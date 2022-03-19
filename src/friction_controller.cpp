#include <friction_controller.hpp>
#include <data_collector.hpp>
#include <yaml/Yaml.hpp>
#include <cmath>
#include <list>

using namespace Yaml;

#define DEG_TO_RAD(x) (x) * 3.14159265358979323846 / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 / 3.14159265358979323846
const int SECOND = 1000000; // num of microsec. in one sec.

// Maximum allowed waiting time during actions
std::chrono::steady_clock::time_point loop_start_time;
std::chrono::steady_clock::time_point dynamic_control_time;
std::chrono::steady_clock::time_point start_cooldown_time;
std::chrono::duration<double, std::micro> loop_interval{};
std::chrono::duration<double, std::micro> total_time_sec{};
std::chrono::duration<double, std::micro> time_diff{};
// Make sure that the control loop runs exactly with the specified frequency
int friction_controller::enforce_loop_frequency(const int dt)
{
    loop_interval = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time);
    if (loop_interval < std::chrono::microseconds(dt)) // Loop is sufficiently fast
    {
        while (loop_interval < std::chrono::microseconds(dt - 1))
            loop_interval = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time);
        return 0;
    }
    else
        return -1; // Loop is too slow
}
// function to check if two vaules are equal to each other
bool equal(double x, double y, double epsilon = 0.00001)
{
    return (std::fabs(x - y) < epsilon);
}

bool friction_controller::cyclic_torque_control(k_api::Base::BaseClient *base, k_api::BaseCyclic::BaseCyclicClient *base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient *actuator_config)
{
    Data_collector data_collector_obj;
    Yaml::Node root;
    Yaml::Parse(root, "../configs/constants.yml");

    const unsigned int ACTUATOR_COUNT = root["actuator_count"].As<unsigned int>();
    const unsigned int TEST_JOINT = root["test_joint"].As<unsigned int>();
    const int RATE_HZ = root["RATE_HZ"].As<int>(); // Hz
    const double Proportional_gain = root["Proportional_gain"].As<double>();
    const double Differential_gain = root["Differential_gain"].As<double>();
    const int DT_MICRO = SECOND / RATE_HZ;
    const double DT_SEC = 1.0 / static_cast<double>(RATE_HZ);
    const double torque_increment_rate = root["initial_torque_increment_rate"].As<double>();
    const double number_of_rate_increments = root["number_of_rate_increments"].As<double>();
    const double breakaway_torque_samples = root["breakaway_torque_samples"].As<double>();
    const double breakaway_torque_iterations = number_of_rate_increments * breakaway_torque_samples;
    const double task_time_limit_sec = root["task_time_limit_sec"].As<double>();
    const double breakaway_torque_velocity_threshold = root["breakaway_torque_velocity_threshold"].As<double>();
    bool start_torque_test = root["start_torque_test"].As<bool>();
    bool get_dynamic_values = false;
    bool get_static_torque = false;
    bool start_test = false; // flag variable to start testing for static torque breakaway point
    bool cool_down = false; // flag variable to cool down the joint after reading data
    string arm_position_configuration = root["arm_position_configuration"].As<string>();
    std::vector<double> dynamic_data_velocity_values;
    int control_loop_delay_count = 0;
    bool return_status = true;
    bool compensate_joint_friction = false;
    bool system_error_occured = false;

    // Low level velocity limits (official Kinova): 2.618 rad/s (149 deg/s) for small and 1.745 rad/s (99 deg/s) for large joints
    std::vector<double> joint_velocity_limits;
    // Motor torque constant K_t (gear ration included - 100:1): official Kinova values
    std::vector<double> motor_torque_constant; // These ones show best result in gravity comp. cases

    // Low-level mode current limits: derived based on safety thresholds outlined in the KINOVA manual
    std::vector<double> joint_current_limits; // Amp
    std::vector<double> home_configuration;
    std::vector<double> zero_configuration, friction_configuration_SDP;
    std::vector<double> joint_inertia;
    std::vector<double> friction_estimation_D_gain;
    std::vector<double> friction_estimation_P_gain;
    std::vector<double> friction_estimation_I_gain;

    for (int i = 0; i < 7; i++)
    {
        joint_velocity_limits.push_back(root["joint_velocity_limits"][i].As<double>());
        motor_torque_constant.push_back(root["motor_torque_constant"][i].As<double>());
        joint_current_limits.push_back(root["joint_current_limits"][i].As<double>());
        home_configuration.push_back(root["home_configuration"][i].As<double>());
        zero_configuration.push_back(root["zero_configuration"][i].As<double>());
        friction_configuration_SDP.push_back(root["friction_configuration_SDP"][i].As<double>());
        joint_inertia.push_back(root["joint_inertia"][i].As<double>());
        friction_estimation_D_gain.push_back(root["friction_estimation_D_gain"][i].As<double>());
        friction_estimation_P_gain.push_back(root["friction_estimation_P_gain"][i].As<double>());
        friction_estimation_I_gain.push_back(root["friction_estimation_I_gain"][i].As<double>());
    }

    for (int i = 0; i < 10; i++)
    {
        dynamic_data_velocity_values.push_back(root["dynamic_data_velocity_values"][i].As<double>());
    }

    Eigen::VectorXd rotor_inertia_eigen(ACTUATOR_COUNT);

    for (unsigned int i = 0; i < ACTUATOR_COUNT; i++)
        rotor_inertia_eigen(i) = joint_inertia[i];

    // Setup the gains
    Eigen::VectorXd friction_estimation_d_gain = Eigen::VectorXd::Map(friction_estimation_D_gain.data(), friction_estimation_D_gain.size());
    Eigen::VectorXd friction_estimation_p_gain = Eigen::VectorXd::Map(friction_estimation_P_gain.data(), friction_estimation_P_gain.size());
    Eigen::VectorXd friction_estimation_i_gain = Eigen::VectorXd::Map(friction_estimation_I_gain.data(), friction_estimation_I_gain.size());

    FrictionObserver friction_observer(ACTUATOR_COUNT, DT_SEC, rotor_inertia_eigen, friction_estimation_d_gain, friction_estimation_p_gain,
                                       friction_estimation_i_gain, friction_observer_type::PD, integration_method::SYMPLECTIC_EULER, 0, 0.0);
    // Prepare state and control variables
    Eigen::VectorXd jnt_ctrl_torque_vec(ACTUATOR_COUNT), jnt_position_vec(ACTUATOR_COUNT), jnt_velocity_vec(ACTUATOR_COUNT), jnt_torque_vec(ACTUATOR_COUNT), jnt_command_current_vec(ACTUATOR_COUNT), jnt_current_vec(ACTUATOR_COUNT),
        friction_torque_vec(ACTUATOR_COUNT), nominal_pos_vec(ACTUATOR_COUNT), nominal_vel_vec(ACTUATOR_COUNT), initial_position_vec(ACTUATOR_COUNT), initial_velocity_vec(ACTUATOR_COUNT);
    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;
    auto servoing_mode = k_api::Base::ServoingModeInformation();

    // checking arm position configuration name
    if (arm_position_configuration != "Home" &&
        arm_position_configuration != "Zero" &&
        arm_position_configuration != "friction_configuration_SDP")
    {
        printf("Unknown position configuration name. Exiting.");
        return false;
    }
    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch (...)
    {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }
    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        for (unsigned int i = 0; i < ACTUATOR_COUNT; i++)
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);
        // Set actuators in torque mode
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, 1);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, 2);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, 4);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, 5);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, 6);

        switch (TEST_JOINT)
        {
        case 2:
            control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::CURRENT);
            actuator_config->SetControlMode(control_mode_message, 3);

            control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
            actuator_config->SetControlMode(control_mode_message, 7);
            break;

        case 6:
            control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
            actuator_config->SetControlMode(control_mode_message, 3);

            control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::CURRENT);
            actuator_config->SetControlMode(control_mode_message, 7);
            break;
            
        default:
            control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
            actuator_config->SetControlMode(control_mode_message, 3);

            control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
            actuator_config->SetControlMode(control_mode_message, 7);

            printf("Wrong joint selected!");
            break;
        }
        // Velocity control
        double theta_dot_desired = 0.0;
        double error = 0.0, previous_error = 0.0;

        int iteration_limit = 0;
        int dynamic_vel_counter = 0;

        if (start_torque_test)
        {
            jnt_ctrl_torque_vec(TEST_JOINT) = root["jnt_ctrl_torque_vec_start"].As<double>();
            std::cout<<"Collecting data"<<endl;
            get_static_torque = true;
            start_test = false;
            iteration_limit = breakaway_torque_iterations + dynamic_data_velocity_values.size();
        }

        for (int iteration_counter = 0; iteration_counter < iteration_limit; iteration_counter++)
        {
            if (iteration_counter >= breakaway_torque_iterations)
            {
                compensate_joint_friction = true;
                get_static_torque = false;
                get_dynamic_values = true;
                theta_dot_desired = dynamic_data_velocity_values[dynamic_vel_counter];
                dynamic_vel_counter++;
                error = 0.0;
                previous_error = 0.0;

                // Initialize friction estimator (feedforward component)
                for (unsigned int i = 0; i < ACTUATOR_COUNT; i++)
                {
                    jnt_position_vec(i) = DEG_TO_RAD(base_feedback.actuators(i).position());
                    jnt_velocity_vec(i) = DEG_TO_RAD(base_feedback.actuators(i).velocity());
                }

                friction_observer.setInitialState(jnt_position_vec, jnt_velocity_vec);
                initial_position_vec = jnt_position_vec;
                initial_velocity_vec = jnt_velocity_vec;

                // velocity control
                if (theta_dot_desired > joint_velocity_limits[TEST_JOINT])
                    theta_dot_desired = joint_velocity_limits[TEST_JOINT];
                else if (theta_dot_desired < -joint_velocity_limits[TEST_JOINT])
                    theta_dot_desired = -joint_velocity_limits[TEST_JOINT];
            }
            // ##########################################################################
            // Real-time loop
            // ##########################################################################
            const std::chrono::steady_clock::time_point control_start_time_sec = std::chrono::steady_clock::now();
            while (total_time_sec.count() / SECOND < task_time_limit_sec)
            {
                loop_start_time = std::chrono::steady_clock::now();
                total_time_sec = std::chrono::duration<double, std::micro>(loop_start_time - control_start_time_sec);

                try
                {
                    base_feedback = base_cyclic->RefreshFeedback();
                }
                catch (Kinova::Api::KDetailedException &ex)
                {
                    std::cout << "Kortex exception 1: " << ex.what() << std::endl;

                    std::cout << "KError error_code 1: " << ex.getErrorInfo().getError().error_code() << std::endl;
                    std::cout << "KError sub_code 1: " << ex.getErrorInfo().getError().error_sub_code() << std::endl;
                    std::cout << "KError sub_string 1: " << ex.getErrorInfo().getError().error_sub_string() << std::endl;
                    // Error codes by themselves are not very verbose if you don't see their corresponding enum value
                    // You can use google::protobuf helpers to get the string enum element for every error code and sub-code
                    std::cout << "Error code string equivalent 1: " << Kinova::Api::ErrorCodes_Name(Kinova::Api::ErrorCodes(ex.getErrorInfo().getError().error_code())) << std::endl;
                    std::cout << "Error sub-code string equivalent 1: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())) << std::endl;
                    system_error_occured = true;
                    break;
                }
                catch (std::runtime_error &ex2)
                {
                    std::cout << "runtime error 1: " << ex2.what() << std::endl;
                    system_error_occured = true;
                    break;
                }
                catch (...)
                {
                    std::cout << "Unknown error 1." << std::endl;
                    system_error_occured = true;
                    break;
                }

                for (unsigned int i = 0; i < ACTUATOR_COUNT; i++)
                {
                    jnt_position_vec(i) = DEG_TO_RAD(base_feedback.actuators(i).position());
                    jnt_velocity_vec(i) = DEG_TO_RAD(base_feedback.actuators(i).velocity());
                    jnt_torque_vec(i) = base_feedback.actuators(i).torque();
                    jnt_current_vec(i) = base_feedback.actuators(i).current_motor();
                }

                // Get the friction-free state
                if (compensate_joint_friction)
                    friction_observer.getNominalState(nominal_pos_vec, nominal_vel_vec);
                else
                {
                    nominal_pos_vec = jnt_position_vec;
                    nominal_vel_vec = jnt_velocity_vec;
                }

                for (unsigned int i = 0; i < ACTUATOR_COUNT; i++)
                {
                    if (arm_position_configuration == "Home")
                    {
                        if (i != TEST_JOINT)
                            base_command.mutable_actuators(i)->set_position(home_configuration[i]);
                    }
                    else if (arm_position_configuration == "Zero")
                    {
                        if (i != TEST_JOINT)
                            base_command.mutable_actuators(i)->set_position(zero_configuration[i]);
                    }
                    else
                    {
                        if (i != TEST_JOINT)
                            base_command.mutable_actuators(i)->set_position(friction_configuration_SDP[i]);
                    }
                }
                base_command.mutable_actuators(TEST_JOINT)->set_position(base_feedback.actuators(TEST_JOINT).position());

                if (start_test && !cool_down)
                {
                    if (get_static_torque)
                    {
                        if (!equal(0.0, jnt_velocity_vec(TEST_JOINT), breakaway_torque_velocity_threshold))
                        {

                            data_collector_obj.save_static_torques_breakawy_point(jnt_ctrl_torque_vec(TEST_JOINT));
                            data_collector_obj.save_static_torques_rate((torque_increment_rate * ((int)(iteration_counter / 10) + 1)));

                            cool_down = true;
                            start_cooldown_time = std::chrono::steady_clock::now();
                            jnt_ctrl_torque_vec(TEST_JOINT) = 0.0;
                        }
                        else
                        {
                            jnt_ctrl_torque_vec(TEST_JOINT) = jnt_ctrl_torque_vec(TEST_JOINT) + (torque_increment_rate * ((int)(iteration_counter / 10) + 1)); // to increase the increment value of torque_value every 10 iterations by 0.05
                            data_collector_obj.save_static_torques_values(jnt_ctrl_torque_vec(TEST_JOINT));
                        }
                    }
                    else if (get_dynamic_values)
                    {
                        dynamic_control_time = std::chrono::steady_clock::now();
                        time_diff = std::chrono::duration<double, std::micro>(dynamic_control_time - control_start_time_sec);

                        if ((time_diff.count() / SECOND) > 3)
                        {
                            cool_down = true;
                            start_cooldown_time = std::chrono::steady_clock::now();
                            jnt_ctrl_torque_vec(TEST_JOINT) = 0.0;
                        }

                        else
                        {
                            // Error calc
                            error = theta_dot_desired - nominal_vel_vec(TEST_JOINT);

                            // PD control
                            jnt_ctrl_torque_vec(TEST_JOINT) = Proportional_gain * error;                                // Proportional gain
                            jnt_ctrl_torque_vec(TEST_JOINT) += Differential_gain * (error - previous_error) / DT_SEC; // Differential gain
                            previous_error = error;
                            // Estimate friction in joints
                            if (compensate_joint_friction)
                            {
                                friction_observer.estimateFrictionTorque(jnt_position_vec, jnt_velocity_vec, jnt_ctrl_torque_vec, jnt_torque_vec, friction_torque_vec);
                                jnt_ctrl_torque_vec(TEST_JOINT) -= friction_torque_vec(TEST_JOINT);
                            }
                        }
                    }
                    else
                    {
                        std::cout << "Uknown experiment mode" << std::endl;
                        system_error_occured = true;
                        break;
                    }
                }
                else if (!start_test && !cool_down)
                {
                    // Error calc
                    error = 0.0 - nominal_vel_vec(TEST_JOINT);

                    // PD control
                    jnt_ctrl_torque_vec(TEST_JOINT) = Proportional_gain * error;                                // Proportional gain
                    jnt_ctrl_torque_vec(TEST_JOINT) += Differential_gain * (error - previous_error) / DT_SEC; // Differential gain
                    previous_error = error;

                    if (equal(0.0, jnt_velocity_vec(TEST_JOINT), 0.0002))
                    {
                        start_test = true;

                        jnt_ctrl_torque_vec(TEST_JOINT) = root["jnt_ctrl_torque_vec_start"].As<double>();
                    }
                }
                else
                {
                    if (std::chrono::duration<double, std::micro>(loop_start_time - control_start_time_sec).count() > 3000000)
                    {
                        cool_down = false;
                        start_test = false;

                        jnt_ctrl_torque_vec(TEST_JOINT) = root["jnt_ctrl_torque_vec_start"].As<double>();
                        break;
                    }
                }

                jnt_command_current_vec(TEST_JOINT) = jnt_ctrl_torque_vec(TEST_JOINT) / motor_torque_constant[TEST_JOINT];
                if (jnt_command_current_vec(TEST_JOINT) > joint_current_limits[TEST_JOINT])
                    jnt_command_current_vec(TEST_JOINT) = joint_current_limits[TEST_JOINT];

                else if (jnt_command_current_vec(TEST_JOINT) < -joint_current_limits[TEST_JOINT])
                    jnt_command_current_vec(TEST_JOINT) = -joint_current_limits[TEST_JOINT];

                base_command.mutable_actuators(TEST_JOINT)->set_current_motor(jnt_command_current_vec(TEST_JOINT));

                // Incrementing identifier ensures actuators can reject out of time frames
                base_command.set_frame_id(base_command.frame_id() + 1);
                if (base_command.frame_id() > 65535)
                    base_command.set_frame_id(0);
                    
                for (unsigned int idx = 0; idx < ACTUATOR_COUNT; idx++)
                    base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());

                try
                {
                    base_feedback = base_cyclic->Refresh(base_command, 0);
                }
                catch (k_api::KDetailedException &ex)
                {
                    std::cout << "Kortex exception 2: " << ex.what() << std::endl;
                    std::cout << "Kortex exception 2: " << ex.what() << std::endl;

                    std::cout << "KError error_code 2: " << ex.getErrorInfo().getError().error_code() << std::endl;
                    std::cout << "KError sub_code 2: " << ex.getErrorInfo().getError().error_sub_code() << std::endl;
                    std::cout << "KError sub_string 2: " << ex.getErrorInfo().getError().error_sub_string() << std::endl;
                    // Error codes by themselves are not very verbose if you don't see their corresponding enum value
                    // You can use google::protobuf helpers to get the string enum element for every error code and sub-code
                    std::cout << "Error code string equivalent 2: " << Kinova::Api::ErrorCodes_Name(Kinova::Api::ErrorCodes(ex.getErrorInfo().getError().error_code())) << std::endl;
                    std::cout << "Error sub-code string equivalent 2: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())) << std::endl;
                    system_error_occured = true;
                    break;
                }
                catch (std::runtime_error &ex2)
                {
                    std::cout << "runtime error 2: " << ex2.what() << std::endl;
                    system_error_occured = true;
                    break;
                }
                catch (...)
                {
                    std::cout << "Unknown error. 2" << std::endl;
                    system_error_occured = true;
                    break;
                }
                if (get_dynamic_values)
                {
                    data_collector_obj.get_dynamic_data(jnt_ctrl_torque_vec[TEST_JOINT], jnt_position_vec[TEST_JOINT], jnt_velocity_vec[TEST_JOINT],
                                                        jnt_torque_vec[TEST_JOINT], jnt_command_current_vec[TEST_JOINT], jnt_current_vec[TEST_JOINT], friction_torque_vec[TEST_JOINT]);
                }

                // Enforce the constant loop time and count how many times the loop was late
                if (enforce_loop_frequency(DT_MICRO) != 0)
                    control_loop_delay_count++;
            }
            if (system_error_occured == true)
                break;

            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            // function to save data to csv file
            if (get_static_torque)
                data_collector_obj.create_static_torque_value_file();
            if (get_dynamic_values)
                data_collector_obj.save_dynamic_data(dynamic_data_velocity_values[dynamic_vel_counter - 1]);
        }

        // Set actuators back in position
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);

        for (unsigned int actuator_id = 1; actuator_id < ACTUATOR_COUNT + 1; actuator_id++)
            actuator_config->SetControlMode(control_mode_message, actuator_id);
    }
    catch (k_api::KDetailedException &ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error &ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }
    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    return return_status;
}
