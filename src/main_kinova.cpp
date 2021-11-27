#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <stdlib.h> /* abs */

#include <chrono>
#include <time.h>
#include <unistd.h>
#include <eigen3/Eigen/Core>
#include <KDetailedException.h>
#include <friction_observer.hpp>
#include <data_reader.hpp>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>

const int SECOND = 1000000; // num of microsec. in one sec.

namespace k_api = Kinova::Api;

#define IP_ADDRESS "192.168.1.10"

#define PORT 10000
#define PORT_REAL_TIME 10001

#define ACTUATOR_COUNT 7
#define DEG_TO_RAD(x) (x) * 3.14159265358979323846 / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 / 3.14159265358979323846
// #define PARAMETERS_COUNT 11
// #define TEST_JOINT 6

float TIME_DURATION = 30.0f; // Duration of the example (seconds)

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};
std::chrono::steady_clock::time_point loop_start_time;
std::chrono::duration <double, std::micro> loop_interval{};
std::chrono::duration <double, std::micro> total_time_sec{};

//Make sure that the control loop runs exactly with the specified frequency
int enforce_loop_frequency(const int dt)
{
    loop_interval = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time);

    if (loop_interval < std::chrono::microseconds(dt)) // Loop is sufficiently fast
    {
        while (loop_interval < std::chrono::microseconds(dt - 1))
            loop_interval = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time);

        return 0;
    }
    else return -1; //Loop is too slow
}

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)> 
    create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

/**************************
 * Example core functions *
 **************************/
void example_move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);

    for (auto action : action_list.action_list()) 
    {
        if (action.name() == "Home")
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
        return;
    }
    else
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
        }
        const auto promise_event = finish_future.get();
    }
    base->Stop();
    // std::this_thread::sleep_for(std::chrono::milliseconds(1500));
}

bool example_cyclic_torque_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient* actuator_config)
{
    Data_reader data_reader_obj;

    int PARAMETERS_COUNT= data_reader_obj.PARAMETERS_COUNT;
    int TEST_JOINT=data_reader_obj.TEST_JOINT;
    constexpr int RATE_HZ = 900; // Hz
    const int DT_MICRO = SECOND / RATE_HZ;
    const double DT_SEC = 1.0 / static_cast<double>(RATE_HZ);
    constexpr double task_time_limit_sec = 10.0;

    constexpr int estimated_loop_iterations = static_cast<int> (task_time_limit_sec * RATE_HZ + 20); //aprox number of iterations +20 

    int iteration_count = 0;
    int control_loop_delay_count = 0;
    bool return_status = true;
    bool compensate_joint_friction = true;

    // Low level velocity limits (official Kinova): 2.618 rad/s (149 deg/s) for small and 1.745 rad/s (99 deg/s) for large joints
    const std::vector<double> joint_velocity_limits {1.74, 1.74, 1.74, 1.74, 2.6, 2.6, 2.6};

    // Motor torque constant K_t (gear ration included - 100:1): official Kinova values
    const std::vector<double> motor_torque_constant {11.0, 11.0, 11.0, 11.0, 7.6, 7.6, 7.6}; // These ones show best result in gravity comp. cases

    // Low-level mode current limits: derived based on safety thresholds outlined in the KINOVA manual
    const std::vector<double> joint_current_limits {9.5, 9.5, 9.5, 9.5, 5.5, 5.5, 5.5}; // Amp

    const std::vector<double> home_configuration {0.0, 15.0, 180.0, 230.0, 0.0, 55.0, 90.0};
    const std::vector<double> zero_configuration {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    const std::vector<double> joint_inertia {0.5580, 0.5580, 0.5580, 0.5580, 0.1389, 0.1389, 0.1389};
    Eigen::VectorXd rotor_inertia_eigen(ACTUATOR_COUNT);
    for (unsigned int i = 0; i < ACTUATOR_COUNT; i++) 
        rotor_inertia_eigen(i) = joint_inertia[i];

    // Setup the gains
    Eigen::VectorXd friction_estimation_d_gain = (Eigen::VectorXd(7) << 30.0, 30.0, 30.0, 30.0, 20.0, 20.0, 20.0).finished();
    Eigen::VectorXd friction_estimation_p_gain = (Eigen::VectorXd(7) << 70.0, 70.0, 70.0, 70.0, 50.0, 50.0, 50.0).finished();
    Eigen::VectorXd friction_estimation_i_gain = (Eigen::VectorXd(7) << 30.0, 30.0, 30.0, 30.0, 20.0, 20.0, 20.0).finished();
    FrictionObserver friction_observer(ACTUATOR_COUNT, DT_SEC, rotor_inertia_eigen, friction_estimation_d_gain, friction_estimation_p_gain,
                                       friction_estimation_i_gain, friction_observer_type::PD, integration_method::SYMPLECTIC_EULER, 0, 0.0);

    // Prepare state and control variables
    Eigen::VectorXd jnt_ctrl_torque_vec(ACTUATOR_COUNT), jnt_position_vec(ACTUATOR_COUNT), jnt_velocity_vec(ACTUATOR_COUNT), jnt_torque_vec(ACTUATOR_COUNT), jnt_command_current_vec(ACTUATOR_COUNT), jnt_current_vec(ACTUATOR_COUNT),
                  friction_torque_vec(ACTUATOR_COUNT), nominal_pos_vec(ACTUATOR_COUNT), nominal_vel_vec(ACTUATOR_COUNT), initial_position_vec(ACTUATOR_COUNT), initial_velocity_vec(ACTUATOR_COUNT);

    // Prepare matrix for saving values
    double parameters[estimated_loop_iterations][PARAMETERS_COUNT];
    // data_reader_obj.get_params(estimated_loop_iterations);

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;
    
    auto servoing_mode = k_api::Base::ServoingModeInformation();


    //Open a file (csv)
    ofstream myfile;
    myfile.open("../data/readings.csv"); 

    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch(...)
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
        for (int i = 0; i < ACTUATOR_COUNT; i++)
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
        actuator_config->SetControlMode(control_mode_message, 3);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, 4);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, 5);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, 6);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::CURRENT);
        actuator_config->SetControlMode(control_mode_message, 7);

        compensate_joint_friction = true;
        double error = 0.0, previous_error = 0.0;

        // Initialize friction estimator (feedforward component)
        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            jnt_position_vec(i) = DEG_TO_RAD(base_feedback.actuators(i).position());
            jnt_velocity_vec(i) = DEG_TO_RAD(base_feedback.actuators(i).velocity());
        }
        friction_observer.setInitialState(jnt_position_vec, jnt_velocity_vec);

        initial_position_vec = jnt_position_vec;
        initial_velocity_vec = jnt_velocity_vec;

        // Velocity control
        double theta_dot_desired = 1.3;
        if      (theta_dot_desired >  joint_velocity_limits[TEST_JOINT]) theta_dot_desired =  joint_velocity_limits[TEST_JOINT];
        else if (theta_dot_desired < -joint_velocity_limits[TEST_JOINT]) theta_dot_desired = -joint_velocity_limits[TEST_JOINT];

        // ##########################################################################
        // Real-time loop
        // ##########################################################################
        const std::chrono::steady_clock::time_point control_start_time_sec = std::chrono::steady_clock::now();
        while (total_time_sec.count() / SECOND < task_time_limit_sec)
        {
            iteration_count++;
            loop_start_time = std::chrono::steady_clock::now();
            total_time_sec = std::chrono::duration<double, std::micro>(loop_start_time - control_start_time_sec);

            try
            {
                base_feedback = base_cyclic->RefreshFeedback();
            }
            catch (Kinova::Api::KDetailedException& ex)
            {
                std::cout << "Kortex exception 1: " << ex.what() << std::endl;

                std::cout << "KError error_code 1: " << ex.getErrorInfo().getError().error_code() << std::endl;
                std::cout << "KError sub_code 1: " << ex.getErrorInfo().getError().error_sub_code() << std::endl;
                std::cout << "KError sub_string 1: " << ex.getErrorInfo().getError().error_sub_string() << std::endl;

                // Error codes by themselves are not very verbose if you don't see their corresponding enum value
                // You can use google::protobuf helpers to get the string enum element for every error code and sub-code 
                std::cout << "Error code string equivalent 1: " << Kinova::Api::ErrorCodes_Name(Kinova::Api::ErrorCodes(ex.getErrorInfo().getError().error_code())) << std::endl;
                std::cout << "Error sub-code string equivalent 1: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())) << std::endl;
                break;
            }
            catch (std::runtime_error& ex2)
            {
                std::cout << "runtime error 1: " << ex2.what() << std::endl;
                break;
            }
            catch(...)
            {
                std::cout << "Unknown error 1." << std::endl;
                break;
            }

            for (int i = 0; i < ACTUATOR_COUNT; i++)
            {
                jnt_position_vec(i) = DEG_TO_RAD(base_feedback.actuators(i).position());
                jnt_velocity_vec(i) = DEG_TO_RAD(base_feedback.actuators(i).velocity());
                jnt_torque_vec  (i) = base_feedback.actuators(i).torque();
                jnt_current_vec (i) = base_feedback.actuators(i).current_motor();
            }

            // Get the friction-free state
            if (compensate_joint_friction) friction_observer.getNominalState(nominal_pos_vec, nominal_vel_vec);
            else
            {
                nominal_pos_vec = jnt_position_vec;
                nominal_vel_vec = jnt_velocity_vec;
            }

            for (int i = 0; i < ACTUATOR_COUNT; i++)
            {
                if (i != TEST_JOINT) base_command.mutable_actuators(i)->set_position(home_configuration[i]);
            }
            base_command.mutable_actuators(TEST_JOINT)->set_position(base_feedback.actuators(TEST_JOINT).position());

            // Error calc
            error = theta_dot_desired - nominal_vel_vec(TEST_JOINT);

            // PD control
            jnt_ctrl_torque_vec(TEST_JOINT)  = 11.5 * error; // P controller
            jnt_ctrl_torque_vec(TEST_JOINT) += 0.0009 * (error - previous_error) / DT_SEC; // D term
            previous_error = error;

            // Estimate friction in joints
            if (compensate_joint_friction)
            {
                friction_observer.estimateFrictionTorque(jnt_position_vec, jnt_velocity_vec, jnt_ctrl_torque_vec, jnt_torque_vec, friction_torque_vec);
                jnt_ctrl_torque_vec(TEST_JOINT) -= friction_torque_vec(TEST_JOINT);
            }
 
            jnt_command_current_vec(TEST_JOINT) = jnt_ctrl_torque_vec(TEST_JOINT) / motor_torque_constant[TEST_JOINT];
            if      (jnt_command_current_vec(TEST_JOINT) >  joint_current_limits[TEST_JOINT]) jnt_command_current_vec(TEST_JOINT) =  joint_current_limits[TEST_JOINT];
            else if (jnt_command_current_vec(TEST_JOINT) < -joint_current_limits[TEST_JOINT]) jnt_command_current_vec(TEST_JOINT) = -joint_current_limits[TEST_JOINT];
            base_command.mutable_actuators(TEST_JOINT)->set_current_motor(jnt_command_current_vec(TEST_JOINT));

            // Incrementing identifier ensures actuators can reject out of time frames
            base_command.set_frame_id(base_command.frame_id() + 1);
            if (base_command.frame_id() > 65535) base_command.set_frame_id(0);
            for (int idx = 0; idx < ACTUATOR_COUNT; idx++)
                base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());

            try
            {
                base_feedback = base_cyclic->Refresh(base_command, 0);
            }
            catch (k_api::KDetailedException& ex)
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
                break;
            }
            catch (std::runtime_error& ex2)
            {
                std::cout << "runtime error 2: " << ex2.what() << std::endl;
                break;
            }
            catch(...)
            {
                std::cout << "Unknown error. 2" << std::endl;
                break;
            }

            parameters[iteration_count-1][0]  = jnt_ctrl_torque_vec[TEST_JOINT];
            parameters[iteration_count-1][1]  = jnt_position_vec[TEST_JOINT];
            parameters[iteration_count-1][2]  = jnt_velocity_vec[TEST_JOINT];
            parameters[iteration_count-1][3]  = jnt_torque_vec[TEST_JOINT];
            parameters[iteration_count-1][4]  = jnt_command_current_vec[TEST_JOINT];
            parameters[iteration_count-1][5]  = jnt_current_vec[TEST_JOINT];
            parameters[iteration_count-1][6]  = friction_torque_vec[TEST_JOINT];
            parameters[iteration_count-1][7]  = nominal_pos_vec[TEST_JOINT];
            parameters[iteration_count-1][8]  = nominal_vel_vec[TEST_JOINT];


            // Enforce the constant loop time and count how many times the loop was late
            if (enforce_loop_frequency(DT_MICRO) != 0) control_loop_delay_count++;
        }

        // Set actuators back in position 
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        for (int actuator_id = 1; actuator_id < ACTUATOR_COUNT + 1; actuator_id++)
            actuator_config->SetControlMode(control_mode_message, actuator_id);
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }
    
    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    try
    {   
        for (int i = 0; i < iteration_count; i++)
        {
            for (int j = 0; j < PARAMETERS_COUNT; j++)
                myfile << parameters[i][j] << " ";
            myfile <<"\n";
        }  
    
        myfile.close();
    }
    catch(const std::exception& e)
    {
        //comment a message
        std::cerr << e.what() << '\n';
    }

    // std::cout << "last current command: " << jnt_command_current(TEST_JOINT) << std::endl;
    std::cout << "Torque control example clean exit. \nTotal run time: " << total_time_sec.count() / SECOND
              << "   Loop iteration count: "                             << iteration_count
              << "   Control Loop delay count: "                         << control_loop_delay_count << std::endl;

    return return_status;
}

int main(int argc, char **argv)
{
    // Create API objects
    
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    
    std::cout << "Creating transport objects" << std::endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    // transport->connect(IP_ADDRESS, PORT);

    std::cout << "Creating transport real time objects" << std::endl;
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    // transport_real_time->connect(IP_ADDRESS, PORT_REAL_TIME);
    
    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("kinova1_area4251");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    // session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    // session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);
    
    // Example core
    std::cout << "starting example" << std::endl;
    // example_move_to_home_position(base);
    auto isOk = example_cyclic_torque_control(base, base_cyclic, actuator_config);
    if (!isOk)
    {
        std::cout << "There has been an unexpected error in example_cyclic_torque_control() function." << endl;;
    }

    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;
}