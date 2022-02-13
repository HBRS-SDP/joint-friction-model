#ifndef Data_collector_HPP
#define Data_collector_HPP
#include<iostream>
#include<fstream>
#include <stdexcept>
#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>
using namespace std;



class Data_collector {     // The class
  public:           // Access specifier

    // int PARAMETERS_COUNT;
    // int TEST_JOINT;
    // double task_time_limit_sec;
    // int estimated_loop_iterations;
    
    std::vector<float> vec = std::vector<float>();
    std::vector<double> static_friction_torque_values = std::vector<double>();
    // Data_collector();
    // ~Data_collector(){};
    void get_data(float jnt_ctrl_torque_vec,float jnt_position_vec,float jnt_velocity_vec,float jnt_torque_vec,float jnt_command_current_vec,float jnt_current_vec,float friction_torque_vec,float nominal_pos_vec,float nominal_vel_vec);
    void save_data();
    void get_params(int estimated_loop_iterations);
    void save_static_torques_values(double torque_value);
    tuple<bool,double,double,double,double,bool> get_static_torques_values(bool start_test,double jnt_ctrl_torque_vec,double jnt_velocity_vec,double error,double previous_error,double theta_dot_desired,double nominal_vel_vec,double DT_SEC);
    void create_static_torque_value_file();
    tuple<bool,double,double,bool> testing();

    
};

#endif

