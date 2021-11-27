#ifndef DATA_READER_HPP
#define DATA_READER_HPP
#include<iostream>
#include<fstream>
#include <stdexcept>
using namespace std;



class Data_reader {     // The class
  public:           // Access specifier

    int PARAMETERS_COUNT;
    int TEST_JOINT;
    double task_time_limit_sec;
    int estimated_loop_iterations ;
    // double parameters;
    Data_reader() ;
    ~Data_reader(){}
    void get_data(int iteration_count,float jnt_ctrl_torque_vec,float jnt_position_vec,float jnt_velocity_vec,float jnt_torque_vec,float jnt_command_current_vec,float jnt_current_vec,float friction_torque_vec,float nominal_pos_vec,float nominal_vel_vec);

    void save_data();
    void get_params(int estimated_loop_iterations);

};

#endif

