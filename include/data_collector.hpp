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
    std::vector<float> sensor_values = std::vector<float>();
    std::vector<double> static_friction_torque_values = std::vector<double>();
    std::vector<double> static_friction_torque_breakaway_point = std::vector<double>();
    std::vector<double> static_friction_torque_rate = std::vector<double>();
    void get_dynamic_data(double jnt_ctrl_torque_vec,double jnt_position_vec,
                          double jnt_velocity_vec,double jnt_torque_vec,double jnt_command_current_vec,double jnt_current_vec,double friction_torque_vec);
    void save_dynamic_data(double velocity_value);
    void save_static_torques_values(double torque_value);
    void create_static_torque_value_file();
    void save_static_torques_breakawy_point(double static_torque_value);
    void save_static_torques_rate(double static_torque_rate);
};
#endif
