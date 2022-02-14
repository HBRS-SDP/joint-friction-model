
#include<data_collector.hpp>
#include<yaml/Yaml.hpp>
#include<iostream>
#include <cmath>
using namespace std;
using namespace Yaml;

void Data_collector::get_data(float jnt_ctrl_torque_vec,float jnt_position_vec,float jnt_velocity_vec,float jnt_torque_vec,float jnt_command_current_vec,float jnt_current_vec,float friction_torque_vec,float nominal_pos_vec,float nominal_vel_vec){

    this->vec.push_back(jnt_ctrl_torque_vec);
    this->vec.push_back(jnt_position_vec);
    this->vec.push_back(jnt_velocity_vec);
    this->vec.push_back(jnt_torque_vec);
    this->vec.push_back(jnt_command_current_vec);
    this->vec.push_back(jnt_current_vec);
    this->vec.push_back(friction_torque_vec);
}
// Function to save data in csv file
void Data_collector:: save_data(){
    Yaml::Node root;
    Yaml::Parse(root, "../configs/constants.yml");
    int parameter_count=root["PARAMETERS_COUNT"].As<int>();
    int rows=this->vec.size();
    int counter=1;
    ofstream myfile;
    myfile.open("../data/readings.csv");
    for (int i = 0; i < rows; i++)
    {
        myfile << this->vec[i]<< " ";
        if(counter==parameter_count){
            counter=1;
            myfile <<"\n";
            continue;
        }
        counter++;
    }     
    myfile.close();
}
// function to compare if two values are equal or not
bool equal(double x, double y, double epsilon = 0.00001)
    {
        return (std::fabs(x - y) < epsilon);
    }
// getting static torque values
tuple<bool,double,double,double,double,bool> Data_collector :: get_static_torques_values(bool start_test,double jnt_ctrl_torque_vec,double jnt_velocity_vec, double error,double previous_error,double theta_dot_desired,double nominal_vel_vec,double DT_SEC){

    struct retVals {      
        bool b1;
        double i1, i2,i3,i4;
        bool b2;
    };
    bool test_start=start_test;
    double l_error = error;
    double l_previous_error = previous_error;

    if (test_start){
        if (!equal(0.0,jnt_velocity_vec, 0.0085)){
            this->save_static_torques_values(jnt_ctrl_torque_vec);
            printf("breakaway torque: %f  velocity: %f", jnt_ctrl_torque_vec, jnt_velocity_vec);
            return {test_start,jnt_ctrl_torque_vec,jnt_velocity_vec,l_error,l_previous_error,true};
        }
        else{
            jnt_ctrl_torque_vec = jnt_ctrl_torque_vec + 0.01;
            this->save_static_torques_values(jnt_ctrl_torque_vec);
            return {test_start,jnt_ctrl_torque_vec,jnt_velocity_vec,l_error,l_previous_error,false};
        }
    }
    else {
        // Error calc
        l_error = theta_dot_desired - nominal_vel_vec;
        // PD control
        jnt_ctrl_torque_vec  = 11.5 * l_error; // P controller
        jnt_ctrl_torque_vec += 0.0009 * (l_error - l_previous_error) / DT_SEC; // D term
        l_previous_error = l_error;
        if (equal(0.0,jnt_velocity_vec, 0.0002))
        {
            test_start = true;
            jnt_ctrl_torque_vec = 0.0;
            return {test_start,jnt_ctrl_torque_vec,jnt_velocity_vec,l_error,l_previous_error,false};
        }
        return {test_start,jnt_ctrl_torque_vec,jnt_velocity_vec,l_error,l_previous_error,false};
    }
}
// saving static torque values
void Data_collector :: save_static_torques_values(double static_torque_value){
    this->static_friction_torque_values.push_back(static_torque_value);
}
// creating a file to save static torque values
void Data_collector :: create_static_torque_value_file(){
    int rows = this->static_friction_torque_values.size();
    ofstream myfile;
    myfile.open("../data/static_torque_values.csv");
    for (int i = 0; i < rows; i++)
    {
        myfile << this->static_friction_torque_values[i]<< " ";
    }     
    myfile.close();
}

tuple<bool,double,double,bool> Data_collector :: testing(){
    // retVals a;
    return {true,7.9797,7.8686,false };
}


