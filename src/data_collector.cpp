#include<data_collector.hpp>
#include<yaml/Yaml.hpp>
#include<iostream>
#include <cmath>
using namespace std;
using namespace Yaml;

// function to recieve values from sensors
void Data_collector::get_dynamic_data(float jnt_ctrl_torque_vec,float jnt_position_vec,float jnt_velocity_vec,float jnt_torque_vec,float jnt_command_current_vec,float jnt_current_vec,float friction_torque_vec,float nominal_pos_vec,float nominal_vel_vec){

    this->vec.push_back(jnt_ctrl_torque_vec);
    this->vec.push_back(jnt_position_vec);
    this->vec.push_back(jnt_velocity_vec);
    this->vec.push_back(jnt_torque_vec);
    this->vec.push_back(jnt_command_current_vec);
    this->vec.push_back(jnt_current_vec);
    this->vec.push_back(friction_torque_vec);
}
// Function to save data in csv file
void Data_collector:: save_dynamic_data(){
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

void Data_collector :: save_static_torques_values(double static_torque_value){

    this->static_friction_torque_values.push_back(static_torque_value);

}

void Data_collector :: save_static_torques_breakawy_point(double static_torque_breakaway_point){

    this->static_friction_torque_breakaway_point.push_back(static_torque_breakaway_point);

}
// creating a file to save static torque values
void Data_collector :: create_static_torque_value_file(){

    // int rows1 = this->static_friction_torque_values.size();
    int rows2 = this->static_friction_torque_breakaway_point.size();
    ofstream myfile;
    // myfile.open("../data/static_torque_values.csv");
    
    // for (int i = 0; i < rows1; i++)
    // {
    //     // cout<<this->static_friction_torque_values[i]<<endl;
    //     myfile << this->static_friction_torque_values[i]<< " ";
    // }     
    // myfile.close();

    myfile.open("../data/static_torque_breakaway_point.csv");
    
    for (int i = 0; i < rows2; i++)
    {
        // cout<<this->static_friction_torque_values[i]<<endl;
        myfile << this->static_friction_torque_breakaway_point[i]<< " ";
    }     
    myfile.close();

}

tuple<bool,double,double,bool> Data_collector :: testing(){
    // retVals a;
    return {true,7.9797,7.8686,false };
}


