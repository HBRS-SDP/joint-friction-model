
#include<data_reader.hpp>
using namespace std;


Data_reader::Data_reader() {     // Constructor
    PARAMETERS_COUNT=7;
    TEST_JOINT=6;
    task_time_limit_sec=10;
    // estimated_loop_iterations = static_cast<int> (task_time_limit_sec * 900 + 20);
    // int parameters[estimated_loop_iterations][PARAMETERS_COUNT];
    
}

void Data_reader::get_data(int iteration_count,float jnt_ctrl_torque_vec,float jnt_position_vec,float jnt_velocity_vec,float jnt_torque_vec,float jnt_command_current_vec,float jnt_current_vec,float friction_torque_vec,float nominal_pos_vec,float nominal_vel_vec){

    // parameters[iteration_count-1][0]  = jnt_ctrl_torque_vec;
    // parameters[iteration_count-1][1]  = jnt_position_vec;
    // parameters[iteration_count-1][2]  = jnt_velocity_vec;
    // parameters[iteration_count-1][3]  = jnt_torque_vec;
    // parameters[iteration_count-1][4]  = jnt_command_current_vec;
    // parameters[iteration_count-1][5]  = jnt_current_vec;
    // parameters[iteration_count-1][6]  = friction_torque_vec;
    // parameters[iteration_count-1][7]  = nominal_pos_vec;
    // parameters[iteration_count-1][8]  = nominal_vel_vec;
}

void Data_reader:: save_data(){
   
    ofstream myfile;
    myfile.open("../data/readings.csv");
    
    
    myfile.close();

}

void Data_reader::get_params(int estimated_loop_iterations){
    estimated_loop_iterations= estimated_loop_iterations;


}




