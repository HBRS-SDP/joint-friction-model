
#include<data_collector.hpp>
using namespace std;


Data_collector::Data_collector() {     // Constructor
    PARAMETERS_COUNT=9; //no of parameters we want
    TEST_JOINT=6;  //joint to be get values from
    task_time_limit_sec=10; //Time duration to run the robot
    estimated_loop_iterations = static_cast<int> (task_time_limit_sec * 900 + 20);
   
    
}
// function to recieve values from sensors
void Data_collector::get_data(float jnt_ctrl_torque_vec,float jnt_position_vec,float jnt_velocity_vec,float jnt_torque_vec,float jnt_command_current_vec,float jnt_current_vec,float friction_torque_vec,float nominal_pos_vec,float nominal_vel_vec){

    this->vec.push_back(jnt_ctrl_torque_vec);
    this->vec.push_back(jnt_position_vec);
    this->vec.push_back(jnt_velocity_vec);
    this->vec.push_back(jnt_torque_vec);
    this->vec.push_back(jnt_command_current_vec);
    this->vec.push_back(jnt_current_vec);
    this->vec.push_back(friction_torque_vec);
    this->vec.push_back(nominal_pos_vec);
    this->vec.push_back(nominal_vel_vec);  
    
}

// Function to save data in csv file
void Data_collector:: save_data(){
    int rows=this->vec.size();
    int counter=1;
    ofstream myfile;
    myfile.open("../data/readings.csv");
    
    for (int i = 0; i < rows; i++)
    {

        myfile << this->vec[i]<< " ";
        if(counter==this->PARAMETERS_COUNT){
            counter=1;
            myfile <<"\n";
            continue;
        }
        counter++;
    }  
    
    myfile.close();

}





