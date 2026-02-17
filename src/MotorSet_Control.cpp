#include <array>
#include <vector>
#include <iostream>
#include <thread>
#include "ServoMotor.h"

int main(){

    // Initialize set of servo motors for robotic arm joints
    ServoMotor ja1, ja2, jb1, jb2;
    ja1.set_label("Servo JA1");
    ja2.set_label("Servo JA2");
    jb1.set_label("Servo JB1");
    jb2.set_label("Servo JB2");

    // Store motor objects in an array
    std::array<ServoMotor, 4> motor_set = {ja1, ja2, jb1, jb2};

    // Create a TEST COMMANDS set for the motor set
    std::array<double, motor_set.size()> cmd_set = {0.5, 1.0, 1.5, 2.0};

    // Generate a thread pool
    double sim_time = 5.0;
    double sample_period = 0.01;
    std::vector<std::thread> thread_pool;

    unsigned int n = std::thread::hardware_concurrency();
    std::cout << n << " concurrent threads are being supported by the hardware." << std::endl;

    for(int i = 0; i < motor_set.size(); i++){
        thread_pool.emplace_back(
            &ServoMotor::set_position_state, 
            &motor_set[i],
            cmd_set[i], sim_time, sample_period
        );
        
        thread_pool.emplace_back(
            &ServoMotor::get_position_state,
            &motor_set[i]
        );
    }

    // Join Threads
    for(auto& th : thread_pool){
        th.join();
    }

    return 0;
}