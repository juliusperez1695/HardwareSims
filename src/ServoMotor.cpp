/*

ServoMotor.cpp
Author: Julius Perez
Description: Defines the ServoMotor class's constructors, destructors, and methods

*/

#include <iostream>
#include <cstdio>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <sciplot/sciplot.hpp>
#include "DCMotor.h"
#include "PIDController.h"
#include "ServoMotor.h"

using namespace sciplot;

std::mutex mtx;
std::condition_variable cv;
bool notified = false;
bool write_complete = false;

ServoMotor :: ServoMotor() {
    // Definition of constructor
    this->motor = DCMotor();
    this->ctrlr = PIDController(15.0, 0.0, 7.0);
}

ServoMotor :: ~ServoMotor() {
    // Definition of destructor - deallocate memory as necessary
}

double ServoMotor :: get_position_state(void) {
    if(this->is_rotating){
        
        // Run a separate thread for reading during position change

        FILE* pipe = popen("gnuplot -persist", "w");
        if (!pipe) return -1;

        fprintf(pipe, "set title 'Live Data Plot'\n");
        fprintf(pipe, "set xlabel 'Time (s)'\n");
        fprintf(pipe, "set ylabel 'Angular Position (rad)'\n");
        fprintf(pipe, "set xrange [0:6]\n");
        fprintf(pipe, "set yrange [0:6]\n");
        //fprintf(pipe, "plot '-' with points title 'Test'\n");

        std::unique_lock<std::mutex> lock(mtx);
        while(!write_complete){
            // Release lock and wait until lambda condition has been met
            cv.wait(lock, [](){ return notified || write_complete; });

            if(write_complete) break;
            
            // Update position and time records
            this->pos_record.push_back(this->pos_state_rad);
            this->t_record.push_back(this->time_elapsed);
            
            fprintf(pipe, "plot '-' with points title 'Motor Position' pt 7 ps 1 lc rgb 'blue'\n");
            for (size_t j = 0; j < pos_record.size(); ++j) {
                fprintf(pipe, "%f %f\n", t_record[j], pos_record[j]);
            }
            fprintf(pipe, "%f %f\n", time_elapsed, pos_state_rad); // Example data point
            fprintf(pipe, "e\n"); // End of data for current plot
            fflush(pipe); // Flush to update the plot
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            // Reset notified variable
            notified = false;
            cv.notify_one();
        }
        pclose(pipe);
        return this->pos_state_rad;
    }else{
        return this->pos_state_rad;
    }

}

std::vector<double> ServoMotor :: get_pos_record(void) {
    return this->pos_record;
}

std::vector<double> ServoMotor :: get_t_record(void) {
    return this->t_record;
}

void ServoMotor :: set_position_state(double setpoint_rad, double sim_time, double dt){
    /* 
    Once verified, set up loop to stop once output position is stable 
        - only inputs to this function should then be setpoint_rad and dt
    */

    this->is_rotating = true;
    
    // simulated time and feedback loop
    for(double t = 0.0; t < sim_time; t += dt){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        {
            std::lock_guard<std::mutex> lock(mtx);
            double meas_pos = motor.get_position();
            double meas_vel = motor.get_velocity();
            double meas_curr = motor.get_current();
            double error = setpoint_rad - meas_pos;

            ctrlr.sample_error(error, dt);
            motor.set_input_volts(ctrlr.get_pid_output(), dt);

            this->pos_state_rad = meas_pos;
            this->time_elapsed = t;

            notified = true;
        }
        // Notify reader thread that record has been updated
        cv.notify_one();
    }

    // Now notify reader thread that writing is complete
    {
        std::lock_guard<std::mutex> lock(mtx);
        write_complete = true;
        notified = true;
    }

    this->is_rotating = false;
    cv.notify_one();

}
