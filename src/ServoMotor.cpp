/*

ServoMotor.cpp
Author: Julius Perez
Description: Defines the ServoMotor class's constructors, destructors, and methods

*/

#include <iostream>
#include <cstdio>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <sciplot/sciplot.hpp>
#include "DCMotor.h"
#include "PIDController.h"
#include "ServoMotor.h"

#ifdef _WIN32
#define popen _popen
#define pclose _pclose
#endif

using namespace sciplot;

std::mutex mtx;
std::condition_variable cv;
bool notified = false;
bool write_complete = false;

ServoMotor :: ServoMotor() {
    // Definition of constructor
    this->motor = DCMotor();
    this->ctrlr = PIDController(15.0, 0.0, 7.0);
    //this->ctrlr = PIDController(6.0, 0.0005, 0.0);
}

ServoMotor :: ~ServoMotor() {
    // Definition of destructor - deallocate memory as necessary
}

double ServoMotor :: get_position_state(void) {

    if(this->is_rotating){
        
        // Run a separate thread for reading during position change
#ifdef _WIN32
        FILE* pipe = popen("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\" -persist", "w");
#else
        FILE* pipe = popen("gnuplot -persist", "w");
#endif
        if (pipe) {
#ifdef _WIN32
            fprintf(pipe, "set terminal windows size 400,300\n");
#elif __APPLE__
            fprintf(pipe, "set terminal qt size 400,300\n");
#else
            fprintf(pipe, "set terminal wxt size 400,300\n");
#endif
            fprintf(pipe, "set title 'Initializing...'\n");
            fprintf(pipe, "plot NaN notitle\n"); // Plots "Nothing" to force the window open
            fflush(pipe);
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Give it a head start
        }else return -1;

        //fprintf(pipe, "set terminal wxt size 400,300 noraise\n");
        fprintf(pipe, "set terminal windows size 400,300\n");
        fflush(pipe);
        std::string title_str = "set title '"+this->label+" Live Data Plot'\n";
        fprintf(pipe, title_str.c_str());
        fprintf(pipe, "set xlabel 'Time (s)'\n");
        fprintf(pipe, "set ylabel 'Angular Position (rad)'\n");
        fprintf(pipe, "set xrange [0:6]\n");
        fprintf(pipe, "set yrange [0:6]\n");

        std::vector<double> t_history, pos_history;
        
        while(!write_complete){

            // Thread: Vector-copy of t_record and pos_record for plotting
            std::vector<double> t_batch, pos_batch;
            {
                // Release lock and wait until lambda condition has been met
                std::unique_lock<std::mutex> lock(mtx);
                cv.wait(lock, []() { return notified || write_complete; });

                // Simulation fills the record which acts as a data buffer, then the plotter clears the buffer via "move"
                t_batch = std::move(t_record);
                pos_batch = std::move(pos_record);
                notified = false;

                if (write_complete) break;
            }
            
            // Append batches to master history
            t_history.insert(t_history.end(), t_batch.begin(), t_batch.end());
            pos_history.insert(pos_history.end(), pos_batch.begin(), pos_batch.end());

            // Plot position record
            if (!t_history.empty()) {
                std::string plot_str = "plot '-' with points title '" + this->label + " Position' pt 7 ps 1 lc rgb 'blue'\n";

                fprintf(pipe, plot_str.c_str());
                for (size_t j = 0; j < t_history.size(); ++j) {
                    fprintf(pipe, "%f %f\n", t_history[j], pos_history[j]);
                }
                fprintf(pipe, "%f %f\n", time_elapsed, pos_state_rad); // Example data point
                fprintf(pipe, "e\n"); // End of data for current plot
                fflush(pipe); // Flush to update the plot
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(30));

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

            // Update position and time records
            this->pos_record.push_back(this->pos_state_rad);
            this->t_record.push_back(this->time_elapsed);

            notified = true;
        }
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

void ServoMotor :: set_label(std::string in_label){
    this->label = in_label;
}
