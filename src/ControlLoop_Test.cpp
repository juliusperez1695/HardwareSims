/*

ControlLoop_Test.cpp
Author: Julius Perez
Description: Tests the feedback position-control loop for the DC Motor

*/

#include <iostream>
#include <vector>
#include <sciplot/sciplot.hpp>
#include "PIDController.h"
#include "DCMotor.h"

using namespace std;
using namespace sciplot;

int main(void){

    double Kp = 15.0;
    double Ki = 0.0;
    double Kd = 7.0;
    PIDController pid_ctrlr(Kp, Ki, Kd);
    DCMotor motor1;
    vector<double> t_vals, curr_vals, vel_vals, pos_vals;

    double cmd_pos = 3.0;
    double sim_time = 5;
    double dt = 0.01;

    // simulated time and feedback loop
    for(double t = 0.0; t < sim_time; t += dt){
        double meas_pos = motor1.get_position();
        double meas_vel = motor1.get_velocity();
        double meas_curr = motor1.get_current();
        double error = cmd_pos - meas_pos;

        pid_ctrlr.sample_error(error, dt);
        motor1.set_input_volts(pid_ctrlr.get_pid_output(), dt);

        t_vals.push_back(t);
        pos_vals.push_back(meas_pos);
        vel_vals.push_back(meas_vel);
        curr_vals.push_back(meas_curr);
    }

    Plot2D plot4, plot5, plot6;

    plot4.xlabel("Time (s)");
    plot4.ylabel("Current (A)");
    plot4.drawCurve(t_vals, curr_vals).label("Motor Current");
    plot4.legend().atOutsideBottom().displayHorizontal();
    Figure fig1 = {{plot4}};
    Canvas canv1 = {{fig1}};
    canv1.show();

    plot5.xlabel("Time (s)");
    plot5.ylabel("Angular Velocity (rad/s)");
    plot5.drawCurve(t_vals, vel_vals).label("Motor Velocity");
    plot5.legend().atOutsideBottom().displayHorizontal();
    Figure fig2 = {{plot5}};
    Canvas canv2 = {{fig2}};
    canv2.show();

    plot6.xlabel("Time (s)");
    plot6.ylabel("Angular Position (rad)");
    plot6.drawCurve(t_vals, pos_vals).label("Motor Position");
    plot6.legend().atOutsideBottom().displayHorizontal();
    Figure fig3 = {{plot6}};
    Canvas canv3 = {{fig3}};
    vector<double>::iterator max_iter = max_element(pos_vals.begin(), pos_vals.end());
    double max_pos = *max_iter;
    cout << "Max. Position = " << max_pos << endl;
    canv3.show();

    return 0;
}