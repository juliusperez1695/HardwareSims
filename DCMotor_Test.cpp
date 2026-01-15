/*

DCMotor_Test.cpp
Author: Julius Perez
Description: For testing DC motor open-loop operation

*/

#include <iostream>
#include <vector>
#include <sciplot/sciplot.hpp>
#include "DCMotor.h"

using namespace std;
using namespace sciplot;

int main(void){

    DCMotor motor1;
    double sim_time = 5.0;
    double V_setpoint = 5;
    double dt = 0.01;
    vector<double> t_vals, curr_vals, vel_vals, pos_vals;

    cout << "Motor Voltage Setpoint = " << V_setpoint << endl;
    //cout << "Current (A),   Ang. Velocity (rad/s),  Ang. Position (rad)" << endl;

    for(double t = 0; t < sim_time; t+=dt){
        motor1.set_input_volts(V_setpoint, dt);
        double curr = motor1.get_current();
        double vel = motor1.get_velocity();
        double pos = motor1.get_position();

        //cout << curr << ", \t" << vel << ", \t\t" << pos << endl;

        t_vals.push_back(t);
        curr_vals.push_back(curr);
        vel_vals.push_back(vel);
        pos_vals.push_back(pos);
    }

    Plot2D plot1, plot2, plot3;

    plot1.xlabel("Time (s)");
    plot1.ylabel("Current (A)");
    plot1.drawCurve(t_vals, curr_vals).labelNone();
    Figure fig1 = {{plot1}};
    Canvas canv1 = {{fig1}};
    canv1.show();

    plot2.xlabel("Time (s)");
    plot2.ylabel("Angular Velocity (rad/s)");
    plot2.drawCurve(t_vals, vel_vals).labelNone();
    Figure fig2 = {{plot2}};
    Canvas canv2 = {{fig2}};
    canv2.show();

    plot3.xlabel("Time (s)");
    plot3.ylabel("Angular Position (rad)");
    plot3.drawCurve(t_vals, pos_vals).labelNone();
    Figure fig3 = {{plot3}};
    Canvas canv3 = {{fig3}};
    canv3.show();

    return 0;
}