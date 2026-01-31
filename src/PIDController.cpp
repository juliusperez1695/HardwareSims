/*

PIDController.cpp
Author: Julius Perez
Description: Definition of the PIDController class

*/

#include "PIDController.h"

PIDController :: PIDController(){
    this->Kp = 15.0;
    this->Ki = 0.0;
    this->Kd = 7.0;
}

PIDController :: PIDController(double Kp, double Ki, double Kd){
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

PIDController :: ~PIDController(){
    // Deallocate memory as necessary
}

void PIDController :: sample_error(double error, double sample_time){
    this->sampled_error = error;
    this->dt = sample_time;
    this->generate_output();
}

void PIDController :: generate_output(void){
    // Calculates the control output for each time step 'dt' using the sampled position error

    // Integral Term
    integrated_error += sampled_error;

    // Derivative Term
    double d_error = sampled_error - previous_error;
    double derivative_error = d_error / dt;

    // Control signal
    output = Kp * sampled_error + Ki * integrated_error + Kd * derivative_error;

    previous_error = sampled_error;

}

double PIDController :: get_pid_output(void) const {
    return output;
}