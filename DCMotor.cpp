/*

DCMotor.cpp
Author: Julius Perez
Description: Defines the DCMotor class's constructors, destructors, and methods

*/

#include "DCMotor.h"

DCMotor :: DCMotor(){
    this->R = 1;          // ohms
    this->J = 0.01;       // kg * m^2
    this->Kt = 0.01;      // N * m / A
    this->Ke = 0.01;      // V / rad / sec
    this->b = 0.1;        // N * m * sec
    this->L = 0.5;        // H
}

DCMotor :: DCMotor(
    double R,
    double J,
    double Kt,
    double Ke,
    double b,
    double L)
{
    this->R = R;
    this->J = J;
    this->Kt = Kt;
    this->Ke = Ke;
    this->b = b;
    this->L = L;
}

DCMotor :: ~DCMotor(){
    // Deallocate memory as required
}

// Mutators
void DCMotor :: set_input_volts(double setpoint, double sample_time){
    this->Vin = setpoint;
    this->dt = sample_time;
    this->update_state();
}

void DCMotor :: update_state(void){
    // Solve ODEs when Vin changes and for each increment of simulated time
    double dwdt = (Kt*current - b*ang_velocity) / J;
    double didt = (Vin - R*current - Ke*ang_velocity) / L;

    // Use Forward Euler method to determine state
    ang_velocity += dwdt * dt;
    ang_position += ang_velocity * dt;
    current += didt * dt;
}

// Accessors
double DCMotor :: get_current(void) const {
    return current;
}

double DCMotor :: get_velocity(void) const {
    return ang_velocity;
}

double DCMotor :: get_position(void) const {
    return ang_position;
}

