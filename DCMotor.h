/*

DCMotor.h
Author: Julius Perez
Description: Header file for declaring the DCMotor class

*/

#ifndef DCMOTOR_H
#define DCMOTOR_H

class DCMotor {

    public:
        // Parameters from motor differential equations
        double R, J, Kt, Ke, b, L;

        // Constructors
        DCMotor();
        DCMotor(
            double R,
            double J,
            double Kt,
            double Ke,
            double b,
            double L
        );

        // Destructor
        ~DCMotor();

        // Mutators
        void set_input_volts(double setpoint, double sample_time);

        // Accessors
        double get_current(void) const;
        double get_velocity(void) const;
        double get_position(void) const;

    private:
        // Input voltage for powering the motor
        double Vin;
        double dt; // sample time

        // State variables
        double current = 0.0;
        double ang_velocity = 0.0;
        double ang_position = 0.0;

        // Update motor state vector by numerically solving ODEs
        void update_state();
};

#endif