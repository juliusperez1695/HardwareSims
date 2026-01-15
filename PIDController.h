/*

PIDController.h
Author: Julius Perez
Description: Declaration of the PIDController class

*/

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController{

    public:
        // PID coefficients
        double Kp, Ki, Kd;

        // Constructor
        PIDController(double Kp, double Ki, double Kd);

        // Destructor
        ~PIDController(void);

        // Mutators
        void sample_error(double error, double sample_time);

        // Accessors
        double get_pid_output(void) const;
    private:
        double output = 0.0;
        double previous_error = 0.0;
        double sampled_error = 0.0;
        double integrated_error = 0.0;
        double dt;

        // Called by sample_error function to update 'output' variable
        void generate_output(void);
};

#endif