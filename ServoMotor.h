/*

ServoMotor.h
Author: Julius Perez
Description: Header file for declaring the ServoMotor class

*/

#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <vector>
#include "DCMotor.h"
#include "PIDController.h"

class ServoMotor {

    public:
        // Constructors
        ServoMotor();

        // Destructor
        ~ServoMotor();

        // Accessors
        double get_position_state(void);
        std::vector<double> get_pos_record(void);
        std::vector<double> get_t_record(void);

        // Mutators
        void set_position_state(double setpoint_rad, double sim_time, double dt);

    private:
        double pos_state_rad = 0.0;
        double time_elapsed = 0.0;
        bool is_rotating = false;
        DCMotor motor;
        PIDController ctrlr;
        std::vector<double> pos_record, t_record;

};

#endif