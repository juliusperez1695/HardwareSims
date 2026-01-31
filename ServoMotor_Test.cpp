#include <vector>
#include <thread>
#include <sciplot/sciplot.hpp>
#include "ServoMotor.h"

using namespace sciplot;

int main(void){
    ServoMotor motor;
    std::vector<double> t_vals, pos_vals;

    std::thread write_thread(
        &ServoMotor::set_position_state, 
        &motor, 
        3.0, 5.0, 0.01
    );
    std::thread read_thread(
        &ServoMotor::get_position_state,
        &motor
    );
    
    write_thread.join();
    read_thread.join();

    t_vals = motor.get_t_record();
    pos_vals = motor.get_pos_record();

    // Plot2D plot;
    
    // plot.xlabel("Time (s)");
    // plot.ylabel("Angular Position (rad)");
    // plot.drawCurve(t_vals, pos_vals).label("Motor Position");
    // plot.legend().atOutsideBottom().displayHorizontal();
    // Figure fig = {{plot}};
    // Canvas canv = {{fig}};
    // canv.show();

    return 0;
}