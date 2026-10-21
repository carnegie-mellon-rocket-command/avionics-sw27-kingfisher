#include <stdlib.h>

// State Estimation/Attitude function (take the sensors and convert into pitch, yaw, roll (i think thats what we need but not sure))


//PID Control (control algorithm to make the rocket go straight)


//Canard Allocation (take torques and make actual servo angles)

float max_angle = 10;//I think this should be max angle in one direction from neutral

float* torque_to_motor_position(float t_roll, float t_pitch, float t_yaw){
    float* motor_positions = (float*)malloc(4*sizeof(float));
    motor_positions[0] = (t_pitch + t_roll) * max_angle;
    motor_positions[1] = (t_yaw + t_roll) * max_angle;
    motor_positions[2] = (-t_pitch + t_roll) * max_angle;
    motor_positions[3] = (-t_yaw + t_roll) * max_angle;
    return motor_positions;
    //returns an array of motor positions from neutral, in order of top, right, bottom, left 
};

//Data Logging and Actuation (write the data we need and actually write the servo)