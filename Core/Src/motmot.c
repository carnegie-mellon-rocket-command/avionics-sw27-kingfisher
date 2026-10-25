#include <stdlib.h>

// State Estimation/Attitude function (take the sensors and convert into pitch, yaw, roll (i think thats what we need but not sure))


//PID Control (control algorithm to make the rocket go straight)


//Canard Allocation (take torques and make actual servo angles)

float max_angle = 8; //This we will get from mechanical

void torque_to_motor_position(float t_roll, float t_pitch, float t_yaw, float motor_positions[4]){
    motor_positions[0] = (t_pitch + t_roll) * max_angle; //Top Fin 1
    motor_positions[1] = (t_yaw + t_roll) * max_angle; //Right Fin 2
    motor_positions[2] = (-t_pitch + t_roll) * max_angle; //Bottom Fin 3
    motor_positions[3] = (-t_yaw + t_roll) * max_angle; //Left Fin 4
    //now motor_positions contains angles for each of the motors

    //Clamps the angles within the max angle   
    for (int i = 0; i < 4; i++) {
        float mangle = motor_positions[i];
        motor_positions[i] = (mangle < -max_angle) ? -max_angle : (mangle > max_angle) ? max_angle: mangle; 
};

//Data Logging and Actuation (write the data we need and actually write the servo)