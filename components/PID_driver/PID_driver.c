#include "PID_driver.h"

#include <stdio.h>
/*
esp_err_t PID_driver_init(float KP, float KI, float KD, float MAX, float MIN, PID_data_t PID){
    PID.kp = KP;
    PID.ki = KI;
    PID.kd = KD;

    PID.maxOutput = MAX;
    PID.minOutput = MIN;
    
    return ESP_OK;
}*/

float PID_driver_update(float value, float setpoint, float time_us, PID_data_t PID){

    //Calculate the error
    float dT = (float)(PID.previous_time_us - time_us)/1000000.0f;
    PID.previous_time_us = time_us;
    
    float error = setpoint - value;

    //Filter the error for derivative part
    float errorFiltered = PID.alpha * error + (1.0f - PID.alpha) * PID.previousErrorFiltered;
    float derivative = (errorFiltered - PID.previousErrorFiltered) / dT;
    PID.previousErrorFiltered = errorFiltered;
    
    float newIntegral = PID.integral + error * dT;

    float output = PID.kp * error + PID.ki * PID.integral + PID.kd * derivative;


    //Saturation filter
    if(output > PID.maxOutput){
        output = PID.maxOutput;
    }
    else if(output < PID.minOutput){
        output = PID.minOutput;
    }
    else{
        //Anti windup
        PID.integral = newIntegral;
    }
    
    return output; 
}