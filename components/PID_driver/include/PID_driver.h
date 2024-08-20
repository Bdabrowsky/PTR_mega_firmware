#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"

typedef struct{
    float kp;
    float ki;
    float kd;

    float previousErrorFiltered;
    float integral;
    float alpha;

    float maxOutput;
    float minOutput;

    int64_t previous_time_us;

}PID_data_t;


esp_err_t PID_driver_init(float KP, float KI, float KD, float MAX, float MIN, PID_data_t PID);
float PID_driver_update(float value, float setpoint, float time_us, PID_data_t PID);

