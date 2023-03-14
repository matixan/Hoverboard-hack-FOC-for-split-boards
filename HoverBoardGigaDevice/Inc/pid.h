#ifndef _PID_H_
#define _PID_H_

#include "gd32f1x0.h"


typedef enum ControlType { CT_PWM, CT_SPEED, CT_ANGLE } ControlType;

typedef struct PidData {
    float set_point;
    float min_out;
    float max_out;
    float dead_band;
    float p_gain;
    float i_gain;
    float d_gain;
    float integral;
    float last_error;
    uint32_t time_stamp;
} PidData;

extern ControlType control_type;
extern float rpm_filtered;


// PWM control
extern float target_pwm;
extern float motor_pwm;
extern float max_pwm_change_rate;
extern uint16_t max_pwm;

// Speed control
extern PidData speedPid;

// Angle control
extern PidData anglePid;
extern unsigned long target_angle;
extern int anlge_max_pwm;
extern float angle_Kp; 
extern float angle_Kd;
extern float angle_PID_error_old;
extern float angle_set_point_old;
extern PidData anglePid;


void pidControllerRun();

#endif  // _PID_H_