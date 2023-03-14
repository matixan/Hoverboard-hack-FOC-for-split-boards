
#include "gd32f1x0.h"
#include <math.h>     
#define ARM_MATH_CM3
#include "arm_math.h" 

#include "setup.h"
#include "defines.h"
#include "config.h"
#include "it.h"
#include "bldc.h"
#include "pid.h"
#include "debug.h"


ControlType control_type = CT_PWM;
float rpm_filtered = 0.0;

// PWM control
float target_pwm = 0.0;
float motor_pwm = 0.0;
float max_pwm_change_rate = 10.0;
uint16_t max_pwm = 500;

// Angle control
unsigned long target_angle = 360*5;
int anlge_max_pwm = 500;
float angle_Kp = ANGLE_PID_KP; 
float angle_Kd = ANGLE_PID_KD;
float angle_PID_error_old = 0.0;
float angle_set_point_old = 0.0;
unsigned long angle_old_ts = 0;


PidData speedPid = {
    0.0,           // set_point
    -1000.0,       // min_out
     1000.0,       // max_out
    1.0,           // dead_band
    SPEED_PID_KP,  // p_gain
    SPEED_PID_KI,  // i_gain
    SPEED_PID_KD,  //  d_gain
    0.0,           // integral
    0.0,           // last_error
    0              // time_stamp
};

PidData anglePid = {
    0.0,           // set_point
    -1000.0,       // min_out
     1000.0,       // max_out
    1.0,           // dead_band
    ANGLE_PID_KP,  // p_gain
    ANGLE_PID_KI,  // i_gain
    ANGLE_PID_KD,  //  d_gain
    0.0,           // integral
    0.0,           // last_error
    0              // time_stamp
};


// PID loop
uint32_t next_pid_ts = 0;

// Increase / decrease motor pwm limiting rate (acceleration / deceleration)
float adjustBldcPWM(float newPWM) {
    float delta = newPWM - motor_pwm;
    delta = delta < 0.0 ? MIN(delta, -max_pwm_change_rate) : MAX(delta, max_pwm_change_rate);
    motor_pwm += delta;
    setBldcPWM(motor_pwm);
    return motor_pwm;
}

float anglePDControl( float input, float setPoint,  float Kp, float Kd, unsigned long now) {
  float DT = (now - angle_old_ts) * 0.000001;
  angle_old_ts = now;

  float error = setPoint - input;

  // Kd is implemented in two parts
  //    The biggest one using only the input part not the SetPoint input-input(t-1).
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  float Kd_setPoint = CLAMP((setPoint - angle_set_point_old), -8, 8); // We limit the input part...
  float output = Kp * error + (Kd * Kd_setPoint - Kd * (input - angle_PID_error_old)) / DT;
  //DEBUG_print(Kd*(error-PID_errorOld); DEBUG_print(FST("\t"));
  //PID_errorOld2 = PID_errorOld;
  angle_PID_error_old = input;  // error for Kd is only the input component
  angle_set_point_old = setPoint;
  return (output);
}

float pidExecute( float input, PidData* pidData, uint32_t now) {
    float result;
    float dt = (now - pidData->time_stamp) * 0.000001;
    pidData->time_stamp = now;
    float error = pidData->set_point - input;
    if (ABS(error) > pidData->dead_band) {
        float p_term = pidData->p_gain * error;
        if (p_term > pidData->max_out || p_term < pidData->min_out) { pidData->integral = 0.0; }
        else if (pidData->i_gain) {
            // Tustin transform of the integral part
            // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
            // method uses the antiwindup Foxboro method : https://core.ac.uk/download/pdf/289952713.pdf
            pidData->integral += pidData->i_gain * dt * 0.5 * (error + pidData->last_error);
            pidData->integral = CLAMP(pidData->integral, pidData->min_out, pidData->max_out);
        }
        float d_term = (pidData->d_gain != 0.0) ? (error - pidData->last_error) * pidData->d_gain / dt : 0.0; 
        result = p_term + pidData->integral + d_term;
    }
    else { result = pidData->integral; }
    pidData->last_error = error;
    return CLAMP(result, pidData->min_out, pidData->max_out);
}


void pidControllerRun() {
    // Execute at 1 kHz
    unsigned long now = micros();
    if (now < next_pid_ts) { return; }
    next_pid_ts = now + 1000;
    rpm_filtered = wheel_speed_rpm_filtered * (1.0/256.0);

    switch (control_type) {
    case CT_PWM:
        adjustBldcPWM(target_pwm);
        break;
    
    case CT_SPEED:
        adjustBldcPWM(pidExecute(wheel_speed_rpm * (1.0/256.0), &speedPid, now));
        break;

    case CT_ANGLE: {
        #if 0
        float newPWM = anglePDControl(wheel_angle, target_angle, angle_Kp, angle_Kd, now);
        adjustBldcPWM(CLAMP(newPWM, -anlge_max_pwm, anlge_max_pwm));
        #else
        float pwm_angle = pidExecute(wheel_angle, &anglePid, now);
        if (speedPid.set_point == 0.0) {
            adjustBldcPWM(pwm_angle);
        }
        else if (pwm_angle < 0) {
            if (speedPid.set_point > 0.0) {
                speedPid.set_point = -speedPid.set_point;
                speedPid.integral = 0.0;
                speedPid.last_error = 0.0;
            }
            float pwm_speed = pidExecute(wheel_speed_rpm * (1.0/256.0), &speedPid, now);
            adjustBldcPWM((pwm_angle > pwm_speed) ? pwm_angle : pwm_speed);
        } else {
            if (speedPid.set_point < 0.0) {
                speedPid.set_point = -speedPid.set_point;
                speedPid.integral = 0.0;
                speedPid.last_error = 0.0;
            }
            float pwm_speed = pidExecute(wheel_speed_rpm * (1.0/256.0), &speedPid, now);
            adjustBldcPWM((pwm_angle < pwm_speed) ? pwm_angle : pwm_speed);
        }
        #endif
        break; }
    }
}
