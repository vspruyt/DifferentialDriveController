/**
 * @author Vincent Spruyt
 * @brief PID controller for differential steering
 *
 * Steering controller for balancing robot.
 * Used for closed loop PID control.
 * Implemented as an integrating process, where the PID controller
 * outputs the power-delta that needs to be added to the current
 * motor power in order to correct for any steering error.
 * 
 * The setpoint unit is RPM.
 */

#ifndef STEERINGCONTROLLER_H
#define STEERINGCONTROLLER_H

//------------------------------------------------------------------------------
// Includes
#include "Arduino.h"
#include <PID_v1.h>
#include <USBSabertooth.h>
#include "limits.h"
#include "float.h"

// Assumes that FILTER_UPDATE_RATE_HZ has been defined before including this header file
#define ANOMALY_BUFFER_SIZE (FILTER_UPDATE_RATE_HZ*ANOMALY_REJECTION_WINDOW_SIZE_S)

struct PidParams{
    double Kp;
    double Ki;
    double Kd;
};

class SteeringController{
public:    
    SteeringController(int countable_events_per_revolution, float update_rate_hz, PidParams params_normal_mode, PidParams params_deadzone_mode);        
    double encoderticks_to_rpm(float tick_count, double samplePeriod);
    double rpm_to_encoderticks(double speed_rpm, double sampleperiod);
    int update_steering(double target_rpm_wheel_difference, float speed_voltage, float encoder1_val, float encoder2_val, float sample_period);    
    void update_pid_params(const PidParams& pid_params);
    void update_pid_params(double Kp, double Ki, double Kd);
    void setDeadzone(int physical_deadzone, int virtual_deadzone);
    void set_pid_params_depending_on_deadzone(float speed_voltage, int steering_offset);
    void calc_motor_commands(float speed_voltage, int steering_offset, int* motor_command1, int* motor_command2);
    void update_last_good_speed_measurement(float encoder1_val, float encoder2_val, int motor_command1, int motor_command2);

    double pid_setpoint=0.0;
    double pid_input=0.0;
    double pid_output=0.0;    

    PID pid;        

private:        
    int countable_events_per_revolution = 0;
    float update_rate_hz;    
    PidParams params_normal_mode;
    PidParams params_deadzone_mode;

    bool motor1_in_deadzone = true;
    bool motor2_in_deadzone = true;

    int virtual_deadzone = 10;
    int physical_deadzone = 80;

    int motor_1_last_good_command = -115;
    float motor_1_last_good_speed = 8.0;
    int motor_2_last_good_command = -115;
    float motor_2_last_good_speed = 8.0;
    
    int map_deadzone(int motor_command, int margin, int deadzone);
    bool check_deadzone(int motor_nr, float speed, float steering_offset);    
    void interpolate_pid_in_deadzone(int motor_nr, float x0, float y0, float x1, float y1, float* encoder_val, float speed);
};

void SteeringController::setDeadzone(int physical_deadzone, int virtual_deadzone){
    physical_deadzone = physical_deadzone;
    virtual_deadzone = virtual_deadzone;
}

SteeringController::SteeringController(int countable_events_per_revolution, float update_rate_hz, PidParams params_normal_mode, PidParams params_deadzone_mode): pid(PID(&pid_input, &pid_output, &pid_setpoint, params_deadzone_mode.Kp, params_deadzone_mode.Ki, params_deadzone_mode.Kd, P_ON_M, DIRECT)), countable_events_per_revolution(countable_events_per_revolution), update_rate_hz(update_rate_hz), params_normal_mode(params_normal_mode), params_deadzone_mode(params_deadzone_mode){
    pid.SetMode(AUTOMATIC); 
    pid.SetOutputLimits(-2047.0, 2047.0);
    pid.SetSampleTime(1000.0/update_rate_hz);
}

double SteeringController::encoderticks_to_rpm(float tick_count, double samplePeriod){    
    return (60.0*tick_count/float(countable_events_per_revolution))/samplePeriod;
};

double SteeringController::rpm_to_encoderticks(double speed_rpm, double sampleperiod){
  return countable_events_per_revolution/((60.0/speed_rpm)/sampleperiod);
};

bool SteeringController::check_deadzone(int motor_nr, float speed, float steering_offset){
    int totspeed = 0;
    if(motor_nr == 1)
        totspeed = speed-steering_offset;
    else
        totspeed = speed+steering_offset;
    
    if(abs(totspeed)<=virtual_deadzone)
        return true;
    else
        return false;
}

void SteeringController::set_pid_params_depending_on_deadzone(float speed_voltage, int steering_offset){
    bool motor1_in_deadzone_before = motor1_in_deadzone;
    motor1_in_deadzone = check_deadzone(1, speed_voltage, steering_offset);
    bool motor2_in_deadzone_before = motor2_in_deadzone;
    motor2_in_deadzone = check_deadzone(2, speed_voltage, steering_offset);

    if(motor1_in_deadzone && !motor1_in_deadzone_before)
        update_pid_params(params_deadzone_mode);   
    else if(motor2_in_deadzone && !motor2_in_deadzone_before)
        update_pid_params(params_deadzone_mode);   
    else if(!motor1_in_deadzone && motor1_in_deadzone_before)
        update_pid_params(params_normal_mode);   
    else if(!motor2_in_deadzone && motor2_in_deadzone_before)
        update_pid_params(params_normal_mode);   
}

void SteeringController::update_last_good_speed_measurement(float encoder1_val, float encoder2_val, int motor_command1, int motor_command2){
    
    if(!motor1_in_deadzone){
        if(abs(motor_1_last_good_command) <= abs(motor_command1) && abs(motor_1_last_good_speed) <= abs(encoder1_val)){
            motor_1_last_good_command = motor_command1;
            motor_1_last_good_speed = encoder1_val;
        }
    }

    if(!motor2_in_deadzone){
        if(abs(motor_2_last_good_command) <= abs(motor_command2) && abs(motor_2_last_good_speed) <= abs(encoder2_val)){
            motor_2_last_good_command = motor_command2;
            motor_2_last_good_speed = encoder2_val;
        }
    }

}

void SteeringController::interpolate_pid_in_deadzone(int motor_nr, float x0, float y0, float x1, float y1, float* encoder_val, float speed){
    float tot_speed = motor_nr==1?speed-pid_output:speed+pid_output;
    float prev_motor_command = constrain(tot_speed, -2047.0, 2047.0);        
    float abs_motor_command = abs(prev_motor_command);
    if(abs_motor_command>=virtual_deadzone)
        abs_motor_command = map(abs_motor_command, virtual_deadzone, 2047.0, physical_deadzone, 2047.0);        
    else
        abs_motor_command = map(abs_motor_command, 0.0, virtual_deadzone, 0.0, physical_deadzone);
        
    prev_motor_command = (prev_motor_command >= 0) ? abs_motor_command :  -abs_motor_command;        

    if(x1 != x0)        
        *encoder_val = (y0*(x1 - prev_motor_command) + y1*(prev_motor_command-x0))/(x1-x0);  
}

void SteeringController::calc_motor_commands(float speed_voltage, int steering_offset, int* motor_command1, int* motor_command2){
    *motor_command1 = constrain(speed_voltage-steering_offset, -2047, 2047);
    *motor_command2 = constrain(speed_voltage+steering_offset, -2047, 2047);

    if(motor1_in_deadzone)
      *motor_command1 = 0;
    if(motor2_in_deadzone)
      *motor_command2 = 0;

    *motor_command1 = map_deadzone(*motor_command1, virtual_deadzone, physical_deadzone);
    *motor_command2 = map_deadzone(*motor_command2, virtual_deadzone, physical_deadzone);
}

int SteeringController::update_steering(double target_rpm_wheel_difference, float speed_voltage, float encoder1_val, float encoder2_val, float sample_period){    
        
    if(motor1_in_deadzone)
        interpolate_pid_in_deadzone(1, motor_1_last_good_command, motor_1_last_good_speed, 0.0, 0.0, &encoder1_val, speed_voltage);
    if(motor2_in_deadzone)
        interpolate_pid_in_deadzone(2, motor_2_last_good_command, motor_2_last_good_speed, 0.0, 0.0, &encoder2_val, speed_voltage);

    double speed1 = encoderticks_to_rpm(encoder1_val, sample_period);
    double speed2 = encoderticks_to_rpm(encoder2_val, sample_period);     

    pid_setpoint = target_rpm_wheel_difference;
    pid_input = speed1 - speed2;
    pid.Compute();      
    int steering_offset = round(pid_output);    
    
    return steering_offset;
}

void SteeringController::update_pid_params(const PidParams& pid_params){
    pid.SetTunings(pid_params.Kp, pid_params.Ki, pid_params.Kd, P_ON_M);
}

void SteeringController::update_pid_params(double Kp, double Ki, double Kd){
    pid.SetTunings(Kp, Ki, Kd, P_ON_M);
}

int SteeringController::map_deadzone(int motor_command, int margin, int deadzone){
  int abs_motor_command = abs(motor_command);
  abs_motor_command = (abs_motor_command < margin) ? 0 : map(abs_motor_command, margin, 2047.0, deadzone, 2047.0) ; // Inserts a Deadzone
  motor_command = (motor_command >= 0) ? abs_motor_command :  -abs_motor_command; // Restores a FullRange Value +-
  return(motor_command);
}

#endif
//------------------------------------------------------------------------------
// End of file
