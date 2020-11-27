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

// Assumes that FILTER_UPDATE_RATE_HZ has been defined before including this header file
#define ANOMALY_BUFFER_SIZE (FILTER_UPDATE_RATE_HZ*ANOMALY_REJECTION_WINDOW_SIZE_S)

class SteeringController{
public:    
    SteeringController(int countable_events_per_revolution, float update_rate_hz);
    double encoderticks_to_rpm(int tick_count, double samplePeriod);
    double rpm_to_encoderticks(double speed_rpm, double sampleperiod);
    double update_steering(double target_rpm_wheel_difference, double measured_rpm_wheel_difference, float sample_period);

    double pid_setpoint=0.0;
    double pid_input=0.0;
    double pid_output=0.0;

private:        
    int countable_events_per_revolution;
    float update_rate_hz;
    double Kp=2.3;
    double Ki=50.0;
    double Kd=0.0;    
    PID pid = PID(&pid_input, &pid_output, &pid_setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);
    int steering_offset = 0;
};

SteeringController::SteeringController(int countable_events_per_revolution, float update_rate_hz): countable_events_per_revolution(countable_events_per_revolution), update_rate_hz(update_rate_hz){
    pid.SetMode(AUTOMATIC); 
    pid.SetOutputLimits(-2047.0, 2047.0);
    pid.SetSampleTime(1000.0/update_rate_hz);
}

double SteeringController::encoderticks_to_rpm(int tick_count, double samplePeriod){    
    return (60.0*tick_count/float(countable_events_per_revolution))/samplePeriod;
};

double SteeringController::rpm_to_encoderticks(double speed_rpm, double sampleperiod){
  return countable_events_per_revolution/((60.0/speed_rpm)/sampleperiod);
};

double SteeringController::update_steering(double target_rpm_wheel_difference, double measured_rpm_wheel_difference, float sample_period){
    pid_setpoint = target_rpm_wheel_difference;
    pid_input = encoderticks_to_rpm(measured_rpm_wheel_difference, sample_period);
    pid.Compute();
    return pid_output;
    steering_offset += round(pid_output);
    steering_offset = constrain(steering_offset, -2048, +2048);
    return steering_offset;
}

#endif
//------------------------------------------------------------------------------
// End of file
