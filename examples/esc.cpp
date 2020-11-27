#include <Arduino.h>
#include <Encoder.h>
#include <limits.h>
#include <PID_v1.h>
#include <USBSabertooth.h>

#define UPDATE_RATE_HZ (100) // Needs to be low enough so we have enough encoder measurements at low speed
#define PRINT_EVERY_N_UPDATES (-1)
#define COUNTABLE_EVENTS_PER_REVOLUTION (1296) // At the output shaft (wheel)
#define MAX_SPEED_RPM (313)

USBSabertoothSerial C(Serial1);
USBSabertooth ST(C, 128);

uint32_t timestamp;
Encoder myEnc1((uint8_t)11, (uint8_t)10);
Encoder myEnc2((uint8_t)15, (uint8_t)14);

double pid_setpoint_m1=0.0;
double prev_motor_command_m1 = 0.0;
double pid_input_m1=0.0;
double pid_output_m1=0.0;

double Kp_m1=10.0;
double Ki_m1=0.01;
double Kd_m1=0.02;
PID pid_m1(&pid_input_m1, &pid_output_m1, &pid_setpoint_m1, Kp_m1, Ki_m1, Kd_m1, P_ON_E, DIRECT);

double pid_setpoint_m2=0.0;
double prev_motor_command_m2 = 0.0;
double pid_input_m2=0.0;
double pid_output_m2=0.0;

double Kp_m2=10.30;
double Ki_m2=0.01;
double Kd_m2=0.02;
PID pid_m2(&pid_input_m2, &pid_output_m2, &pid_setpoint_m2, Kp_m2, Ki_m2, Kd_m2, P_ON_E, DIRECT);

int getExpectedEventCount(double speed_rpm, double sampleperiod){
  return COUNTABLE_EVENTS_PER_REVOLUTION/((60.0/speed_rpm)/sampleperiod);
}

const int max_expected_event_count = getExpectedEventCount(MAX_SPEED_RPM, 1.0/UPDATE_RATE_HZ);

void setup()
{  
  timestamp = micros();  
  Serial.begin(115200);  
  Serial1.begin(115200);
  // ST.setRamping(1980); // (approximately 2 seconds)
  ST.setTimeout(500); // Stop motor 0.5 seconds after signal was lost  
  
  pid_input_m1 = 0.0;
  pid_setpoint_m1 = 0.0;
    
  pid_m1.SetMode(AUTOMATIC); 
  pid_m1.SetOutputLimits(-2047.0, 2047.0);
  pid_m1.SetSampleTime(1000.0/UPDATE_RATE_HZ);  

  pid_input_m2 = 0.0;
  pid_setpoint_m2 = 0.0;
    
  pid_m2.SetMode(AUTOMATIC); 
  pid_m2.SetOutputLimits(-2047.0, 2047.0);
  pid_m2.SetSampleTime(1000.0/UPDATE_RATE_HZ);  
}

bool do_PID_update(float rpm_target, int raw_input, float sample_period, double* setpoint_var, double* input_var, PID* pid){  
  *setpoint_var = getExpectedEventCount(rpm_target, sample_period);  
  *setpoint_var = constrain(pid_setpoint_m1, -max_expected_event_count, max_expected_event_count);
  *input_var = raw_input;
  return pid->Compute();
}

int drive_motor(USBSabertooth* driver, int motor_nr, double pid_output, int prev_motor_command){
  int motor_command = prev_motor_command+round(pid_output);  
  motor_command = constrain(motor_command, -2047, 2047);
  driver->motor(motor_nr, -motor_command);
  return motor_command;
}

void loop()
{
  
  // Timing stuff to keep a constant sample rate
  static uint8_t counter = 0;  
  uint32_t curr_time = micros();
  uint32_t time_diff = curr_time - timestamp;
  if (time_diff < (1000000 / UPDATE_RATE_HZ)) return;
  float samplePeriod = time_diff/1000000.0;
  timestamp = curr_time;


  // Read the new motor encoder positions
  long newPosition1 = -myEnc1.readAndReset();
  long newPosition2 = myEnc2.readAndReset();

  // Our raw 'setpoint input'. For now, this is just a potentiometer that
  // is attached to analog pin 8.  
  float rpm_target = analogRead(8)*(MAX_SPEED_RPM/1023.0);  
  
  // Let our PIDs calculate the desired motor power
  do_PID_update(rpm_target, newPosition1, samplePeriod, &pid_setpoint_m1, &pid_input_m1, &pid_m1);
  do_PID_update(rpm_target, newPosition2, samplePeriod, &pid_setpoint_m2, &pid_input_m2, &pid_m2);

  // Actually drive the motors based on the PID output
  prev_motor_command_m1 = drive_motor(&ST, 1, pid_output_m1, prev_motor_command_m1);
  prev_motor_command_m2 = drive_motor(&ST, 2, pid_output_m2, prev_motor_command_m2);  

  Serial.print(prev_motor_command_m1);
  Serial.print("\t");
  Serial.println(prev_motor_command_m2);

  // only print the calculated output once in a while
  if (counter++ <= PRINT_EVERY_N_UPDATES){
    return;
  }
  
  // reset the counter
  counter = 0;

  // Print out some debugging info  
  // Serial.print("Sample period (ms): ");
  // Serial.println(samplePeriod*1000);  

  // Serial.print("Speed (m/s): ");
  // Serial.print(speed1, 4);
  
  // Serial.println(UPDATE_RATE_HZ);
  // Serial.println(samplePeriod, 4);

  // Serial.print("Speed (RPM): ");
  // Serial.print((60.0*newPosition1/float(COUNTABLE_EVENTS_PER_REVOLUTION))/samplePeriod);
  // Serial.print(" (");
  // Serial.print(newPosition1);
  // Serial.print(")");
  // Serial.print(" / ");
  // // Serial.print(speed2, 4);  
  // Serial.print((60.0*newPosition2/float(COUNTABLE_EVENTS_PER_REVOLUTION))/samplePeriod);
  // Serial.print(" (");
  // Serial.print(newPosition2);
  // Serial.println(")");

  // Serial.print(pid_setpoint_m1);
  // Serial.print("\t");
  // Serial.println(pid_input_m1);

  

}