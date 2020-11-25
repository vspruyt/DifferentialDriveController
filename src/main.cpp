#include <Arduino.h>
#include <Encoder.h>
#include <limits.h>
#include <PID_v1.h>
#include <SabertoothSimplified.h>

#define UPDATE_RATE_HZ (100) // Needs to be low enough so we have enough encoder measurements at low speed
#define PRINT_EVERY_N_UPDATES (-1)
#define COUNTABLE_EVENTS_PER_REVOLUTION (1296) // At the output shaft (wheel)
#define WHEEL_DIAMETER_IN_METER (0.124968)
#define MAX_SPEED_RPM (313)

SabertoothSimplified ST(Serial1);

uint32_t timestamp;
Encoder myEnc1((uint8_t)11, (uint8_t)10);
Encoder myEnc2((uint8_t)15, (uint8_t)14);

const double wheel_circumference_in_meter = WHEEL_DIAMETER_IN_METER * M_PI;
const double distance_m_per_encoder_tick = wheel_circumference_in_meter/COUNTABLE_EVENTS_PER_REVOLUTION;

double pid_setpoint_m1=0.0;
double prev_motor_command_m1 = 0.0;
double pid_input_m1=0.0;
double pid_output_m1=0.0;

double Kp_m1=0.30;
double Ki_m1=0.001;
double Kd_m1=0.002;
PID pid_m1(&pid_input_m1, &pid_output_m1, &pid_setpoint_m1, Kp_m1, Ki_m1, Kd_m1, DIRECT);

double pid_setpoint_m2=0.0;
double prev_motor_command_m2 = 0.0;
double pid_input_m2=0.0;
double pid_output_m2=0.0;

double Kp_m2=0.30;
double Ki_m2=0.001;
double Kd_m2=0.002;
PID pid_m2(&pid_input_m2, &pid_output_m2, &pid_setpoint_m2, Kp_m2, Ki_m2, Kd_m2, DIRECT);

int getExpectedEventCount(double speed_rpm, double sampleperiod){
  return COUNTABLE_EVENTS_PER_REVOLUTION/((60.0/speed_rpm)/sampleperiod);
}

const int max_expected_event_count = getExpectedEventCount(MAX_SPEED_RPM, 1.0/UPDATE_RATE_HZ);

void setup()
{  
  timestamp = micros();  
  Serial.begin(115200);  
  Serial1.begin(19200);
  SabertoothTXPinSerial.begin(19200); // This is the baud rate you chose with the DIP switches.
  
  pid_input_m1 = 0.0;
  pid_setpoint_m1 = 0.0;
    
  pid_m1.SetMode(AUTOMATIC); 
  pid_m1.SetOutputLimits(-127.0, 127.0);
  pid_m1.SetSampleTime(1000.0/UPDATE_RATE_HZ);  

  pid_input_m2 = 0.0;
  pid_setpoint_m2 = 0.0;
    
  pid_m2.SetMode(AUTOMATIC); 
  pid_m2.SetOutputLimits(-127.0, 127.0);
  pid_m2.SetSampleTime(1000.0/UPDATE_RATE_HZ);  
}

bool do_PID_update(int raw_setpoint, int raw_input, float sample_period, double* setpoint_var, double* input_var, PID* pid){
  *setpoint_var = getExpectedEventCount(raw_setpoint*(MAX_SPEED_RPM/1023.0), sample_period);  
  *setpoint_var = constrain(pid_setpoint_m1, -max_expected_event_count, max_expected_event_count);
  *input_var = raw_input;
  return pid->Compute();
}

int drive_motor(SabertoothSimplified* driver, int motor_nr, double pid_output, int prev_motor_command){
  int motor_command = prev_motor_command+round(pid_output);  
  motor_command = constrain(motor_command, -127, 127);
  driver->motor(motor_nr, motor_command);
  return motor_command;
}

void loop()
{
  
  // Timing stuff to keep a constant 400Hz sample rate
  static uint8_t counter = 0;  
  uint32_t curr_time = micros();
  uint32_t time_diff = curr_time - timestamp;
  if (time_diff < (1000000 / UPDATE_RATE_HZ)) return;
  float samplePeriod = time_diff/1000000.0;
  timestamp = curr_time;


  // Read the new motor encoder positions
  long newPosition1 = myEnc1.readAndReset();
  long newPosition2 = -myEnc2.readAndReset();

  // Calculate the actual speed in meter per seconds.
  // Not needed for the PID motor control, but we'll send this
  // back to our main controller board which might want to do
  // something with it.
  float speed1 = distance_m_per_encoder_tick*newPosition1/samplePeriod;
  float speed2 = distance_m_per_encoder_tick*newPosition2/samplePeriod;
  
  // Our raw 'setpoint input'. For now, this is just a potentiometer that
  // is attached to analog pin 8.
  int potval = analogRead(8);

  // Let our PIDs calculate the desired motor power
  do_PID_update(potval, newPosition1, samplePeriod, &pid_setpoint_m1, &pid_input_m1, &pid_m1);
  do_PID_update(potval, newPosition2, samplePeriod, &pid_setpoint_m2, &pid_input_m2, &pid_m2);
  
  // Actually drive the motors based on the PID output
  prev_motor_command_m1 = drive_motor(&ST, 1, pid_output_m1, prev_motor_command_m1);
  prev_motor_command_m2 = drive_motor(&ST, 2, pid_output_m2, prev_motor_command_m2);


  // only print the calculated output once in a while
  if (counter++ <= PRINT_EVERY_N_UPDATES){
    return;
  }
  
  // reset the counter
  counter = 0;

  // Print out some debugging info  
  Serial.print("Sample period (ms): ");
  Serial.println(samplePeriod*1000);  

  Serial.print("Speed (m/s): ");
  Serial.print(speed1, 4);
  Serial.print(" (");
  Serial.print(newPosition1);
  Serial.print(")");
  Serial.print(" / ");
  Serial.print(speed2, 4);
  Serial.print(" (");
  Serial.print(newPosition2);
  Serial.println(")");

}