#include <Arduino.h>
#include <Encoder.h>
#include <limits.h>
#include <PID_v1.h>
#include <USBSabertooth.h>
#include "SteeringController.h"

#define UPDATE_RATE_HZ (40) // Needs to be low enough so we have enough encoder measurements at low speed
#define PRINT_EVERY_N_UPDATES (-1)
#define COUNTABLE_EVENTS_PER_REVOLUTION (1296) // At the output shaft (wheel)
#define MAX_SPEED_RPM (313)

USBSabertoothSerial C(Serial1);
USBSabertooth ST(C, 128);

uint32_t timestamp;
Encoder myEnc1((uint8_t)11, (uint8_t)10);
Encoder myEnc2((uint8_t)15, (uint8_t)14);

SteeringController steering(COUNTABLE_EVENTS_PER_REVOLUTION, UPDATE_RATE_HZ);

void setup()
{  
  timestamp = micros();  
  Serial.begin(115200);  
  while (!Serial) {
    ; // wait for serial port to connect.
  }

  Serial1.begin(115200);
  ST.setTimeout(500); // Stop motor 0.5 seconds after signal was lost  
}

void loop()
{
  
  // Timing stuff to keep a constant sample rate  
  uint32_t curr_time = micros();
  uint32_t time_diff = curr_time - timestamp;
  if (time_diff < (1000000 / UPDATE_RATE_HZ)) return;
  float samplePeriod = time_diff/1000000.0;
  timestamp = curr_time;


  // Read the new motor encoder positions
  long newPosition1 = -myEnc1.readAndReset();
  long newPosition2 = myEnc2.readAndReset();
  
  double steering_target = analogRead(8)*(MAX_SPEED_RPM/1023.0);
  double steering_offset = steering.update_steering(steering_target, newPosition1-newPosition2, samplePeriod);  
  
//   Actually drive the motors based on the PID output
  int speed = 0;  
  int motor_command1 = constrain(speed-steering_offset, -2047, 2047);
  ST.motor(1, motor_command1);
  int motor_command2 = constrain(speed+steering_offset, -2047, 2047);
  ST.motor(2, motor_command2);  

  Serial.println(samplePeriod, 4);
  // Serial.print(steering.pid_setpoint);
  // Serial.print("\t");
  // Serial.println(steering.pid_input);  
  
}