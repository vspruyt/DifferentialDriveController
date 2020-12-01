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


int map_deadzone(int motor_command, int margin, int deadzone){
  int16_t abs_motor_command = abs(motor_command);
  abs_motor_command = (abs_motor_command < margin) ? 0 : map(abs_motor_command, margin, 2047, deadzone, 2047) ; // Inserts a Deadzone
  motor_command = (motor_command >= 0) ? abs_motor_command :  -abs_motor_command; // Restores a FullRange Value +-
  return(motor_command);
}

int virtual_deadband = 100;
int physical_deadband = 100;
int fixed_steering_target_ctr = 0;
bool steering_target_hystersis_active = false;
bool steering_hystersis_target_cause = 0;
double last_good_steering_value = 0;
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

  double speed1 = steering.encoderticks_to_rpm(newPosition1, samplePeriod);
  double speed2 = steering.encoderticks_to_rpm(newPosition2, samplePeriod);
  
  double steering_target = analogRead(8);
  steering_target = map(double(steering_target), 0.0, 1023.0, -313.0, 313.0);    

  // float min_steering_diff_motor_command = physical_deadband;
  // float actual_speed = (speed1+speed2)/2.0;

  Serial.print("\nSteering Target:");
  Serial.println(steering_target);  
  // Serial.print("Actual speed:");
  // Serial.println(speed1-);

  // float abs_steering_target = abs(steering_target);
  // if(abs_steering_target < 20){
  //   steering_target = 0;        
  //   steering_target_hystersis_active = true;
  // }
  // else{
  //   if(steering_target_hystersis_active){
  //     steering_target = 0;                
  //     Serial.println("YESSSS");
  //     if(abs_steering_target > 30){                
  //       steering_target_hystersis_active = false;
  //     }
  //   }    
  // }

  double real_steering_target = steering_target;
  if(steering_target_hystersis_active){
    // steering_target = last_good_steering_value;
    // if(steering_target<0)
    //   steering_target -= 5;
    // else
    //   steering_target += 5;

    int sign = 1;
    if(real_steering_target < 0)
      sign = -1;

    if(real_steering_target*sign < 50)
      steering_target = 0.0;
    else{
      steering_target = last_good_steering_value;
      Serial.println("OWWWWWWWWWW");    
    }
    
    Serial.print("YESSSS: ");    
    Serial.println(last_good_steering_value);    
  }    
  
  
  
  double steering_offset = steering.update_steering(steering_target, speed1-speed2); 

  // Serial.print("\nSteering Offset:");
  // Serial.println(steering_offset);

  // if(abs(steering_offset) < virtual_deadband+5){
  //   steering_target = 0;    
  //   // steering_offset = 0;
  // }
    
  // Actually drive the motors based on the PID output
  int speed = 0;  
  // int d = map(analogRead(8), 0, 1023, 0, 300);
  // int d = map(analogRead(8), 0, 1023, 0, 2047);
  // Serial.println(steering_target);

  // int kick_off_offset = 0;
  // if(abs(newPosition1) == 0 && abs(speed+steering_offset) > 100)
  //   kick_off_offset = 5;

  
  int motor_command1 = constrain(speed-steering_offset, -2047, 2047);    
  motor_command1 = map_deadzone(motor_command1, virtual_deadband, physical_deadband);
  // if(!(abs(motor_command1)>20 && abs(motor_command1)<80)){    
  //   if(motor_command1<0)
  //     motor_command1 -= 80;
  //   else if(motor_command1>0)
  //     motor_command1 += 80;  
    // Serial.println(motor_command1);
    
    // Serial.println(samplePeriod, 4);
    // Serial.print("M1: ");
    // Serial.print(steering.encoderticks_to_rpm(newPosition1, samplePeriod));
    // Serial.print(motor_command1);
    // Serial.print(" / ");
    // Serial.print(constrain(speed-steering_offset, -2047, 2047));

    // bool motor_1_will_be_in_deadzone = abs(speed-steering_offset)<=5;
    // bool motor_2_will_be_in_deadzone = abs(speed+steering_offset)<=5;
        
    if(abs(speed-steering_offset)<=virtual_deadband || abs(speed+steering_offset)<=virtual_deadband){      
      bool old_steering_target_hystersis_active = steering_target_hystersis_active;
      steering_target_hystersis_active = true;        
      if(!old_steering_target_hystersis_active){
        steering_hystersis_target_cause = steering_target;
        
        return;
      }
    }
    if(steering_target_hystersis_active){        
        fixed_steering_target_ctr++;
        // if(fixed_steering_target_ctr > 20){
        if(abs(real_steering_target)>min(abs(last_good_steering_value)+5, steering_hystersis_target_cause+30)){          
          fixed_steering_target_ctr = 0;
          steering_target_hystersis_active = false;          
        }
      }  
      else{
        last_good_steering_value = real_steering_target;
        Serial.print("Last Good: ");
        Serial.println(real_steering_target);
      }
    
    // else
    Serial.print(speed1);
    Serial.print(" / ");
    Serial.println(motor_command1);
      ST.motor(1, motor_command1);
  // }
  
  // if(abs(newPosition2) == 0 && abs(speed+steering_offset) > 100)
  //   kick_off_offset = 5;

  int motor_command2 = constrain(speed+steering_offset, -2047, 2047);
  motor_command2 = map_deadzone(motor_command2, virtual_deadband, physical_deadband);
  // if(!(abs(motor_command2)>20 && abs(motor_command2)<80)){
  //   if(motor_command2<0)
  //     motor_command2 -= 80;
  //   else if(motor_command2>0)
  //     motor_command2 += 80;
  // Serial.println(motor_command2);  
    // Serial.print(" / M2: ");     
    // Serial.println(steering.encoderticks_to_rpm(newPosition2, samplePeriod));
    // Serial.print(motor_command2);
    // Serial.print(" / ");
    // Serial.println(constrain(speed+steering_offset, -2047, 2047));
    // if(motor_1_will_be_in_deadzone ^ motor_2_will_be_in_deadzone){

    // }
    // else
    
      ST.motor(2, motor_command2); 
  // }


  // M1: 69
  // M2: 76
  // M1: -70
  // M2: -82
  
  // Serial.println("");
  

// 63-70
//60-78
  // int d = analogRead(8);
  // d = map(d, 0, 1023, -500, 500);

  // Serial.print(d);
  // int d = map_deadzone(122, 0, 100);
  // int d = map(11, 10, 2047, 100, 2047);
  // Serial.print(" / ");
  // Serial.println(d);

  
  // Serial.println(d);
  // ST.motor(1, d);
  // ST.motor(2, d);
    
  

  // Serial.println(samplePeriod, 4);
  // Serial.print(steering.pid_setpoint);
  // Serial.print("\t");
  // Serial.println(steering.pid_input);  
  
}