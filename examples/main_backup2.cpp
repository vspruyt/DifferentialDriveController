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
#define MAX_STEERING_WHEEL_DIFFERENCE_RPM_FACTOR (1.0)

USBSabertoothSerial C(Serial1);
USBSabertooth ST(C, 128);

uint32_t timestamp;
Encoder myEnc1((uint8_t)11, (uint8_t)10);
Encoder myEnc2((uint8_t)15, (uint8_t)14);

const double Kp_normal_mode=1.5;
const double Ki_normal_mode=70.0;
const double Kd_normal_mode=0.00;

const double Kp_deadzone_mode=0.0;
const double Ki_deadzone_mode=2.0;
const double Kd_deadzone_mode=0.00;

SteeringController steering(COUNTABLE_EVENTS_PER_REVOLUTION, UPDATE_RATE_HZ, Kp_normal_mode, Ki_normal_mode, Kd_normal_mode);

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


float map_deadzone(float motor_command, float margin, float deadzone){
  float abs_motor_command = abs(motor_command);
  abs_motor_command = (abs_motor_command < margin) ? 0 : map(abs_motor_command, margin, 2047.0, deadzone, 2047.0) ; // Inserts a Deadzone
  motor_command = (motor_command >= 0) ? abs_motor_command :  -abs_motor_command; // Restores a FullRange Value +-
  return(motor_command);
}

int virtual_deadband = 10;
int physical_deadband = 80;
int fixed_steering_target_ctr = 0;
bool steering_target_hystersis_active = false;
bool steering_hystersis_target_cause = 0;
double last_good_steering_value = 0;

bool motor_1_steering_deadband_hystersis = false;
bool motor_2_steering_deadband_hystersis = false;
float motor_1_last_good_speed = 8.0; // A good default for my motor
float motor_2_last_good_speed = 8.0; // A good default for my motor
float motor_1_last_good_command = -115.0;
float motor_2_last_good_command = -115.0;

bool motor_1_in_deadzone_mode = false;
bool motor_2_in_deadzone_mode = false;

void loop()
{
  
  // Timing stuff to keep a constant sample rate  
  uint32_t curr_time = micros();
  uint32_t time_diff = curr_time - timestamp;
  if (time_diff < (1000000 / UPDATE_RATE_HZ)) return;
  float samplePeriod = time_diff/1000000.0;
  timestamp = curr_time;


  int speed = 0;  

  // Read the new motor encoder positions
  float newPosition1 = -myEnc1.readAndReset();
  float newPosition2 = myEnc2.readAndReset();

  if(motor_1_steering_deadband_hystersis){
    // Interpolate between last good speed outside of deadband, and zero rpm.
    float x0 = motor_1_last_good_command;
    float y0 = motor_1_last_good_speed;    
    float x1 = 0.0;
    float y1 = 0.0;

    float prev_motor_1_command = constrain(speed-steering.pid_output, -2047.0, 2047.0);    
    // int sign = prev_motor_1_command>=0?1:-1;
    // prev_motor_1_command = sign*100+prev_motor_1_command;    
    float abs_motor_command = abs(prev_motor_1_command);
    if(abs_motor_command>=virtual_deadband){
      abs_motor_command = map(abs_motor_command, virtual_deadband, 2047.0, physical_deadband, 2047.0);
    }
    else{
      abs_motor_command = map(abs_motor_command, 0.0, virtual_deadband, 0.0, physical_deadband);
    }    
    prev_motor_1_command = (prev_motor_1_command >= 0) ? abs_motor_command :  -abs_motor_command;
      
    // prev_motor_1_command = map_deadzone(prev_motor_1_command, virtual_deadband, physical_deadband);

    if(x1 == x0){
      // newPosition1 = y1;
    }
    else
      newPosition1 = (y0*(x1 - float(prev_motor_1_command)) + y1*(float(prev_motor_1_command)-x0))/(x1-x0);        

    Serial.print("\nM1 Last good command: ");
    Serial.println(x0);
    Serial.println("M1 Last good speed: ");
    Serial.println(y0);
    Serial.println("M1 Prev motor command: ");
    Serial.println(prev_motor_1_command);
    Serial.println("M1 Interpolated: ");
    Serial.println(newPosition1);
  }

  if(motor_2_steering_deadband_hystersis){
    // Interpolate between last good speed outside of deadband, and zero rpm.
    float x0 = motor_2_last_good_command;
    float y0 = motor_2_last_good_speed;    
    float x1 = 0.0;
    float y1 = 0.0;

    float prev_motor_2_command = constrain(speed+steering.pid_output, -2047.0, 2047.0);    
    float abs_motor_command = abs(prev_motor_2_command);
    if(abs_motor_command>=virtual_deadband){
      abs_motor_command = map(abs_motor_command, virtual_deadband, 2047.0, physical_deadband, 2047.0);
    }
    else{
      abs_motor_command = map(abs_motor_command, 0.0, virtual_deadband, 0.0, physical_deadband);
    }    
    prev_motor_2_command = (prev_motor_2_command >= 0) ? abs_motor_command :  -abs_motor_command;
    // prev_motor_2_command = map_deadzone(prev_motor_2_command, virtual_deadband, physical_deadband);

    if(x1 == x0){
      // newPosition2 = y1;
    }
    else
      newPosition2 = (y0*(x1 - float(prev_motor_2_command)) + y1*(float(prev_motor_2_command)-x0))/(x1-x0);          

    // Serial.print("\nM2 Last good command: ");
    // Serial.println(x0);
    // Serial.println("M2 Last good speed: ");
    // Serial.println(y0);
    // Serial.println("M2 Prev motor command: ");
    // Serial.println(prev_motor_2_command);
    // Serial.println("M2 Interpolated: ");
    // Serial.println(newPosition2);

  }
  
  double speed1 = steering.encoderticks_to_rpm(newPosition1, samplePeriod);
  double speed2 = steering.encoderticks_to_rpm(newPosition2, samplePeriod);  
  
  double steering_target = analogRead(8);
  steering_target = map(double(steering_target), 0.0, 1023.0, -MAX_STEERING_WHEEL_DIFFERENCE_RPM_FACTOR*MAX_SPEED_RPM, MAX_STEERING_WHEEL_DIFFERENCE_RPM_FACTOR*MAX_SPEED_RPM);    
  Serial.println(steering_target);

  // Serial.println("Target speed diff: ");
  // Serial.println(steering_target);
  // Serial.println("Real speed diff: ");
  // Serial.println(speed1-speed2);
  // Serial.println("Real speed 1: ");
  // Serial.println(speed1);
  // Serial.println("Real speed 2: ");
  // Serial.println(speed2);
  // Serial.println("");

  // float min_steering_diff_motor_command = physical_deadband;
  // float actual_speed = (speed1+speed2)/2.0;

  // Serial.print("\nSteering Target:");
  // Serial.println(steering_target);  
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

  // double real_steering_target = steering_target;
  // if(steering_target_hystersis_active){
    // steering_target = last_good_steering_value;
    // if(steering_target<0)
    //   steering_target -= 5;
    // else
    //   steering_target += 5;

  //   int sign = 1;
  //   if(real_steering_target < 0)
  //     sign = -1;

  //   if(real_steering_target*sign < 50)
  //     steering_target = 0.0;
  //   else{
  //     steering_target = last_good_steering_value;
  //     if(steering_target>0)
  //       steering_target -=10;
  //     else if(steering_target<0)
  //       steering_target +=10;
  //     Serial.println("OWWWWWWWWWW");    
  //   }
    
  //   Serial.print("YESSSS: ");    
  //   Serial.println(last_good_steering_value);    
  // }    
  
  
  if(motor_1_steering_deadband_hystersis || motor_2_steering_deadband_hystersis){
    bool pid_needs_update = false;
    if(motor_1_steering_deadband_hystersis && !motor_1_in_deadzone_mode){      
      motor_1_in_deadzone_mode = true;
      pid_needs_update = true;
    }
    if(motor_2_steering_deadband_hystersis && !motor_2_in_deadzone_mode){      
      motor_2_in_deadzone_mode = true;
      pid_needs_update = true;
    }
    if(pid_needs_update){
      steering.update_pid_params(Kp_deadzone_mode, Ki_deadzone_mode, Kd_deadzone_mode);   
      // Serial.println("DEADZONE MODE");
    }
  }
  else{

    bool pid_needs_update = false;
    if(!motor_1_steering_deadband_hystersis && motor_1_in_deadzone_mode){      
      motor_1_in_deadzone_mode = false;
      pid_needs_update = true;
    }
    if(!motor_2_steering_deadband_hystersis && motor_2_in_deadzone_mode){      
      motor_2_in_deadzone_mode = false;
      pid_needs_update = true;
    }
    if(pid_needs_update){
      steering.update_pid_params(Kp_normal_mode, Ki_normal_mode, Kd_normal_mode);      
      // Serial.println("NORMAL MODE");
    }
  }
    

  double steering_offset = steering.update_steering(steering_target, speed1-speed2); 

  // Serial.print("\nSteering Offset:");
  // Serial.println(steering_offset);

  // if(abs(steering_offset) < virtual_deadband+5){
  //   steering_target = 0;    
  //   // steering_offset = 0;
  // }
    
  // Actually drive the motors based on the PID output
  
  // int d = map(analogRead(8), 0, 1023, 0, 300);
  // int d = map(analogRead(8), 0, 1023, 0, 2047);
  // Serial.println(steering_target);

  // int kick_off_offset = 0;
  // if(abs(newPosition1) == 0 && abs(speed+steering_offset) > 100)
  //   kick_off_offset = 5;

    if(abs(speed-steering_offset)<=virtual_deadband){      
      motor_1_steering_deadband_hystersis = true;
    }
    else if(motor_1_steering_deadband_hystersis){
      if(abs(speed-steering_offset)>virtual_deadband+5){
        motor_1_steering_deadband_hystersis = false;
      }
    }    

    
    if(abs(speed+steering_offset)<=virtual_deadband){      
      motor_2_steering_deadband_hystersis = true;
    }
    else if(motor_2_steering_deadband_hystersis){
      if(abs(speed+steering_offset)>virtual_deadband+5){
        motor_2_steering_deadband_hystersis = false;
      }
    }    
  
  int motor_command1 = constrain(speed-steering_offset, -2047, 2047);
  if(motor_1_steering_deadband_hystersis){
    motor_command1 = 0;
  }  
  motor_command1 = map_deadzone(motor_command1, virtual_deadband, physical_deadband);
  if(!motor_1_steering_deadband_hystersis){
    if(abs(motor_1_last_good_command) <= abs(motor_command1) && abs(motor_1_last_good_speed) <= abs(newPosition1)){
      motor_1_last_good_command = motor_command1;
      motor_1_last_good_speed = newPosition1;
    }
  }
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

    

    

    // if(abs(speed-steering_offset)<=virtual_deadband || abs(speed+steering_offset)<=virtual_deadband){      
    //   bool old_steering_target_hystersis_active = steering_target_hystersis_active;
    //   steering_target_hystersis_active = true;        
    //   if(!old_steering_target_hystersis_active){
    //     steering_hystersis_target_cause = steering_target;
        
    //     return;
    //   }
    // }
    // if(steering_target_hystersis_active){        
    //     fixed_steering_target_ctr++;
    //     // if(fixed_steering_target_ctr > 20){
    //     if(abs(real_steering_target)>min(abs(last_good_steering_value)+5, steering_hystersis_target_cause+30)){          
    //       fixed_steering_target_ctr = 0;
    //       steering_target_hystersis_active = false;          
    //     }
    //   }  
    //   else{
    //     last_good_steering_value = real_steering_target;
    //     Serial.print("Last Good: ");
    //     Serial.println(real_steering_target);
    //   }
    
    // else
    // Serial.print(steering_offset);
    // Serial.print(" / ");
    // Serial.println(motor_command1);
      ST.motor(1, motor_command1);
  // }
  
  // if(abs(newPosition2) == 0 && abs(speed+steering_offset) > 100)
  //   kick_off_offset = 5;

  int motor_command2 = constrain(speed+steering_offset, -2047, 2047);
  if(motor_2_steering_deadband_hystersis){
    motor_command2 = 0;
  }
  motor_command2 = map_deadzone(motor_command2, virtual_deadband, physical_deadband);

  if(!motor_2_steering_deadband_hystersis){
    if(abs(motor_2_last_good_command) <= abs(motor_command2) && abs(motor_2_last_good_speed) <= abs(newPosition2)){
      motor_2_last_good_command = motor_command2;
      motor_2_last_good_speed = newPosition2;
    }
  }
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
  Serial.print(steering.pid_setpoint);
  Serial.print("\t");
  Serial.print(steering.pid_input);  
  Serial.print("\t");
  Serial.println(steering.pid.GetKi());  
  // Serial.print("\t");
  // Serial.print(newPosition1);  
  // Serial.print("\t");
  // Serial.print(newPosition2);  
  // Serial.print("\t");
  // Serial.println(motor_command1);    
  
}