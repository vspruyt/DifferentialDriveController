#include <Arduino.h>
#include <Encoder.h>
#include <limits.h>
#include <PID_v1.h>
#include <USBSabertooth.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"
#include "SteeringController.h"

#define UPDATE_RATE_HZ (40.0) // Needs to be low enough so we have enough encoder measurements at low speed
#define PRINT_EVERY_N_UPDATES (-1)
#define COUNTABLE_EVENTS_PER_REVOLUTION (1296) // At the output shaft (wheel)
#define MAX_SPEED_RPM (313)
#define MAX_STEERING_WHEEL_DIFFERENCE_RPM (313)
#define BLE_TIMEOUT_MICROSECONDS (1000000)

#define SERIAL_COMM_MAIN_CONTROLLER Serial5
#define SERIAL_COMM_MAIN_CONTROLLER_BAUDRATE 57600

#define FILTER_TIME_CONST   (4.0)       // time constant: time in samples to reach ~ 63% of a steady value
                                        // the rise time of a signal to > 90% is approx 2.5 times this value
                                        // at 3kHz sampling a value of 100 == rise time to 95% of approx 100mS
                                        
#define FILTER_WEIGHT (FILTER_TIME_CONST/(FILTER_TIME_CONST +1.0))

bool new_main_controller_reading_available = false;
byte main_controller_header[] = {1, 2};
double desired_speed_from_main_controller = 0;

// Bluetooth low energy
Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);
// A message consists of a start-of-header byte, a start-of-text byte, and the actual payload.
// The payload contains an identifier byte (e.g. go left or go forward), and a quantifier 
// float (e.g. desired velocity) represented by 4 bytes.
// So in total, a message has a byte size of 7.
uint8_t ble_data_buffer[5];
bool new_ble_reading_available = false;
byte ble_header[] = {1, 2}; //{'!', 'B'};
uint32_t timestamp_last_ble_message;

bool bluetooth_connected = false;

USBSabertoothSerial C(Serial1);
USBSabertooth ST(C, 128);

Encoder myEnc1((uint8_t)15, (uint8_t)14);
Encoder myEnc2((uint8_t)11, (uint8_t)10);

uint32_t timestamp_outer_loop;
uint32_t timestamp_inner_loop;

// PidParams steering_pid_params_normal_mode{1.5, 70.0, 0.0};
PidParams steering_pid_params_normal_mode{1.5, 60.0, 0.0};
// PidParams steering_pid_params_deadzone_mode{2.0, 5.0, 0.0};
// PidParams steering_pid_params_normal_mode{0.0, 28.0, 0.0};
PidParams steering_pid_params_deadzone_mode{0.1, 5.0, 0.0};

int steering_virtual_deadzone = 10;
int steering_physical_deadzone = 60;

double speeding_pid_input = 0.0;
double speeding_pid_output = 0.0;
double smoothed_speeding_pid_output = 0.0;
double speeding_pid_setpoint = 0.0;
double target_speeding_pid_setpoint = 0.0;
// double speeding_pid_Kp = 0.001;
// double speeding_pid_Ki = 0.04;

// double speeding_pid_Kp = 0.06;
// double speeding_pid_Ki = 0.00;
// double speeding_pid_Kd = 0.0001;

// PID speeding_pid = PID(&speeding_pid_input, &speeding_pid_output, &speeding_pid_setpoint, speeding_pid_Kp, speeding_pid_Ki, speeding_pid_Kd, P_ON_E, REVERSE);

// THIS ONE WORKS BUT NOT FOR REMOTE CONTROLL
// double aggressive_speeding_pid_Kp = 0.2;
// double aggressive_speeding_pid_Ki = 0.1;
// double aggressive_speeding_pid_Kd = 0.05;

// THIS ONE WORKS BUT NOT ON HILLS
// double aggressive_speeding_pid_Kp = 0.06;
// double aggressive_speeding_pid_Ki = 0.00;
// double aggressive_speeding_pid_Kd = 0.0001;

// PID speeding_pid = PID(&speeding_pid_input, &speeding_pid_output, &speeding_pid_setpoint, aggressive_speeding_pid_Kp, aggressive_speeding_pid_Ki, aggressive_speeding_pid_Kd, P_ON_E, REVERSE);

// double aggressive_speeding_pid_Kp = 0.06;
// double aggressive_speeding_pid_Ki = 0.00;
// double aggressive_speeding_pid_Kd = 0.0001;

// VERY GOOD
double aggressive_speeding_pid_Kp = 0.06;
double aggressive_speeding_pid_Ki = 0.03;
double aggressive_speeding_pid_Kd = 0.005;

double conservative_speeding_pid_Kp = 0.00;
double conservative_speeding_pid_Ki = 0.03;
double conservative_speeding_pid_Kd = 0.002;


PID speeding_pid = PID(&speeding_pid_input, &speeding_pid_output, &speeding_pid_setpoint, aggressive_speeding_pid_Kp, aggressive_speeding_pid_Ki, aggressive_speeding_pid_Kd, P_ON_E, REVERSE);


// double speeding_pid_Kp = 0.20;
// double speeding_pid_Ki = 0.1;
// double speeding_pid_Kd = 0.05;

// PID speeding_pid = PID(&speeding_pid_input, &speeding_pid_output, &speeding_pid_setpoint, speeding_pid_Kp, speeding_pid_Ki, speeding_pid_Kd, P_ON_M, REVERSE);


SteeringController steering(COUNTABLE_EVENTS_PER_REVOLUTION, UPDATE_RATE_HZ, steering_pid_params_normal_mode, steering_pid_params_deadzone_mode);

void setup()
{    
  digitalWrite(13, HIGH);    // Light up the internal led to show it's game on
  delay(20);
  timestamp_outer_loop = micros();    
  timestamp_last_ble_message = timestamp_outer_loop;
  timestamp_inner_loop = timestamp_outer_loop;
  Serial.begin(115200);  
  // while (!Serial) {
    // ; // wait for serial port to connect.
  // }

  speeding_pid.SetMode(AUTOMATIC); 
  speeding_pid.SetOutputLimits(-20, 20);
  speeding_pid.SetSampleTime(1000.0/UPDATE_RATE_HZ);

  // Configure the communication port to the main controller
  SERIAL_COMM_MAIN_CONTROLLER.begin(SERIAL_COMM_MAIN_CONTROLLER_BAUDRATE); 

  Serial1.begin(115200);
  ST.setTimeout(500); // Stop motor 0.5 seconds after signal was lost  

  steering.setDeadzone(steering_physical_deadzone, steering_virtual_deadzone);

  // Configure the bluetooth chip    
  if(ble.begin(VERBOSE_MODE)){
    ble.echo(false);    
    ble.verbose(false);
    ble.setMode(BLUEFRUIT_MODE_DATA);             
    
  }
  else{
    Serial.println("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?");
  }
  delay(20);
}

uint16_t crc16_update(uint16_t crc, uint8_t a) {
  int i;
  crc ^= a;
  for (i = 0; i < 8; i++) {
    if (crc & 1) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc = (crc >> 1);
    }
  }
  return crc;
}


const int serial_timeout_ms = 5;
bool readFromSerial(Stream& input_device, int message_length, uint8_t* buffer, const byte* header, int crc_size=0) {
  if(input_device.available() < message_length+2+crc_size)
    return false;  
  bool header_started = false;
  bool message_started = false;  
  while (input_device.available()) {

    // If the header was sent AND the message was started, we can start reading
    // out the message values
    if(header_started && message_started){   
      uint32_t start_yield_ts = millis();         
      while(input_device.available() < message_length+crc_size){
        if(millis()-start_yield_ts > serial_timeout_ms){
          Serial.println("SERIAL TIMEOUT!");         
          while (input_device.available())
            input_device.read(); 
          return false;
        }
        yield();            
      }

      for(int i=0; i<message_length; i++){        
        buffer[i] = input_device.read();            
      }

      if(crc_size > 0){
        uint16_t crc;
        for(int i=0; i<crc_size; i++){
          ((uint8_t*)&crc)[i] = input_device.read();
        }

        uint16_t expected_crc = 0xFFFF;        
        for (uint16_t i = 0; i < message_length; i++) {
          expected_crc = crc16_update(expected_crc, buffer[i]);
        }

        if(expected_crc != crc){
          Serial.println("BAD PACKET!");
          while (input_device.available())
            input_device.read(); 
          return false;
        }                
      }
      return true;                  
    }

    else{
      // get the new byte:
      uint8_t inbyte = (uint8_t)input_device.read();      

      // If the header is already started, we now expect the start-of-message.
      // If that is not the case, the header was actually not started yet.
      if(header_started && !message_started){
        if(inbyte==header[1]){
          message_started = true;
          continue;
        }
        else
          header_started = false;
      }

      // Check if the header is being sent
      if(!header_started){
        // check if the byte is a start-of-header byte
        if(inbyte==header[0]){
          header_started = true;
          continue;
        }
      }
    }
  }

  return false;
}

void serialEvent5() {      
  if(readFromSerial(SERIAL_COMM_MAIN_CONTROLLER, sizeof(desired_speed_from_main_controller), (uint8_t*) &desired_speed_from_main_controller, main_controller_header, 2)){
    new_main_controller_reading_available = true;    
  }
}

void bleEvent(){    
  if(readFromSerial(ble, 5, ble_data_buffer, ble_header)){
    new_ble_reading_available = true;
  }          
}

double speed = 0;  
double steering_target = 0;
int steering_offset = 0;

double corrected_speed = 0;

int smoothed_motor_command1 = 0;
int smoothed_motor_command2 = 0;

float samplePeriod_outer_loop;
float samplePeriod_inner_loop;

void loop()
{    

    // ST.motor(1, 80);    
    // ST.motor(2, 80);
    // return;

    bool got_main_controller_reading = false;
    if(new_main_controller_reading_available){ 
      corrected_speed = desired_speed_from_main_controller;
      // Serial.println(desired_speed_from_main_controller);
      new_main_controller_reading_available = false; 
      got_main_controller_reading = true;
    }

    // Timing stuff to keep a constant sample rate  
    bool do_steering = true;
    bool do_remote_control = true;
    uint32_t curr_time = micros();
    uint32_t time_diff = curr_time - timestamp_outer_loop;
    if (time_diff < (1000000 / UPDATE_RATE_HZ)){
      do_steering = false;
      do_remote_control = false;
    }

    if(!(do_steering || do_remote_control) && !got_main_controller_reading)
      return;    

    if(got_main_controller_reading){        
        samplePeriod_inner_loop = (curr_time - timestamp_inner_loop)/1000000.0;    
        timestamp_inner_loop = curr_time;
    }
    if(do_steering || do_remote_control){
      samplePeriod_outer_loop = time_diff/1000000.0;   
      timestamp_outer_loop = curr_time;      
    }
    
    // do_steering = false; // Disable steering for now    

    float newPosition1 = 0.0;
    float newPosition2 = 0.0;  
    double speed1 = 0.0;
    double speed2 = 0.0;
    if(do_steering || do_remote_control){      
      // Serial.println(samplePeriod_outer_loop, 4);

      // Read the new motor encoder positions
      newPosition1 = myEnc1.readAndReset();
      newPosition2 = -myEnc2.readAndReset();        

      // Serial.print(newPosition1);
      // Serial.print(" / ");
      // Serial.println(newPosition2);

      speed1 = steering.encoderticks_to_rpm(newPosition1, samplePeriod_outer_loop);
      speed2 = steering.encoderticks_to_rpm(newPosition2, samplePeriod_outer_loop);    
      double avg_speed = (speed1+speed2)/2.0;  
      // Serial.print(speed1);
      // Serial.print(" / ");
      // Serial.println(speed2);

      // Check if there is a bluetooth command ready in the buffer
      bleEvent();
      if(new_ble_reading_available){      
        bluetooth_connected = true;
        timestamp_last_ble_message = timestamp_outer_loop;
        new_ble_reading_available=false;                  
        float val = *((float*)&(ble_data_buffer[1]));
        if(ble_data_buffer[0] == 0){
          speed = val;
          speed = map(speed, -1.0, 1.0, -2000, 2000);    
          int sign = speed<0?-1:1;
          target_speeding_pid_setpoint = sign*constrain(speed*speed/4000.0, 0, 2000); // Quadratic mapping to get more control at slow speeds
        }
        else{
          steering_target = val;
          steering_target = map(steering_target, -1.0, 1.0, -MAX_STEERING_WHEEL_DIFFERENCE_RPM, MAX_STEERING_WHEEL_DIFFERENCE_RPM);
        }
        // Serial.print("Speed: ");
        // Serial.print(speed);
        // Serial.print(" / Steering: ");
        // Serial.println(steering_target);
                      
        char outstr[30];
        sprintf(outstr, "\001Speed: %.1f\nSteering: %.1f\n", speed, steering_target);      
        ble.print(outstr);            
      }
      else{
        if(curr_time - timestamp_last_ble_message > BLE_TIMEOUT_MICROSECONDS){
          bluetooth_connected = false;        
        }      
      }
      
      if(abs(target_speeding_pid_setpoint - speeding_pid_setpoint) < 200)
        speeding_pid_setpoint = target_speeding_pid_setpoint;
      else
        speeding_pid_setpoint = 0.7*speeding_pid_setpoint + 0.3*target_speeding_pid_setpoint;      

      // Serial.print(speeding_pid_setpoint);
      // Serial.print(" / ");      
      // speeding_pid_input = avg_speed;
      speeding_pid_input = (FILTER_WEIGHT * speeding_pid_input ) + (1.0-FILTER_WEIGHT) * avg_speed;       
      // Serial.print(speeding_pid_input);
      // Serial.print(" -> ");
    
    
    if(abs(speeding_pid_setpoint - speeding_pid_input) < 10 && abs(speeding_pid_setpoint - avg_speed) < 25 && abs(speeding_pid_setpoint)<4)
      speeding_pid.SetTunings(conservative_speeding_pid_Kp, conservative_speeding_pid_Ki, conservative_speeding_pid_Kd, P_ON_E);
    else
      speeding_pid.SetTunings(aggressive_speeding_pid_Kp, aggressive_speeding_pid_Ki, aggressive_speeding_pid_Kd, P_ON_E);
    
    // Serial.print(avg_speed);
    // Serial.print(" / ");
    // Serial.print(speeding_pid_input);
    // Serial.print(" / ");
    // Serial.print(speeding_pid_setpoint);    
    // Serial.print(" / ");
    // Serial.print(abs(speeding_pid_setpoint - speeding_pid_input));
    // Serial.print(" / ");
    
    // Serial.println(speeding_pid.GetKp());

      speeding_pid.Compute();
      // Serial.print(speeding_pid_output);
      // Serial.print(" / ");           
      // smoothed_speeding_pid_output = (FILTER_WEIGHT * smoothed_speeding_pid_output ) + (1.0-FILTER_WEIGHT) * speeding_pid_output;       
      smoothed_speeding_pid_output = speeding_pid_output;
      // Serial.println(smoothed_speeding_pid_output);
      // Serial.println(abs(speeding_pid_setpoint - speeding_pid_input));

      uint16_t crc = 0xFFFF;
      uint8_t* buf = (uint8_t*)&smoothed_speeding_pid_output;
      for (uint16_t i = 0; i < sizeof(smoothed_speeding_pid_output); i++) {
        crc = crc16_update(crc, buf[i]);
      }
      // Serial.print("CRC: ");
      // Serial.println(crc, HEX);
      
      // smoothed_speeding_pid_output = 0.0;
      SERIAL_COMM_MAIN_CONTROLLER.print("\001\002"); // Start-of-header and start-of-text bytes
      SERIAL_COMM_MAIN_CONTROLLER.write((uint8_t*) &(smoothed_speeding_pid_output), sizeof(smoothed_speeding_pid_output));
      SERIAL_COMM_MAIN_CONTROLLER.write((uint8_t*) &crc, 2);
      

      // Serial.print("Speed M1: ");
      // Serial.print(speed1, 4);
      // Serial.print(" / Speed M2: ");
      // Serial.println(speed2, 4);    
    }    


    // double corrected_speed = desired_speed_from_main_controller;

    // float prev_motor_command = 177;

    // ST.motor(1, prev_motor_command*2);    
    // ST.motor(2, prev_motor_command);
    // Serial.print("Encoder1: ");
    // Serial.println(newPosition1);    
    // Serial.print("Encoder2: ");
    // Serial.println(newPosition2);
    // Serial.print("Speed1: ");
    // Serial.println(speed1);    
    // Serial.print("Speed2: ");
    // Serial.println(speed2);
    // Serial.print("Corrected speed: ");
    // Serial.println(corrected_speed);   
    // return; 
    
    
    // float abs_motor_command = abs(prev_motor_command);
    // if(abs_motor_command>=steering_virtual_deadzone)
    //     abs_motor_command = map(abs_motor_command, steering_virtual_deadzone, 2047.0, steering_physical_deadzone, 2047.0);        
    // else
    //     abs_motor_command = map(abs_motor_command, 0.0, steering_virtual_deadzone, 0.0, steering_physical_deadzone);
        
    // prev_motor_command = (prev_motor_command >= 0) ? abs_motor_command :  -abs_motor_command;        

    // float x0 = 300;
    // float y0 = 24.0;
    // float x1 = 0.0;
    // float y1 = 0.0;
    // float interpolated_val = (y0*(x1 - prev_motor_command) + y1*(prev_motor_command-x0))/(x1-x0);  
    // Serial.print("Interpolated: ");
    // Serial.println(interpolated_val);

    // Serial.println("");
        
    // return;
    

    // double steering_target = analogRead(8);
    // steering_target = map(double(steering_target), 0.0, 1023.0, -MAX_STEERING_WHEEL_DIFFERENCE_RPM, MAX_STEERING_WHEEL_DIFFERENCE_RPM);

    // steering.motor1_in_deadzone = true;
    // steering.motor2_in_deadzone = true;

    if(do_steering){      
      steering_offset = steering.update_steering(steering_target, corrected_speed, newPosition1, speed1, newPosition2, speed2, samplePeriod_outer_loop);             
      steering.set_pid_params_depending_on_deadzone(corrected_speed, steering_offset);
      // steering.calc_motor_commands(corrected_speed, steering_offset, &motor_command1, &motor_command2);  
    }

    // steering.motor1_in_deadzone = true;
    // steering.motor2_in_deadzone = true;    
        
    
    // if(!bluetooth_connected){
    //   motor_command1 = 0;
    //   motor_command2 = 0;
    //   speed = 0;
    // }

    // Serial.print("Maincontroller: ");
    // Serial.println(desired_speed_from_main_controller);
    // Serial.print("Speed: ");
    // Serial.println(corrected_speed);
    // Serial.print("Command: ");
    // Serial.println(motor_command1);    
        
    int motor_command1 = constrain(desired_speed_from_main_controller-steering_offset, -2047, 2047);
    int motor_command2 = constrain(desired_speed_from_main_controller+steering_offset, -2047, 2047);

    // int dz = 60;
    // motor_command1 = steering.map_deadzone(motor_command1, 30, dz);
    // motor_command2 = steering.map_deadzone(motor_command2, 30, dz);

    int dz = 20;
    motor_command1 = steering.map_deadzone(motor_command1, 10, dz);
    motor_command2 = steering.map_deadzone(motor_command2, 10, dz);

    if(do_steering)
      steering.update_last_good_speed_measurement(newPosition1, newPosition2, motor_command1, motor_command2);    

    // Serial.println(motor_command1);
  
    smoothed_motor_command1 = (FILTER_WEIGHT * smoothed_motor_command1 ) + (1.0-FILTER_WEIGHT) * motor_command1;       
    smoothed_motor_command2 = (FILTER_WEIGHT * smoothed_motor_command2 ) + (1.0-FILTER_WEIGHT) * motor_command2;

    int final_motor_command1 = motor_command1;
    int final_motor_command2 = motor_command2;
    
    if(final_motor_command1 == 0)
      final_motor_command1 = smoothed_motor_command1;
    if(final_motor_command2 == 0)
      final_motor_command2 = smoothed_motor_command2;
        
    if(got_main_controller_reading){
      Serial.print(samplePeriod_inner_loop, 4);
      Serial.print(" / ");
      Serial.print(final_motor_command1);
      Serial.print(" / ");
      Serial.print(final_motor_command2);
      Serial.print(" / ");
      Serial.println(do_steering || do_remote_control);

      ST.motor(1, final_motor_command1);
      ST.motor(2, final_motor_command2); 
    }           

    // if(do_steering){
    //   Serial.print(steering.pid_setpoint);
    //   Serial.print("\t");
    //   Serial.print(steering.pid_input);  
    //   Serial.print("\t");
    //   Serial.println(steering.pid.GetKi());      
      // Serial.println(samplePeriod_outer_loop, 4);    
    // }

    // Serial.print("M1: ");
    // Serial.print(motor_command1);
    // Serial.print("->");
    // Serial.print(newPosition1);
    // Serial.print("\t");
    // Serial.print("M2: ");
    // Serial.print(motor_command2);
    // Serial.print("->");
    // Serial.println(newPosition2);  
    // Serial.print("\t");
    // Serial.println(steering.pid.GetKi());          
}