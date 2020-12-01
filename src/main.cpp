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

#define UPDATE_RATE_HZ (40) // Needs to be low enough so we have enough encoder measurements at low speed
#define PRINT_EVERY_N_UPDATES (-1)
#define COUNTABLE_EVENTS_PER_REVOLUTION (1296) // At the output shaft (wheel)
#define MAX_SPEED_RPM (313)
#define MAX_STEERING_WHEEL_DIFFERENCE_RPM (313)
#define BLE_TIMEOUT_MICROSECONDS (1000000)

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

Encoder myEnc1((uint8_t)11, (uint8_t)10);
Encoder myEnc2((uint8_t)15, (uint8_t)14);

uint32_t timestamp;

PidParams steering_pid_params_normal_mode{1.5, 70.0, 0.0};
PidParams steering_pid_params_deadzone_mode{0.2, 5.0, 0.0};

int steering_virtual_deadzone = 10;
int steering_physical_deadzone = 80;


SteeringController steering(COUNTABLE_EVENTS_PER_REVOLUTION, UPDATE_RATE_HZ, steering_pid_params_normal_mode, steering_pid_params_deadzone_mode);


void setup()
{    
  timestamp = micros();  
  timestamp_last_ble_message = timestamp;
  Serial.begin(115200);  
  while (!Serial) {
    ; // wait for serial port to connect.
  }

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
  
}

bool readFromSerial(Stream& input_device, int message_length, uint8_t* buffer, const byte* header) {
  if(input_device.available() < message_length+2)
    return false;  
  bool header_started = false;
  bool message_started = false;  
  while (input_device.available()) {

    // If the header was sent AND the message was started, we can start reading
    // out the message values
    if(header_started && message_started){            
      while(input_device.available() < message_length)
        yield();            

      for(int i=0; i<message_length; i++){        
        buffer[i] = input_device.read();
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

void bleEvent(){    
  if(readFromSerial(ble, 5, ble_data_buffer, ble_header)){
    new_ble_reading_available = true;
  }          
}

double speed = 0;  
double steering_target = 0;

void loop()
{
    // Timing stuff to keep a constant sample rate  
    uint32_t curr_time = micros();
    uint32_t time_diff = curr_time - timestamp;
    if (time_diff < (1000000 / UPDATE_RATE_HZ)) return;
    float samplePeriod = time_diff/1000000.0;
    timestamp = curr_time;

    // Check if there is a bluetooth command ready in the buffer
    bleEvent();
    if(new_ble_reading_available){      
      bluetooth_connected = true;
      timestamp_last_ble_message = timestamp;
      new_ble_reading_available=false;                  
      float val = *((float*)&(ble_data_buffer[1]));
      if(ble_data_buffer[0] == 0){
        speed = val;
        speed = map(speed, -1.0, 1.0, 1800, -1800);
      }
      else{
        steering_target = val;
        steering_target = map(steering_target, -1.0, 1.0, MAX_STEERING_WHEEL_DIFFERENCE_RPM, -MAX_STEERING_WHEEL_DIFFERENCE_RPM);
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

    // Read the new motor encoder positions
    float newPosition1 = -myEnc1.readAndReset();
    float newPosition2 = myEnc2.readAndReset();   

    // double steering_target = analogRead(8);
    // steering_target = map(double(steering_target), 0.0, 1023.0, -MAX_STEERING_WHEEL_DIFFERENCE_RPM, MAX_STEERING_WHEEL_DIFFERENCE_RPM);

    int motor_command1=0, motor_command2=0;
    int steering_offset = steering.update_steering(steering_target, speed, newPosition1, newPosition2, samplePeriod); 
    steering.set_pid_params_depending_on_deadzone(speed, steering_offset);
    steering.calc_motor_commands(speed, steering_offset, &motor_command1, &motor_command2);
    steering.update_last_good_speed_measurement(newPosition1, newPosition2, motor_command1, motor_command2);    

    if(!bluetooth_connected){
      motor_command1 = 0;
      motor_command2 = 0;
    }

    ST.motor(1, motor_command1);
    ST.motor(2, motor_command2);
        

    // Serial.print(steering.pid_setpoint);
    // Serial.print("\t");
    // Serial.print(steering.pid_input);  
    // Serial.print("\t");
    // Serial.println(steering.pid.GetKi());      
    // Serial.println(samplePeriod, 4);    
}