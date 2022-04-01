#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>

#include "I2Cdev.h"
#include "MPU6050.h"
#include "pitches.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//=== MACRO DECLARATIONS =========================================================================================================

#define verbose 1

#define daq_select_pin 10
#define digipot_select_pin 15

#define charge_sense_pin 21
#define charger_enable_pin 17

#define buzzer_pin 22
#define led_pin 23

#define buck_keepalive_pin 23

#define bldc_enable 7

#define bldc_0_tach 5
#define bldc_0_direction 6
#define bldc_0_brake 3

#define bldc_1_tach 8
#define bldc_1_direction 2
#define bldc_1_brake 4

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//=== VARIABLES & CONSTANTS ======================================================================================================

MPU6050 imu;

// User configurable variables -----------------------------------

const float over_voltage_setpoint = 4.25;                            // Cell OV Set-Point (units of V)
const float under_voltage_setpoint = 2.7;                            // Cell UV Set-Point (units of V)

const float full_voltage_setpoint = 4.2;                             // Cell full Set-Point (units of V)
const float empty_voltage_setpoint = 3.65;                           // Cell empty Set-Point (units of V)

const byte motor_max_speed = 128;                                    // Max speed for motors

const float kp = 0;
const float ki = 0;
const float kd = 0;

const float gyro_weight  = 0;
const float accel_weight = 0;


// BMS Variables -------------------------------------------------

int temp_int[3]           = {0,0,0};                                 // BMS temperatures (unitless)
float temp[12]            = {0,0,0};                                 // Temps (units of K) | {NTC1, NTC2, DIE}

boolean cell_bal[12]      = {0,0,0,0,0,0,0,0,0,0,0,0};               // Balance FET states, 0 = off

unsigned int cell_voltage_int[12]  = {0,0,0,0,0,0,0,0,0,0,0,0};      // Cell voltages (unitless)
float cell_voltage[12]    = {0,0,0,0,0,0,0,0,0,0,0,0};               // Cell voltages (units of V)

byte battery_fuel = 100;                                             // Battery fuel, 0 to 100 (% Charge)

float mean_cell_voltage;                                             // Floats for cell statistics (units of V)
float max_cell_voltage;
float min_cell_voltage;

byte CVR[18]              = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};   // CVRxx register buffer
byte TMP[5]               = {0,0,0,0,0};                             // TMPx  register buffer


// LTC6802-2 Config Variables ------------------------------------

byte CFGR0 = B00110001;       // WTD   | GPIO2 (led) | GPIO1 | LVLPL| CELL10| CDC2  | CDC1  | CDC0     
                              // CDC bits != 0, Chip NOT in standby 
                              // CDC = 0, STBY || CDC = 1, for Self-Test & UV/OV comparator off

byte CFGR1 = B00000000;       // DCC8  | DCC7  | DCC6  | DCC5 | DCC4  | DCC3  | DCC2  | DCC1     
                              // Discharge no cell
                              
byte CFGR2 = B00000000;       // MC4L  | MC3L  | MC2L  | MC1L | DCC12 | DCC11 | DCC10 | DCC9     
                              // Mask no cell, Discharge no cell
                              
byte CFGR3 = B00000000;       // MC12L | MC11L | MC10L | MC9L | MC8L  | MC7L  | MC6L  | MC5L     
                              // Mask no cell
                              
byte CFGR4 = 113;             // Set cell UVLO to 1.5mV * 16 * 113 = 2.712V
byte CFGR5 = 180;             // Set cell OVLO to 1.5mV * 16 * 180 = 4.32V


// Management Variables ------------------------------------------

byte spi_config_mode = 0;                 // For keeping track of SPI configs & thus saving time
                                          // 0 = Unconfigured, 1 = LTC6802-2, 2 = Digital Pot

boolean battery_fuel_first_call = true;   // For making sure calc_battery_fuel is accurate                  

boolean led_state = 0;                    // For blinkenlights
boolean mute = 0;

unsigned long bldc_0_tach_timer, bldc_1_tach_timer;

unsigned int bldc_0_pulselength;
unsigned int bldc_1_pulselength;

float const pi = 3.141592654;             // Yummy


// Motion variables ---------------------------------------------

float bldc_0_speed;            // Speed of motors (unitless)
float bldc_1_speed;          

int ax_offset, ay_offset, az_offset, wx_offset, wy_offset, wz_offset;       // Sensor offsets

int16_t ax_raw, ay_raw, az_raw, wx_raw, wy_raw, wz_raw;                     // Raw, unitless data from the IMU
  
float ax, ay, az;              // Acceleration data (units: meters/second)                                
float wx, wy, wz;              // Rotation data (units: rad/sec)



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//=== SETUP ======================================================================================================================

void setup(){

  #ifdef verbose
    Serial.begin(115200); delay(1);
    Serial.print("Segbot boot sequence initiated");
  #endif
  
  // Set pin modes ----------------------------------
 
  system_set_pin_impedances();
  
  // Default Pin States -----------------------------

  digitalWrite(buck_keepalive_pin, HIGH);            // Keep alive the power supplies 
  
  play_melody(4);                                    // Dee-doo-dee
  
  bldc_freewheel();                                  // Let's not maim people
  bldc_unbrake();
  
  digitalWrite(bldc_0_direction, LOW); 
  digitalWrite(bldc_1_direction, HIGH); 

  // Get gyro & accel offsets from eeprom -----------

  imu_retrieve_offsets();                            // Get IMU offsets from the EEPROM, if existing

  // Init SPI stuff ---------------------------------
  
  SPI.begin();

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(12, INPUT);
  pinMode(11, OUTPUT);

  digitalWriteFast(9, HIGH);  
  digitalWriteFast(10, HIGH);
  digitalWriteFast(20, HIGH);  
  digitalWriteFast(21, HIGH);
  digitalWriteFast(15, HIGH);
  
  digipot_update(0, 127);
  digipot_update(1, 127);

  delay(10);

                                                     // Write to the DAQ the default values
  
  // Init IMU ----------------------------------------
  
 Wire.begin();                                      // Begin i2c communication
  delay(1);
  
  imu.initialize();                                  // Initialize the IMU
  delay(10);
  
  imu.setDLPFMode(MPU6050_DLPF_BW_256);              // Turn off the IMU's internal low pass filter
  
  imu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);    // Set sensitivity to +-250deg/sec & +-2g. DON'T CHANGE.
  imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);    // See MPU6050 library for details
  
  //imu_retrieve_offsets();                            // Get IMU offsets from EEPROM
  imu_calc_offsets();
  
  // Run self test -----------------------------
  
  //system_self_test(); 
  
  // Tach ISR initialization -------------------
  
  //attachInterrupt(bldc_0_tach, bldc_0_tach_isr, RISING);
  //attachInterrupt(bldc_1_tach, bldc_1_tach_isr, RISING);
  
  delay(200);
  daq_write_configs();
  delay(20);
  daq_write_configs();
};



byte lolol = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//=== LOOP =======================================================================================================================

void loop(){
 //imu_measure();  
  /*
 #ifdef verbose
    Serial.print("\n\n\n\n\n\n");
    Serial.print("aX: "); if(ax>0)Serial.print(" "); Serial.print(ax); Serial.println("\t m/s^2");
    Serial.print("aY: "); if(ay>0)Serial.print(" "); Serial.print(ay); Serial.println("\t m/s^2");
    Serial.print("aZ: "); if(az>0)Serial.print(" "); Serial.print(az); Serial.println("\t m/s^2");
    
    Serial.print("wX: "); if(wx>0)Serial.print(" "); Serial.print(wx); Serial.println("\t rad/s");
    Serial.print("wY: "); if(wy>0)Serial.print(" "); Serial.print(wy); Serial.println("\t rad/s");
    Serial.print("wZ: "); if(wz>0)Serial.print(" "); Serial.print(wz); Serial.println("\t rad/s");
  #endif
  */
//delay(100);


//daq_begin_conversion();
//delay(12);

//daq_read_voltages();
//delay(12);
//daq_write_configs();

//daq_run_self_test();

/*
  digitalWrite(bldc_enable, HIGH);

  digitalWrite(bldc_1_brake, LOW);
  digitalWrite(bldc_1_direction, LOW);
  digitalWrite(bldc_0_brake, LOW);
  digitalWrite(bldc_0_direction, LOW);
*/

  digipot_update(1,0);
  digipot_update(0,5);
  digitalWrite(bldc_enable, HIGH);
  //digitalWrite(bldc_0_direction, HIGH);
  //digipot_update(0, lolol);
  //digipot_update(1, lolol);
  //lolol ++;
  //if(lolol > 128) lolol = 0;
  

  
};



