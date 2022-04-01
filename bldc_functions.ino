/* 

    This page contains functions relating to the IMU. It is not portable.
    
*/

//================================================================================================================================
void bldc_freewheel(){                                        /// These functions feewheel and engage the motors, respectively.
  digitalWrite(bldc_enable, LOW); 
};

void bldc_engage(){
  digitalWrite(bldc_enable, HIGH); 
};

//================================================================================================================================
void bldc_brake(){                                            /// These functions brake and unbrake the motors. Don't call when 
                                                              // moving fast
  digitalWrite(bldc_0_brake, HIGH); 
  digitalWrite(bldc_1_brake, HIGH); 
};

void bldc_unbrake(){
  digitalWrite(bldc_0_brake, LOW); 
  digitalWrite(bldc_1_brake, LOW); 
};


//================================================================================================================================
float bldc_infer_speeds(){                                     /// This function finds motor speeed based on the tach ISR timers 
  bldc_0_speed = (1 / (float)bldc_0_pulselength);              // (below). It returns a non-normalized, unitless speed.  
  bldc_1_speed = (1 / (float)bldc_1_pulselength);                    
};

void bldc_0_tach_isr(){
  bldc_0_pulselength = millis() - bldc_0_tach_timer;
  bldc_0_tach_timer = millis();
};

void bldc_1_tach_isr(){
  bldc_1_pulselength = millis() - bldc_1_tach_timer;
  bldc_1_tach_timer = millis();
};

//=================================================================================================================================
void bldc_update_speed(byte wheel, int wheel_speed){             /// This function updates the selected motor's speed
 
  if(wheel_speed >= 0){
     if(wheel == 0) digitalWrite(bldc_0_direction, LOW); 
     if(wheel == 1) digitalWrite(bldc_1_direction, HIGH);
   }else{
     if(wheel == 0) digitalWrite(bldc_0_direction, HIGH); 
     if(wheel == 1) digitalWrite(bldc_1_direction, LOW);    
  };
 
  byte pot_value = map(wheel_speed, 0, 100, 20, 127); 
  
  digipot_update(wheel, pot_value);
};



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//======= MCP4231 SPECIFIC FUNCTIONS ==============================================================================================
void digipot_ready_spi(){                                     /// This function gets the spi library ready for the MCP4231
  if(spi_config_mode != 2){
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV128);
    spi_config_mode = 2;
  };
};

//=================================================================================================================================
void digipot_update(byte pot_register, byte pot_value){        /// This function writes a value to the digital potentiometer
  
  digipot_ready_spi();
  
  byte pot_address;
  
  if(pot_register == 0) pot_address = B00000000;
  else if(pot_register == 1) pot_address = B00010000;
  else return;
  
  digitalWrite(digipot_select_pin, LOW);
  SPI.transfer(pot_address);
  SPI.transfer(pot_value);
  digitalWrite(digipot_select_pin, HIGH);
  
};




void digipot_read(byte pot_register){        /// This function writes a value to the digital potentiometer
  
  digipot_ready_spi();
  
  byte pot_address;
  
  if(pot_register == 0) pot_address = B00001100;
  else if(pot_register == 1) pot_address = B00011100;
  else return;
  
  digitalWrite(digipot_select_pin, LOW);
  SPI.transfer(pot_address);
  Serial.println(SPI.transfer(0));
  digitalWrite(digipot_select_pin, HIGH);
  
};
