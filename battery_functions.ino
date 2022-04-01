/* 

    This page contains functions relating to the battery and/or battery management system. It is not portable.
    
*/

//================================================================================================================================
void battery_charge(){                                  //// This routine charges the battery. The segbot does not run while this
                                                        //// happens, but in the future this could change
  bldc_brake();

  daq_begin_conversion();                               // Begin a DAQ conversion & Wait for it to finish
  delay(15);
  
  daq_read_voltages();                                  // Read the cell voltages & Find statistics, battery fuel
  calc_cell_statistics();   
  calc_battery_fuel();
   
  if( digitalRead(charge_sense_pin) == LOW              // No sense charging if there is no charger
      || battery_fuel > 95){                            // Or if there's a full battery
    
    stop_balancing_cells();
    digitalWrite(charger_enable_pin, LOW);
   
    play_melody(2);                                     // Doo-doo-doo
 
    return;
  };
  
  digitalWrite(charger_enable_pin, HIGH);               // Well, if it's good to go, connect the charger yo
  
  play_melody(3);                                       // Doo-dee
  
  long timer = millis();
  
  while(true){
    
    if(timer + 1500 < millis()){                        // Run this approx once per 1.5s
      
      timer = millis();
      
      daq_begin_conversion();                           // Begin a DAQ conversion & Wait for it to finish
      delay(15);
      
      daq_read_voltages();                              // Read the cell voltages & Find statistics, battery fuel
      calc_cell_statistics();   
      calc_battery_fuel();                                         
      
      if(battery_fuel > 98 || digitalRead(charge_sense_pin) == LOW){   // If charged, or charger unplugged... well we're done.
        
        play_melody(1);                                 // Dee-doo
        stop_balancing_cells();
        digitalWrite(charger_enable_pin, LOW);
        digitalWrite(led_pin, LOW);
        break;
      }; 
      
      balance_cells(0.075);                             // Otherwise, charge and balance cells, max voltage imbalance, 75mV
      
      led_state = !led_state;                           // Blink LED, also
      digitalWrite(led_pin, led_state);
    };
    
    /* Battery meter code or something can go here */
    
  };
};

//=================================================================================================================================
void balance_cells(float hysteresis){                    //// This function checks if cells need balancing, and shunts accordingly

  if (battery_fuel < 75){                                // Cells should not be balanced if battery is under 75%
     stop_balancing_cells();
     return;
  };
  
  daq_read_voltages();
  
  for(int n = 0; n < 12; n++){                           // Check if the cell is too far astray, if it is, shunt it
    if(cell_voltage[n] > (mean_cell_voltage + hysteresis) || cell_voltage[n] > over_voltage_setpoint){
      cell_bal[n] = true;
    }else{
      cell_bal[n] = false;
    };
  };
  
  daq_update_balance_fets();
};

//=================================================================================================================================
void stop_balancing_cells(){                            //// This function stops cell balancing
 for(int n = 0; n < 12; n++){                                   
   cell_bal[n] = 0;
  };
  daq_update_balance_fets();                            // Update the balance fets, twice for good measure
  daq_update_balance_fets();
};

//=====================================================================================================================-===========
void calc_cell_statistics(){                           //// This function finds the cell voltage statistics
 
  for(int n = 0; n < 12; n++){                         // Calculate the mean cell voltage
    mean_cell_voltage += cell_voltage[n];
  };
  
  mean_cell_voltage = mean_cell_voltage / (float)10;
  
  max_cell_voltage = 0;                                // Calculate the max cell voltage
  for(int n = 0; n < 12; n++){
    if(max_cell_voltage < cell_voltage[n]) max_cell_voltage = cell_voltage[n];
  };
  
  min_cell_voltage = 10000;                            // Calculate the min cell voltage
  for(int n = 0; n < 12; n++){
    if(min_cell_voltage > cell_voltage[n]) min_cell_voltage = cell_voltage[n];
  };
};

//================================================================================================================================
void calc_battery_fuel(){                              //// This function calculates the battery fuel, assumes cell_voltage[] 
                                                       //// data is recent and the calc_cell_statistiscs has also been called.                                                                
  fuel_repeat:                                                                 
      
  float weighted_cell_voltage =                            // So, when the battery is near full, we use the mean_cell_voltage to 
        mean_cell_voltage * ((float)battery_fuel / 100)    // calculate fuel. When the battery is near empty, we use the 
        + min_cell_voltage *                               // min_cell_voltage to do this so we don't accidentally discharge a 
        (1 - (float)battery_fuel / 100);                   // cell too much.
  
  float bf_float =                                         // This normalizes the battery fuel from 0 to 100
        (weighted_cell_voltage - empty_voltage_setpoint) * 
        (100.00 - full_voltage_setpoint) / 
        (full_voltage_setpoint - empty_voltage_setpoint);
        
  battery_fuel = bf_float;                                 // Cast it to a byte, since this doesn't need to be precise
  
  if(battery_fuel_first_call == true){                     // This function relies on n-1, so run it twice the first time 
    battery_fuel_first_call == false;                      // it's called. Yes I used goto, it's called only once, deal with it 
    goto fuel_repeat;                                                    
  };                                              
};



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//======= LTC6802-2 DAQ-SPECIFIC FUNCTIIONS =======================================================================================

void daq_ready_spi(){                                   //// This function gets the spi library ready for the LTC6802-2
  if(spi_config_mode != 1){
    SPI.setDataMode(SPI_MODE3);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV128);
    spi_config_mode = 1;
  };
};

//=================================================================================================================================
void daq_write_configs(){                               //// This function writes the DAQ config registers

  daq_ready_spi();                                      // Config SPI for BMS IC
  digitalWrite(daq_select_pin, LOW);                    // Begin DAQ communication
  SPI.transfer(0x01);                                   // WRCFG Configuration command
  SPI.transfer(CFGR0);
  SPI.transfer(CFGR1);
  SPI.transfer(CFGR2);
  SPI.transfer(CFGR3); 
  SPI.transfer(CFGR4); 
  SPI.transfer(CFGR5);
  digitalWrite(daq_select_pin, HIGH);                   // End DAQ communication
};


//==================================================================================================================================================
void daq_update_balance_fets(){                         //// This function updates the FET drivers based on the status of cel_bal[]
  
  // DCC8  | DCC7  | DCC6  | DCC5 | DCC4  | DCC3  | DCC2  | DCC1     
  CFGR1 = (                                             // Figure out what number to write to the registers... Binary flag time.
    128 * cell_bal[7] + 
    64  * cell_bal[6] + 
    32  * cell_bal[5] + 
    16  * cell_bal[4] +
    8   * cell_bal[3] + 
    4   * cell_bal[2] + 
    2   * cell_bal[1] + 
    1   * cell_bal[0]
  );
  
  // MC4L  | MC3L  | MC2L  | MC1L | DCC12 | DCC11 | DCC10 | DCC9
  CFGR2 = (
    8 * cell_bal[11] + 
    4 * cell_bal[10] + 
    2 * cell_bal[9] + 
    1 * cell_bal[8]
  );
  
  daq_write_configs();                                  // Then write it
};

//=================================================================================================================================
void daq_begin_conversion(){                            //// This function begins A/Dc, but doesn't wait for it to finish (13ms)

  daq_ready_spi();  // Config SPI for BMS IC
  
  daq_led(1);
  
  digitalWrite(daq_select_pin, LOW);                    // Begin DAQ communication
  SPI.transfer(0x10);                                   // STCVAD "Start AD conversion for all cells"
  digitalWrite(daq_select_pin, HIGH);                   // End DAQ communication & Exit polling mode
  
  #ifdef verbose
    Serial.println("\nVoltage conversion begun.");
  #endif
};

//=================================================================================================================================
void daq_begin_temp_conversion(){                       //// Function begins thermal A/Dc, doesn't wait for it to finish (5ms)

  daq_ready_spi();                                      // Config SPI for BMS IC
  digitalWrite(daq_select_pin, LOW);                    // Begin DAQ communication
  SPI.transfer(0x30);                                   // STTMPAD "All temperature conversions"
  digitalWrite(daq_select_pin, HIGH);                   // End DAQ communication & Exit polling mode

  #ifdef verbose
    Serial.println("\nTemperature conversion begun.");
  #endif
};

//=================================================================================================================================
void daq_read_voltages(){                              //// This function reads the DAQ and fills cell_voltage[] with voltages
                                                       //// It DOES NOT begin an ADC conversion
  #ifdef verbose
    if(!mute) Serial.print("\nReading voltage registers...");
  #endif                                                     
                                                       
  daq_ready_spi();                                     // Config SPI for BMS IC
  digitalWrite(daq_select_pin, LOW);                   // Begin DAQ communication
  SPI.transfer(B10001111);                             // Address command byte (BMS ic is addressed at 0)
  SPI.transfer(0x04);                                  // RDCV "Read cell register group" command
  for(int n = 0; n < 18; n++){                         // Store cell voltage data from registers
    CVR[n] = SPI.transfer(0);
  };
  byte PEC = SPI.transfer(0);                          // Get CRC byte, do nothing with it
    
  digitalWrite(daq_select_pin, HIGH);                  // End DAQ communication
  
  #ifdef verbose                                       // Debugging stuffs
    if(!mute) Serial.println(" Done!\nVoltage data dump:"); 
  #endif
  
  
  int index_1 = 0;                                     // Arrays increase to different degrees, so can't use loop's index
  int index_2 = 0;                                                  

  for(int n = 0; n < 6; n++){  
    
    int cvr_bitstash = CVR[index_2+1];                 // Get CVR upper bits of cell measurement I 
    
    cvr_bitstash = cvr_bitstash & B00001111;           // Bitmask AND 0000-0000-0000-1111, to ignore cell measurement II
    cvr_bitstash = cvr_bitstash << 8;                  // Shift 8 bits higher
            
    cell_voltage_int[index_1] = CVR[index_2] + cvr_bitstash;   // Add lower bits to upper bits, cell_voltage_int[0] is now filled
    
    cvr_bitstash = CVR[index_2+1];                     // Get CVR lower bits of cell measurement II
    cvr_bitstash = cvr_bitstash & B11110000;           // Bitmask AND 0000-0000-1111-0000, to ignore cell measurement I
    cvr_bitstash = cvr_bitstash >> 4;                  // Shift them bits to the low side
    
    int cvr_bitstash2 = CVR[index_2+2];                // Get upper 8 bits of cell measurement II
    cvr_bitstash2 = cvr_bitstash2 << 4;                // Shift them bits to the high side
    
    cell_voltage_int[index_1+1] = cvr_bitstash + cvr_bitstash2;    // Combine them bits into a 12 bit int, cell_voltage_int[1]
     
    
    cell_voltage[index_1]     = (float)0.0015 * cell_voltage_int[index_1];  // Now, convert cell_voltage_int to something with units
    cell_voltage[index_1 + 1] = (float)0.0015 * cell_voltage_int[index_1+1];  // Voltage = 1.5mV * integer value

     
    #ifdef verbose                                     // Debugging stuffs
      if(!mute){
        Serial.print("  Cell_"); Serial.print(index_1  ); Serial.print(": "); Serial.print(cell_voltage[index_1]  ,3);Serial.print("V   raw: "); Serial.println(cell_voltage_int[index_1  ]);
        Serial.print("  Cell_"); Serial.print(index_1+1); Serial.print(": "); Serial.print(cell_voltage[index_1+1],3);Serial.print("V   raw: "); Serial.println(cell_voltage_int[index_1+1]);
      };
    #endif
    
    index_1 = index_1 + 2;                             // Increment cell_voltage_int[] index by 2
    index_2 = index_2 + 3;                             // Increment CVR[] index by 3
  
  };
  
  daq_led(0);

};


//=================================================================================================================================
void daq_read_temperatures(){                          //// This function reads the DAQ and fills temp_int[] with data
                                                       /// It DOES NOT begin an ADC conversion
  #ifdef verbose                                       // Debugging stuffs
    if(!mute) Serial.print("\nReading temperature registers... "); 
  #endif
  
  daq_ready_spi();                                     // Config SPI for BMS IC
  digitalWrite(daq_select_pin, LOW);                   // Begin DAQ communication
  SPI.transfer(B10001111);                             // Address command byte (BMS ic is addressed at 16)
  SPI.transfer(0x08);                                  // RDTMP "Read temperature group" command
  for(int n = 0; n < 5; n++){                          // Store temp data from registers
    TMP[n] = SPI.transfer(0);
  };
  byte PEC = SPI.transfer(0);                          // Get CRC byte, do nothing with it
  digitalWrite(daq_select_pin, HIGH);                  // End DAQ communication
  
                                          
  int tmp_bitstash = TMP[1];                           // Get CVR upper bits of NTC1
  tmp_bitstash = tmp_bitstash & 0x0F;                  // Bitmask AND 0000-0000-0000-1111, to ignore NTC2
  tmp_bitstash = tmp_bitstash << 8;                    // Shift 8 bits higher
        
  temp_int[0] = TMP[0] + tmp_bitstash;                 // Add lower bits to upper bits, temp_int[0] is now filled
    
  tmp_bitstash = TMP[1];                               // Get TMP lower bits of NTC2
  tmp_bitstash = tmp_bitstash & 0xF0;                  // Bitmask AND 0000-0000-1111-0000, to ignore NTC1
  tmp_bitstash = tmp_bitstash >> 4;                    // Shift them bits to the low side
    
  int tmp_bitstash2 = TMP[2];                          // Get upper 8 bits of NTC2
  tmp_bitstash2 = tmp_bitstash2 << 4;                  // Shift them bits to the high side
    
  temp_int[1] = tmp_bitstash + tmp_bitstash2;          // Combine them bits into a 12 bit int, temp_int[1] is now filled
  
  tmp_bitstash = TMP[4];                               // Get TMP upper bits of DIE TEMP
  tmp_bitstash = tmp_bitstash & 0x0F;                  // Bitmask AND 0000-0000-0000-1111, to ignore NTC2
  tmp_bitstash = tmp_bitstash << 8;                    // Shift 8 bits higher
        
  temp_int[2] = TMP[3] + tmp_bitstash;                 // Add lower bits to upper bits, temp_int[2] is now filled
    
  temp[0] = (float)temp_int[0] * 0.0015;     // Units of V
  temp[1] = (float)temp_int[1] * 0.0015;     // Below, units of C. This took 3 hours because of limited precision, Good lord.
  
  temp[0] = ((298.15 * 4250) / (4250 + 298.15 * (log((temp[0] * 100000)/(3.068 - temp[0])) - log(100000)))) - 273.15;
  temp[1] = ((298.15 * 4250) / (4250 + 298.15 * (log((temp[1] * 100000)/(3.068 - temp[1])) - log(100000)))) - 273.15;

  temp[2] = (((float)temp_int[2] * 0.0015) / 0.008) - 273.15;    // Units of C
  
  #ifdef verbose                                                 // Debugging stuffs
    if(!mute){
      Serial.println("Done!\nTemperature data dump:"); 
      Serial.print("  NTC1: "); Serial.print(temp[0]); Serial.print("'C   raw: "); Serial.println(temp_int[0]);
      Serial.print("  NTC2: "); Serial.print(temp[1]); Serial.print("'C   raw: "); Serial.println(temp_int[1]);
      Serial.print("  DIE:  "); Serial.print(temp[2]); Serial.print("'C   raw: "); Serial.println(temp_int[2]);
    };
  #endif
  
};
  

//=================================================================================================================================
boolean daq_run_self_test(){                               //// This function runs a DAQ self-test, takes about 50ms & returns true
                                                           ////  or false
  boolean passed_adc = true;
  boolean passed_thermal = true;
  
  #ifdef verbose
    mute = 1;
    Serial.print("\nDAQ A/Dc self-test... ");
  #endif 
                                                           
  daq_ready_spi();                                         // Config SPI for BMS IC
  digitalWrite(daq_select_pin, LOW);                       // Begin DAQ communication for self test 1
  SPI.transfer(0x1E);                                      // STCVAD "Cell self test 1; all CV=0x555"
  digitalWrite(daq_select_pin, HIGH);                      // End DAQ communication & Exit polling mode
  
  delay(20);                                               // Wait for A/Dc to finish
  
  daq_read_voltages();                                     // Get cell voltages
  
  boolean passed = true;
  
  for(int n = 0; n < 12; n++){                             // See if it failed self-test #1
    if(cell_voltage_int[n] != 0x555) passed_adc = false;              
  };
  
  digitalWrite(daq_select_pin, LOW);                       // Begin DAQ communication for self test 2
  SPI.transfer(0x1F);                                      // STCVAD "Cell self test 2; all CV=0xAAA"
  digitalWrite(daq_select_pin, HIGH);                      // End DAQ communication & Exit polling mode
  
  delay(20);                                               // Wait for A/Dc to finish
  
  daq_read_voltages();                                     // Get cell voltages
   
  for(int n = 0; n < 12; n++){                             // See if it failed self-test #2
    if(cell_voltage_int[n] != 0xAAA) passed_adc = false;            
  };
  
  #ifdef verbose
    if(passed_adc == false) Serial.println("FAILED!");
    else Serial.println("Passed");

    Serial.print("DAQ thermal self-test... ");
  #endif 
                                                           
  daq_ready_spi();                                         // Config SPI for BMS IC
  digitalWrite(daq_select_pin, LOW);                       // Begin DAQ communication for self test 1
  SPI.transfer(0x3E);                                      // STCVAD "Cell self test 1; all CV=0x555"
  digitalWrite(daq_select_pin, HIGH);                      // End DAQ communication & Exit polling mode
  
  delay(20);                                               // Wait for A/Dc to finish
  
  daq_read_temperatures();                                     // Get cell voltages
    
  for(int n = 0; n < 3; n++){                             // See if it failed self-test #1
    if(temp_int[n] != 0x555) passed_thermal = false;              
  };
  
  digitalWrite(daq_select_pin, LOW);                       // Begin DAQ communication for self test 2
  SPI.transfer(0x3F);                                      // STCVAD "Cell self test 2; all CV=0xAAA"
  digitalWrite(daq_select_pin, HIGH);                      // End DAQ communication & Exit polling mode
  
  delay(20);                                               // Wait for A/Dc to finish
  
  daq_read_temperatures();                                     // Get cell voltages
   
  for(int n = 0; n < 3; n++){                             // See if it failed self-test #2
    if(temp_int[n] != 0xAAA) passed_thermal = false;            
  };
  
  #ifdef verbose
    if(passed == false) Serial.println("FAILED!");
    else Serial.println("Passed");
    mute = 0;
  #endif
  
  
       
  return passed;
};


//=================================================================================================================================
void daq_led(boolean ledstate){                                      // This function sets the DAQ LED
  switch(ledstate){
    case 0: CFGR0 = B00110001; break;
    case 1: CFGR0 = B01110001; break;
  };
  
  daq_write_configs();
};
