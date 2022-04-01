/* 

    This page contains functions relating to the IMU. It is not portable.
    
*/

//================================================================================================================================
void imu_measure(){                                                       //// This function polls the IMU and updates inertial 
  imu.getMotion6(&ax_raw, &ay_raw, &az_raw, &wx_raw, &wy_raw, &wz_raw);   //// meaurements. [TESTED: WORKS]
  
  ax_raw += ax_offset;
  ay_raw += ay_offset;
  az_raw += az_offset;
  
  wx_raw += wx_offset;
  wy_raw += wy_offset;
  wz_raw += wz_offset;

  ax = ((float)ax_raw / 16384) * 9.80655;          // Acceleration is in units of meters seconds^-2
  ay = ((float)ay_raw / 16384) * 9.80655;
  az = ((float)az_raw / 16384) * 9.80655;

  wx = ((float)wx_raw / 131) * (pi / 180);         // Rotation is in units of radians seconds^-1
  wy = ((float)wy_raw / 131) * (pi / 180);
  wz = ((float)wz_raw / 131) * (pi / 180);
};

//================================================================================================================================
void imu_calc_offsets(){                                       //// This function guesses the IMU offsets
                                                               //// [TESTED: WORKS]
  #ifdef verbose
     Serial.println("Calculating IMU offsets...");
  #endif

  long ax_offset_sigma, ay_offset_sigma, az_offset_sigma, wx_offset_sigma, wy_offset_sigma, wz_offset_sigma = 0;

  for(byte n = 0; n < 25; n++){
    imu.getMotion6(&ax_raw, &ay_raw, &az_raw, &wx_raw, &wy_raw, &wz_raw);
    
    delay(10);
    
    ax_offset_sigma += (0 - ax_raw);
    ay_offset_sigma += (0 - ay_raw);
    az_offset_sigma += (0 - (az_raw - 16384));
    
    wx_offset_sigma += (0 - wx_raw);
    wy_offset_sigma += (0 - wy_raw);
    wz_offset_sigma += (0 - wz_raw);
    
    delay(40);
  };
  
  ax_offset = ax_offset_sigma / 25;
  ay_offset = ay_offset_sigma / 25;
  az_offset = az_offset_sigma / 25;
  wx_offset = wx_offset_sigma / 25;
  wy_offset = wy_offset_sigma / 25;
  wz_offset = wz_offset_sigma / 25;
  
  #ifdef verbose
    Serial.print("ax: "); Serial.println(ax_offset);
    Serial.print("ay: "); Serial.println(ay_offset);
    Serial.print("az: "); Serial.println(az_offset);
    Serial.print("wx: "); Serial.println(wx_offset);
    Serial.print("wy: "); Serial.println(wy_offset);
    Serial.print("wz: "); Serial.println(wz_offset);
  #endif
  
  imu_store_offsets();
};

//================================================================================================================================
void imu_store_offsets(){                                       //// This function stores IMU offsets in EEPROM
                                                                //// [TESTED: WORKS]
  EEPROM.write(0x06, 1);                                        // Set flag
 
  EEPROM.write(0x07, ((ax_offset + 32768) & B11111111));        // BIT TWIDDLING YO
  EEPROM.write(0x08, ((ax_offset + 32768) >> 8)); 
  EEPROM.write(0x09, ((ay_offset + 32768) & B11111111)); 
  EEPROM.write(0x0A, ((ay_offset + 32768) >> 8)); 
  EEPROM.write(0x0B, ((az_offset + 32768) & B11111111)); 
  EEPROM.write(0x0C, ((az_offset + 32768) >> 8)); 
  
  EEPROM.write(0x0D, ((wx_offset + 32768) & B11111111)); 
  EEPROM.write(0x0E, ((wx_offset + 32768) >> 8)); 
  EEPROM.write(0x0F, ((wy_offset + 32768) & B11111111));
  EEPROM.write(0x10, ((wy_offset + 32768) >> 8)); 
  EEPROM.write(0x12, ((wz_offset + 32768) & B11111111));
  EEPROM.write(0x13, ((wz_offset + 32768) >> 8)); 
      
  #ifdef verbose
    Serial.println("IMU offsets stored!");
  #endif
    
};

//================================================================================================================================
void imu_retrieve_offsets(){                                        //// Gets IMU offsets from EEPROM, if they exist 
                                                               //// [TESTED: WORKS]
  #ifdef verbose
     Serial.println("Grabbing IMU offsets from EEPROM...");
  #endif

  if(EEPROM.read(0x06) != 0){
    ax_offset = EEPROM.read(0x07) + (EEPROM.read(0x08) << 8);                      
    ay_offset = EEPROM.read(0x09) + (EEPROM.read(0x0A) << 8); 
    az_offset = EEPROM.read(0x0B) + (EEPROM.read(0x0C) << 8); 
  
    wx_offset = EEPROM.read(0x0D) + (EEPROM.read(0x0E) << 8);
    wy_offset = EEPROM.read(0x0F) + (EEPROM.read(0x10) << 8);
    wz_offset = EEPROM.read(0x11) + (EEPROM.read(0x12) << 8); 
    
    // But, needs to be a signed number so....
    
    ax_offset = ax_offset - 32768;
    ay_offset = ay_offset - 32768;
    az_offset = az_offset - 32768;
    
    wx_offset = wx_offset - 32768;
    wy_offset = wy_offset - 32768;
    wz_offset = wz_offset - 32768;
    
    #ifdef verbose
      Serial.print("ax: "); Serial.println(ax_offset);
      Serial.print("ay: "); Serial.println(ay_offset);
      Serial.print("az: "); Serial.println(az_offset);
      Serial.print("wx: "); Serial.println(wx_offset);
      Serial.print("wy: "); Serial.println(wy_offset);
      Serial.print("wz: "); Serial.println(wz_offset);
    #endif

  } else {                                                     // If nothing is written, offsets are assumed zero.
    
    #ifdef verbose
       Serial.println("No offsets recorded!");
    #endif
    
    ax_offset, ay_offset, az_offset, 
    wx_offset, wy_offset, wz_offset = 0; 
  };
};

