/* 

    This page contains functions relating to system startup, configuration & operation. It is not portable.
    
*/

//================================================================================================================================
void system_set_pin_impedances(){                              //// This function sets pin impedances
  pinMode(digipot_select_pin,  OUTPUT);                        //// [TESTED: WORKS]
  pinMode(daq_select_pin,      OUTPUT);

  pinMode(buck_keepalive_pin,  OUTPUT);
  pinMode(charger_enable_pin,  OUTPUT);
  pinMode(charge_sense_pin,    INPUT);

  pinMode(led_pin,             OUTPUT);
  pinMode(buzzer_pin,          OUTPUT);

  pinMode(bldc_0_direction,    OUTPUT);
  pinMode(bldc_1_direction,    OUTPUT);

  pinMode(bldc_0_brake,        OUTPUT);
  pinMode(bldc_1_brake,        OUTPUT); 
  
  #ifdef verbose
    Serial.println("Pin impedances defined OK");
  #endif
};

//================================================================================================================================
void system_self_test(){                                      /// This function runs a self-test, gives alert and powers down 
                                                              // if it fails. If verbose, it does not power down.
  #ifdef verbose
    Serial.println("Running system self test...");
    Serial.println(daq_run_self_test() ? "DAQ: Passed" : "DAQ: FAILED");              // Verbose self-test
    Serial.println(imu.testConnection() ? "IMU I2C: Passed" : "IMU I2C: FAILED");
  #else
    byte passed = 0;                                                                  // Mute self-test
    passed += daq_run_self_test();
    passed += imu.testConnection();
 
    if(passed == 0) system_power_down(); 
  #endif
};


//================================================================================================================================
void system_power_down(){                                             //// This function shuts down the MCU, hopefully
  digitalWrite(buck_keepalive_pin, LOW); 
};


//================================================================================================================================
void play_melody(byte melody){                                         //// Play a melody
  switch(melody){
    case 1:
      tone(buzzer_pin, NOTE_G4, 90);         // Dee-doo
      delay(110);
      tone(buzzer_pin, NOTE_C4, 90);  
      break;
  
    case 2:
      tone(buzzer_pin, NOTE_C4, 90);          // Doo doo doo
      delay(110);
      tone(buzzer_pin, NOTE_C4, 90); 
      delay(110);
      tone(buzzer_pin, NOTE_C4, 90); 
      break;

    case 3:
      tone(buzzer_pin, NOTE_C4, 90);           // Doo-dee
      delay(110);
      tone(buzzer_pin, NOTE_G4, 90); 
      break;

    case 4:
      tone(buzzer_pin, NOTE_G4, 90);           // Dee-doo-dee
      delay(110);
      tone(buzzer_pin, NOTE_C4, 90);
      delay(110);
      tone(buzzer_pin, NOTE_G4, 90); 
      break;

  };
};
