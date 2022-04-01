//==================================================================================================================================================
void freewheel_motors(){
};


//==================================================================================================================================================
void brake_motors(){
};


//==================================================================================================================================================
int infer_motor_speed(boolean motor_index){
  //pulseIn(9,9);
};



//==================================================================================================================================================
void spi_ready_digipot(){                                              /// This function gets the spi library ready for the MCP4231
  if(spi_config_mode != 2){
    SPI.setDataMode(SPI_MODE3);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV32);
    spi_config_mode = 2;
  };
};


//==================================================================================================================================================
void update_digipot(byte pot_register, byte pot_value){
  
  byte pot_address;
  
  if(pot_register == 0) pot_address = B00000000;
  else if(pot_register == 1) pot_address = B00010000;
  else return;
  
  digitalWrite(digipot_select_pin, LOW);
  SPI.transfer(pot_address);
  SPI.transfer(pot_value);
  digitalWrite(digipot_select_pin, HIGH);
  
};
