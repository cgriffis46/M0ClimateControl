/*
  Written by : Cory S Griffis
  (C) 09/20/2023
  Provided under GNU license. 
  No warranty provided. 
*/

bool loadAlertValues(SHT31_Alert_t *HighAlerts,SHT31_Alert_t *LowAlerts){
  
    char ok[2 + 1];
  uint32_t address;
//  uint8_t buffer1[256];
  //uint8_t passwordbuffer[sizeof(password)];
for(address = 0; address <sizeof(float);address++){
      #ifdef USE_SPI_FRAM
        floattobytearray.bytes[address] = fram.read8(_MEM_HIGH_TEMP_SET+address);
      #endif
      #ifdef USE_I2C_FRAM
        floattobytearray.bytes[address] = fram.read(_MEM_HIGH_TEMP_SET+address);
      #endif
      #ifdef USE_I2C_EEPROM
        floattobytearray.bytes[address] = fram.read(_MEM_HIGH_TEMP_SET+address);
      #endif
      //floattobytearray.bytes[address] = char(buffer1[address]);
    //  Serial.println("could not read fram");
  }
HighAlerts->SetTemp = floattobytearray.f;

for(address = 0; address <sizeof(float);address++){
      #ifdef USE_SPI_FRAM
        floattobytearray.bytes[address] = fram.read8(_MEM_HIGH_TEMP_CLEAR+address);
      #endif
      #ifdef USE_I2C_FRAM
        floattobytearray.bytes[address] = fram.read(_MEM_HIGH_TEMP_CLEAR+address);
      #endif
      #ifdef USE_I2C_EEPROM
        floattobytearray.bytes[address] = fram.read(_MEM_HIGH_TEMP_CLEAR+address);
      #endif
      //floattobytearray.bytes[address] = char(buffer1[address]);
    //  Serial.println("could not read fram");
  }
HighAlerts->ClearTemp = floattobytearray.f;

for(address = 0; address <sizeof(float);address++){
      #ifdef USE_SPI_FRAM
        floattobytearray.bytes[address] = fram.read8(_MEM_LOW_TEMP_SET+address);
      #endif
      #ifdef USE_I2C_FRAM
        floattobytearray.bytes[address] = fram.read(_MEM_LOW_TEMP_SET+address);
      #endif
      #ifdef USE_I2C_EEPROM
        floattobytearray.bytes[address] = fram.read(_MEM_LOW_TEMP_SET+address);
      #endif
      //floattobytearray.bytes[address] = char(buffer1[address]);
    //  Serial.println("could not read fram");
  }
LowAlerts->SetTemp = floattobytearray.f;

for(address = 0; address <sizeof(float);address++){
      #ifdef USE_SPI_FRAM
        floattobytearray.bytes[address] = fram.read8(_MEM_LOW_TEMP_CLEAR+address);
      #endif
      #ifdef USE_I2C_FRAM
        floattobytearray.bytes[address] = fram.read(_MEM_LOW_TEMP_CLEAR+address);
      #endif
      #ifdef USE_I2C_EEPROM
        floattobytearray.bytes[address] = fram.read(_MEM_LOW_TEMP_CLEAR+address);
      #endif
      //floattobytearray.bytes[address] = char(buffer1[address]);
    //  Serial.println("could not read fram");
  }
LowAlerts->ClearTemp = floattobytearray.f;

for(address = 0; address <sizeof(float);address++){
      #ifdef USE_SPI_FRAM
        floattobytearray.bytes[address] = fram.read8(_MEM_HIGH_HUMIDITY_SET+address);
      #endif
      #ifdef USE_I2C_FRAM
        floattobytearray.bytes[address] = fram.read(_MEM_HIGH_HUMIDITY_SET+address);
      #endif
      #ifdef USE_I2C_EEPROM
        floattobytearray.bytes[address] = fram.read(_MEM_HIGH_HUMIDITY_SET+address);
      #endif
      //floattobytearray.bytes[address] = char(buffer1[address]);
    //  Serial.println("could not read fram");
  }
HighAlerts->SetHumidity = floattobytearray.f;

for(address = 0; address <sizeof(float);address++){
      #ifdef USE_SPI_FRAM
        floattobytearray.bytes[address] = fram.read8(_MEM_HIGH_HUMIDITY_CLEAR+address);
      #endif
      #ifdef USE_I2C_FRAM
        floattobytearray.bytes[address] = fram.read(_MEM_HIGH_HUMIDITY_CLEAR+address);
      #endif
      #ifdef USE_I2C_EEPROM
        floattobytearray.bytes[address] = fram.read(_MEM_HIGH_HUMIDITY_CLEAR+address);
      #endif
      //floattobytearray.bytes[address] = char(buffer1[address]);
    //  Serial.println("could not read fram");
  }
HighAlerts->ClearHumidity = floattobytearray.f;

for(address = 0; address <sizeof(float);address++){
      #ifdef USE_SPI_FRAM
        floattobytearray.bytes[address] = fram.read8(_MEM_LOW_HUMIDITY_SET+address);
      #endif
      #ifdef USE_I2C_FRAM
        floattobytearray.bytes[address] = fram.read(_MEM_LOW_HUMIDITY_SET+address);
      #endif
      #ifdef USE_I2C_EEPROM
        floattobytearray.bytes[address] = fram.read(_MEM_LOW_HUMIDITY_SET+address);
      #endif
      //floattobytearray.bytes[address] = char(buffer1[address]);
    //  Serial.println("could not read fram");
  }
  LowAlerts->SetHumidity = floattobytearray.f;

for(address = 0; address <sizeof(float);address++){
      #ifdef USE_SPI_FRAM
        floattobytearray.bytes[address] = fram.read8(_MEM_LOW_HUMIDITY_CLEAR+address);
      #endif
      #ifdef USE_I2C_FRAM
        floattobytearray.bytes[address] = fram.read(_MEM_LOW_HUMIDITY_CLEAR+address);
      #endif
      #ifdef USE_I2C_EEPROM
        floattobytearray.bytes[address] = fram.read(_MEM_LOW_HUMIDITY_CLEAR+address);
      #endif
      //floattobytearray.bytes[address] = char(buffer1[address]);
    //  Serial.println("could not read fram");
  }
  LowAlerts->ClearHumidity = floattobytearray.f;

  return true;
}