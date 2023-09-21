/*
  Written by : Cory S Griffis
  (C) 09/20/2023
  Provided under GNU license. 
  No warranty provided. 
*/

bool saveAlertValues(SHT31_Alert_t *HighAlerts,SHT31_Alert_t *LowAlerts){

  uint32_t address;


floattobytearray.f = HighAlerts->SetTemp;
for(address = 0; address <sizeof(float);address++){
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(_MEM_HIGH_TEMP_SET+address,floattobytearray.bytes[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(_MEM_HIGH_TEMP_SET+address,floattobytearray.bytes[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(_MEM_HIGH_TEMP_SET+address,floattobytearray.bytes[address]);
    #endif 
  }

floattobytearray.f = HighAlerts->ClearTemp;
for(address = 0; address <sizeof(float);address++){
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(_MEM_HIGH_TEMP_CLEAR+address,floattobytearray.bytes[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(_MEM_HIGH_TEMP_CLEAR+address,floattobytearray.bytes[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(_MEM_HIGH_TEMP_CLEAR+address,floattobytearray.bytes[address]);
    #endif 
  }


floattobytearray.f = LowAlerts->SetTemp;
for(address = 0; address <sizeof(float);address++){
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(_MEM_LOW_TEMP_SET+address,floattobytearray.bytes[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(_MEM_LOW_TEMP_SET+address,floattobytearray.bytes[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(_MEM_LOW_TEMP_SET+address,floattobytearray.bytes[address]);
    #endif 
  }

floattobytearray.f = LowAlerts->ClearTemp;
for(address = 0; address <sizeof(float);address++){
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(_MEM_LOW_TEMP_CLEAR+address,floattobytearray.bytes[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(_MEM_LOW_TEMP_CLEAR+address,floattobytearray.bytes[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(_MEM_LOW_TEMP_CLEAR+address,floattobytearray.bytes[address]);
    #endif 
  }

floattobytearray.f = HighAlerts->SetHumidity;
for(address = 0; address <sizeof(float);address++){
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(_MEM_HIGH_HUMIDITY_SET+address,floattobytearray.bytes[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(_MEM_HIGH_HUMIDITY_SET+address,floattobytearray.bytes[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(_MEM_HIGH_HUMIDITY_SET+address,floattobytearray.bytes[address]);
    #endif 
  }

floattobytearray.f = HighAlerts->ClearHumidity;
for(address = 0; address <sizeof(float);address++){
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(_MEM_HIGH_HUMIDITY_CLEAR+address,floattobytearray.bytes[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(_MEM_HIGH_HUMIDITY_CLEAR+address,floattobytearray.bytes[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(_MEM_HIGH_HUMIDITY_CLEAR+address,floattobytearray.bytes[address]);
    #endif 
  }

floattobytearray.f = LowAlerts->SetHumidity;
for(address = 0; address <sizeof(float);address++){
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(_MEM_LOW_HUMIDITY_SET+address,floattobytearray.bytes[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(_MEM_LOW_HUMIDITY_SET+address,floattobytearray.bytes[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(_MEM_LOW_HUMIDITY_SET+address,floattobytearray.bytes[address]);
    #endif 
  }

floattobytearray.f = LowAlerts->ClearHumidity;
for(address = 0; address <sizeof(float);address++){
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(_MEM_LOW_HUMIDITY_CLEAR+address,floattobytearray.bytes[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(_MEM_LOW_HUMIDITY_CLEAR+address,floattobytearray.bytes[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(_MEM_LOW_HUMIDITY_CLEAR+address,floattobytearray.bytes[address]);
    #endif 
  }
  return true;
}

