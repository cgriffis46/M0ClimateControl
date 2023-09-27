/*!
 *  @file Adafruit_SHT31.cpp
 *
 *  @mainpage Adafruit SHT31 Digital Humidity & Temp Sensor
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the SHT31 Digital Humidity & Temp Sensor
 *
 *  Designed specifically to work with the SHT31 Digital sensor from Adafruit
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2857
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include "Adafruit_SHT31.h"

/*!
 * @brief  SHT31 constructor using i2c
 * @param  *theWire
 *         optional wire
 */
Adafruit_SHT31::Adafruit_SHT31(TwoWire *theWire) {
  _wire = theWire;
  humidity = NAN;
  temp = NAN;
  TempHighAlert = false; TempLowAlert = false; HumidityHighAlert = false; HumidityLowAlert = false;
}

/**
 * Destructor to free memory in use.
 */
Adafruit_SHT31::~Adafruit_SHT31() {
  if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }
}

/**
 * Initialises the I2C bus, and assigns the I2C address to us.
 *
 * @param i2caddr   The I2C address to use for the sensor.
 *
 * @return True if initialisation was successful, otherwise False.
 */
bool Adafruit_SHT31::begin(uint8_t i2caddr) {
  if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }

  i2c_dev = new Adafruit_I2CDevice(i2caddr, _wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  reset();
  return readStatus() != 0xFFFF;
}

/**
 * Gets the current status register contents.
 *
 * @return The 16-bit status register.
 */
uint16_t Adafruit_SHT31::readStatus(void) {
  writeCommand(SHT31_READSTATUS);

  uint8_t data[3];
  i2c_dev->read(data, 3);

  uint16_t stat = data[0];
  stat <<= 8;
  stat |= data[1];
  // Serial.println(stat, HEX);
  return stat;
}

/**
 * Performs a reset of the sensor to put it into a known state.
 */
void Adafruit_SHT31::reset(void) {
  writeCommand(SHT31_SOFTRESET);
  delay(10);
}

/**
 * Enables or disabled the heating element.
 *
 * @param h True to enable the heater, False to disable it.
 */
void Adafruit_SHT31::heater(bool h) {
  if (h)
    writeCommand(SHT31_HEATEREN);
  else
    writeCommand(SHT31_HEATERDIS);
  delay(1);
}

/*!
 *  @brief  Return sensor heater state
 *  @return heater state (TRUE = enabled, FALSE = disabled)
 */
bool Adafruit_SHT31::isHeaterEnabled() {
  uint16_t regValue = readStatus();
  return (bool)bitRead(regValue, SHT31_REG_HEATER_BIT);
}

/**
 * Gets a single temperature reading from the sensor.
 *
 * @return A float value indicating the temperature.
 */
float Adafruit_SHT31::readTemperature(void) {
  if (!readTempHum())
    return NAN;

  return temp;
}

/**
 * Gets a single relative humidity reading from the sensor.
 *
 * @return A float value representing relative humidity.
 */
float Adafruit_SHT31::readHumidity(void) {
  if (!readTempHum())
    return NAN;

  return humidity;
}

/**
 * Gets a reading of both temperature and relative humidity from the sensor.
 *
 * @param temperature_out  Where to write the temperature float.
 * @param humidity_out     Where to write the relative humidity float.
 * @return True if the read was successful, false otherwise
 */
bool Adafruit_SHT31::readBoth(float *temperature_out, float *humidity_out) {
  if (!readTempHum()) {
    *temperature_out = *humidity_out = NAN;
    return false;
  }

  *temperature_out = temp;
  *humidity_out = humidity;
  return true;
}

/**
 * Performs a CRC8 calculation on the supplied values.
 *
 * @param data  Pointer to the data to use when calculating the CRC8.
 * @param len   The number of bytes in 'data'.
 *
 * @return The computed CRC8 value.
 */
static uint8_t crc8(const uint8_t *data, int len) {
  /*
   *
   * CRC-8 formula from page 14 of SHT spec pdf
   *
   * Test data 0xBE, 0xEF should yield 0x92
   *
   * Initialization data 0xFF
   * Polynomial 0x31 (x8 + x5 +x4 +1)
   * Final XOR 0x00
   */

  const uint8_t POLYNOMIAL(0x31);
  uint8_t crc(0xFF);

  for (int j = len; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}

/**
 * Internal function to perform a temp + humidity read.
 *
 * @return True if successful, otherwise false.
 */
bool Adafruit_SHT31::readTempHum(void) {
  uint8_t readbuffer[6];

  if (!writeCommand(SHT31_MEAS_HIGHREP))
    return false;

  delay(20);

  if (!i2c_dev->read(readbuffer, sizeof(readbuffer)))
    return false;

  if (readbuffer[2] != crc8(readbuffer, 2) ||
      readbuffer[5] != crc8(readbuffer + 3, 2))
    return false;

  int32_t stemp = (int32_t)(((uint32_t)readbuffer[0] << 8) | readbuffer[1]);
  // simplified (65536 instead of 65535) integer version of:
  // temp = (stemp * 175.0f) / 65535.0f - 45.0f;
  stemp = ((4375 * stemp) >> 14) - 4500;
  temp = (float)stemp / 100.0f;

  uint32_t shum = ((uint32_t)readbuffer[3] << 8) | readbuffer[4];
  // simplified (65536 instead of 65535) integer version of:
  // humidity = (shum * 100.0f) / 65535.0f;
  shum = (625 * shum) >> 12;
  humidity = (float)shum / 100.0f;

  return true;
}

/**
 * Internal function to perform and I2C write.
 *
 * @param cmd   The 16-bit command ID to send.
 */
bool Adafruit_SHT31::writeCommand(uint16_t command) {
  uint8_t cmd[2];

  cmd[0] = command >> 8;
  cmd[1] = command & 0xFF;

  return i2c_dev->write(cmd, 2);
}

bool Adafruit_SHT31::FetchData(float *t, float *h){
  uint8_t cmd[2];
  uint8_t readbuffer[6];
 
  cmd[0] = SHT31_FETCH_DATA_MSB;
  cmd[1] = SHT31_FETCH_DATA_LSB; 
  
  // Uncomment for RP2040 
  //i2c_write_blocking(&this->i2c, SHT31_DEFAULT_ADDR, cmd, 2, false);
  //i2c_read_blocking(&this->i2c,SHT31_DEFAULT_ADDR,readbuffer, 6,false);

  writeCommand(SHT31_FETCH_DATA);

    if (!i2c_dev->read(readbuffer, sizeof(readbuffer)))
      return false;

     if (readbuffer[2] != crc8(readbuffer, 2) ||
      readbuffer[5] != crc8(readbuffer + 3, 2))
    return false;

  int32_t stemp = (int32_t)(((uint32_t)readbuffer[0] << 8) + readbuffer[1]);
  // simplified (65536 instead of 65535) integer version of:
  // temp = (stemp * 175.0f) / 65535.0f - 45.0f;
  stemp = ((4375 * stemp) >> 14) - 4500;
  this->temp = (float)stemp / 100.0f;
  *t = temp;
  uint32_t shum = ((uint32_t)readbuffer[3] << 8) + readbuffer[4];
  // simplified (65536 instead of 65535) integer version of:
  // humidity = (shum * 100.0f) / 65535.0f;
  shum = (625 * shum) >> 12;
  this->humidity = (float)shum / 100.0f;
  *h = this->humidity;

  if(this->humidity>=this->HighAlert.SetHumidity){this->HumidityHighAlert = true;}
  if(this->humidity<=this->HighAlert.ClearHumidity){this->HumidityHighAlert = false;}
  if(this->temp>=this->HighAlert.SetTemp){this->TempHighAlert = true;}
  if(this->temp<=this->HighAlert.ClearTemp){this->TempHighAlert = false;}

  if(this->humidity<=this->LowAlert.SetHumidity){this->HumidityLowAlert = true;}
  if(this->humidity>=this->LowAlert.ClearHumidity){this->HumidityLowAlert = false;}
  if(this->temp<=this->LowAlert.SetTemp){this->TempLowAlert = true;}
  if(this->temp>=this->LowAlert.ClearTemp){this->TempLowAlert = false;}

  return true;
}

void Adafruit_SHT31::setHighAlert(const SHT31_Alert_t* alert){
  uint16_t TempSet,TempClear, HumiditySet, HumidityClear;
  uint16_t rhtSet,rhtClear;
  uint8_t rhtSetMSB,rhtSetLSB,rhtClearMSB,rhtClearLSB;

  uint8_t cmd[5];

  memcpy(&this->HighAlert,alert,sizeof(SHT31_Alert_t));

  switch (this->unit) {
    case F:
    {
      TempSet = static_cast<uint16_t>(((alert->SetTemp+49)/315)*65535.0f);
      TempClear = static_cast<uint16_t>(((alert->ClearTemp+49)/315)*65535.0f);
      break;
    }
    default:
    {
      TempSet = static_cast<uint16_t>(((alert->SetTemp+45)/175)*65535.0f);
      TempClear = static_cast<uint16_t>(((alert->ClearTemp+45)/175)*65535.0f);
      break;
    }
  }

  HumiditySet = static_cast<uint16_t>((alert->SetHumidity*65535.0f)/100);
  HumidityClear = static_cast<uint16_t>((alert->ClearHumidity*65535.0f)/100);

  rhtSet = (HumiditySet&0xFE00);
  rhtSet |= (TempSet>>7);
  rhtSetMSB = (uint8_t)(rhtSet>>8);
  rhtSetLSB = (uint8_t)(rhtSet&0xFF);

  rhtClear =(HumidityClear&0xFE00);
  rhtClear |= (TempClear>>7);
  rhtClearMSB = (uint8_t)(rhtClear>>8);
  rhtClearLSB = (uint8_t)(rhtClear&0xFF);


  cmd[0] = 0x61; // MSB
  cmd[1] = 0x1D;
  cmd[2] = rhtSetMSB;
  cmd[3] = rhtSetLSB;
  cmd[4] = crc8(&cmd[2],2);
  //i2c_write_blocking(&this->i2c, this->devAddress, cmd, 5, false);
  i2c_dev->write(cmd, 5);

  //printf("Set Byte %x %x \n",cmd[2],cmd[3]);
  cmd[0] = 0x61; // MSB
  cmd[1] = 16;
  cmd[2] = rhtClearMSB;
  cmd[3] = rhtClearLSB;
  cmd[4] = crc8(&cmd[2],2);
  //i2c_write_blocking(&this->i2c, this->devAddress, cmd, 5, false);
  i2c_dev->write(cmd, 5);
  //printf("clear  Byte %x %x \n",cmd[2],cmd[3]);
}

void Adafruit_SHT31::setLowAlert(const SHT31_Alert_t* alert){
  uint16_t TempSet,TempClear, HumiditySet, HumidityClear;
  uint16_t rhtSet,rhtClear;
  uint8_t rhtSetMSB,rhtSetLSB,rhtClearMSB,rhtClearLSB;

  uint8_t cmd[5];

    memcpy(&this->LowAlert,alert,sizeof(SHT31_Alert_t));

  switch (this->unit) {
    case F: 
    {
      TempSet = static_cast<uint16_t>(((alert->SetTemp+49)/315)*65535.0f);
      TempClear = static_cast<uint16_t>(((alert->ClearTemp+49)/315)*65535.0f);
      break;
    }
    default:
    {
      TempSet = static_cast<uint16_t>(((alert->SetTemp+45)/175)*65535.0f);
      TempClear = static_cast<uint16_t>(((alert->ClearTemp+45)/175)*65535.0f);
      break;
    }
  }

  HumiditySet = static_cast<uint16_t>((alert->SetHumidity*65535.0f)/100);
  HumidityClear = static_cast<uint16_t>((alert->ClearHumidity*65535.0f)/100);

  rhtSet = (HumiditySet&0xFE00);
  rhtSet |= (TempSet>>7);
  rhtSetMSB = (uint8_t)(rhtSet>>8);
  rhtSetLSB = (uint8_t)(rhtSet&0xFF);

  rhtClear =(HumidityClear&0xFE00);
  rhtClear |= (TempClear>>7);
  rhtClearMSB = (uint8_t)(rhtClear>>8);
  rhtClearLSB = (uint8_t)(rhtClear&0xFF);

  cmd[0] = 0x61; // MSB
  cmd[1] = 0x0B;
  cmd[2] = rhtSetMSB;
  cmd[3] = rhtSetLSB;
  cmd[4] = crc8(&cmd[2],2);
  //i2c_write_blocking(&this->i2c, this->devAddress, cmd, 5, false);
  i2c_dev->write(cmd, 5);
  //printf("Set Byte %x %x \n",cmd[2],cmd[3]);
  cmd[0] = 0x61; // MSB
  cmd[1] = 00;
  cmd[2] = rhtClearMSB;
  cmd[3] = rhtClearLSB;
  cmd[4] = crc8(&cmd[2],2);
  //i2c_write_blocking(&this->i2c, this->devAddress, cmd, 5, false);
  i2c_dev->write(cmd, 5);
  //printf("clear  Byte %x %x \n",cmd[2],cmd[3]);
}

void Adafruit_SHT31::ReadLowAlert(SHT31_Alert_t* alert){
  memcpy(&this->LowAlert,alert,sizeof(SHT31_Alert_t));
}

void Adafruit_SHT31::ReadHighAlert(SHT31_Alert_t* alert){
  memcpy(&this->HighAlert,alert,sizeof(SHT31_Alert_t));
}

void Adafruit_SHT31::PeriodicMode(SHT31_Sample_Rate_t mps){
  uint8_t cmd[2];

  switch (mps) {
    case _05mps_high_Res: {
      cmd[0] = SHT31_PERIODIC_05mps_HIGHREP_MSB;
      cmd[1] = SHT31_PERIODIC_05mps_HIGHREP_LSB;
    }
    case _05_med_Res :{
      cmd[0] = SHT31_PERIODIC_05mps_MEDREP_MSB;
      cmd[1] = SHT31_PERIODIC_05mps_MEDREP_LSB;
    }
    case _05_low_Res:{
      cmd[0] = SHT31_PERIODIC_05mps_LOWREP_MSB;
      cmd[1] = SHT31_PERIODIC_05mps_LOWREP_LSB;
    }
    case _1mps_high_Res:{ 
      cmd[0] = SHT31_PERIODIC_1mps_HIGHREP_MSB;
      cmd[1] = SHT31_PERIODIC_1mps_HIGHREP_LSB;
    }
    case _1mps_med_Res:{
      cmd[0] = SHT31_PERIODIC_1mps_MEDREP_MSB;
      cmd[1] = SHT31_PERIODIC_1mps_MEDREP_LSB;
    }
    case _1mps_low_Res:{
      cmd[0] = SHT31_PERIODIC_1mps_LOWREP_MSB;
      cmd[1] = SHT31_PERIODIC_1mps_LOWREP_LSB;
    }
    case _2mps_high_Res:{
      cmd[0] = SHT31_PERIODIC_2mps_HIGHREP_MSB;
      cmd[1] = SHT31_PERIODIC_2mps_HIGHREP_LSB;
    }
    case _2mps_med_Res:{
      cmd[0] = SHT31_PERIODIC_2mps_MEDREP_MSB;
      cmd[1] = SHT31_PERIODIC_2mps_MEDREP_LSB;
    }
    case _2mps_low_Res:{
      cmd[0] = SHT31_PERIODIC_2mps_LOWREP_MSB;
      cmd[1] = SHT31_PERIODIC_2mps_LOWREP_LSB;
    }
    case _4mps_high_Res:{
      cmd[0] = SHT31_PERIODIC_4mps_HIGHREP_MSB;
      cmd[1] = SHT31_PERIODIC_4mps_HIGHREP_LSB;
    }
    case _4mps_med_Res:{
      cmd[0] = SHT31_PERIODIC_4mps_MEDREP_MSB;
      cmd[1] = SHT31_PERIODIC_4mps_MEDREP_LSB;
    }
    case _4mps_low_Res:{
      cmd[0] = SHT31_PERIODIC_4mps_LOWREP_MSB;
      cmd[1] = SHT31_PERIODIC_4mps_LOWREP_LSB;
    }
    case _10mps_high_Res:{
      cmd[0] = SHT31_PERIODIC_10mps_HIGHREP_MSB;
      cmd[1] = SHT31_PERIODIC_10mps_HIGHREP_LSB;
    }
    case _10mps_med_Res:{
      cmd[0] = SHT31_PERIODIC_10mps_MEDREP_MSB;
      cmd[1] = SHT31_PERIODIC_10mps_MEDREP_LSB;
    }
    case _10mps_low_Res:{
      cmd[0] = SHT31_PERIODIC_10mps_LOWREP_MSB;
      cmd[1] = SHT31_PERIODIC_10mps_LOWREP_LSB;
    }
    default: {
      cmd[0] = SHT31_PERIODIC_1mps_LOWREP_MSB;
      cmd[1] = SHT31_PERIODIC_1mps_LOWREP_LSB;
    }
  }

  i2c_dev->write(cmd, 2);
  // i2c_write_blocking(&this->i2c, this->devAddress, cmd, 2, false);
}

  bool Adafruit_SHT31::HighTempActive(){
    return TempHighAlert;
  }

  bool Adafruit_SHT31::LowTempActive(){
    return TempLowAlert;
  }

  bool Adafruit_SHT31::HighHumidityActive(){
    return HumidityHighAlert;
  }

  bool Adafruit_SHT31::LowHumidityActive(){
    return HumidityLowAlert;
  }