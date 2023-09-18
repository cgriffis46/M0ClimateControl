/*!
 *  @file Adafruit_SHT31.h
 *
 *  This is a library for the SHT31 Digital Humidity & Temp Sensor
 *
 *  Designed specifically to work with the  Digital Humidity & Temp Sensor
 *  -----> https://www.adafruit.com/product/2857
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution
 */

#ifndef ADAFRUIT_SHT31_H
#define ADAFRUIT_SHT31_H

#include "Arduino.h"
#include <Adafruit_I2CDevice.h>

#define SHT31_DEFAULT_ADDR 0x44 /**< SHT31 Default Address */
#define SHT31_MEAS_HIGHREP_STRETCH                                             \
  0x2C06 /**< Measurement High Repeatability with Clock Stretch Enabled */
#define SHT31_MEAS_MEDREP_STRETCH                                              \
  0x2C0D /**< Measurement Medium Repeatability with Clock Stretch Enabled */
#define SHT31_MEAS_LOWREP_STRETCH                                              \
  0x2C10 /**< Measurement Low Repeatability with Clock Stretch Enabled*/
#define SHT31_MEAS_HIGHREP                                                     \
  0x2400 /**< Measurement High Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_MEDREP                                                      \
  0x240B /**< Measurement Medium Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_LOWREP                                                      \
  0x2416 /**< Measurement Low Repeatability with Clock Stretch Disabled */
#define SHT31_READSTATUS 0xF32D   /**< Read Out of Status Register */
#define SHT31_CLEARSTATUS 0x3041  /**< Clear Status */
#define SHT31_SOFTRESET 0x30A2    /**< Soft Reset */
#define SHT31_HEATEREN 0x306D     /**< Heater Enable */
#define SHT31_HEATERDIS 0x3066    /**< Heater Disable */
#define SHT31_REG_HEATER_BIT 0x0d /**< Status Register Heater Bit */

#define SHT31_FETCH_DATA 0xE000
#define SHT31_FETCH_DATA_MSB 0xE0
#define SHT31_FETCH_DATA_LSB 0x00


extern TwoWire Wire; /**< Forward declarations of Wire for board/variant
                        combinations that don't have a default 'Wire' */


#define SHT31_PERIODIC_05mps_HIGHREP_MSB 0x20
#define SHT31_PERIODIC_05mps_HIGHREP_LSB 0x32

#define SHT31_PERIODIC_05mps_MEDREP_MSB 0x20
#define SHT31_PERIODIC_05mps_MEDREP_LSB 0x24

#define SHT31_PERIODIC_05mps_LOWREP_MSB 0x20
#define SHT31_PERIODIC_05mps_LOWREP_LSB 0x2F

#define SHT31_PERIODIC_1mps_HIGHREP_MSB 0x21
#define SHT31_PERIODIC_1mps_HIGHREP_LSB 0x30

#define SHT31_PERIODIC_1mps_MEDREP_MSB 0x21
#define SHT31_PERIODIC_1mps_MEDREP_LSB 0x26

#define SHT31_PERIODIC_1mps_LOWREP_MSB 0x21
#define SHT31_PERIODIC_1mps_LOWREP_LSB 0x2D

#define SHT31_PERIODIC_2mps_HIGHREP_MSB 0x22
#define SHT31_PERIODIC_2mps_HIGHREP_LSB 0x36

#define SHT31_PERIODIC_2mps_MEDREP_MSB 0x22
#define SHT31_PERIODIC_2mps_MEDREP_LSB 0x20

#define SHT31_PERIODIC_2mps_LOWREP_MSB 0x22
#define SHT31_PERIODIC_2mps_LOWREP_LSB 0x2B

#define SHT31_PERIODIC_4mps_HIGHREP_MSB 0x23
#define SHT31_PERIODIC_4mps_HIGHREP_LSB 0x34

#define SHT31_PERIODIC_4mps_MEDREP_MSB 0x23
#define SHT31_PERIODIC_4mps_MEDREP_LSB 0x22

#define SHT31_PERIODIC_4mps_LOWREP_MSB 0x23
#define SHT31_PERIODIC_4mps_LOWREP_LSB 0x29

#define SHT31_PERIODIC_10mps_HIGHREP_MSB 0x27
#define SHT31_PERIODIC_10mps_HIGHREP_LSB 0x37

#define SHT31_PERIODIC_10mps_MEDREP_MSB 0x27
#define SHT31_PERIODIC_10mps_MEDREP_LSB 0x21

#define SHT31_PERIODIC_10mps_LOWREP_MSB 0x27
#define SHT31_PERIODIC_10mps_LOWREP_LSB 0x2A

struct SHT31_Alert_t{
  float SetTemp, ClearTemp;
  float SetHumidity, ClearHumidity;
};

typedef enum {
  C = 0,
  F = 1
}temp_unit;

typedef enum{
  _05mps_high_Res = 0,
  _05_med_Res = 1,
  _05_low_Res = 2,
  _1mps_high_Res = 3,
  _1mps_med_Res = 4,
  _1mps_low_Res = 5,
  _2mps_high_Res = 6,
  _2mps_med_Res = 7,
  _2mps_low_Res = 8,
  _4mps_high_Res = 9,
  _4mps_med_Res = 10,
  _4mps_low_Res = 11,
  _10mps_high_Res = 12,
  _10mps_med_Res = 13,
  _10mps_low_Res = 14,
}SHT31_Sample_Rate_t;



/**
 * Driver for the Adafruit SHT31-D Temperature and Humidity breakout board.
 */
class Adafruit_SHT31 {
public:
  Adafruit_SHT31(TwoWire *theWire = &Wire);
  ~Adafruit_SHT31();

  bool begin(uint8_t i2caddr = SHT31_DEFAULT_ADDR);
  float readTemperature(void);
  float readHumidity(void);
  bool readBoth(float *temperature_out, float *humidity_out);
  uint16_t readStatus(void);
  void reset(void);
  void heater(bool h);
  bool isHeaterEnabled();
  void PeriodicMode(SHT31_Sample_Rate_t mps);
  bool FetchData(float *t, float *h);
  void setHighAlert(const SHT31_Alert_t* alert);
  void setLowAlert(const SHT31_Alert_t* alert);
  void ReadLowAlert(SHT31_Alert_t* alert);
  void ReadHighAlert(SHT31_Alert_t* alert);
  void clearStatus();
  bool HighTempActive();
  bool LowTempActive();
  bool HighHumidityActive();
  bool LowHumidityActive();

private:
  /**
   * Placeholder to track humidity internally.
   */
  float humidity;

  /**
   * Placeholder to track temperature internally.
   */
  float temp;

  bool readTempHum(void);
  bool writeCommand(uint16_t cmd);
//  static uint8_t crc8(const uint8_t *data, int len);
  temp_unit unit = C;
  TwoWire *_wire;                     /**< Wire object */
  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface

  SHT31_Alert_t HighAlert;
  SHT31_Alert_t LowAlert;

  bool TempHighAlert, TempLowAlert, HumidityHighAlert,HumidityLowAlert;

};

#endif
