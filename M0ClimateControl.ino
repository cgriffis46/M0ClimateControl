  /*
    Modbus Climate Controller
    Cory S Griffis
    (c)2023-09-18 

    Simple Climate Controller based on Modbus
    Turn on/off devices based on High and Low Temperature and Humidity Alerts
    ex. 
      Turn on a Humdifier or Dehumidifier based on High/Low humidity ranges 
      Turn on a fan using a High Temperature alert
      Turn on a heater using the Low temperature alert
    
    Released under GNU public license 

    TODO: Needs NVRAM to store setpoint values. 

    MODBUS REGISTERS
    INPUT REGISTERS 
      0x00 - temperature in Celcius (32-bit float)
      0x01 - 
      0x02 - humidity RH% (32-bit float)
      0x03 - 
    DISCRETE INPUTS 
      0x04 - temp high alert active
      0x05 - temp low alert active
      0x06 - humidity high alert active
      0x07 - humidity low alert active
    HOLDING REGISTERS (32-bit float)
      0x08 - Temperature High Set
      0x09 - 
      0x0A - Temperature High Clear
      0x0B - 
      0x0C - Humidity High Set
      0x0D - 
      0x0E - Humidity High Clear
      0x0F - 
      0x10 - Temperature Low Set
      0x11 - 
      0x12 - Temperature Low Clear
      0x13 - 
      0x14 - Humidity Low Set
      0x15 - 
      0x16 - Humidity Low Clear
      0x17 - 
  */
// Defines 
#define USE_SHT31
#define _USE_MODBUS true
#define USE_I2C_EEPROM true
#define _USE_RTC_PCF8523 true
#define SDchipSelect 10
#define _HIGH_HEAT_PIN 0
#define _LOW_HEAT_PIN 1
#define _HIGH_HUMIDITY_PIN 2
#define _LOW_HUMIDITY_PIN 3

// FreeRTOS Libraries 
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOS_SAMD21.h>
#include <croutine.h>
#include <deprecated_definitions.h>
#include <error_hooks.h>
#include <event_groups.h>
#include <list.h>
#include <message_buffer.h>
#include <mpu_prototypes.h>
#include <mpu_wrappers.h>
#include <portable.h>
#include <portmacro.h>
#include <projdefs.h>
#include <queue.h>
#include <runTimeStats_hooks.h>
#include <semphr.h>
#include <stack_macros.h>
#include <stream_buffer.h>
#include <task.h>
#include <timers.h>

// Standard Libraries
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
// Datalogger 
#include <SD.h>

// Memory Definitions 
#define _MEM_HIGH_TEMP_SET 0x0200
#define _MEM_HIGH_TEMP_CLEAR 0x0204
#define _MEM_LOW_TEMP_SET 0x0208
#define _MEM_LOW_TEMP_CLEAR 0x020C

#define _MEM_HIGH_HUMIDITY_SET 0x0210
#define _MEM_HIGH_HUMIDITY_CLEAR 0x0214
#define _MEM_LOW_HUMIDITY_SET 0x0218
#define _MEM_LOW_HUMIDITY_CLEAR 0x021C

#define _HIGH_TEMP_SET_DEFAULT 80.0
#define _HIGH_TEMP_CLEAR_DEFAULT 75.0
#define _LOW_TEMP_SET_DEFAULT 60.0
#define _LOW_TEMP_CLEAR_DEFAULT 65.0

#define _HIGH_HUMIDITY_SET_DEFAULT 80.0
#define _HIGH_HUMIDITY_CLEAR_DEFAULT 75.0
#define _LOW_HUMIDITY_SET_DEFAULT 50.0
#define _LOW_HUMIDITY_CLEAR_DEFAULT 55.0

// RTC Definitions 
#ifdef _USE_DS3231
  #include <RTClib.h>
  RTC_DS3231 rtc;
  #ifndef _USE_RTC
    #define _USE_RTC true
  #endif
#endif

#ifdef _USE_RTC_PCF8523
  #ifndef _USE_RTC
    #define _USE_RTC
  #endif
  #include "RTClib.h"
  RTC_PCF8523 rtc;
#endif

#ifdef USE_I2C_EEPROM
// NVRAM definition for I2C EEPROM
  #include <Adafruit_EEPROM_I2C.h> // EEPROM Library
  Adafruit_EEPROM_I2C fram = Adafruit_EEPROM_I2C(); // EEPROM Object
#endif

#ifdef USE_SPI_FRAM
// NVRAM definition (if using SPI FRAM)
  #include <Adafruit_FRAM_SPI.h>
  uint8_t FRAM_SCK = 14; // SCK Pin
  uint8_t FRAM_MISO = 12; // MISO Pin
  uint8_t FRAM_MOSI = 13; // MOSI Pin
  uint8_t FRAM_CS = 15; // CS Pin
  Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_SCK, FRAM_MISO, FRAM_MOSI, FRAM_CS); // FRAM Object
#endif

#ifdef USE_SHT31
// temperature & humidity sensor
  #include "Adafruit_SHT31.h" 
  static void xSHT31Task(void *pvParameters);
  Adafruit_SHT31 sht31 = Adafruit_SHT31();
  bool enableHeater = false;
  uint8_t loopCnt = 0;
  float temperature, humidity;
  SHT31_Alert_t highAlert,LowAlert;
  TaskHandle_t xSHT31TaskHandle;

  #ifndef _USE_TH_SENSOR
    #define _USE_TH_SENSOR
  #endif
  #ifndef _INFCE_SEND_TEMP_HUMIDITY
   #define _INFCE_SEND_TEMP_HUMIDITY
  #endif
#endif

#ifdef _USE_MODBUS
  static void xModbusTask(void *pvParameters);
  static void xModbusPollTask(void *pvParameters);

  #include <SPI.h>
  #include <Ethernet.h>
  #include <ArduinoModbus.h>
  #include <EthernetUdp.h>
  #define _MODBUSINPUTREGISTERS 4 // Modbus Input Registers
  #define _MODBUSHOLDINGREGISTERS 16 // Modbus Holding Registers 
  #define _MODBUSCOILS 5 // Modbus Coils
  #define _MODBUSDISCRETEINPUTS 4 // Modbus Discrete Inputs 

  EthernetServer ethServer(502); // The Ethernet server 
  EthernetClient ModbusClient;   // The Client 
  ModbusTCPServer modbusTCPServer; // The Modbus Server

  TaskHandle_t xModbusTaskHandle;      // Task updates registers and determines if holding registers changed
  TaskHandle_t xModbusPollTaskHandle;  // Task polls for clients 

  bool ModbusDiscreteInputs[_MODBUSDISCRETEINPUTS]; // Modbus Discrete Inputs
  bool ModbusCoils[_MODBUSCOILS]; // Mpdbus Coils
  uint16_t ModbusInputRegisters[_MODBUSINPUTREGISTERS]; // Modbus Input Registers
  uint16_t ModbusHoldingRegisters[_MODBUSHOLDINGREGISTERS];

  // structure to break apart a 32-bit float into 2 16-bit Modbus registers
  union FloatToInt16{
    float f;
    uint16_t ModbusInt[2];
  };

  // High alerts and Low alerts for Holding registers 
  SHT31_Alert_t ModbusHighAlert,ModbusLowAlert;

#endif

// If using Feather M0 with RF69 module 
  #define RFM69_CS    8
  #define RFM69_INT   3
  #define RFM69_RST   4
  #define LED        13

// Convert float to byte array for 8-bit EEPROM
union{
  float f;
  uint8_t bytes[4];
} floattobytearray;

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0x98, 0x76, 0xB6, 0x12, 0x3c, 0xa6
};

// default IP address
IPAddress ip(192, 168, 1, 177);

// webserver definitions 
static void xHTTPUpdateTask(void *pvParameters); // Webserver Task 
EthernetClient HTTPClient; // The Web Client
TaskHandle_t xHTTPClientTaskHandle; // Task Handle for Webserver Task
// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);

// NTP Client definitions
#include <NTPClient.h> // NTP Client Library 
DateTime now; // Current DateTime
DateTime ntptime; // NTP Time
// WiFiUDP wifiUdp;
EthernetUDP eth0udp; // UDP Socket for NTP Client
NTPClient timeClient(eth0udp); // NTP Client 
static void xNTPClientTask(void *pvParameters); // NTP Client Task
TaskHandle_t xNTPClientTaskHandle;  // NTP Client Task Handle

// Output Pin Definitions
static void xSetOutputPinsTask(void *pvParameters); // Task updates Digital Outputs
TaskHandle_t xSetOutputPinsTaskHandle; // Task handle for task to update Digital Outputs

// Semaphore Definitions 
SemaphoreHandle_t I2CBusSemaphore; // Arbitrate I2C bus access
SemaphoreHandle_t Eth0Semaphore; // Arbitrate Eth0 bus access

void setup() {
  FloatToInt16 conversion;
  Serial.begin(115200);
//  while(!Serial);
  SPI.begin();
  I2CBusSemaphore = xSemaphoreCreateMutex(); // Create I2C Semaphore
  Eth0Semaphore = xSemaphoreCreateMutex(); // Create ETH0 Semaphore
#ifdef _USE_RTC
// Initialize the RTC 
    Serial.println("Initialize RTC");
    if(rtc.begin(&Wire)){      
//      rtc.start();
      now = rtc.now();
  }
  else {
    Serial.println("Could not initialize RTC!");
  }
#endif // USE_RTC
  // Initialize NVRAM
  Serial.println("Initialize NVRAM");
  if(fram.begin(0x50)){
     //loadCredentials();            // Load WLAN credentials from network
     //LoadSensorsFromDisk();
     //LoadWundergroundCredentials();// Load Wunderground Interface credentials
     loadAlertValues(&highAlert,&LowAlert);
  }
  else{
    Serial.println("could not initialize fram");
  }

  // Setup output pins
  pinMode(_HIGH_HEAT_PIN, OUTPUT); // Digital Output for High Temperature alert 
  pinMode(_LOW_HEAT_PIN, OUTPUT);  // Digital Output for Low Temperature alert 
  pinMode(_HIGH_HUMIDITY_PIN, OUTPUT); // Digital Output for High Humidity alert 
  pinMode(_LOW_HUMIDITY_PIN, OUTPUT); // Digital Output for Low Humidity alert 

  sht31.begin(0x44);   // Initialize SHT31 Temp/humidity sensor
  sht31.setHighAlert(&highAlert); // Write High Temp/Humidity alerts to SHT31
  sht31.setLowAlert(&LowAlert); // Write Low Temp/Humdity alerts to SHT31
  sht31.PeriodicMode(_10mps_low_Res); // Periodic mode 
  xTaskCreate(xSHT31Task,     "Sensor Task",       256, NULL, tskIDLE_PRIORITY + 3, &xSHT31TaskHandle); // start Sensor Update task

  pinMode(8, INPUT_PULLUP); // RF69 Enable pin
  pinMode(10, OUTPUT); // Ethernet Feather CS pin

  SD.begin(SDchipSelect); 

  // Initialize Ethernet 
  Ethernet.init(10);
  // start the Ethernet connection and the server:
  Ethernet.begin(mac);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  server.begin(); // start webserver
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
  HTTPClient.setConnectionTimeout(2000);
  xTaskCreate(xHTTPUpdateTask,     "HTTP Update Task",       256, NULL, tskIDLE_PRIORITY + 9, &xHTTPClientTaskHandle); // Start web server task

#ifdef _USE_MODBUS
    // Configure Modbus Registers
    modbusTCPServer.begin();
    modbusTCPServer.configureInputRegisters(0x00, _MODBUSINPUTREGISTERS);
    modbusTCPServer.configureDiscreteInputs(0x04, _MODBUSDISCRETEINPUTS);
    modbusTCPServer.configureHoldingRegisters(0x08,_MODBUSHOLDINGREGISTERS);
    // Write current High Temperature setpoint to Modbus 
    conversion.f = highAlert.SetTemp;
    modbusTCPServer.holdingRegisterWrite(0x08, conversion.ModbusInt[0]);
    modbusTCPServer.holdingRegisterWrite(0x09, conversion.ModbusInt[1]);
    // Write current High Temperature clear point to Modbus 
    conversion.f = highAlert.ClearTemp;
    modbusTCPServer.holdingRegisterWrite(0x0A, conversion.ModbusInt[0]);
    modbusTCPServer.holdingRegisterWrite(0x0B, conversion.ModbusInt[1]);
    // Write current High Humidity setpoint to Modbus 
    conversion.f = highAlert.SetHumidity;
    modbusTCPServer.holdingRegisterWrite(0x0C, conversion.ModbusInt[0]);
    modbusTCPServer.holdingRegisterWrite(0x0D, conversion.ModbusInt[1]);
    // Write current High Humidity clear point to Modbus 
    conversion.f = highAlert.ClearHumidity;
    modbusTCPServer.holdingRegisterWrite(0x0E, conversion.ModbusInt[0]);
    modbusTCPServer.holdingRegisterWrite(0x0F, conversion.ModbusInt[1]);
    // Write current Low Temperature setpoint to Modbus 
    conversion.f = LowAlert.SetTemp;
    modbusTCPServer.holdingRegisterWrite(0x10, conversion.ModbusInt[0]);
    modbusTCPServer.holdingRegisterWrite(0x11, conversion.ModbusInt[1]);
    // Write current Low Temperature clear point to Modbus 
    conversion.f = LowAlert.ClearTemp;
    modbusTCPServer.holdingRegisterWrite(0x12, conversion.ModbusInt[0]);
    modbusTCPServer.holdingRegisterWrite(0x13, conversion.ModbusInt[1]);
    // Write current Low Humidity setpoint to Modbus 
    conversion.f = LowAlert.SetHumidity;
    modbusTCPServer.holdingRegisterWrite(0x14, conversion.ModbusInt[0]);
    modbusTCPServer.holdingRegisterWrite(0x15, conversion.ModbusInt[1]);
    // Write current Low Humidity clear point to Modbus 
    conversion.f = LowAlert.ClearHumidity;
    modbusTCPServer.holdingRegisterWrite(0x16, conversion.ModbusInt[0]);
    modbusTCPServer.holdingRegisterWrite(0x17, conversion.ModbusInt[1]);
  xTaskCreate(xModbusTask,     "Modbus Update Task",       256, NULL, tskIDLE_PRIORITY + 8, &xModbusTaskHandle); // Start register update task
  xTaskCreate(xModbusPollTask,     "Modbus Poll",       256, NULL, tskIDLE_PRIORITY + 8, &xModbusPollTaskHandle); // Start client poll task
#endif
  timeClient.begin(); // Start NTP Client 
  xTaskCreate(xNTPClientTask,     "NTP Task",       256, NULL, tskIDLE_PRIORITY + 5, &xNTPClientTaskHandle); // Start NTP Update task
  xTaskCreate(xSetOutputPinsTask,     "NTP Task",       256, NULL, tskIDLE_PRIORITY + 8, &xSetOutputPinsTaskHandle); // Start task to update output values
  vTaskStartScheduler(); // Start task scheduler

}

// RTOS Scheduler will not allow us to get here
void loop() {
}

#ifdef _USE_MODBUS
// Updates Modbus Input Registers and Discrete Inputs. 
// Determines if any Holding registers have changed and
// saves alert values to NVRAM
static void xModbusTask(void *pvParameters){
while(true){

    FloatToInt16 conversion;
    // Write Input Registers to Modbus
    // 0x00 - Temperature (32 bit float)
    // 0x01 - 
    // 0x02 - Humidity (32 bit float)
    // 0x03 - 
    // Write Current Temperature to Modbus
    conversion.f = temperature;
    ModbusInputRegisters[0] = conversion.ModbusInt[0];
    ModbusInputRegisters[1] = conversion.ModbusInt[1];
    // Write Current Humidity to Modbus
    conversion.f = humidity;
    ModbusInputRegisters[2] = conversion.ModbusInt[0];
    ModbusInputRegisters[3] = conversion.ModbusInt[1];
    modbusTCPServer.writeInputRegisters(0x00, ModbusInputRegisters, _MODBUSINPUTREGISTERS);

    // Write Discrete Input registers to Modbus 
    // 0x04 - High Temperature alert is active
    // 0x05 - Low Temperature alert is active
    // 0x06 - High Humidity alert is active
    // 0x07 - Low Humidity alert is active
    modbusTCPServer.discreteInputWrite(0x04, sht31.HighTempActive()); // High Temperature alert is Active
    modbusTCPServer.discreteInputWrite(0x05, sht31.LowTempActive()); // Low Temperature alert is Active
    modbusTCPServer.discreteInputWrite(0x06, sht31.HighHumidityActive()); // High Humidity alert is Active
    modbusTCPServer.discreteInputWrite(0x07, sht31.LowHumidityActive()); // Low Humidity alert is Active

  // Read High Temperature alert Setpoint
  conversion.ModbusInt[0] = modbusTCPServer.holdingRegisterRead(0x08);
  conversion.ModbusInt[1] = modbusTCPServer.holdingRegisterRead(0x09);
  ModbusHighAlert.SetTemp = conversion.f;

  // Read High Temperature alert Clear point
  conversion.ModbusInt[0] = modbusTCPServer.holdingRegisterRead(0x0A);
  conversion.ModbusInt[1] = modbusTCPServer.holdingRegisterRead(0x0B);
  ModbusHighAlert.ClearTemp = conversion.f;

  // Read High Humidity alert Setpoint
  conversion.ModbusInt[0] = modbusTCPServer.holdingRegisterRead(0x0C);
  conversion.ModbusInt[1] = modbusTCPServer.holdingRegisterRead(0x0D);
  ModbusHighAlert.SetHumidity = conversion.f;

  // Read High Humidity alert clear point
  conversion.ModbusInt[0] = modbusTCPServer.holdingRegisterRead(0x0E);
  conversion.ModbusInt[1] = modbusTCPServer.holdingRegisterRead(0x0F);
  ModbusHighAlert.ClearHumidity = conversion.f;

  // Read Low temperature alert setpoint 
  conversion.ModbusInt[0] = modbusTCPServer.holdingRegisterRead(0x10);
  conversion.ModbusInt[1] = modbusTCPServer.holdingRegisterRead(0x11);
  ModbusLowAlert.SetTemp = conversion.f;

  // Read low temperature alert clear point
  conversion.ModbusInt[0] = modbusTCPServer.holdingRegisterRead(0x12);
  conversion.ModbusInt[1] = modbusTCPServer.holdingRegisterRead(0x13);
  ModbusLowAlert.ClearTemp = conversion.f;

  // read low humidity alert setpoint 
  conversion.ModbusInt[0] = modbusTCPServer.holdingRegisterRead(0x14);
  conversion.ModbusInt[1] = modbusTCPServer.holdingRegisterRead(0x15);
  ModbusLowAlert.SetHumidity = conversion.f;

  // read low humidity alert clear point
  conversion.ModbusInt[0] = modbusTCPServer.holdingRegisterRead(0x16);
  conversion.ModbusInt[1] = modbusTCPServer.holdingRegisterRead(0x17);
  ModbusLowAlert.ClearHumidity = conversion.f;

  // Determine which holding registers changed. Update as necessary. 
  bool AlertValuesChanged = false; // Only save update if values change 
  if(ModbusHighAlert.SetTemp!=highAlert.SetTemp){   // High Temperature alert Setpoint changed 
      if(ModbusHighAlert.SetTemp>=-20&&ModbusHighAlert.SetTemp<=200){ // Sanity check
        highAlert.SetTemp = ModbusHighAlert.SetTemp; // Update High Temperature Setpoint
        AlertValuesChanged = true;}} // Notify Holding Registers Changed
  if(ModbusHighAlert.ClearTemp!=highAlert.ClearTemp){   // High Temperature alert Clear point changed 
      if(ModbusHighAlert.ClearTemp>=-20&&ModbusHighAlert.ClearTemp<=200){ // Sanity check
        highAlert.ClearTemp = ModbusHighAlert.ClearTemp; //Update High Temperature Clear point
        AlertValuesChanged = true;}}  // Notify Holding Registers Changed
  if(ModbusHighAlert.SetHumidity!=highAlert.SetHumidity){   // High Humidity alert Setpoint changed
      if(ModbusHighAlert.SetHumidity>=0&&ModbusHighAlert.SetHumidity<=100){ // Sanity check 
        highAlert.SetHumidity = ModbusHighAlert.SetHumidity; // Update High Humidity alert
        AlertValuesChanged = true;}}  // Notify Holding Registers Changed
  if(ModbusHighAlert.ClearHumidity!=highAlert.ClearHumidity){   // High Humidity alert Clear point changed
      if(ModbusHighAlert.ClearHumidity>=0&&ModbusHighAlert.ClearHumidity<=100){ // Sanity check
        highAlert.ClearHumidity = ModbusHighAlert.ClearHumidity; // Update High Humidity Clear point 
        AlertValuesChanged = true;}} // Notify Holding Registers Changed
  if(ModbusLowAlert.SetTemp!=LowAlert.SetTemp){  // Low temperature setpoint changed
      if(ModbusLowAlert.SetTemp>=-20&&ModbusLowAlert.SetTemp<=200){ // Sanity check
        LowAlert.SetTemp = ModbusLowAlert.SetTemp; // Update low temperature alert setpoint
        AlertValuesChanged = true;}} // Notify Holding Registers Changed
  if(ModbusLowAlert.ClearTemp!=LowAlert.ClearTemp){ // low temperature clear point changed
      if(ModbusLowAlert.ClearTemp>=-20&&ModbusLowAlert.ClearTemp<=200){ // sanity check
        LowAlert.ClearTemp = ModbusLowAlert.ClearTemp; // Update low temperature alert clear point 
        AlertValuesChanged = true;}}// Notify Holding Registers Changed
  if(ModbusLowAlert.SetHumidity!=LowAlert.SetHumidity){  // low humidity setpoint changed
      if(ModbusLowAlert.SetHumidity>=0&&ModbusLowAlert.SetHumidity<=100){// Sanity check
        LowAlert.SetHumidity = ModbusLowAlert.SetHumidity; // Update Low Humidity alert Setpoint
        AlertValuesChanged = true;}}// Notify Holding Registers Changed
  if(ModbusLowAlert.ClearHumidity!=LowAlert.ClearHumidity){  // low humidity clear point changed
      if(ModbusLowAlert.ClearHumidity>=0&&ModbusLowAlert.ClearHumidity<=100){ // Sanity check
        LowAlert.ClearHumidity = ModbusLowAlert.ClearHumidity; // Update Low Humidity Alert Setpoint
        AlertValuesChanged = true;}}// Notify Holding Registers Changed
  if(AlertValuesChanged){    // if any alert values changed, update the physical sensor. 
    sht31.setHighAlert(&highAlert); // write high temperature/humidity values to the SHT31
    sht31.setLowAlert(&LowAlert); // write the Low temperature/humidity values to the SHT31
    saveAlertValues(&highAlert,&LowAlert); //Writes High/Low Temperature alert values to NVRAM
    AlertValuesChanged = false; // reset flag 
  }
  vTaskDelay( (100 * 1000) / portTICK_PERIOD_US ); // Sleep for 100ms
}
}
  #endif

// Sensor update task reads the SHT31 sensor 
static void xSHT31Task(void *pvParameters){
while(true){
// Read SHT31 sensor and update temperature and humidity. 
float t, h;
if(xSemaphoreTake(I2CBusSemaphore,1)){
  if(sht31.FetchData(&t, &h)){

    if (! isnan(t)) {  // check if 'is not a number'
      temperature = t;
      Serial.print("Temp *C = "); Serial.print(temperature); Serial.print("\t\t");
    } else { 
      Serial.println("Failed to read temperature");
    }
  
    if (! isnan(h)) {  // check if 'is not a number'
      humidity = h;
      Serial.print("Hum. % = "); Serial.println(humidity);
    } else { 
      Serial.println("Failed to read humidity");
    }
  }
  else {
    Serial.println("Failed to read temp and humidity");
  }
  xSemaphoreGive( I2CBusSemaphore );
  // Get current High and Low alert levels
  sht31.ReadHighAlert(&highAlert);
  sht31.ReadLowAlert(&LowAlert);
}
  vTaskDelay( (1000 * 1000) / portTICK_PERIOD_US );
}
}

// Webserver task polls for web clients and serves webpages 
// to be inplemented
static void xHTTPUpdateTask(void *pvParameters){
while(true){
  if(xSemaphoreTake(Eth0Semaphore,5)){
    HTTPClient = server.available();
    if (HTTPClient) {
      Serial.println("new client");
      // an HTTP request ends with a blank line
      bool currentLineIsBlank = true;
    if (HTTPClient.connected()) {
      while (HTTPClient.available()) {
        char c = HTTPClient.read();
        //Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the HTTP request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard HTTP response header
          HTTPClient.println("HTTP/1.1 200 OK");
          HTTPClient.println("Content-Type: text/html");
          HTTPClient.println("Connection: close");  // the connection will be closed after completion of the response
          HTTPClient.println("Refresh: 5");  // refresh the page automatically every 5 sec
          HTTPClient.println();
          HTTPClient.println("<!DOCTYPE HTML>");
          HTTPClient.println("<html>");
          /*
          // output the value of each analog input pin
          for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
            int sensorReading = analogRead(analogChannel);
            HTTPClient.print("analog input ");
            HTTPClient.print(analogChannel);
            HTTPClient.print(" is ");
            HTTPClient.print(sensorReading);
            HTTPClient.println("<br />");
          } */
          HTTPClient.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } 
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
         //vTaskDelay( 2/portTICK_PERIOD_MS );  
    }
      // give the web browser time to receive the data
      delay(1);
      // close the connection:
      HTTPClient.stop();
    }
    // Give up the semaphore and sleep 250ms
    xSemaphoreGive( Eth0Semaphore );
    vTaskDelay( 250/portTICK_PERIOD_MS ); 
  }
  // We didn't get the semaphore, so sleep only 1ms
  vTaskDelay(1/portTICK_PERIOD_MS );
}
}

// NTP Task polls NTP and updates the RTC
static void xNTPClientTask(void *pvParameters){
while(true){
bool shouldUpdateRTC = false;

if(xSemaphoreTake(Eth0Semaphore,1)){
       // timeClient must be called every loop to update NTP time 
      if(timeClient.update()) {
          if(timeClient.isTimeSet()){
            shouldUpdateRTC = true;
            // adjust the external RTC

            //Serial.println("Updated RTC time!");
            //String timedate = String(now.year())+String("-")+String(now.month())+String("-")+String(now.day())+String(" ")+String(now.hour())+String(":")+String(now.minute())+String(":")+String(now.second());
            //Serial.println(now.timestamp());
            }
      }
      else {
      //          Serial.println("Could not update NTP time!");
      }
    xSemaphoreGive( Eth0Semaphore );
  }
  else {
    shouldUpdateRTC = false;
  }

#ifdef _USE_RTC
if(shouldUpdateRTC){
  if(xSemaphoreTake(I2CBusSemaphore,5)){
  rtc.adjust(DateTime(timeClient.getEpochTime()));
  xSemaphoreGive( I2CBusSemaphore );}
  vTaskDelay( 500/portTICK_PERIOD_MS );
}
else {
  vTaskDelay( 500/portTICK_PERIOD_MS );
}
#endif
}
}

static void xSetOutputPinsTask(void *pvParameters){
while(true){
  // Set output pins 
  digitalWrite(_HIGH_HEAT_PIN, sht31.HighTempActive());
  digitalWrite(_LOW_HEAT_PIN, sht31.LowTempActive());
  digitalWrite(_HIGH_HUMIDITY_PIN,sht31.HighHumidityActive());
  digitalWrite(_LOW_HUMIDITY_PIN, sht31.LowHumidityActive());
  vTaskDelay( 1000/portTICK_PERIOD_US ); 
}
}

static void xDataloggerTask(void *pvParameters){

}

// Poll modbus for clients and respond. 
// Reading/Writing registers will be handled in a separate
// task to reduce the time we have the mutex. 
static void xModbusPollTask(void *pvParameters){
while(true){
  if(xSemaphoreTake(Eth0Semaphore,1)){
      if(ModbusClient.connected()){ // client is already connected 
      modbusTCPServer.poll();
    } else 
    {
      // listen for incoming clients
      ModbusClient = ethServer.available();
      if(ModbusClient) {
        // let the Modbus TCP accept the connection 
        modbusTCPServer.accept(ModbusClient);
        // poll for Modbus TCP requests, while client connected
        modbusTCPServer.poll();
      } 
    }
    xSemaphoreGive( Eth0Semaphore );
    vTaskDelay(100/portTICK_PERIOD_MS ); 
  }
  else {
    vTaskDelay( 1000/portTICK_PERIOD_US );
  }
}
}
