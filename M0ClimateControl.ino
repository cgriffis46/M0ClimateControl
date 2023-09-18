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
#define USE_SHT31
#define _USE_MODBUS true

#define _HIGH_HEAT_PIN 0
#define _LOW_HEAT_PIN 1
#define _HIGH_HUMIDITY_PIN 2
#define _LOW_HUMIDITY_PIN 3

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#ifdef USE_SHT31
  #include "Adafruit_SHT31.h"
  
  Adafruit_SHT31 sht31 = Adafruit_SHT31();
  bool enableHeater = false;
  uint8_t loopCnt = 0;

  float temperature, humidity;

  SHT31_Alert_t highAlert,LowAlert;

  #ifndef _USE_TH_SENSOR
    #define _USE_TH_SENSOR
  #endif
  #ifndef _INFCE_SEND_TEMP_HUMIDITY
   #define _INFCE_SEND_TEMP_HUMIDITY
  #endif
#endif

#ifdef _USE_MODBUS
  #include <SPI.h>
  #include <Ethernet.h>
  #include <ArduinoModbus.h>

  #define _MODBUSINPUTREGISTERS 4 // Modbus Input Registers
  #define _MODBUSHOLDINGREGISTERS 16 // Modbus Holding Registers 
  #define _MODBUSCOILS 5 // Modbus Coils
  #define _MODBUSDISCRETEINPUTS 4 // Modbus Discrete Inputs 

  EthernetServer ethServer(502); // Ethernet server 

  ModbusTCPServer modbusTCPServer;

  bool ModbusDiscreteInputs[_MODBUSDISCRETEINPUTS];
  bool ModbusCoils[_MODBUSCOILS];
  uint16_t ModbusInputRegisters[_MODBUSINPUTREGISTERS];
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


// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0x98, 0x76, 0xB6, 0x12, 0x3c, 0xa6
};

// default IP address
IPAddress ip(192, 168, 1, 177);

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
  SPI.begin();

  // Setup output pins
  pinMode(_HIGH_HEAT_PIN, OUTPUT);
  pinMode(_LOW_HEAT_PIN, OUTPUT);
  pinMode(_HIGH_HUMIDITY_PIN, OUTPUT);
  pinMode(_LOW_HUMIDITY_PIN, OUTPUT);

  // default values for Alerts 
  LowAlert.SetTemp = 0;
  LowAlert.ClearTemp = 5;
  LowAlert.SetHumidity = 30;
  LowAlert.ClearHumidity = 35;

  highAlert.SetTemp = 30;
  highAlert.ClearTemp = 25;
  highAlert.SetHumidity = 90;
  highAlert.ClearHumidity = 85;

  // Initialize SHT31 Temp/humidity sensor
  sht31.begin(0x44);
  sht31.setHighAlert(&highAlert);
  sht31.setLowAlert(&LowAlert);
  sht31.PeriodicMode(_10mps_low_Res);

  pinMode(8, INPUT_PULLUP); // RF69 Enable pin
  pinMode(10, OUTPUT); // Ethernet Feather CS pin

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

  // start the server
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

#ifdef _USE_MODBUS
  // Configure Modbus Registers
  modbusTCPServer.begin();
  modbusTCPServer.configureInputRegisters(0x00, _MODBUSINPUTREGISTERS);
  modbusTCPServer.configureDiscreteInputs(0x04, _MODBUSDISCRETEINPUTS);
  modbusTCPServer.configureHoldingRegisters(0x08,_MODBUSHOLDINGREGISTERS);
#endif

}

void loop() {

  // put your main code here, to run repeatedly:
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an HTTP request ends with a blank line
    bool currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the HTTP request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard HTTP response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
//          // output the value of each analog input pin
//          for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
//            int sensorReading = analogRead(analogChannel);
//            client.print("analog input ");
//            client.print(analogChannel);
//            client.print(" is ");
//            client.print(sensorReading);
//            client.println("<br />");
//          }
          client.println("</html>");
          break;
        }
        if (c == '\n') {
//          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }

  #ifdef _USE_MODBUS

  FloatToInt16 conversion;

  // Update Modbus Input Registers

    // Write Current Temperature to Modbus
    conversion.f = temperature;
    ModbusInputRegisters[0] = conversion.ModbusInt[0];
    ModbusInputRegisters[1] = conversion.ModbusInt[1];

    // Write Current Humidity to Modbus
    conversion.f = humidity;
    ModbusInputRegisters[2] = conversion.ModbusInt[0];
    ModbusInputRegisters[3] = conversion.ModbusInt[1];

    // Write Input Registers to Modbus
    // 0x00 - Temperature (32 bit float)
    // 0x01 - 
    // 0x02 - Humidity (32 bit float)
    // 0x03 - 
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

    // Write Current Modbus Holding Registers 

    // Get current High and Low alert levels
    sht31.ReadHighAlert(&highAlert);
    sht31.ReadLowAlert(&LowAlert);

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

  // Check if we have a modbus client connected and poll Modbus if necessary
  if(client.connected()){ // client is already connected 
    modbusTCPServer.poll();
  } else {
    // listen for incoming clients
    EthernetClient client = ethServer.available();
    if(client) {
      // let the Modbus TCP accept the connection 
      modbusTCPServer.accept(client);
      // poll for Modbus TCP requests, while client connected
      modbusTCPServer.poll();
    }
  }

  // Read Modbus Holding registers 

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
  bool ShouldUpdateModbus = false; 
  if(ModbusHighAlert.SetTemp!=highAlert.SetTemp){   // High Temperature alert Setpoint changed 
      if(ModbusHighAlert.SetTemp>=-20&&ModbusHighAlert.SetTemp<=200){ // Sanity check
        highAlert.SetTemp = ModbusHighAlert.SetTemp; // Update High Temperature Setpoint
        ShouldUpdateModbus = true;}} // Notify Holding Registers Changed
  if(ModbusHighAlert.ClearTemp!=highAlert.ClearTemp){   // High Temperature alert Clear point changed 
      if(ModbusHighAlert.ClearTemp>=-20&&ModbusHighAlert.ClearTemp<=200){ // Sanity check
        highAlert.ClearTemp = ModbusHighAlert.ClearTemp; //Update High Temperature Clear point
        ShouldUpdateModbus = true;}}  // Notify Holding Registers Changed
  if(ModbusHighAlert.SetHumidity!=highAlert.SetHumidity){   // High Humidity alert Setpoint changed
      if(ModbusHighAlert.SetHumidity>=0&&ModbusHighAlert.SetHumidity<=100){ // Sanity check 
        highAlert.SetHumidity = ModbusHighAlert.SetHumidity; // Update High Humidity alert
        ShouldUpdateModbus = true;}}  // Notify Holding Registers Changed
  if(ModbusHighAlert.ClearHumidity!=highAlert.ClearHumidity){   // High Humidity alert Clear point changed
      if(ModbusHighAlert.ClearHumidity>=0&&ModbusHighAlert.ClearHumidity<=100){ // Sanity check
        highAlert.ClearHumidity = ModbusHighAlert.ClearHumidity; // Update High Humidity Clear point 
        ShouldUpdateModbus = true;}} // Notify Holding Registers Changed
  if(ModbusLowAlert.SetTemp!=LowAlert.SetTemp){  // Low temperature setpoint changed
      if(ModbusLowAlert.SetTemp>=-20&&ModbusLowAlert.SetTemp<=200){ // Sanity check
        LowAlert.SetTemp = ModbusLowAlert.SetTemp; // Update low temperature alert setpoint
        ShouldUpdateModbus = true;}} // Notify Holding Registers Changed
  if(ModbusLowAlert.ClearTemp!=LowAlert.ClearTemp){ // low temperature clear point changed
      if(ModbusLowAlert.ClearTemp>=-20&&ModbusLowAlert.ClearTemp<=200){ // sanity check
        LowAlert.ClearTemp = ModbusLowAlert.ClearTemp; // Update low temperature alert clear point 
        ShouldUpdateModbus = true;}}// Notify Holding Registers Changed
  if(ModbusLowAlert.SetHumidity!=LowAlert.SetHumidity){  // low humidity setpoint changed
      if(ModbusLowAlert.SetHumidity>=0&&ModbusLowAlert.SetHumidity<=100){// Sanity check
        LowAlert.SetHumidity = ModbusLowAlert.SetHumidity; // Update Low Humidity alert Setpoint
        ShouldUpdateModbus = true;}}// Notify Holding Registers Changed
  if(ModbusLowAlert.ClearHumidity!=LowAlert.ClearHumidity){  // low humidity clear point changed
      if(ModbusLowAlert.ClearHumidity>=0&&ModbusLowAlert.ClearHumidity<=100){ // Sanity check
        LowAlert.ClearHumidity = ModbusLowAlert.ClearHumidity; // Update Low Humidity Alert Setpoint
        ShouldUpdateModbus = true;}}// Notify Holding Registers Changed
  if(ShouldUpdateModbus){    // if any alert values changed, update the physical sensor. 
    sht31.setHighAlert(&highAlert); // write high temperature/humidity values to the SHT31
    sht31.setLowAlert(&LowAlert); // write the Low temperature/humidity values to the SHT31
    ShouldUpdateModbus = false; // only update when alert values change 
  }
  #endif

// Read SHT31 sensor and update temperature and humidity. 
  float t, h;
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

  // Set output pins 
  digitalWrite(_HIGH_HEAT_PIN, sht31.HighTempActive());
  digitalWrite(_LOW_HEAT_PIN, sht31.LowTempActive());
  digitalWrite(_HIGH_HUMIDITY_PIN,sht31.HighHumidityActive());
  digitalWrite(_LOW_HUMIDITY_PIN, sht31.LowHumidityActive());

}
