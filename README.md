# M0ClimateControl
 Modbus over TCP based Climate Controller

![M0ModbusClimateControl_bb](https://github.com/cgriffis46/M0ClimateControl/assets/78368880/0062478d-ee3b-47bf-926c-0e5941a846f0)


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
      Need to implement SHT31 Interrupt function 
      Authenticate Modbus clients 
      Implement temperature unit

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
      
DIGITAL OUTPUTS: 
  	A0 - High Temperature Alert
  	A1 - Low Temperature Alert
  	A2 - High Humidity Alert
  	A3 - Low Humidity Alert
