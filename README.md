# modbus-ttl-485
## Wire

UNO to TTL convert

DI <--> 8 as TX
SO <--> 9 as RX
GND <--> GND
VCC <--> 5V

TTL convert to 485 device
A <--> A
B <--> B

## Libs
SensorModbusMaster at https://github.com/EnviroDIY/SensorModbusMaster

SoftwareSerial for UNO as the serial port which is connecting to 485 device.


