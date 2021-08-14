/*****************************************************************************
GetValues.ino

This example writes a setting value to a holding register, reads it to confirm
the value has changed, and then reads several data values from holding registers.

The register numbers in this example happen to be for an S::CAN oxy::lyser.
*****************************************************************************/

// ---------------------------------------------------------------------------
// Include the base required libraries
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <SensorModbusMaster.h>

// ---------------------------------------------------------------------------
// Set up the sensor specific information
//   ie, pin locations, addresses, calibrations and related settings
// ---------------------------------------------------------------------------

// Define the sensor's modbus address
byte modbusAddress = 0x64;   // The sensor's modbus address, or SlaveID
long modbusBaudRate = 9600; // The baud rate the sensor uses

// Define pin number variables
const int sensorPwrPin = -1;  // The pin sending power to the sensor
const int adapterPwrPin = -1; // The pin sending power to the RS485 adapter
const int DEREPin = 3;       // The pin controlling Recieve Enable and Driver Enable
                              // on the RS485 adapter, if applicable (else, -1)
                              // Setting HIGH enables the driver (arduino) to send text
                              // Setting LOW enables the receiver (sensor) to send text

// Construct software serial object for Modbus
#if defined(ARDUINO_AVR_UNO)
// The Uno only has 1 hardware serial port, which is dedicated to comunication with the computer
// If using an Uno, you will be restricted to using AltSofSerial or SoftwareSerial
#include <SoftwareSerial.h>
const int SSRxPin = 9; // Recieve pin for software serial (Rx on RS485 adapter)
const int SSTxPin = 8; // Send pin for software serial (Tx on RS485 adapter)
SoftwareSerial modbusSerial(SSRxPin, SSTxPin);
#else
// This is just a assigning another name to the same port, for convienence
// Unless it is unavailable, always prefer hardware serial.
HardwareSerial* modbusSerial = &Serial1;
#endif

// Construct the modbus instance
modbusMaster modbus;

// ---------------------------------------------------------------------------
// Main setup function
// ---------------------------------------------------------------------------
void setup()
{
    // Set various pins as needed
    if (DEREPin >= 0)
    {
        pinMode(DEREPin, OUTPUT);
    }
    if (sensorPwrPin >= 0)
    {
        pinMode(sensorPwrPin, OUTPUT);
        digitalWrite(sensorPwrPin, HIGH);
    }
    if (adapterPwrPin >= 0)
    {
        pinMode(adapterPwrPin, OUTPUT);
        digitalWrite(adapterPwrPin, HIGH);
    }

    // Turn on the "main" serial port for debugging via USB Serial Monitor
    Serial.begin(57600);

    // Turn on your modbus serial port
#if defined(ARDUINO_AVR_UNO)
    modbusSerial.begin(modbusBaudRate);
    // NOTE:  Software serial only supports 8N1
#else
    //Serial1.begin(modbusBaudRate, SERIAL_8O1);
    // ^^ use this for 8 data bits - odd parity - 1 stop bit
    // Serial1.begin(modbusBaudRate, SERIAL_8E1);
    // ^^ use this for 8 data bits - even parity - 1 stop bit
    // Serial1.begin(modbusBaudRate, SERIAL_8N2);
    // ^^ use this for 8 data bits - no parity - 2 stop bits
     Serial1.begin(modbusBaudRate);
    // ^^ use this for 8 data bits - no parity - 1 stop bits
    // Despite being technically "non-compliant" with the modbus specifications
    // 8N1 parity is very common.
#endif

    // Turn on debugging, if desired
//     modbus.setDebugStream(&Serial);

    // Start the modbus instance
    modbus.begin(modbusAddress, modbusSerial, DEREPin);
}

// ---------------------------------------------------------------------------
// Main setup function
// ---------------------------------------------------------------------------
void loop()
{
    // Get data values from read-only input registers (0x04)
    // Just for show, we will do the exact same thing 2 ways
    // All values will be read as bigEndian

    // Some variables to hold results
    uint16_t spec = 0;
    int16_t type = 0;
    uint16_t curr = 0;

    // Method 1:
    // Get three values one at a time from 3 different holding registers.
    // This code is easier to follow, but it requires more back-and-forth between
    // the Arduino and the sensor so it is a little "slower".
    spec = modbus.uint16FromRegister(0x03, 0x200, bigEndian);
    type = modbus.uint16FromRegister(0x03, 0x201, bigEndian);
    curr = modbus.uint16FromRegister(0x03, 0x202, bigEndian);

    // Print results
    Serial.print("Device Spec:");
    Serial.println(spec);
    Serial.print("Device Type:");
    Serial.println(type);
    Serial.print("Current in A:");
    Serial.println(curr);
    Serial.println();
}
