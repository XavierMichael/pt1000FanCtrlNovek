/*************************************************** 
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Adafruit_MAX31865.h>

// Use software SPI: CS, DI, DO, CLK
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF                4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL            1000.0
#define TEMP_THRESHOLD      25.0
#define RELAY_PIN           5
void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");

  /**************************************************************************/
/*!
    @brief Initialize the SPI interface and set the number of RTD wires used
    @param wires The number of wires in enum format. Can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @return True
*/
/**************************************************************************/
  thermo.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary
  pinMode(RELAY_PIN, OUTPUT);
}


void loop() {
/**************************************************************************/
/*!
    @brief Read the temperature in degrees Celsius from the RTD through calculation of 
    the resistance. Uses
   http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
   technique
    @param RTDnominal The 'nominal' resistance of the RTD sensor, usually 100
    or 1000
    @param refResistor The value of the matching reference resistor, usually
    430 or 4300
    @returns Temperature in C
*/
/**************************************************************************/
  float temp = thermo.temperature(RNOMINAL, RREF);
  
  if(temp > TEMP_THRESHOLD)
  {
    for(int i=0; i<3; i++)
    {
        temp = thermo.temperature(RNOMINAL, RREF);
        delay(500)
    }
    if(temp> TEMP_THRESHOLD)
    {
        digitalWrite(RELAY_PIN, HIGH);
    }
  } 
  else
  {
    digitalWrite(RELAY_PIN, LOW);
  }

}
