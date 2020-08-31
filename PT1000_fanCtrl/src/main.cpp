#include <Arduino.h>

#ifndef ADAFRUIT_MAX31865_LIBRARY
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
  #define ADAFRUIT_MAX31865_LIBRARY   0
#endif //ADAFRUIT_MAX31865_LIBRARY

#ifndef SPI_CUSTOM_CODE
    #define SPI_CUSTOM_CODE             1
#endif //SPI_CUSTOM_CODE

#if ADAFRUIT_MAX31865_LIBRARY
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
#endif //ADAFRUIT_MAX31865_LIBRARY

#if SPI_CUSTOM_CODE
  #include <SPI.h>
  #define CS_PIN      10
  #define MAX31856_CONFIG_REG     0x00
  #define MAX31856_CONFIG_3WIRE   0x10
  typedef enum max31865_numwires {
    MAX31865_2WIRE = 0,
    MAX31865_3WIRE = 1,
    MAX31865_4WIRE = 0
  } max31865_numwires_t;
  void setWires(max31865_numwires_t);
  uint8_t readRegister8(uint8_t addr);
//    void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);
  void writeRegister8(uint8_t addr, uint8_t reg);
#endif //SPI_CUSTOM_CODE

void setup() {
  Serial.begin(115200);
  #if ADAFRUIT_MAX31865_LIBRARY
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
  #endif //ADAFRUIT_MAX31865_LIBRARY
  #if SPI_CUSTOM_CODE
    //Initialize the SPI Communication
    SPI.begin();
    //Set clock for SPI communication below 2MHz(72/64 = 4.5Mhz)
    SPI.setClockDivider(SPI_CLOCK_DIV64);
    //Set the data mode to mode 1
    SPI.setDataMode(SPI_MODE1);
    // Set CS_PIN as Output
    pinMode(CS_PIN, OUTPUT);
    //Set the CS_PIN as HIGH(So Master does not connect with slave)
    digitalWrite(SS, HIGH);
    setWires(MAX31865_4WIRE);
  #endif //SPI_CUSTOM_CODE
}


void loop() {
    #if ADAFRUIT_MAX31865_LIBRARY
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
              delay(500);
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
    #endif //ADAFRUIT_MAX31865_LIBRARY
}

#if SPI_CUSTOM_CODE
  void setWires(max31865_numwires_t wires) 
  {
    uint8_t t = readRegister8(MAX31856_CONFIG_REG);
    if (wires == MAX31865_3WIRE) {
      t |= MAX31856_CONFIG_3WIRE;
    } else {
      // 2 or 4 wire
      t &= ~MAX31856_CONFIG_3WIRE;
    }
    writeRegister8(MAX31856_CONFIG_REG, t);
  }
  uint8_t readRegister8(uint8_t addr)
  {
    addr &= 0x7F; // make sure top bit is not set
    digitalWrite(CS_PIN, LOW);
    uint8_t ret = SPI.transfer(addr);
    digitalWrite(CS_PIN, HIGH);
//      readRegisterN(addr, &ret, 1);
  
    return ret;    
  }
  void writeRegister8(uint8_t addr, uint8_t data)
  {
    addr |= 0x80; // make sure top bit is set
  
    uint8_t buffer[2] = {addr, data};
    digitalWrite(CS_PIN, LOW);
    SPI.write(buffer, 2); 
    digitalWrite(CS_PIN, HIGH);           
  }
//    void readRegisterN(uint8_t addr, uint8_t buffer[],uint8_t n) 
//    {
//      addr &= 0x7F; // make sure top bit is not set
//    
//      spi_dev.write_then_read(&addr, 1, buffer, n);
//    }
#endif //SPI_CUSTOM_CODE