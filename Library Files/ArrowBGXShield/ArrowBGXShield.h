/************************************************************************
File: ArrowBGXShield.h
Version: V1.0
Release Date: February 4, 2019
Authors: Eric Beppler, Andrew Reisdorph, Arrow Electronics

Description: This file declares the classes and member functions for
the sensor and control systems of the SLASH1000. These systems
are detailed in the SLASH1000 User Guide. 

Three classes are declared: PCA9633, SI7050, VEML6030
Fucntions are provided for each class to initialize and control
the related chip and subsystem.


Contact ESC@Arrow.com for all inquiries with the subject SLASH1000


This file is distributed by ARrow Electronics under the MIT License. 
See License.txt for more details.
************************************************************************/

#ifndef ArrowBGX_h
#define ArrowBGX_h

#include "Arduino.h"
#include "ArrowBGXShieldDefines.h"


#if defined(ARDUINO_AVR_UNO)
  #include "SoftwareSerial.h"
#else
  #define TWCR 0
#endif


class SI7050
{
  public:
    SI7050(uint8_t adr);
    void startTempMeasure(void);
    float getTempMeasureC(void);
    float getTempMeasureF(void);
  private:
    int _adr;
};


class PCA9633
{
  public:
    PCA9633(uint8_t adr);
    void init(void);
    void onBoardMode(void);
    void outputMode(void);
    void rgbwControl(uint8_t red, uint8_t green, uint8_t blue, uint8_t white );
    //pwmcontrol
  private:
    int _adr;
    void _I2CWrite(uint8_t commmand, uint8_t data);
    uint8_t _I2CRead(uint8_t command);
};


class VEML6030
{
  public:
    VEML6030(uint8_t adr);
    void init(void);
    void defaultConfig(void);
    uint16_t getALS(void);
    uint16_t getWhite(void);
  private:
    int _adr;
    void writeCommand(uint8_t reg, uint16_t data);
    uint16_t readData(uint8_t reg);
};

#endif