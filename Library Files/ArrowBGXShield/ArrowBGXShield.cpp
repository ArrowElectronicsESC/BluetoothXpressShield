/************************************************************************
File: ArrowBGXShield.cpp
Version: V1.0
Release Date: February 4, 2019
Authors: Eric Beppler, Andrew Reisdorph, Arrow Electronics

Description: This file defines the classes and member functions for
the sensor and control systems of the SLASH1000. These systems
are detailed in the SLASH1000 User Guide. 

Three classes are defined: PCA9633, SI7050, VEML6030
Fucntions are provided for each class to initialize and control
the related chip and subsystem.


Contact ESC@Arrow.com for all inquiries with the subject SLASH1000


This file is distributed by ARrow Electronics under the MIT License. 
See License.txt for more details.
************************************************************************/

#include "Arduino.h"
#include "ArrowBGXShield.h"
#include <Wire.h>

//Vairable includes to allow for UNO and MKR operation
#if defined(ARDUINO_AVR_UNO)
	#include "SoftwareSerial.h"
#else
  #define TWCR 0
#endif

/**********************************************
* Class: PCA9633
* Params: 
*	uint8_t adr: 7 bit I2C address of the PCA9633
* Description:
*	This class controls the configuration and operation
* 	of the 0-10V lighting output, onboard LEDs, and 
* 	PWM lighting Controller.
* Returns: 
*	PCA9633 Object
***********************************************/ 
PCA9633::PCA9633(uint8_t adr){
	_adr = adr; //Set the address for the device; 0x60 is the default 7bit address

}

/**********************************************
* Function: _I2CWrite
* Params: 
*	uint8_t command: The command register to write the data 
*	to on the PCA9633
*	uint8_t data: The data to be written to the register specified by command 
* Description:
*	This is a helper function to faciliatte writes to the PCA9633 command registers. 
* Returns: void
***********************************************/ 
void PCA9633::_I2CWrite(uint8_t commmand, uint8_t data){
	Wire.beginTransmission(_adr);
	Wire.write(commmand); // control register
	Wire.write(data);  //  send byte data
	Wire.endTransmission();
}

/**********************************************
* Function: _I2CRead
* Params:
*	uint8_t command: The PCA9633 register to read from
* Description:
*	this is a helper function to retrive the value stored in the PCA9633 command registers
* Returns: 
*	uint8_t: the 8 bit data contained in the specified register
***********************************************/ 
uint8_t PCA9633::_I2CRead(uint8_t commmand){
	uint8_t readData;
	Wire.beginTransmission(_adr);
	Wire.write(commmand); // control register
	Wire.endTransmission();

	Wire.requestFrom(_adr, SINGLE_READ, SEND_STOP); // request one byte

	readData = Wire.read();
	return readData;
}

/**********************************************
* Function: init
* Params: void
* Description:
*	This function initializes the PCA9633 to allow for 
*	0-10V outputs on four channels. Individual control, 
*	a totem config, and inverted PWM singal are slected.
* Returns: void
***********************************************/ 
void PCA9633::init(void){
	uint8_t settings = NO_AUTO_INC | SLEEP_DISABLE | ALT_ADDR_DISABLE;
	_I2CWrite(MODE1, settings);
	settings = GROUP_DIMMING | INVRT | CHANGE_ON_ACK | TOTEM_CONFIG;
	_I2CWrite(MODE2, settings);
	settings = LED_INDV_GRP(LED0) | LED_INDV_GRP(LED1) | LED_INDV_GRP(LED2) | LED_INDV_GRP(LED3); 
	_I2CWrite(LEDOUT, settings);
}
/**********************************************
* Function: onBoardMode
* Params: void
* Description:
*	This function sets the PWM signal to non 
*	inverted so that the 0-10V outputs function 
*	properly. the onboard LEDs will not function 
*	with this setting.
* Returns: void
***********************************************/ 
void PCA9633::onBoardMode(void){
	uint8_t settings = GROUP_DIMMING | NON_INVRT | CHANGE_ON_ACK | TOTEM_CONFIG;
	_I2CWrite(MODE2, settings);
}
/**********************************************
* Function: outputMode
* Params: void
* Description:
* This function inverts the PWM signal such 
*	that the onboard LEDs can be used. These 
*	LEDs are sinking. The 0-10V outputs will 
*	not function properly under this configuration
* Returns: void
***********************************************/ 
void PCA9633::outputMode(void){
	uint8_t settings = GROUP_DIMMING | INVRT | CHANGE_ON_ACK | TOTEM_CONFIG;
	_I2CWrite(MODE2, settings);
}
/**********************************************
* Function: rgbwControl
* Params: 
*	uint8_t red: The new intensity value for the
*	 red channel (0 to 255MAX)
*	uint8_t green: The new intensity value for 
*	the green channel (0 to 255MAX)
*	uint8_t blue: The new intensity value for 
*	the blue channel (0 to 255MAX)
*	uint8_t white:  The new intensity value for
*	 the white channel (0 to 255MAX)
* Description:
*	This function writes new values to all 4 
*	color channels of the I2C PWM controller. 
*	All values are immediatly updated and can 
*	range from 0 (off) to 255 (Max).
* Returns: void
***********************************************/ 
void PCA9633::rgbwControl(uint8_t red, uint8_t green, uint8_t blue, uint8_t white ){
	_I2CWrite(PWM0, white);
	_I2CWrite(PWM1, blue);
	_I2CWrite(PWM2, red);
	_I2CWrite(PWM3, green);
	uint8_t	settings = LED_INDV_GRP(LED0) | LED_INDV_GRP(LED1) | LED_INDV_GRP(LED2) | LED_INDV_GRP(LED3); 
	_I2CWrite(LEDOUT, settings);
	return;
}

/**********************************************
* Class: SI7050
* Params: 
*	uint8_t adr: 7 bit I2C address of the SI7050
* Description:
*	This class controls the configuration and operation
* 	of the temperature sensor.
* Returns: 
*	SI7050 Object
***********************************************/ 
SI7050::SI7050(uint8_t adr){
	_adr = adr; //Set the address for the device 0x40 is the default 7bit address

}
/**********************************************
* Function: startTempMeasure
* Params: void
* Description: This function starts the 
*	temperature conversion process so that a 
*	later function call can retrieve the value. 
*	This must be called prior to calling any 
*	'get' function. 
* Returns: void
***********************************************/ 
void SI7050::startTempMeasure(void){
    Wire.beginTransmission(_adr);
    // Send temperature measurement command, NO HOLD MASTER
    Wire.write(START_TEMP);
    // Stop I2C transmission
    Wire.endTransmission();
}
/**********************************************
* Function: getTempMeasureC
* Params: void
* Description:
* 	this function reads the temperature value 
*	from the SI7050 and converts the data to 
*	Celcius. This function must be called 20ms 
*	or more after a 'startTempMeasure'
* Returns: 
*	float: The temperture reading converted to 
*	degrees celcius
***********************************************/ 
float SI7050::getTempMeasureC(void){
	unsigned int data[2] = {};
    Wire.requestFrom(_adr, 2);
    // Read 2 bytes of data
    // temp msb, temp lsb
    if(Wire.available() == 2)
    {
      data[0] = Wire.read();
      data[1] = Wire.read();
    }
    float temp  = ((data[0] * 256.0) + data[1]);
    float ctemp = ((175.72 * temp) / 65536.0) - 46.85;
    return ctemp;
}
/**********************************************
* Function: getTempMeasureF
* Params: void
* Description:
* 	this function reads the temperature value 
*	from the SI7050 and converts the data to 
*	Farenhiet. This function must be called 20ms 
*	or more after a 'startTempMeasure'
* Returns: 
*	float: The temperture reading converted to 
*	degrees F
***********************************************/ 
float SI7050::getTempMeasureF(void){
	unsigned int data[2];
    Wire.requestFrom(_adr, 2);
    // Read 2 bytes of data
    // temp msb, temp lsb
    if(Wire.available() == 2)
    {
      data[0] = Wire.read();
      data[1] = Wire.read();
    }
    float temp  = ((data[0] * 256.0) + data[1]);
    float ctemp = ((175.72 * temp) / 65536.0) - 46.85;
    float ftemp = ctemp * 1.8 + 32;
    return ftemp;
}


/**********************************************
* Class: VEML6030
* Params: 
*	uint8_t adr: 7 bit I2C address of the VEML6030
* Description:
*	This class controls the configuration and operation
* 	of the ambient light sensor.
* Returns: 
*	VEML6030 Object
***********************************************/ 
VEML6030::VEML6030(uint8_t adr){
	_adr = adr; //Set the address for the device 0x10 is the default 7bit address
}
/**********************************************
* Function: defaultConfig
* Params: void
* Description:
* 	This function sets the VEML6030 to its 
*	default configuration, allowing for easy
*	reading of light values
* Returns: void
***********************************************/ 
void VEML6030::defaultConfig(void){
	writeCommand(0, 0x0000);
	writeCommand(1, 0x0000);
	writeCommand(2, 0x0000);

}

/**********************************************
* Function: getALS
* Params: void
* Description:
* Returns: 
*	uint16_t:
***********************************************/ 
uint16_t VEML6030::getALS(void){
	float luxCorrectionFactor [4] = {0.0576,0.0288,0.4608,0.2304};
	float integrationFactor [6] = {1.0,0.5,0.25,0.125,2.0,4.0};
	uint16_t conversion =  readData(CONF_REF);
	uint16_t gain = ((conversion && 0x1800) >> 11);
	uint16_t integration = ((conversion && 0x03C0) >> 6);
	integration = (integration > 0x4) ? (integration >> 1) : integration;
	return readData(ALS_REG) * luxCorrectionFactor[gain] * integrationFactor[integration];
}
/**********************************************
* Function: getWhite
* Params: void
* Description:	
*	This function read the VEML6030 register 
*	containing white light data and returns 
*	the un-modified values
* Returns: 
*	uint16_t: The unscaled white light sensor 
*	reading provided by the VEML6030. This 
*	must be converted to a lux value. See
*	 the VEML6030 Datasheet for details.
***********************************************/ 
uint16_t VEML6030::getWhite(void){
	return readData(WHITE_REG);
}
/**********************************************
* Function: writeCommand
* Params:
*	uint8_t reg: The register of the VEML6030 to write to
*	uint16_t data: The data to be written to the specified register. MSB Big Endian/
* Description:
*	This function writes the provided data to a single register of the VEML6030
* Returns:  void
***********************************************/ 
void VEML6030::writeCommand(uint8_t reg, uint16_t data){
	Wire.beginTransmission(_adr);
    Wire.write(reg);
    Wire.write( (uint8_t) (data & 0xFF));
    Wire.write( (uint8_t) ((data >> 8) & 0xFF) );
    Wire.endTransmission(); 
}
/**********************************************
* Function: readData
* Params:
* 	uint8_t reg: The register of the VEML6030 to read
* Description:
*	This function reads the data from a single register of the VEML6030 
* Returns: 
*	uint16_t : the 16 bit data in MSB, BIG Endian format contained int the requested register
***********************************************/ 
uint16_t VEML6030::readData(uint8_t reg){
	uint8_t byteLow = 0;
	uint8_t byteHigh = 0;
	Wire.beginTransmission(_adr);
    Wire.write(reg);
    Wire.endTransmission(false);     delay(10);

    Wire.requestFrom(_adr, 2);
    if (Wire.available() == 2){
    	byteLow = Wire.read();  //Read in high and low bytes (big endian)
		byteHigh = Wire.read();
    }

	return (((uint16_t)byteHigh << 8) | byteLow);
}