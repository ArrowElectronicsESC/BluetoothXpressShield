/************************************************************************
File: BluetoothExpress.cpp
Version: V1.0
Release Date: February 4, 2019
Authors: Eric Beppler, Andrew Reisdorph, Arrow Electronics

Description: This file contains the definitions for the BGX13 class.
This class is designed to interface with the BGX13S From Silicon Labs 
The functions are base on the command set provided with the BGX13

Details on the function of the commands can be found in the BGX13 documentation



Contact ESC@Arrow.com for all inquiries with the subject SLASH1000


This file is distributed by Arrow Electronics under the MIT License. 
See License.txt for more details.
************************************************************************/
#include "Arduino.h"
#include "BluetoothExpress.h"

//Variable Includes to allow for MKR and Uno operation
#if defined(ARDUINO_AVR_UNO)
  #include <SoftwareSerial.h>
#else
#endif

/*Overloaded constructor prevents compile errors 
when using the MKR and no fotware serial library */
#if defined(ARDUINO_AVR_UNO)
BGX13::BGX13(SoftwareSerial *arduinoSoftSerial){
	_state = 0;
  _bgxSerial = arduinoSoftSerial;
  pinMode(BUS_MODE_PIN, OUTPUT);
  _swSerial = true;
  SET_COMMAND_MODE(); //Sets command mode
}
#endif

BGX13::BGX13(HardwareSerial  *arduinoHardwareSerial){
  _state = 0;
  _bgxSerial = arduinoHardwareSerial;
  pinMode(BUS_MODE_PIN, OUTPUT);
  _swSerial = false;
  SET_COMMAND_MODE(); //Sets command mode
}

void BGX13::serialBegin(long baud) {
  if(_swSerial){
  #if defined(ARDUINO_AVR_UNO)
    static_cast<SoftwareSerial*>(_bgxSerial)->begin(baud);
    static_cast<SoftwareSerial*>(_bgxSerial)->listen();
  #endif
  } else {
    static_cast<HardwareSerial*>(_bgxSerial)->begin(baud);
  }

  while (!_bgxSerial) {
    ; // wait for serial port to connect.
  }
}


void BGX13::serialConnect(long baud){


  serialBegin(baud);
  delay(5);
  //_bgxSerial->println("clrb");
  factoryReset();
  delay(500);
  SET_COMMAND_MODE();
  selectGPIOFunction(4, "str_active");
  setVariable("bu", "s c level");
  selectGPIOFunction(0, "str_select");
  gpioSetOut(0, OUTPUT_DIRECTION_LOW);
  selectGPIOFunction(2, "stdio"); 
  gpioSetIn(2, INPUT_DIRECTION_IN_PULLUP);  
  selectGPIOFunction(3, "stdio");
  gpioSetIn(3, INPUT_DIRECTION_IN_PULLUP);
  selectGPIOFunction(7, "stdio");
  selectGPIOFunction(6, "stdio");
  gpioSetIn(7, INPUT_DIRECTION_IN);
  gpioSetIn(6, INPUT_DIRECTION_IN);
  
  RESET_UART_RX; 
  delay(10);
  SET_STREAM_MODE(); //Sets stream mode
  delay(5);
}

void BGX13::waitForStreamMode(void) {
  // 
  int char_iter;
  int matchCount = 0;
  for (char_iter = 0; (char_iter < _uart_rx_write_ptr) && (char_iter < strlen(stream)); char_iter++) {
    if (_uart_rx_buffer[char_iter] == stream[char_iter]) {
      matchCount++;
    }
  }
  if (matchCount > 4) {
    _state = RUNNING;
    _bgxSerial->println("Streaming");
  }
  if (_uart_rx_write_ptr > 0) {
    _bgxSerial->println(_uart_rx_buffer);
  }
  _uart_rx_write_ptr = 0;
}

int BGX13::BGXRead(void){
  int byte_counter;
  delay(READ_DELAY);
  int bytes_available = _bgxSerial->available();
  
  //for (byte_counter = 0; byte_counter < bytes_available; byte_counter++) {
  //  _uart_rx_buffer[_uart_rx_write_ptr++]=_bgxSerial->read();
  //}

  char next_char = _bgxSerial->read();
  while (next_char != -1 && _uart_rx_write_ptr < UART_BUFFER_SIZE) {
    _uart_rx_buffer[_uart_rx_write_ptr++] = next_char;
    next_char = _bgxSerial->read();
    //delay(10);
  }

  _uart_rx_buffer[_uart_rx_write_ptr] = NULL;

  return bytes_available;
}

//MUST HAVE SERIAL INITIALIZED PRIOR TO USE _ FOR DEBUG ONLY
int BGX13::printBGXBuffer(void){
  if(_uart_rx_write_ptr > 0){
    Serial.println(_uart_rx_buffer);
    _uart_rx_write_ptr=0;
  }
  return 0;
  //gets and pushes _uart_rx_buffer to the UART
}

int BGX13::getBGXBuffer(char* data){
  int count = BGXRead();
  strcpy(data, _uart_rx_buffer);
  _uart_rx_write_ptr = 0;
  return count;
  //gets and pushes _uart_rx_buffer to passed string
}



void BGX13::sendCommand(int readTime = 0) {
  SET_COMMAND_MODE(); //Sets COMMAND mode
  BGXRead();
  _uart_rx_write_ptr=0;
  _bgxSerial->println(_uart_tx_buffer);
  RESET_UART_RX;

  unsigned long start = millis();
  do
  {
    BGXRead();
  }while ((millis() - start) < readTime);
  //BGXRead(); //Eat the output

  
  SET_STREAM_MODE(); //Sets stream mode
}

void BGX13::sendString(char* toSend) {
  SET_STREAM_MODE(); //Sets COMMAND mode
  BGXRead();
  _uart_rx_write_ptr = 0;
  _bgxSerial->println(toSend);
  RESET_UART_RX;
  BGXRead(); //Eat the output
}

void BGX13::advertiseHigh(void){
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, ADVERTISE_HIGH);
  sendCommand();
}

void BGX13::advertiseLow(void){
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, ADVERTISE_LOW);
  sendCommand();
}

void BGX13::advertiseOff(void){
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, ADVERTISE_OFF);
  sendCommand();
}


void BGX13::clearBondingInfo(void) {
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, CMD_CLEAR_BONDING_INFO);
  sendCommand();
}

void BGX13::disconnect(void){
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, DISCONNECT);
  sendCommand();
}

void BGX13::factoryReset(void) {
  SET_COMMAND_MODE(); //Sets COMMAND mode
  BGXRead();
  _uart_rx_write_ptr = 0;
  _bgxSerial->println(GET_BT_ADDR);
  BGXRead();
  _uart_rx_buffer[ADDRESS_END_INDEX] = NULL;
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, FACTORY_RESET "%s", &_uart_rx_buffer[ADDRESS_INDEX]);
  _bgxSerial->println(_uart_tx_buffer);
  BGXRead();
  SET_STREAM_MODE(); //Sets stream mode
}

/* Directions are as follows:
   "in", //0
    "ipd", //1
    "ipu", //2
    "inw", //3
    "ipuw", //4
    "ipdw" //5
*/
//TODO(andrew): Test this since it doesn't use std::String anymore
void BGX13::gpioSetIn(int number, BGX_input_direction direction, int debounce) {
  //String command = "gdi ";
  if (direction > 5) {
    return;
  }
  int charsAdded = snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, "gdi %d %s", number, inDirections[direction]);
  //command += number;
  //command += SPACE;
  //command += inDirections[direction];

  if (debounce >= 0 && debounce < 10) {
    //command += " db";
    //command += debounce;
    snprintf(_uart_tx_buffer + charsAdded, (UART_BUFFER_SIZE - charsAdded), "db %d", debounce);
  }

  //snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, command.c_str());
  sendCommand();
}

/* Directions are as follows:
    "olo", //0
    "ohi", //1
    "hiz"  //2

    Push Pull Mode is as follows:
    "pp", //0
    "od", //1
    "os"  //2

    Drive Strenght is as follows:
    "drvst", //0
    "drvwk" //1
*/
void BGX13::gpioSetOut(int number, BGX_output_direction direction, int mode, bool enablePullResistor, int driveStrength){
  String command = "gdi ";
  if(direction > 2) return;
  if(mode > 2) return;
  if(driveStrength > 1) return;
  if(mode > 0 && direction == 2) return;
  command += number;
  command += SPACE;
  command += outDirections[direction];
  //command += SPACE;
  //command += pushPullModes[mode];
  if (mode == 1 || mode == 2) {
    command += SPACE;
    if (enablePullResistor) {
      command += "pren";
    } else {
      command += "prdi";
    }
    command += SPACE;
    command += driveStrengths[driveStrength];
  }

  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, command.c_str());
  sendCommand();
}

void BGX13::reboot(void){
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, REBOOT);
  sendCommand();
}

void BGX13::saveConfiguration(void){
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, SAVE);
  sendCommand();
}

void BGX13::scan(int timeScan ){
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, SCAN);
  sendCommand(timeScan);
}

void BGX13::sleepMode(void){
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, SLEEP);
  sendCommand();
}

void BGX13::streamMode(void){
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, STREAM_MODE);
  sendCommand();
}

void BGX13::updateUartSettings(void){
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, UPDATE_UART_SETTINGS);
  sendCommand();
}

void BGX13::userFunctionAssign(int number, char* function){
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, "ufu %d %s", number, function);
  sendCommand();
}

void BGX13::getLastUserFunctionResult(){
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, GET_LAST_USER_FUNC_STATUS);
  sendCommand();
}

void BGX13::userFunctionRun(int number) {
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, "urun %d", number);
  sendCommand();
}

void BGX13::getVersion(char *ver){
  SET_COMMAND_MODE(); //Sets COMMAND mode
  BGXRead(); //needed to ensure the buffer is clear
  _uart_rx_write_ptr = 0;
  _bgxSerial->println("ver");
  BGXRead();
  strcpy(ver, &_uart_rx_buffer[VERSION_BEGIN]);
  SET_STREAM_MODE(); //Sets stream mode
}

void BGX13::wake(void){
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, WAKE);
  sendCommand(); 
}

void BGX13::getVariable(char *variable, char *buffer, int length) {
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, "get %s", variable);
  sendCommand();
  BGXRead();
  strncpy(buffer, _uart_rx_buffer, length);
}

void BGX13::selectGPIOFunction(int gpioNumber, char *function) {
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, "gfu %d %s", gpioNumber, function);
  sendCommand();
}

void BGX13::setVariable(char *variable, char *name) {
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, "set %s %s", variable, name);
  sendCommand();
}

int BGX13::getGPIOValue(int number) {
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, "gge %d", number);
  sendCommand();
  BGXRead();
  // Index 7 is the index of the return value of the gge function after the bgx echos the function name
  return (_uart_rx_buffer[7] == '1');
}

void BGX13::setGPIOValue(int number, int value) {
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, "gse %d %d", number, value);
  sendCommand();
}

void BGX13::userEventTrigger(int functionNumber, bool assertHigh, int gpioNumber) {
  snprintf(_uart_tx_buffer, UART_BUFFER_SIZE, "uevt %d %s %d", functionNumber, (assertHigh ? "hi" : "lo"), gpioNumber);
  sendCommand();
}

BGX13::~BGX13(void){
  __asm__("nop");
}