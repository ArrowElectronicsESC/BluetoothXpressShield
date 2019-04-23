
//#define ARDUINO_UNO
#include "Arduino.h"
#include <ArrowBGXShield.h>
#include <BluetoothExpress.h>
#include <Wire.h>

#define OUTPUT_MODE 0  //Set to 1 to enable 0-10V output, 0 to use onboard lights
#define ARD_ID '0' //ID per arduino
#define SWELL 1 //Used as demo mode before connection

#define SERIAL_INPUT_BUFFER_SIZE       32
#define PRINT_BUFFER_SIZE              124
#define GET_ARDUINO_ID_COMMAND_LENGTH    1
#define LIGHT_CHANGE_COMMAND_LENGTH      7
#define GET_ALL_SENSORS_COMMAND_LENGTH   1
#define GET_TEMPERATURE_COMMAND_LENGTH   1
#define GET_LUX_COMMAND_LENGTH           1

enum ErrorCode {
  SUCCESS = 0,
    BAD_COMMAND_LENGTH,
    WRONG_ARDUINO_ID,
    BAD_CHECKSUM
  };

union LedColor {
  struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t white;
  };
  uint8_t all[4];
};

struct RgbaiColorCommand {
  unsigned char command_id;
  unsigned char arduino_id;
  LedColor color;
  unsigned char checksum;
};

int getSerialInput(unsigned char *buffer, int size, int timeout);
ErrorCode handleCommand(unsigned char *buffer, int size, bool bt);
void readSensors(float *temperature, float *ambient_light);
void updateSwell(uint32_t swell_index);
LedColor normalizeColor(LedColor color);
void setLightColor(LedColor color);
void convertBufferAtoI(unsigned char *);

//Conditional for different arduino versions
#if defined(ARDUINO_AVR_UNO)
#include "SoftwareSerial.h"
SoftwareSerial bgxSerial(4, 5);
BGX13 bgx(&bgxSerial);
#else
//HardwareSerial * bgxSerial = (HardwareSerial*)&Serial1;
BGX13 bgx( &Serial1 );
#endif

//Object declaration
PCA9633 *dimmer = NULL;
SI7050 *temp = NULL;
VEML6030 *lux = NULL;

char print_buffer[124];
char serial_input_buffer[SERIAL_INPUT_BUFFER_SIZE] = {0};
char bluetooth_input_buffer[UART_BUFFER_SIZE] = {0};
bool host_connected = false;
uint8_t swell[] = {0, 0, 0, 0, 0, 1, 5, 10, 20, 50, 90, 150, 255, 150, 90, 50, 20, 10, 5, 1, 0, 0, 0, 0}; //Used to test lights with a "chase effect" - Also used for the init section
#define SWELL_COUNT 24
#define SWELL_OFFSET (SWELL_COUNT/4)
uint32_t swell_count = 0; //Index to use for chase math




void setup() {
  //Initialize Arduino Fucntions
  Serial.println("BGX Ready");
  Wire.begin();
  Serial.begin(115200);

  bgx.serialConnect(115200);
  Serial.println("BGX Ready");
  temp = new SI7050(0x40);

  dimmer = new PCA9633(0x60);
  dimmer->init();
  if(OUTPUT_MODE){
    dimmer->outputMode();
  } else{
    dimmer->onBoardMode();
  }
  Serial.println("Dimmer Ready");

  lux = new VEML6030(0x10);
  lux->defaultConfig(); //Initialize Lux Sensor
  Serial.println("LUX Sensor Ready");


  Serial.println("Temp Sensor Ready"); //Temp is auto -Initialized

  delay(100); //Let all init periods expire

  Serial.println("System Ready");
}

void loop() {
  ErrorCode status = SUCCESS;
  int num_bytes_received = getSerialInput(serial_input_buffer, SERIAL_INPUT_BUFFER_SIZE, 100);
  int bt_bytes_recieved = bgx.getBGXBuffer(&bluetooth_input_buffer[0]);
  if(bt_bytes_recieved){
    Serial.println(bluetooth_input_buffer);
  }
  if ( (num_bytes_received > 0 || bt_bytes_recieved > 0 ) && !host_connected ) { // Host is now connected
    host_connected = true;
  }

  if (host_connected) {
    if (num_bytes_received) {
      status = handleCommand(serial_input_buffer, num_bytes_received, 0);
      //bgx.sendString(serial_input_buffer);
      //Serial.println(serial_input_buffer[0]);
      //Serial.write(status);
      num_bytes_received = 0;
    }
    if (bt_bytes_recieved) {
      status = handleCommand(bluetooth_input_buffer, bt_bytes_recieved, 1);
      bt_bytes_recieved = 0;
    }
  } else {
    idleActions();
#if SWELL
    updateSwell(swell_count++);
    delay(400);
#endif
  }
}

int getSerialInput(char *buffer, int size, int timeout) {
  int num_bytes_received = 0;
  unsigned long start_time = millis();

  while (!Serial.available()) {
    // Wait for bytes to come in
    if (timeout && ((millis() - start_time) > timeout)) {
      return 0;
    }
  }

  while (Serial.available() && (num_bytes_received < size) ) {
    buffer[num_bytes_received] = Serial.read();
    num_bytes_received++;
    delay(1);
  }
  //Serial.println(num_bytes_received);
  return num_bytes_received;
}

unsigned char getChecksum( char *buffer, int length) {
  unsigned char sum = 0;

  for (int byte_iter = 0; byte_iter < length; byte_iter++) {
    sum += buffer[byte_iter];
  }

  return sum;
}

ErrorCode handleCommand( char *buffer, int size, bool bt) {
  ErrorCode status = SUCCESS;
  char print_buff[PRINT_BUFFER_SIZE];

  unsigned char command_id = buffer[0];
  switch (command_id) {
    case '\0':
      break;

    // Change Light Color
    case 'L':
      if (size >= LIGHT_CHANGE_COMMAND_LENGTH) {
        convertBufferAtoI(buffer); //Needed if controlling over serial like interface
        RgbaiColorCommand color_command = *((RgbaiColorCommand *)buffer);
        if (color_command.arduino_id == ARD_ID || 1) {
          if (color_command.checksum == getChecksum(buffer, size - 1) || true) {
            setLightColor(color_command.color);
            snprintf(print_buff, PRINT_BUFFER_SIZE, "Lights Set R: %d G: %d B: %d W: %d", color_command.color.red, color_command.color.green, color_command.color.blue, color_command.color.white);
          } else {
            status = BAD_CHECKSUM;
          }
        } else {
          status = WRONG_ARDUINO_ID;
        }
      } else { //Used for light off right now
        RgbaiColorCommand color_command = *((RgbaiColorCommand *)buffer);
        if (color_command.arduino_id == ARD_ID) {
          //setLightColor({0});
          dimmer->rgbwControl(0, 0, 0, 0);
          snprintf(print_buff, PRINT_BUFFER_SIZE, "LIGHTS OFF");
        } else {
          host_connected = false; //Send L1 to start lioght show
        }
      }
      if (status) {
        snprintf(print_buff, PRINT_BUFFER_SIZE, "ERROR: %d", status);
      }
      break;

    // Get Temperature
    case 'T':
      if (size >= GET_TEMPERATURE_COMMAND_LENGTH) {
        float temperature;
        readSensors(&temperature, NULL);
        //Serial.println(temperature);
        snprintf(print_buff, PRINT_BUFFER_SIZE, "Temperature: %d.%02d", int(temperature), int(temperature * 100) % 100);
      } else {
        status = BAD_COMMAND_LENGTH;
      }
      break;

    // Get Lux
    case 'X':
      if (size >= GET_LUX_COMMAND_LENGTH) {
        float lux;
        readSensors(NULL, &lux);
        //Serial.println(lux);
        snprintf(print_buff, PRINT_BUFFER_SIZE, "Lux Reading: %d", int(lux));
      } else {
        status = BAD_COMMAND_LENGTH;
      }
      break;

    // Get All Sensor Data
    case 'A':
      if (size >= GET_ALL_SENSORS_COMMAND_LENGTH) {
        float temperature;
        float lux;
        readSensors(&temperature, &lux);
        snprintf(print_buff, PRINT_BUFFER_SIZE, "Temperature: %d.%02d || Lux Reading: %d", int(temperature), int(temperature * 100) % 100, int(lux));
        //Serial.println(temperature);
        //Serial.println(lux);
      } else {
        status = BAD_COMMAND_LENGTH;
      }
      break;
    case 'I':
      if (size >= GET_ARDUINO_ID_COMMAND_LENGTH) {
        //Serial.write((byte)ARD_ID);
        snprintf(print_buff, PRINT_BUFFER_SIZE, "%d", (int)ARD_ID);
        //Serial.println();
      } else {
        //Serial.write(0xFF);
        snprintf(print_buff, PRINT_BUFFER_SIZE, "%d", (int)0xff);
        //Serial.println();
        status = BAD_COMMAND_LENGTH;
      }
    default:
      //Serial.write(command_id);//0xFF);

      break;
  }
  if (bt) {
    bgx.sendString((char*)print_buff);
  } else {
    Serial.println((char*)&print_buff[0]);
  }

  return status;
}

void readSensors(float *temperature, float *ambient_light) {
  if (temperature && temp) {
    temp->startTempMeasure();
    delay(100);
    *temperature = temp->getTempMeasureC();
  }

  if (ambient_light && lux) {
    *ambient_light = lux->getWhite();
  }
}

void updateSwell(uint32_t swell_index) {
  dimmer->rgbwControl(swell[(swell_index + (SWELL_OFFSET*1))  % SWELL_COUNT],
                      swell[(swell_index + (SWELL_OFFSET*0))  % SWELL_COUNT],
                      swell[(swell_index + (SWELL_OFFSET*2)) % SWELL_COUNT],
                      swell[(swell_index + (SWELL_OFFSET*3)) % SWELL_COUNT]);
}


void setLightColor(LedColor color) {
  dimmer->rgbwControl(color.red, color.green,
                      color.blue, color.white);
}

void idleActions(void) {
  temp->startTempMeasure();
  float ambient = lux->getWhite();
  delay(100);
  float ctemp = temp->getTempMeasureC();
  snprintf(print_buffer, 120, "Temp (Celcius) | Light (Lumens) | Light Value (R G B W/255)"
           "\r\n   %3d.%02d      |     %5d      |   %3d %3d %3d %3d   \r\n",
           int(ctemp), int(ctemp * 100) % 100, int(ambient), swell[(swell_count + 6)  % 13] / 2,
           swell[(swell_count + 0)  % 13],
           swell[(swell_count + 12) % 13],
           swell[(swell_count + 18) % 13] / 4);
  //bgx.sendString(print_buffer);
  Serial.print(print_buffer);
}

void convertBufferAtoI(char * buff) {
  for (int color = 0; color < 4; color++) {
    char str[4];
    memcpy( str, &buff[(2 + color * 3)], 3 );
    str[3] = NULL;
    int temp = atoi(str);
    buff[2 + color] = temp > 255 ? 255 : temp;
  }
  buff[6] = buff[14];
  buff[7] = '\0';
}
