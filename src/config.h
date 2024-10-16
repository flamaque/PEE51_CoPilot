#ifndef CONFIG_H
#define CONFIG_H
//#include <Wire.h>
//#include "esp_attr.h"
//#include <stdio.h>
//#include "soc/lldesc.h"
//#include <nvs.h>
//#include <spiffs.h>

#include <Arduino.h>
#include "MQUnifiedsensor.h"
#include <HardwareSerial.h>
#include <U8g2lib.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include "BluetoothSerial.h"
#include <SD.h> //Changed uint8_t max_files=5 to uint8_t max_files=2 (Includes spi and fs already)
#include <time.h>
#include <nvs_flash.h>
#include "DFRobot_ESP_EC.h"
#include <EEPROM.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "ESP_PH.h" // library van de sensormaker
#include <ArduinoJson.h>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <esp_task_wdt.h>
#include <vector>

// #include "JsonStreamingParser.h"
// #include "JsonListener.h"
#define ARDUINOJSON_STRING_LENGTH_SIZE 2      //Max characters 65,635
#define ARDUINOJSON_SLOT_ID_SIZE 2            //Max-nodes 65,635
#define ARDUINOJSON_USE_LONG_LONG 0           //Store jsonVariant as long
#define ARDUINOJSON_USE_DOUBLE 0              //Store floating point NOT as double 

extern char command[20];
void sendCmd(const char* cmd);
#define BEARER_PROFILE_GPRS "AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n"
#define BEARER_PROFILE_APN "AT+SAPBR=3,1,\"APN\",\"%s\"\r\n"
#define QUERY_BEARER "AT+SAPBR=2,1\r\n"
#define OPEN_GPRS_CONTEXT "AT+SAPBR=1,1\r\n"
#define CLOSE_GPRS_CONTEXT "AT+SAPBR=0,1\r\n"
#define HTTP_INIT "AT+HTTPINIT\r\n"
#define HTTP_INIT2 "AT+HTTPINIT=?\r\n"
#define HTTP_CID "AT+HTTPPARA=\"CID\",1\r\n"
#define HTTP_PARA "AT+HTTPPARA=\"URL\",\"%s\"\r\n"
#define HTTP_GET "AT+HTTPACTION=0\r\n"
#define HTTP_POST "AT+HTTPACTION=1\n"
#define HTTP_DATA "AT+HTTPDATA=%d,%d\r\n"
#define HTTP_READ "AT+HTTPREAD\r\n"
#define HTTP_CLOSE "AT+HTTPTERM\r\n"
#define HTTP_CONTENT "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n"
#define HTTPS_ENABLE "AT+HTTPSSL=1\r\n"
#define HTTPS_DISABLE "AT+HTTPSSL=0\r\n"
#define NORMAL_MODE "AT+CFUN=1,1\r\n"
#define REGISTRATION_STATUS "AT+CREG?\r\n"
#define SIGNAL_QUALITY "AT+CSQ\r\n"
#define READ_VOLTAGE "AT+CBC\r\n"
#define SLEEP_MODE_2 "AT+CSCLK=2\r\n"
#define SLEEP_MODE_1 "AT+CSCLK=1\r\n"
#define SLEEP_MODE_0 "AT+CSCLK=0\r\n"
#define READ_GPS "AT+CIPGSMLOC=1,1\r\n"
#define READ_GPS_2 "AT+CIPGSMLOC=2,1\r\n"
void sendFileToSIM800L(File file);

extern uint8_t voltPin;

void BluetoothListen(void *parameter);
void DisplayMeasurements(void *parameter);
void Measuring(void *parameter);
void sendArray(void *parameter);
void Counting(void *parameter);

extern OneWire oneWire;                 // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
extern DallasTemperature sensors;       // Pass our oneWire reference to Dallas Temperature.
extern DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

extern float t, pHvalue;

extern QueueHandle_t measurementQueue; // Define the queue handle

/*      Configuration     */
//extern bool sendhttp;
//extern String payload;
extern bool Posting;

/*      Display      */
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C 
extern U8G2_SSD1306_128X64_NONAME_1_HW_I2C bigOled;

/*      MQ-7 MQ-8 sensor       */
void mq7_init(MQUnifiedsensor& MQ7);
void mq8_init(MQUnifiedsensor& MQ8);

//For GSMSerial output on OLED
extern U8G2LOG u8g2log;
extern volatile uint8_t stateBigOled, stateDebug;

/*      GSM Functions    */
void saveTimestamp(uint64_t timestamp);
uint64_t getSavedTimestamp();
extern uint64_t savedTimestamp, timestamp_ms, unixTimestamp;
void parseDatetime();
extern String datetime_gsm;

extern String apn, apn_User, apn_Pass;
extern char httpapi[];

void parseCLTSResponse();
String readGsmResponse5();
void getTime();
void post_http2(const char* jsonPayload); 
uint64_t convertToUnixTimestamp(String date, String time);
void readGsmResponse();
String readGsmResponse3();
String readGsmResponse4();
void initialize_gsm();
void initialize_gsm2();
extern String response;
extern uint8_t GSMType;
extern String date_getTime;

/*      Display Setup     */
void init_displays();

/* voltage sensor */
float readVoltage();

/*      DS18B20 sensor       */
extern uint8_t DS18B20_PIN;
void printDS18B20Address();
//void AllDS18B20Sensors();

/*      Flow sensor       */
void pcnt_example_init(pcnt_unit_t unit, int pulse_gpio_num);
extern volatile float flowRate, flowRate2;
/*      NTC sensor       */
extern float steinhart, temp_flow;
extern uint8_t NTC_PIN, TEMPERATURENOMINAL;
extern int serialResistance;
extern const uint8_t NUMSAMPLES;
extern uint16_t nominalResistance, bCoefficient;

float Read_NTC();

/*      Bluetooth          */
extern BluetoothSerial SerialBT;
extern String message;
extern char incomingChar;
void sendFileOverBluetooth(const char *path);

void sendLargeFileOverBluetooth(const char *path);
void sendLargeFileOverBluetooth2(const char *path);
/*        SD Card         */
extern SPIClass spi;
extern uint8_t CS_PIN;
void SD_init();
void printDirectory(File dir, int numTabs);
void read_configuration();
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void readFile(fs::FS &fs, const char * path);
void logMeasurement(String measurement);
void SD_Card_Speed_Test();
bool copyFile(int chunkSize, const char *destinationFile);

void renameFile(fs::FS &fs, const char * path1, const char * path2);
void deleteFile(fs::FS &fs, const char * path);
//void removeDir(fs::FS &fs, const char * path);
//void createDir(fs::FS &fs, const char * path);

/*      Button Setup     */
extern uint8_t buttonbigOled, ButtonDebug;
extern volatile bool buttonDebugPressed;
void buttonInterrupt_bigOled();
void buttonInterrupt_debug();

/*      Ph Sensor         */
extern ESP_PH ph;
extern uint8_t PH_PIN;
float pH();
//float readTemperature();

/*      Conductivity Sensor   */
extern DFRobot_ESP_EC ec;
//extern float voltage_cond, temperature_cond;
extern uint8_t EC_PIN; // Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
float Cond();

/*      Current Sensor   */
extern uint8_t CurrentPin;
float CurrentSensor_724();

#endif // CONFIG_H
