#ifndef CONFIG_H
#define CONFIG_H
//#include "DFRobot_PH.h"
//#include <Wire.h>
//#include "esp_attr.h"
#include <Arduino.h>
#include "MQUnifiedsensor.h"
#include <HardwareSerial.h>
#include <U8g2lib.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include "BluetoothSerial.h"
#include <SD.h> //Changed uint8_t max_files=5 to uint8_t max_files=2
//#include <SPI.h>
//#include "FS.h"
#include <time.h>
#include <nvs_flash.h>
//#include <nvs.h>
#include "DFRobot_ESP_EC.h"
#include <EEPROM.h>
//#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
//#include "soc/lldesc.h"
#include "ESP_PH.h" // library van de sensormaker
#include <ArduinoJson.h>
#include <sstream>
#include <iomanip>

extern int voltPin, CurrentPin, EC_PIN, PH_PIN;

float flowSens();

void BluetoothListen(void *parameter);
void DisplayMeasurements(void *parameter);
void Measuring(void *parameter);
void sendArray(void *parameter);
void Counting(void *parameter);

//extern const int bufferSize;
//extern StaticJsonDocument<bufferSize> doc2;
extern OneWire oneWire;                                               // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
extern DallasTemperature sensors;                                        // Pass our oneWire reference to Dallas Temperature.
extern DeviceAddress tempDeviceAddress;                                            // We'll use this variable to store a found device address

//extern JsonObject values;
extern float t, pHvalue;

extern QueueHandle_t measurementQueue; // Define the queue handle

/*      Configuration     */
extern bool sendhttp;
extern String payload;
extern bool Posting;
/*      Display      */
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C 
extern U8G2_SSD1306_128X64_NONAME_1_HW_I2C bigOled;

/*      MQ-7 MQ-8 sensor       */
void mq7_init(MQUnifiedsensor& MQ7);
void mq8_init(MQUnifiedsensor& MQ8);

//For GSMSerial output on OLED
extern U8G2LOG u8g2log;
extern volatile int stateBigOled;

/*      GSM Functions    */
void saveTimestamp(uint64_t timestamp);
uint64_t getSavedTimestamp();
extern uint64_t savedTimestamp;
void parseDatetime();
extern String datetime_gsm;

extern String apn, apn_User, apn_Pass;
extern char httpapi[];

//void GA6_init();
void getTime();
void getTimeNow();  
void post_http(String j); 
void post_http2(String a); 
time_t convertToUnixTimestamp(String date, String time);
void readGsmResponse();
String readGsmResponse3();
void initialize_gsm();
void initialize_gsm2();
extern String response;

/*      Display Setup     */
void init_displays();

/* voltage sensor */
float readVoltage();
float Dennis();

/*      DS18B20 sensor       */
//extern struct Measurement measurement;
extern int DS18B20_PIN;
void printDS18B20Address();
void AllDS18B20Sensors();

/*      Flow sensor       */
void pcnt_example_init(pcnt_unit_t unit, int pulse_gpio_num);
extern volatile float flowRate, flowRate2;
/*      NTC sensor       */
extern float steinhart;
extern float temp_flow;
float Read_NTC();

/*      Bluetooth          */
extern BluetoothSerial SerialBT;
extern String message;
extern char incomingChar;
void sendFileOverBluetooth(const char* path);
void readFileAndSendOverBluetooth(fs::FS &fs, const char *path);
void sendFileOverBluetoothInOneGo2(const char* path);

/*        SD Card         */
extern int CS_PIN;
void SD_init();
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
//void createDir(fs::FS &fs, const char * path);
//void removeDir(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
//void renameFile(fs::FS &fs, const char * path1, const char * path2);
//void deleteFile(fs::FS &fs, const char * path);
//void testFileIO(fs::FS &fs, const char * path);
void readFile(fs::FS &fs, const char * path);
void logMeasurement(String measurement);
void sendFileOverBluetoothInOneGo(const char* path);

/*      Configuration     */
extern int buttonbigOled;
void buttonInterrupt_bigOled();

/*      Ph Sensor         */
extern ESP_PH ph;
extern int PH_PIN;
float pH();
float readTemperature();

/*      Conductivity Sensor   */
extern DFRobot_ESP_EC ec;
//extern float voltage_cond, temperature_cond;
extern int EC_PIN; // Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
float Cond();

/*      Current Sensor   */
extern int CurrentPin;
float CurrentSensor_quick();
float CurrentSensor_ACS724();


#endif // CONFIG_H
