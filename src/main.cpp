#include "config.h"
#include <driver/adc.h>

/*      GSM Module Setup     */
HardwareSerial gsmSerial(2); // Use UART2
int GSM_RX_PIN = 17;        
int GSM_TX_PIN = 16;        
int GSM_RST_PIN = 4;
String apn = "data.lycamobile.nl";
String apn_User = "lmnl";
String apn_Pass = "plus";
char httpapi[] = "http://jrbubuntu.ddns.net:5000/api/telemetry"; // Not yet tested as String
//char httpapi[] = "http://145.131.6.212/api/v1/HR/gl3soo07qchjimbsdwln/telemetry";
String mobileNumber = "+31614504288";

extern time_t timestamp; // Remove extern
uint64_t savedTimestamp;

/*      MQ-7 CO sensor                  */
int Pin_MQ7 = 14; // 35
MQUnifiedsensor MQ7("ESP32", 5, 12, Pin_MQ7, "MQ-7");
/*      MQ-8 H2 sensor                   */
int Pin_MQ8 = 32;
MQUnifiedsensor MQ8("ESP32", 5, 12, Pin_MQ8, "MQ-8");

/*      DHT22 - Temperature and Humidity */
#include "DHT.h"
int DHT_SENSOR_PIN = 25;
#define DHT_SENSOR_TYPE DHT22
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

/*      Setup Temperature sensor        */
int DS18B20_PIN = 27;

/*      Setup Flowsensor                */
volatile float flowRate, flowRate2, flowRate3 = 0.00;
int flowSensorPin = 36; 
const float flowSensorCalibration = 21.00;

int flowSensor2Pin = 26;
const float flowSensorCalibration2 = 7.50;
// const int flowSensor3Pin = 14;
const float flowSensorCalibration3 = 11.0;
float frequency2 = 0.0;

#define PCNT_INPUT_SIG_IO1 flowSensorPin  // Pulse Input GPIO for PCNT_UNIT_0
#define PCNT_INPUT_SIG_IO2 flowSensor2Pin // Pulse Input GPIO for PCNT_UNIT_1
// #define PCNT_INPUT_SIG_IO3 flowSensor3Pin // Pulse Input GPIO for PCNT_UNIT_2
#define PCNT_UNIT1 PCNT_UNIT_0
#define PCNT_UNIT2 PCNT_UNIT_1
// #define PCNT_UNIT3 PCNT_UNIT_2

/*      Flowsensor Temperature        */
int NTC_PIN = 12; // ADC2_5 
uint16_t nominalResistance  =   50000;
/// The resistance value of the serial resistor used in the conductivity sensor circuit.
uint16_t serialResistance    =  3230;
uint16_t bCoefficient         = 3950;
#define TEMPERATURENOMINAL 25
#define NUMSAMPLES         100 
//#define VERBOSE_SENSOR_ENABLED 
float temp_flow;                   // Global temperature reading

/*      Bluetooth                       */
BluetoothSerial SerialBT;
String message = "";
char incomingChar;

/*      SD card                         */
int CS_PIN = 5;
SemaphoreHandle_t fileMutex = NULL; // Handler for the log.txt file

/*      Switch screen                   */
int buttonbigOled = 13; // Pin connected to the button
extern bool buttonBigPressed;
U8G2_WITH_HVLINE_SPEED_OPTIMIZATION

/*          Conductivity sensor              */
int EC_PIN = 39; // 4;

/*          Current sensor                  */
int CurrentPin = 33;

/*          Voltage sensor                   */
int voltPin = 35;

/*          pH sensor                       */
ESP_PH ph;
int PH_PIN = 34;

/*          Test for Array of JSON Objects         */
// Define the queue handle
QueueHandle_t measurementQueue; //= nullptr // Define the queue handle
const int queueLength = 2;  //5    // was 10, werkte goed maar met gaten in graph

// Ctrl + d for multiple cursors
int currentMeasurementIndex = 0;

int h2Amount = 2;
int coAmount = 2;
int flowRateAmount = 10;
int flowRate2Amount = 10;
int temperatureAmount = 5;
int humidityAmount = 4;
int phValueAmount = 2;
int ecValueAmount = 2;
int ds18b20Amount = 8;
int voltAmount = 25;
int powerAmount = 25;
int acsAmount = 25;
int TempFlowAmount = 5;

const int numMeasurements = std::max({temperatureAmount, phValueAmount, humidityAmount, ecValueAmount, flowRateAmount, flowRate2Amount, acsAmount, ds18b20Amount, h2Amount, coAmount, voltAmount, TempFlowAmount, powerAmount});
const int totMeasurements = temperatureAmount + phValueAmount + humidityAmount + ecValueAmount + flowRateAmount + flowRate2Amount + acsAmount + ds18b20Amount + h2Amount + coAmount + voltAmount + TempFlowAmount + powerAmount;
extern float phValue, AcsValueF, ecValue;
const int dht22_tempInterval = numMeasurements / temperatureAmount;
const int phValueInterval = numMeasurements / phValueAmount;
const int dht22_humInterval = numMeasurements / humidityAmount;
const int ecValueInterval = numMeasurements / ecValueAmount;
const int flowRateInterval = numMeasurements / flowRateAmount;
const int flowRate2Interval = numMeasurements / flowRate2Amount;
const int acsValueFInterval = numMeasurements / acsAmount;
const int ds18b20Interval = numMeasurements / ds18b20Amount;
const int voltInterval = numMeasurements / voltAmount;
const int powerInterval = numMeasurements / powerAmount;
const int h2Interval = numMeasurements / h2Amount;
const int coInterval = numMeasurements / coAmount;
const int FlowTempinterval = numMeasurements / TempFlowAmount;
String intervals = "{\"Intervals\":{\"numMeasurements\":" + String(numMeasurements) + ",\"temperatureInterval\":" + String(dht22_tempInterval) + ",\"phValueInterval\":" + String(phValueInterval) + ",\"humidityInterval\":" + String(dht22_humInterval) + ",\"ecValueInterval\":" + String(ecValueInterval) + ",\"flowRateInterval\":" + String(flowRateInterval) + ",\"flowRate2Interval\":" + String(flowRate2Interval) + ",\"acsValueFInterval\":" + String(acsValueFInterval) + ",\"ds18b20Interval\":" + String(ds18b20Interval) + ",\"voltInterval\":" + String(voltInterval) + ",\"h2Interval\":" + String(h2Interval) + ",\"coInterval\":" + String(coInterval) + ",\"FlowTempinterval\":" + String(FlowTempinterval) + ",\"powerInterval\":" + String(powerInterval) + "},";
const int bufferSize = 6144; //6144; //8192 Waarschijnlijk te groot
char jsonBuffer[bufferSize];
int bufferIndex = 0;

int GSM_RX_PIN2, GSM_TX_PIN2, GSM_RST_PIN2, Pin_MQ72, Pin_MQ82, DHT_SENSOR_PIN2, DS18B20_PIN2, flowSensorPin2, flowSensor2Pin2;

/*          Test for Array of JSON Objects         */
void sendArray(void *parameter)
{
  Serial.println("Now running sendArray task.");
  char receivedBuffer[bufferSize];
  memset(receivedBuffer, 0, sizeof(receivedBuffer));
  unsigned long previousTime = 0;
  for (;;)
  {
    if (measurementQueue != NULL)
    {
      if (uxQueueMessagesWaiting(measurementQueue) > 0)
      {
        int queueSize = uxQueueMessagesWaiting(measurementQueue);
        Serial.println("Amount in queue (sendArray): " + String(queueSize));
        if (xQueueReceive(measurementQueue, &receivedBuffer, 0) == pdPASS)
        {
          // xQueueReceive(measurementQueue, &receivedBuffer, portMAX_DELAY);
          // printf("Received item: %s \n", receivedBuffer);
          // printf("Received item size: %d \n", sizeof(&receivedBuffer));
          unsigned long currentTime = millis();
          unsigned long timeBetweenUsage = currentTime - previousTime;
          previousTime = currentTime;
          post_http2(receivedBuffer);
          Serial.println("Time between usage: " + String(timeBetweenUsage) + "ms");
        }
      }
    }
    else
    {
      Serial.println("measurementQueue was equal to NULL.");
    }
    // Monitor stack and heap usage
    // UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // size_t freeHeap = xPortGetFreeHeapSize();
    // Serial.print("SendArray stack high water mark: ");
    // Serial.println(highWaterMark);
    // Serial.print("Free heap size SendArray: ");
    // Serial.println(freeHeap);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

float t, phvalue, stroom, Volt, DS18B20_1, DS18B20_2, DS18B20_3, DS18B20_4, DS18B20_5, humidity, ecValue, ppmCO, ppmH;
float* ds18b20Sensors[] = {&DS18B20_1, &DS18B20_2, &DS18B20_3, &DS18B20_4, &DS18B20_5};

void Measuring(void *parameter)
{
  vTaskDelay(10 / portTICK_PERIOD_MS);
  Serial.println("Inside Measuring task.");
  Serial.println("MaxMeasurements: " + String(numMeasurements));
  static int temperatureCount = 0, phValueCount = 0, humidityCount = 0, ecValueCount = 0, flowRateCount2 = 0, flowRateCount = 0, acsValueFCount = 0, ds18b20Count = 0, voltCount = 0, coCount = 0, h2Count = 0, FTCount =0, powerCount = 0;
  unsigned long start_time, end_time, duration_temperature, duration_phValue, duration_humidity, duration_ecValue, duration_flowRate, duration_flowRate2, duration_acsValueF, duration_ds18b20, duration_h2, duration_volt;
  duration_temperature = duration_phValue = duration_humidity = duration_ecValue = duration_flowRate = duration_acsValueF = duration_ds18b20 = duration_h2 = duration_volt = 0;
  TickType_t measureTime, stopTime, endTimeDHTtemp, endTimepH, endTimeDHThum, endTimeEC, endTimeFlow1, endTimeFlow2, endTimeStroom, endTimeDS18B20, endTimeH2, endTimeCO, endTimeVolt, endTimeFlowTemp;
  TickType_t startTimeMeasurement, startTime, startDHTTtempTime, startDHThumTime, startECValueTime, startFlowRateTime, startFlowRate2Time, startStroomTime, startpHTime, startDS18B20Time, startH2Time, startCOTime, startVoltTime, startFlowTempTime;
  // Serial.println("dht22_tempInterval: " + String(dht22_tempInterval));
  // Serial.println("dht22_humInterval: " + String(dht22_humInterval));
  // Serial.println("phValueInterval: " + String(phValueInterval));
  // Serial.println("ecValueInterval: " + String(ecValueInterval));
  // Serial.println("flowRateInterval: " + String(flowRateInterval));
  // Serial.println("flowRate2Interval: " + String(flowRate2Interval));
  // Serial.println("acsValueFInterval: " + String(acsValueFInterval));
  // Serial.println("ds18b20Interval: " + String(ds18b20Interval));
  // Serial.println("voltInterval: " + String(voltInterval));
  // Serial.println("coInterval: " + String(coInterval));
  // Serial.println("h2Interval: " + String(h2Interval));

  //  Add the intervals objects
  // JsonObject intervals = doc2["intervals"].to<JsonObject>();
  // intervals["numMeasurements"] = numMeasurements;
  // intervals["TInt"] = dht22_tempInterval;
  // intervals["pHInt"] = phValueInterval;
  // intervals["AInt"] = acsValueFInterval;
  // intervals["VInt"] = voltInterval;
  // intervals["TDSInt"] = ds18b20Interval;
  // intervals["HumInt"] = dht22_humInterval;
  // intervals["CondInt"] = ecValueInterval;
  // intervals["Flow1Int"] = flowRateInterval;
  // intervals["Flow2Int"] = flowRate2Interval;
  // intervals["COInt"] = coInterval;
  // intervals["H2Int"] = h2Interval;

  // JsonArray values = doc2.createNestedArray("values");
  // JsonObject values = doc2["values"].to<JsonObject>();
  //  values["T_g"];
  //  values["pH"];
  //  values["A"];
  //  values["V"];
  //  values["T1"];
  //  values["T2"];
  //  values["T3"];
  //  values["T4"];
  //  values["T5"];
  //  values["Hum"];
  //  values["Cond"];
  //  values["Flow1"];
  //  values["Flow2"];
  //  values["CO"];
  //  values["H2"];
  for (;;)
  {
    startTimeMeasurement = xTaskGetTickCount();
    JsonDocument doc2;
    JsonArray measurementsArray = doc2.to<JsonArray>();
    std::stringstream ss;
    ss.str("");
    
    for (int i = 0; i < numMeasurements; i++)
    {
      //JsonObject measurement = measurementsArray.createNestedObject();
      JsonObject measurement = measurementsArray.add<JsonObject>();
      measurement["ts"] = savedTimestamp + millis();
      JsonObject values = measurement["values"].to<JsonObject>();
      
      if (i % dht22_tempInterval == 0)
      {
        t = dht_sensor.readTemperature();
        ss.str("");
        ss << std::fixed << std::setprecision(2) << t;
        values["T_g"] = isnan(t) || isinf(t) ? 0.0 : t; //ss.str(); //Maybe just "t"
        //values["T_g"] = isnan(t) || isinf(t) ? 0.0 : ss.str().c_str();
      }

      if (i % ds18b20Interval == 0)
      {
        sensors.requestTemperatures();
        int numberOfDevices = sensors.getDeviceCount();
        for (int j = 0; j < numberOfDevices && j < 5; j++)
        {
          if (sensors.getAddress(tempDeviceAddress, j))
          {
            float tempC = sensors.getTempC(tempDeviceAddress);
            ss.str("");
            ss << std::fixed << std::setprecision(3) << tempC;
            values["T" + String(j+1)] = isnan(tempC) || isinf(tempC) ? 0 : tempC; //ss.str();        
            *ds18b20Sensors[j] = tempC; // save tempC to corresponding sensor variable    
          }          
        }
      }

      if (i % phValueInterval == 0)
      {
        phvalue = pH();
        ss.str("");
        ss << std::fixed << std::setprecision(3) << phvalue;
        values["pH"] = isnan(phvalue) || isinf(phvalue) ? 0 : phvalue; //ss.str();
      }

      if (i % acsValueFInterval == 0)
      {
        //stroom = CurrentSensor_ACS724();
        stroom = CurrentSensor_quick();
        ss.str("");
        ss << std::fixed << std::setprecision(3) << stroom;
        values["A"] = isnan(stroom) || isinf(stroom) ? 0 : stroom; //ss.str();
      }

      if (i % dht22_humInterval == 0)
      {
        humidity = dht_sensor.readHumidity();
        ss.str("");
        ss << std::fixed << std::setprecision(3) << humidity;
        values["Hum"] = isnan(humidity) || isinf(humidity) ? 0 : humidity; //ss.str();
      }

      if (i % ecValueInterval == 0)
      {
        ecValue = Cond();
        ss.str("");
          ss << std::fixed << std::setprecision(3) << ecValue;
        values["Cond"] = isnan(ecValue) || isinf(ecValue) ? 0 : ecValue; //ss.str();
      }

      if (i % flowRateInterval == 0)
      {
        //flowRate = flowSens();
        ss.str("");
        ss << std::fixed << std::setprecision(2) << flowRate; 
        values["Flow1"] = isnan(flowRate) || isinf(flowRate) ? 0 : flowRate; //ss.str();
      }

      if (i % flowRate2Interval == 0)
      {
        ss.str("");
        ss << std::fixed << std::setprecision(2) << flowRate2;
        values["Flow2"] = isnan(flowRate2) || isinf(flowRate2) ? 0 : ss.str();
      }

      if (i % FlowTempinterval == 0)
      {
        startFlowTempTime = xTaskGetTickCount();
        temp_flow = Read_NTC();   // Read temperature
        ss.str("");
        ss << std::fixed << std::setprecision(1) << temp_flow;
        values["FT"] = isnan(temp_flow) || isinf(temp_flow) ? 0 : temp_flow; //ss.str();
      }
      
      if (i % coInterval == 0)
      {
        MQ7.update();
        ppmCO = MQ7.readSensor();
        ss.str("");
        ss << std::fixed << std::setprecision(3) << ppmCO;
        values["CO"] = isnan(ppmCO) || isinf(ppmCO) ? 0 : ppmCO; //ss.str();
      }

      if (i % h2Interval == 0)
      {
        MQ8.update();
        ppmH = MQ8.readSensor();
        ss.str("");
        ss << std::fixed << std::setprecision(3) << ppmH; 
        values["H2"] = isnan(ppmH) || isinf(ppmH) ? 0 : ppmH; //ss.str();
      }

      if (i % voltInterval == 0)
      {
        Volt = readVoltage();
        ss.str("");
        ss << std::fixed << std::setprecision(3) << Volt;
        values["V"] = isnan(Volt) || isinf(Volt) ? 0 : Volt; //ss.str();        
      }
      
      if (i % powerInterval == 0)
      {
        float power = Volt * stroom;
        ss.str("");
        ss << std::fixed << std::setprecision(3) << power;
        values["P"] = isnan(power) || isinf(power) ? 0 : power; ;//ss.str();
      }
      currentMeasurementIndex++;
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    stopTime = xTaskGetTickCount();
    measureTime = stopTime - startTimeMeasurement;
    vTaskDelay(50 / portTICK_PERIOD_MS);

    if (currentMeasurementIndex >= (numMeasurements - 1)) // MaxMeasurements -1
    {    
      // printCMD();
/*
      Serial.println("Measurement duration: " + String(measureTime));
      Serial.println("All of the following durations are singular, to obtain the total time you need to multiply by the number of measurements");
      Serial.println("Temp duration:   " + String(endTimeDHTtemp - startDHTTtempTime) + "| Humi duration: " + String(endTimeDHThum- startDHThumTime) + "| DS18B20 duration:  " + String(endTimeDS18B20 - startDS18B20Time));
      Serial.println("pH duration:     " + String(endTimepH - startpHTime)            + "| EC duration:   " + String(endTimeEC - startECValueTime)   + "| Flowrate duration: " + String(endTimeFlow1 - startFlowRateTime) + "| Flowrate2 duration: " + String(endTimeFlow2 - startFlowRate2Time));
      Serial.println("Stroom duration: " + String(endTimeStroom - startStroomTime)    + "| Volt duration: " + String(endTimeVolt - startVoltTime)    + "| MQ8 H2 duration:   " + String(endTimeH2 - startH2Time)          + "| MQ7 Co duration:    " + String(endTimeCO - startCOTime));
      Serial.println("FlowTemp duration: " + String(endTimeFlowTemp - startFlowTempTime));
      Serial.println();
*/
      /* Add linebreaks and whitespaces to the end of the JSON document */
      // serializeJsonPretty(doc, jsonBuffer);

      /* Minifies the JSON document e.g. no linebreaks and no whitespaces */
      ArduinoJson::serializeJson(doc2, jsonBuffer);
      Serial.println("jsonBuffer created in Measuring task and sent to queue: ");
      Serial.println(jsonBuffer);

      if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(3000)) == pdTRUE)
      {
        if (doc2 != nullptr)
        {
          logMeasurement((&doc2)->as<String>().c_str());
          xSemaphoreGive(fileMutex);
        }
      }
      else
      {
        Serial.println("Measuring Task: logMeasurement could not take fileMutex");
      }

      // Send an item
      if (measurementQueue != NULL)
      {
        if (xQueueSend(measurementQueue, &jsonBuffer, portMAX_DELAY) == pdPASS)
        {
          Serial.println("Successfully posted buffer to queue");
          Serial.println("Flowsensor2 Frequency: " + String(frequency2));
          int queueSize = uxQueueMessagesWaiting(measurementQueue);
          Serial.println("Amount in queue (MeauringTask): " + String(queueSize));
          vTaskDelay(pdMS_TO_TICKS(10));
          memset(jsonBuffer, 0, sizeof(jsonBuffer));
        }
        else
        {
          Serial.println("Failed to post buffer to queue, deleting this buffer.");
          memset(jsonBuffer, 0, sizeof(jsonBuffer));
        }
      }      
    }
    currentMeasurementIndex = 0, temperatureCount = 0, phValueCount = 0, humidityCount = 0, ecValueCount = 0, flowRateCount = 0, flowRateCount2 = 0, acsValueFCount = 0, ds18b20Count = 0, h2Count = 0, voltCount = 0, FTCount = 0, coCount = 0, powerCount = 0;

    vTaskDelay(50 / portTICK_PERIOD_MS);
    // Monitor stack and heap usage
    // UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // size_t freeHeap = xPortGetFreeHeapSize();
    // Serial.print("MeasuringTask stack high water mark: ");
    // Serial.println(highWaterMark);
    // Serial.print("Free heap size MeasuringTask: ");
    // Serial.println(freeHeap);
  }
  Serial.println("Measuring task has ended.");
}

void DisplayMeasurements(void *parameter)
{
  vTaskDelay(15 / portTICK_PERIOD_MS);
  Serial.println("Inside Display Measurements task.");
  for (;;)
  {
    String flowDis = "Flow: " + String(flowRate) + " L/min";
    String flowDis2 = "Flow2: " + String(flowRate2) + " L/min";
    String flowDis3 = "Flow3: " + String(flowRate3) + " L/min";
    String tempDis = "Temp: " + String(t) + " °C";
    String humidityDis = "Hum_: " + String(humidity) + " %";
    String coDis = "CO__: " + String(ppmCO) + " ppm";
    String h2Dis = "H2__: " + String(ppmH) + " ppm";
    String DS18B20_1_Dis = "DS_1: " + String(DS18B20_1) + " °C";
    String DS18B20_2_Dis = "DS_2: " + String(DS18B20_2) + " °C";
    String DS18B20_3_Dis = "DS_3: " + String(DS18B20_3) + " °C";
    String DS18B20_4_Dis = "DS_4: " + String(DS18B20_4) + " °C";
    String DS18B20_5_Dis = "DS_5: " + String(DS18B20_5) + " °C";
    String Current_Dis = "Amp_: " + String(stroom) + " A";
    String pH_Dis = "pH__: " + String(phvalue) + "";
    String VoltDis = "Volt: " + String(Volt) + " V";
    String ecDis = "EC__: " + String(ecValue) + " ms/cm";
    String temp_flowDis = "Tflow: " + String(temp_flow) + " °C";
    // #1 u8g2_font_micro_mr
    // #2 u8g2_font_3x5im_mr
    // For horizontal display u8g2_font_tinytim_tr
    if (stateBigOled == 2)
    {
      bigOled.firstPage();
      do
      {
        bigOled.setFont(u8g2_font_micro_mr); // Was u8g2_font_ncenB08_tr
        bigOled.setDisplayRotation(U8G2_R1);
        bigOled.drawStr(0, 8, h2Dis.c_str());
        bigOled.drawStr(0, 16, VoltDis.c_str());
        bigOled.drawStr(0, 24, Current_Dis.c_str());
        bigOled.drawStr(0, 32, pH_Dis.c_str());
        bigOled.drawStr(0, 40, ecDis.c_str());
        bigOled.drawStr(0, 48, humidityDis.c_str());
        bigOled.drawStr(0, 56, tempDis.c_str());
        bigOled.drawStr(0, 64, DS18B20_1_Dis.c_str());
        bigOled.drawStr(0, 72, DS18B20_2_Dis.c_str());
        bigOled.drawStr(0, 80, DS18B20_3_Dis.c_str());
        bigOled.drawStr(0, 88, DS18B20_4_Dis.c_str());
        bigOled.drawStr(0, 96, DS18B20_5_Dis.c_str());
        bigOled.drawStr(0, 104, flowDis.c_str());
        bigOled.drawStr(0, 112, flowDis2.c_str());
        bigOled.drawStr(0, 120, temp_flowDis.c_str());
        bigOled.drawStr(0, 128, coDis.c_str());
      } while (bigOled.nextPage());
    }
    if (stateBigOled == 3)
    {
      bigOled.firstPage();
      do
      {
        bigOled.setFont(u8g2_font_3x5im_mr); // Was u8g2_font_ncenB08_tr
        bigOled.setDisplayRotation(U8G2_R0);
        bigOled.drawStr(0, 8, VoltDis.c_str());
        bigOled.drawStr(65, 8, Current_Dis.c_str());
        bigOled.drawStr(0, 16, h2Dis.c_str());
        bigOled.drawStr(0, 24, ecDis.c_str());
        bigOled.drawStr(0, 32, tempDis.c_str());
        bigOled.drawStr(65, 32, DS18B20_1_Dis.c_str());
        bigOled.drawStr(0, 40, DS18B20_2_Dis.c_str());
        bigOled.drawStr(65, 40, DS18B20_3_Dis.c_str());
        bigOled.drawStr(0, 48, DS18B20_4_Dis.c_str());
        bigOled.drawStr(65, 48, DS18B20_5_Dis.c_str());
        bigOled.drawStr(0, 56, pH_Dis.c_str());
        bigOled.drawStr(65, 56, humidityDis.c_str());
        bigOled.drawStr(0, 64, flowDis.c_str());
        bigOled.drawStr(70, 64, flowDis2.c_str());
      } while (bigOled.nextPage());
    }
    if (stateBigOled == 4)
    {
      bigOled.firstPage();
      do
      {
        bigOled.setFont(u8g2_font_3x5im_mr); // Was u8g2_font_ncenB08_tr
        bigOled.setDisplayRotation(U8G2_R1);
        bigOled.drawStr(0, 8, VoltDis.c_str());
        bigOled.drawStr(0, 16, Current_Dis.c_str());
        bigOled.drawStr(0, 24, h2Dis.c_str());
        bigOled.drawStr(0, 32, ecDis.c_str());
        bigOled.drawStr(0, 40, pH_Dis.c_str());
        bigOled.drawStr(0, 48, coDis.c_str());
        bigOled.drawStr(0, 56, humidityDis.c_str());
        bigOled.drawStr(0, 64, tempDis.c_str());
        bigOled.drawStr(0, 72, DS18B20_1_Dis.c_str());
        bigOled.drawStr(0, 80, DS18B20_2_Dis.c_str());
        bigOled.drawStr(0, 88, DS18B20_3_Dis.c_str());
        bigOled.drawStr(0, 96, DS18B20_4_Dis.c_str());
        bigOled.drawStr(0, 104, DS18B20_5_Dis.c_str());
        bigOled.drawStr(0, 112, flowDis.c_str());
        bigOled.drawStr(0, 120, flowDis2.c_str());
        bigOled.drawStr(0, 120, temp_flowDis.c_str());
      } while (bigOled.nextPage());
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  Serial.println("Display task has ended.");
}

void BluetoothListen(void *parameter)
{
  vTaskDelay(20 / portTICK_PERIOD_MS);
  Serial.println("Inside Bluetooth task.");
  for (;;)
  {
    if (SerialBT.available())
    {
      char incomingChar = SerialBT.read();
      if (incomingChar != '\n')
      {
        message += String(incomingChar);
      }
      else
      {
        message = "";
      }
      Serial.println("Received message:" + message);

      // Check if the command is to request a file
      if (message == "1" || message == "2" || message == "3" || message == "4" || message == "4" || message == "s" || message == "d")
      {
        // Acquire the mutex before accessing the file
        if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE)
        {
          // File operations go here
          if (message == "1")
          {
            Serial.println("Sending config.txt");
            sendFileOverBluetooth("/config.txt");
            // ... (other file operations)
          }
          else if (message == "2")
          {
            Serial.println("Sending log.txt");
            sendFileOverBluetooth("/log.txt");
            // ... (other file operations)
          }
          else if (message == "3")
          {
            size_t freeHeapBefore = esp_get_free_heap_size();
            Serial.println("Free heap before sending file: " + String(freeHeapBefore) + " bytes");

            sendFileOverBluetoothInOneGo("/log.txt");

            size_t freeHeapAfter = esp_get_free_heap_size();
            Serial.println("Free heap after sending file: " + String(freeHeapAfter) + " bytes");

            if (freeHeapAfter >= freeHeapBefore)
            {
              Serial.println("No memory leak detected");
            }
            else
            {
              Serial.println("Potential memory leak detected: " + String(freeHeapBefore - freeHeapAfter) + " bytes");
            }
          }
          else if (message == "4")
          {
            Serial.println("Sending log.txt");
            sendFileOverBluetoothInOneGo2("/log.txt");
            // ... (other file operations)
          }
          else if (message == "s")
          {
            buttonInterrupt_bigOled();
            const char *text = "Display switched";
            size_t size = strlen(text);
            size_t bytesWritten = SerialBT.write(reinterpret_cast<const uint8_t *>(text), size);
            if (bytesWritten != size)
            {
              // Handle error
            }
          }
          else if (message == "d")
          {
            buttonBigPressed = true;
            const char *text = "Display switched";
            size_t size = strlen(text);
            size_t bytesWritten = SerialBT.write(reinterpret_cast<const uint8_t *>(text), size);
            if (bytesWritten != size)
            {
              // Handle error
            }
          }

          // Release the mutex after the file operations are complete
          xSemaphoreGive(fileMutex);
        }
        else
        {
          Serial.println("Failed to acquire file mutex");
        }
      }
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  Serial.println("Bluetooth task has ended.");
}

void Counting(void *parameter)
{
  vTaskDelay(25 / portTICK_PERIOD_MS);
  Serial.println("Counting task has started.");
  for (;;)
  {
    int16_t count1 = 0, count2 = 0, count3 = 0;
    static int16_t last_count1 = 0, last_count2 = 0, last_count3 = 0;
    static uint32_t last_time = 0;
    uint32_t current_time = millis();

    pcnt_get_counter_value(PCNT_UNIT1, &count1);
    pcnt_get_counter_value(PCNT_UNIT2, &count2);
    // pcnt_get_counter_value(PCNT_UNIT3, &count3);
    uint32_t elapsed_time = current_time - last_time; // Time in milliseconds
    
    // Calculate frequency for PCNT_UNIT1
    if (elapsed_time > 0)
    {
      int16_t pulses1 = count1 - last_count1;
      float frequency1 = (float)pulses1 / (elapsed_time / 1000.0); // Frequency in Hz
      flowRate = frequency1 / flowSensorCalibration;
      // Update last count for unit 1
      last_count1 = count1;      
      // Serial.printf("Frequency on GPIO %d: %.2f Hz\n", PCNT_INPUT_SIG_IO1, frequency1);
      // Serial.println("flowRate: " + String(flowRate));
    }

    // Calculate frequency for PCNT_UNIT2
    if (elapsed_time > 0)
    {
      int16_t pulses2 = count2 - last_count2;
      frequency2 = (float)pulses2 / (elapsed_time / 1000.0); // Frequency in Hz
      flowRate2 = frequency2 / flowSensorCalibration2;
      // Update last count for unit 2
      last_count2 = count2;
      // Serial.printf("Frequency on GPIO %d: %.2f Hz\n", PCNT_INPUT_SIG_IO2, frequency2);
      // Serial.println("flowRate2: " + String(flowRate2));
    }
    /*
       if (elapsed_time > 0)
         {
           int16_t pulses3 = count3 - last_count3;
           float frequency3 = (float)pulses3 / (elapsed_time / 1000.0); // Frequency in Hz
           flowRate = frequency3 / flowSensorCalibration3;
           // Update last count for unit 2
           last_count3 = count3;
         // Serial.printf("Frequency on GPIO %d: %.2f Hz\n", PCNT_INPUT_SIG_IO3, frequency3);
         // Serial.println("flowRate3: " + String(flowRate3));
         }
       */
    // Update last time
    last_time = current_time;

    vTaskDelay(1100 / portTICK_PERIOD_MS); // 1000
  }
  Serial.println("Counting task has ended.");
}

TaskHandle_t Task1, Task2, Task3, Task4, Task5 = NULL;

float Read_NTC()
{
  uint8_t i;
  uint16_t sample;
  float average = 0;
  
  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++)
  {
    sample = analogRead(NTC_PIN);
    average += sample;
    delay(5);
  }
  average /= NUMSAMPLES;

  #ifdef VERBOSE_SENSOR_ENABLED  
  Serial.print("1 sample analog reading "); 
  Serial.println(sample);
  Serial.printf("Average analog reading: %.2f\n", average);
  #endif
 
  // convert the value to resistance
  float resistance = 4095 / average - 1;
  resistance = serialResistance * resistance;

  #ifdef VERBOSE_SENSOR_ENABLED
  Serial.printf("Thermistor resistance: %.2f\n", resistance);
  #endif
 
  float steinhart;
  steinhart = resistance  / nominalResistance;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= bCoefficient;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
 
  #ifdef VERBOSE_SENSOR_ENABLED
  Serial.printf("Temperature: %.3f *C\n", steinhart);
  #endif
  
  return steinhart;
}

void processLine(String line)
{
  line.trim(); // Remove any leading or trailing whitespace
  int colonIndex = line.indexOf(':');
  if (colonIndex == -1)
  {
    Serial.println("Invalid line: " + line);
    return; // Skip if no colon found
  }
  String pinName = line.substring(0, colonIndex);
  String pinValueStr = line.substring(colonIndex + 1);
  pinValueStr.trim(); // Remove any leading or trailing whitespace
  int pinValue = pinValueStr.toInt();

  if (pinName == "GSM_RX_PIN" || pinName == "GSM_TX_PIN" || pinName == "GSM_RST_PIN" || pinName == "Pin_MQ7" || pinName == "Pin_MQ8" || pinName == "DHT_SENSOR_PIN" || pinName == "DS18B20_PIN" || pinName == "flowSensorPin" || pinName == "flowSensor2Pin" || pinName == "buttonbigOled" || pinName == "EC_PIN" || pinName == "CurrentPin" || pinName == "PH_PIN")
  {
    if (pinValue < 0 || pinValue > 39)
    {
      Serial.println("Invalid pin value: " + pinValueStr);
      Serial.println("For pin: " + pinName);
      return; // Validate pin number range for ESP32
    }
    if (pinName == "GSM_RX_PIN")
    {
      GSM_RX_PIN = pinValue;
    }
    else if (pinName == "GSM_TX_PIN")
    {
      GSM_TX_PIN = pinValue;
    }
    else if (pinName == "GSM_RST_PIN")
    {
      GSM_RST_PIN = pinValue;
    }
    else if (pinName == "Pin_MQ7")
    {
      Pin_MQ7 = pinValue;
    }
    else if (pinName == "Pin_MQ8")
    {
      Pin_MQ8 = pinValue;
    }
    else if (pinName == "DHT_SENSOR_PIN")
    {
      DHT_SENSOR_PIN = pinValue;
    }
    else if (pinName == "DS18B20_PIN")
    {
      DS18B20_PIN = pinValue;
    }
    else if (pinName == "flowSensorPin")
    {
      flowSensorPin = pinValue;
    }
    else if (pinName == "flowSensor2Pin")
    {
      flowSensor2Pin = pinValue;
    }
    else if (pinName == "buttonbigOled")
    {
      buttonbigOled = pinValue;
    }

    else if (pinName == "EC_PIN")
    {
      EC_PIN = pinValue;
    }
    else if (pinName == "CurrentPin")
    {
      CurrentPin = pinValue;
    }
    else if (pinName == "PH_PIN")
    {
      PH_PIN = pinValue;
    }
    else if (pinName == "CS_PIN")
    {
      CS_PIN = pinValue;
    }
    else if (pinName == "NTC_PIN")
    {
      NTC_PIN = pinValue;
    }
    else if (pinName == "voltPin")
    {
      voltPin = pinValue;
    }
    
    }

  if (pinName == "temperatureAmount" || pinName == "phValueAmount" || pinName == "humidityAmount" || pinName == "ecValueAmount" || pinName == "flowRateAmount" || pinName == "flowRate2Amount" || pinName == "ds18b20Amount" || pinName == "acsAmount" || pinName == "h2Amount" || pinName == "voltAmount")
  {
    if (pinValue < 0 || pinValue > 500)
    {
      Serial.println("Invalid pin value: " + pinValueStr);
      Serial.println("For pin: " + pinName);
      return; // Validate range for ESP32
    }
    else if (pinValue == 0)
    {
      Serial.println("Measurement for pin: " + pinName + " is disabled");
      Serial.println("Pin value: " + pinValueStr);
      // return;
    }
    if (pinName == "temperatureAmount")
    {
      temperatureAmount = pinValue;
    }
    else if (pinName == "phValueAmount")
    {
      phValueAmount = pinValue;
    }
    else if (pinName == "humidityAmount")
    {
      humidityAmount = pinValue;
    }
    else if (pinName == "ecValueAmount")
    {
      ecValueAmount = pinValue;
    }
    else if (pinName == "flowRateAmount")
    {
      flowRateAmount = pinValue;
    }
    else if (pinName == "flowRate2Amount")
    {
      flowRate2Amount = pinValue;
    }
    else if (pinName == "acsAmount")
    {
      acsAmount = pinValue;
    }
    else if (pinName == "ds18b20Amount")
    {
      ds18b20Amount = pinValue;
    }
    else if (pinName == "h2Amount")
    {
      h2Amount = pinValue;
    }
    else if (pinName == "voltAmount")
    {
      voltAmount = pinValue;
    }
  }
  if (pinName == "mobileNumber")
  {
    if (pinValueStr.length() == 12 && pinValueStr.startsWith("+"))
    {
      mobileNumber = pinValueStr;
    }
    else
    {
      Serial.println("Invalid mobile number format: " + pinValueStr);
      Serial.println("Number length: " + pinValueStr.length());
      return;
    }
  }
}

void read_configuration()
{
  File file = SD.open("/config.txt");
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }
  vTaskDelay(50 / portTICK_PERIOD_MS);

  while (file.available())
  {
    String line = file.readStringUntil('\n');
    processLine(line);
  }
  vTaskDelay(50 / portTICK_PERIOD_MS);

  file.close();
  vTaskDelay(50 / portTICK_PERIOD_MS);
  // Output the pin numbers for verification
  Serial.print("GSM_RX_PIN: ");
  Serial.println(GSM_RX_PIN);
  Serial.print("GSM_TX_PIN: ");
  Serial.println(GSM_TX_PIN);
  Serial.print("GSM_RST_PIN: ");
  Serial.println(GSM_RST_PIN);
  Serial.print("Pin_MQ7: ");
  Serial.println(Pin_MQ7);
  Serial.print("Pin_MQ8: ");
  Serial.println(Pin_MQ8);
  Serial.print("DHT_SENSOR_PIN: ");
  Serial.println(DHT_SENSOR_PIN);
  Serial.print("DS18B20_PIN: ");
  Serial.println(DS18B20_PIN);
  Serial.print("flowSensorPin: ");
  Serial.println(flowSensorPin);
  Serial.print("flowSensor2Pin: ");
  Serial.println(flowSensor2Pin);
  Serial.print("NTC_PIN: ");
  Serial.println(NTC_PIN);
  Serial.print("buttonbigOled: ");
  Serial.println(buttonbigOled);
  Serial.print("EC_PIN: ");
  Serial.println(EC_PIN);
  Serial.print("CurrentPin: ");
  Serial.println(CurrentPin);
  Serial.print("PH_PIN: ");
  Serial.println(PH_PIN);
  Serial.print("CS_PIN: ");
  Serial.println(CS_PIN);
  Serial.print("voltPin: ");
  Serial.println(voltPin);

  Serial.print("temperatureAmount: ");
  Serial.println(temperatureAmount);
  Serial.print("phValueAmount: ");
  Serial.println(phValueAmount);
  Serial.print("humidityAmount: ");
  Serial.println(humidityAmount);
  Serial.print("ecValueAmount: ");
  Serial.println(ecValueAmount);
  Serial.print("flowRateAmount: ");
  Serial.println(flowRateAmount);
  Serial.print("flowRate2Amount: ");
  Serial.println(flowRate2Amount);
  Serial.print("temperatureAmount: ");
  Serial.println(temperatureAmount);
  Serial.print("acsAmount: ");
  Serial.println(acsAmount);
  Serial.print("ds18b20Amount: ");
  Serial.println(ds18b20Amount);
  Serial.print("h2Amount: ");
  Serial.println(h2Amount);
  Serial.print("voltAmount: ");
  Serial.println(voltAmount);

  Serial.print("Mobile phonenumber: ");
  Serial.println(mobileNumber);

  temperatureAmount = temperatureAmount;
  phValueAmount = phValueAmount;
  humidityAmount = humidityAmount;
  ecValueAmount = ecValueAmount;
  flowRateAmount = flowRateAmount;
  flowRate2Amount = flowRate2Amount;
  acsAmount = acsAmount;
  ds18b20Amount = ds18b20Amount;
  h2Amount = h2Amount;
  voltAmount = voltAmount;
  // const int MaxMeasurements = std::max({temperatureAmount, phValueAmount, humidityAmount, ecValueAmount, flowRateAmount, flowRate2Amount, acsAmount, ds18b20Amount, h2Amount, voltAmount});
}

void printVariables() {
  Serial.print("GSM_RX_PIN: "); Serial.println(GSM_RX_PIN);
  Serial.print("GSM_TX_PIN: "); Serial.println(GSM_TX_PIN);
  Serial.print("GSM_RST_PIN: "); Serial.println(GSM_RST_PIN);
  Serial.print("mobileNumber: "); Serial.println(mobileNumber);
  Serial.print("Pin_MQ7: "); Serial.println(Pin_MQ7);
  Serial.print("Pin_MQ8: "); Serial.println(Pin_MQ8);
  Serial.print("DHT_SENSOR_PIN: "); Serial.println(DHT_SENSOR_PIN);
  Serial.print("DS18B20_PIN: "); Serial.println(DS18B20_PIN);
  Serial.print("flowSensorPin: "); Serial.println(flowSensorPin);
  Serial.print("flowSensor2Pin: "); Serial.println(flowSensor2Pin);
  Serial.print("NTC_PIN: "); Serial.println(NTC_PIN);
  Serial.print("buttonbigOled: "); Serial.println(buttonbigOled);
  Serial.print("EC_PIN: "); Serial.println(EC_PIN);
  Serial.print("CurrentPin: "); Serial.println(CurrentPin);
  Serial.print("PH_PIN: "); Serial.println(PH_PIN);
  Serial.print("CS_PIN: "); Serial.println(CS_PIN);
  Serial.print("voltPin: "); Serial.println(voltPin);

  Serial.print("h2Amount: "); Serial.println(h2Amount);
  Serial.print("coAmount: "); Serial.println(coAmount);
  Serial.print("flowRateAmount: "); Serial.println(flowRateAmount);
  Serial.print("flowRate2Amount: "); Serial.println(flowRate2Amount);
  Serial.print("temperatureAmount: "); Serial.println(temperatureAmount);
  Serial.print("humidityAmount: "); Serial.println(humidityAmount);
  Serial.print("phValueAmount: "); Serial.println(phValueAmount);
  Serial.print("ecValueAmount: "); Serial.println(ecValueAmount);
  Serial.print("ds18b20Amount: "); Serial.println(ds18b20Amount);
  Serial.print("voltAmount: "); Serial.println(voltAmount);
  Serial.print("acsAmount: "); Serial.println(acsAmount);
  Serial.print("TempFlowAmount: "); Serial.println(TempFlowAmount);
}

void setup()
{
  Serial.begin(115200); // Initialize Serial for debug output
  vTaskDelay(100 / portTICK_PERIOD_MS);
  //Serial.println("All values before setup:");
  //printVariables(); 
  SD_init();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  if (SD.begin(CS_PIN))
  {
    //Serial.println("Reading config file.");
    //read_configuration();
    //Serial.println("All values after setup:");
    //printVariables(); 
  }
  
  init_displays();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  bigOled.firstPage();
  do
  {
    bigOled.setFont(u8g2_font_tinytim_tr); // u8g2_font_ncenB08_tr
    bigOled.drawStr(0, 20, "Giving time");
    bigOled.drawStr(0, 40, "for SIM800L ");
    bigOled.drawStr(0, 60, "to start up.");
  } while (bigOled.nextPage());

  gsmSerial.begin(115200, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN, false); // 38400 Initialize gsmSerial with appropriate RX/TX pins
  vTaskDelay(1000 / portTICK_PERIOD_MS);                              // Give some time for the serial communication to establish
  gsmSerial.println("AT");
  vTaskDelay(100 / portTICK_PERIOD_MS);
  gsmSerial.println("AT+IPR=115200");
  vTaskDelay(100 / portTICK_PERIOD_MS);
  gsmSerial.println("AT&W");
  vTaskDelay(100 / portTICK_PERIOD_MS);
  gsmSerial.println("AT+CSQ");
  //readGsmResponse();
  // gsmSerial.println("AT&V"); //Show saved GSM settings
  nvs_flash_init();
  stateBigOled = 1;
  getTime();
  savedTimestamp = getSavedTimestamp();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  initialize_gsm();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  mq7_init(MQ7);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  mq8_init(MQ8);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  dht_sensor.begin();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  printDS18B20Address();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  ph.begin();

  // Conductivity sensor
  EEPROM.begin(32); // needed EEPROM.begin to store calibration k in eeprom
  ec.begin();       // by default lib store calibration k since 10 change it by set ec.begin(30); to start from 30

  // Bluetooth
  SerialBT.begin("ESP32_BT");
  if (!SerialBT.connected())
  {
    Serial.println("Failed to connect to remote device. Make sure Bluetooth is turned on!");
  }
  vTaskDelay(100 / portTICK_PERIOD_MS);

  /* Flow sensor */
  pcnt_example_init(PCNT_UNIT1, PCNT_INPUT_SIG_IO1);
  pcnt_example_init(PCNT_UNIT2, PCNT_INPUT_SIG_IO2);
  //pcnt_example_init(PCNT_UNIT3, PCNT_INPUT_SIG_IO3);

  /* for switching screens  */
  pinMode(buttonbigOled, INPUT_PULLUP);                                                    // Set button pin as input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(buttonbigOled), buttonInterrupt_bigOled, FALLING); // Attach interrupt to the button pin
  vTaskDelay(100 / portTICK_PERIOD_MS);
  pinMode(CurrentPin, INPUT);
  interrupts();
  char buftest[bufferSize];
  measurementQueue = xQueueCreate(queueLength, sizeof(buftest));
  if (measurementQueue == nullptr)
  {
    Serial.println("Failed to create measurementQueue.");
    while (1)
    {
      // Handle the error appropriately
    }
  }
  else
  {
    Serial.println("measurementQueue created successfully.");
  }

  fileMutex = xSemaphoreCreateMutex();
  if (fileMutex == NULL)
  {
    Serial.println("Failed to create file mutex");
    // Handle the error appropriately
  }

  vTaskDelay(100 / portTICK_PERIOD_MS);
  esp_err_t esp_wifi_stop();
  Serial.println(String(ESP.getHeapSize() / 1024) + " Kb");
  size_t free_size = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  Serial.println("Free heap size: " + String(free_size) + ", largest free block: " + String(largest_free_block));
  vTaskDelay(100 / portTICK_PERIOD_MS);
  analogReadResolution(12); //ADC_ATTENDB_MAX
  analogSetWidth(12); 
  if(adcAttachPin(NTC_PIN) != ESP_OK) {
    Serial.println("Failed to attach ADC to NTC_PIN, current GPIO: " + String(NTC_PIN));    
  }
  
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  BaseType_t xReturned = 0;

  // if ((xReturned = xTaskCreate(sendArray, "Send Array", 10240, NULL, 2, &Task1)) != pdPASS)
  // {
  //   Serial.println("Send Array task Not created: " + String(xReturned));
  //   return;
  // }
  // vTaskSuspend(Task1);
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  // if ((xReturned = xTaskCreate(Measuring, "Measuring", 9216, NULL, 2, &Task2)) != pdPASS)
  // {
  //   Serial.println("Measurementtask Not created: " + String(xReturned));
  //   return;
  // }
  // vTaskDelay(100 / portTICK_PERIOD_MS);

  // if ((xReturned = xTaskCreate(DisplayMeasurements, "Display Measurements", 2048, NULL, 0, &Task3)) != pdPASS)
  // {
  //   Serial.println("Display Measurements task Not created: " + String(xReturned));
  //   return;
  // }
  // vTaskDelay(100 / portTICK_PERIOD_MS);

  // if ((xReturned = xTaskCreate(BluetoothListen, "Listen to Bluetooth", 2048, NULL, 0, &Task4)) != pdPASS)
  // {
  //   Serial.println(" Listen to Bluetoothtask Not created: " + String(xReturned));
  //   return;
  // }
  // vTaskDelay(100 / portTICK_PERIOD_MS);

  // if ((xReturned = xTaskCreate(Counting, "Count pulses", 1024, NULL, 1, &Task5)) != pdPASS)
  // {
  //   Serial.println("Count pulses task Not created: " + String(xReturned));
  //   return;
  // }
  // vTaskResume(Task1);
  // Serial.println("Created tasks succesfully.");

  // Serial.println("Display hight: " + String(bigOled.getDisplayHeight()) + "Display width: " + String(bigOled.getDisplayWidth()));
  // xTaskCreatePinnedToCore(MeasureAndForm, "MeasureAndForm", 4500, NULL, 1, &Task1, 1);
  
  temp_flow = Read_NTC();   // Read temperature
  Serial.println("Temperature of flowsenor: ");
  Serial.println(temp_flow);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  xTaskCreatePinnedToCore(sendArray, "Send Array", 8192, NULL, 3, &Task1, 0); // 8192 (Stack overflow)
  xTaskCreatePinnedToCore(Measuring, "Measuring", 6144, NULL, 2, &Task2, 1); //6144
  xTaskCreatePinnedToCore(DisplayMeasurements, "Display Measurements", 2048, NULL, 0, &Task3, 0);
  xTaskCreatePinnedToCore(BluetoothListen, "Listen to Bluetooth", 3072, NULL, 0, &Task4, 0); //5120
  xTaskCreatePinnedToCore(Counting, "Count pulses", 1024, NULL, 1, &Task5, 1);

  free_size = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  Serial.println("Free heap size: " + String(free_size) + ", largest free block: " + String(largest_free_block));

  stateBigOled = 2;
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

void loop()
{
  // Toggle stateBigOled from 1 to 4
  if (buttonBigPressed)
  {
    stateBigOled = (stateBigOled % 4) + 1;
    Serial.print("stateBigOled: ");
    Serial.println(stateBigOled);
    if (stateBigOled != 1)
    {
      Posting = true;
    }
    buttonBigPressed = false; // Reset button press flag
  }
  if (Serial.available())
  {
    String inputString = Serial.readString(); // Read the contents of serial buffer as a string
    Serial.println();
    Serial.print("-- Input (");
    Serial.print(inputString.length());
    Serial.println(") --");

    if (inputString.startsWith("1"))
    {
      Serial.println("Running pHSensor()");
      Serial.println("Received pH: " + String(pH()));
    }
    else if (inputString.startsWith("2"))
    {
      Serial.println("Switch OLED display");
      buttonBigPressed = true;
    }
    else if (inputString.startsWith("3"))
    {
      Serial.println("No function set");
    }
    else if (inputString.startsWith("4"))
    {
      Serial.println("No function set.");
    }
    else if (inputString.startsWith("5"))
    {
      Serial.println("Running MQ8()");
      MQ8.update();
      Serial.println("Received current: " + String(MQ8.readSensor()));
    }
    else if (inputString.startsWith("6"))
    {
      Serial.println("Running Cond()");
      Cond();
      Serial.println("Received EC: " + String(Cond()));
    }
    // readGsmResponse();
  }
  vTaskDelay(500 / portTICK_PERIOD_MS); // Small delay to avoid overwhelming the loop
}

// void MeasureAndForm(void *parameter)
// {
//   vTaskDelay(10 / portTICK_PERIOD_MS);
//   Serial.println("Inside MeasureAndForm task.");
//   Serial.println("MaxMeasurements: " + String(MaxMeasurements));
//   memset(&measurement, 0, sizeof(measurement));
//   static int temperatureCount = 0, phValueCount = 0, humidityCount = 0, ecValueCount = 0, flowRateCount2 = 0, flowRateCount = 0;
//   static int acsValueFCount = 0, ds18b20Count = 0, voltCount = 0, coCount = 0, h2Count = 0;
//   unsigned long start_time, end_time, duration_temperature, duration_phValue, duration_humidity, duration_ecValue;
//   unsigned long duration_flowRate, duration_flowRate2, duration_acsValueF, duration_ds18b20, duration_h2, duration_volt;
//   duration_temperature = duration_phValue = duration_humidity = duration_ecValue = duration_flowRate = duration_acsValueF = duration_ds18b20 = duration_h2 = duration_volt = 0;
//   Serial.println("phValueInterval: " + String(phValueInterval));
//   Serial.println("ecValueInterval: " + String(ecValueInterval));
//   Serial.println("flowRateInterval: " + String(flowRateInterval));
//   Serial.println("flowRate2Interval: " + String(flowRate2Interval));
//   Serial.println("acsValueFInterval: " + String(acsValueFInterval));
//   Serial.println("ds18b20Interval: " + String(ds18b20Interval));
//   Serial.println("voltInterval: " + String(voltInterval));
//   Serial.println("coInterval: " + String(coInterval));
//   Serial.println("h2Interval: " + String(h2Interval));
//   for (;;)
//   {
//     TickType_t startTime = xTaskGetTickCount();
//     if (currentMeasurementIndex % dht22_tempInterval == 0)
//     {
//       measurement[currentMeasurementIndex].temperature = dht_sensor.readTemperature();
//       if (isnan(measurement[currentMeasurementIndex].temperature) || isinf(measurement[currentMeasurementIndex].temperature))
//       {
//         measurement[currentMeasurementIndex].temperature = 0;
//       }
//       temperatureCount++;
//       TickType_t endTime = xTaskGetTickCount();
//       Serial.println("DHT22 Temperature duration: " + String(endTime - startTime));
//       vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
//     if (currentMeasurementIndex % dht22_humInterval == 0)
//         {
//         TickType_t startTime = xTaskGetTickCount();
//           measurement[currentMeasurementIndex].humidity = dht_sensor.readHumidity();
//           if (isnan(measurement[currentMeasurementIndex].humidity) || isinf(measurement[currentMeasurementIndex].humidity))
//           {
//             measurement[currentMeasurementIndex].humidity = 0;
//           }
//           humidityCount++;
//           TickType_t endTime = xTaskGetTickCount();
//           //Serial.println("DHT22 Humidity duration: " + String(endTime - startTime));
//           vTaskDelay(2000 / portTICK_PERIOD_MS);
//         }
//      if (currentMeasurementIndex % ecValueInterval == 0)
//         {
//         TickType_t startTime = xTaskGetTickCount();
//           measurement[currentMeasurementIndex].ecValue = Cond();
//           if (isnan(measurement[currentMeasurementIndex].ecValue) || isinf(measurement[currentMeasurementIndex].ecValue))
//           {
//             measurement[currentMeasurementIndex].ecValue = 0;
//           }
//           ecValueCount++;
//           TickType_t endTime = xTaskGetTickCount();
//           //Serial.println("EC duration: " + String(endTime - startTime));
//       vTaskDelay(2000 / portTICK_PERIOD_MS);
//         }
//         if (currentMeasurementIndex % flowRateInterval == 0)
//         {
//         TickType_t startTime = xTaskGetTickCount();
//           measurement[currentMeasurementIndex].flowRate = flowRate;
//           if (isnan(measurement[currentMeasurementIndex].flowRate) || isinf(measurement[currentMeasurementIndex].flowRate))
//           {
//             measurement[currentMeasurementIndex].flowRate = 0;
//           }
//           flowRateCount++;
//           TickType_t endTime = xTaskGetTickCount();
//           //Serial.println("Flowrate duration: " + String(endTime - startTime));
//       vTaskDelay(2000 / portTICK_PERIOD_MS);
//         }
//         if (currentMeasurementIndex % acsValueFInterval == 0)
//         {
//         TickType_t startTime = xTaskGetTickCount();
//           measurement[currentMeasurementIndex].AcsValueF = CurrentSensor_quick();
//           if (isnan(measurement[currentMeasurementIndex].AcsValueF) || isinf(measurement[currentMeasurementIndex].AcsValueF))
//           {
//             measurement[currentMeasurementIndex].AcsValueF = 0;
//           }
//           acsValueFCount++;
//           TickType_t endTime = xTaskGetTickCount();
//           //Serial.println("Current duration: " + String(endTime - startTime));
//       vTaskDelay(2000 / portTICK_PERIOD_MS);
//         }
//         if (currentMeasurementIndex % ds18b20Interval == 0)
//         {
//         TickType_t startTime = xTaskGetTickCount();
//           AllDS18B20Sensors(measurement[currentMeasurementIndex]);
//           ds18b20Count++;
//           TickType_t endTime = xTaskGetTickCount();
//           //Serial.println("DS18B20 duration: " + String(endTime - startTime));
//       vTaskDelay(2000 / portTICK_PERIOD_MS);
//         }
//     /*
//         if (currentMeasurementIndex % phValueInterval == 0)
//         {
//         TickType_t startTime = xTaskGetTickCount();
//           measurement[currentMeasurementIndex].phValue = pH();
//           if (isnan(measurement[currentMeasurementIndex].phValue) || isinf(measurement[currentMeasurementIndex].phValue))
//           {
//             measurement[currentMeasurementIndex].phValue = 0;
//           }
//           phValueCount++;
//           TickType_t endTime = xTaskGetTickCount();
//           //Serial.println("pH duration: " + String(endTime - startTime));
//         }
//         if (currentMeasurementIndex % flowRate2Interval == 0)
//         {
//         TickType_t startTime = xTaskGetTickCount();
//           measurement[currentMeasurementIndex].flowRate2 = flowRate2;
//           if (isnan(measurement[currentMeasurementIndex].flowRate2) || isinf(measurement[currentMeasurementIndex].flowRate2))
//           {
//             measurement[currentMeasurementIndex].flowRate2 = 0;
//           }
//           flowRateCount2++;
//           TickType_t endTime = xTaskGetTickCount();
//           //Serial.println("Flowrate2 duration: " + String(endTime - startTime));
//         }
//         if (currentMeasurementIndex % acsValueFInterval == 0)
//         {
//         TickType_t startTime = xTaskGetTickCount();
//           measurement[currentMeasurementIndex].AcsValueF = CurrentSensor_quick();
//           if (isnan(measurement[currentMeasurementIndex].AcsValueF) || isinf(measurement[currentMeasurementIndex].AcsValueF))
//           {
//             measurement[currentMeasurementIndex].AcsValueF = 0;
//           }
//           acsValueFCount++;
//           TickType_t endTime = xTaskGetTickCount();
//           //Serial.println("Current duration: " + String(endTime - startTime));
//         }
//         if (currentMeasurementIndex % ds18b20Interval == 0)
//         {
//         TickType_t startTime = xTaskGetTickCount();
//           AllDS18B20Sensors(measurement[currentMeasurementIndex]);
//           ds18b20Count++;
//           TickType_t endTime = xTaskGetTickCount();
//           //Serial.println("DS18B20 duration: " + String(endTime - startTime));
//         }
//         if (currentMeasurementIndex % h2Interval == 0)
//         {
//         TickType_t startTime = xTaskGetTickCount();
//           MQ8.update();
//           measurement[currentMeasurementIndex].ppmH = MQ8.readSensor();
//           if (isnan(measurement[currentMeasurementIndex].ppmH) || isinf(measurement[currentMeasurementIndex].ppmH))
//           {
//             measurement[currentMeasurementIndex].ppmH = 0;
//           }
//           h2Count++;
//           TickType_t endTime = xTaskGetTickCount();
//           //Serial.println("MQ8 H2 duration: " + String(endTime - startTime));
//         }
//         if (currentMeasurementIndex % coInterval == 0)
//         {
//         TickType_t startTime = xTaskGetTickCount();
//           MQ7.update();
//           measurement[currentMeasurementIndex].ppmCO = MQ7.readSensor();
//           if (isnan(measurement[currentMeasurementIndex].ppmCO) || isinf(measurement[currentMeasurementIndex].ppmCO))
//           {
//             measurement[currentMeasurementIndex].ppmCO = 0;
//           }
//           coCount++;
//           TickType_t endTime = xTaskGetTickCount();
//           //Serial.println("MQ7 Co duration: " + String(endTime - startTime));
//         }
//         if (currentMeasurementIndex % voltInterval == 0)
//         {
//         TickType_t startTime = xTaskGetTickCount();
//           measurement[currentMeasurementIndex].Volt = readVoltage(); // 27.22; // Placeholder for voltage
//           if (isnan(measurement[currentMeasurementIndex].Volt) || isinf(measurement[currentMeasurementIndex].Volt))
//           {
//             measurement[currentMeasurementIndex].Volt = 0;
//           }
//           voltCount++;
//           TickType_t endTime = xTaskGetTickCount();
//           //Serial.println("Volt duration: " + String(endTime - startTime));
//         }
//     /*
//     // Print the durations for each block
//     Serial.print("Temperature measurement time: "); Serial.println(duration_temperature);
//     Serial.print("pH measurement time: "); Serial.println(duration_phValue);
//     Serial.print("Humidity measurement time: "); Serial.println(duration_humidity);
//     Serial.print("EC value measurement time: "); Serial.println(duration_ecValue);
//     Serial.print("Flow rate measurement time: "); Serial.println(duration_flowRate);
//     Serial.print("ACS value measurement time: "); Serial.println(duration_acsValueF);
//     Serial.print("DS18B20 measurement time: "); Serial.println(duration_ds18b20);
//     Serial.print("H2 measurement time: "); Serial.println(duration_h2);
//     Serial.print("Voltage measurement time: "); Serial.println(duration_volt);
//     */
//     measurement[currentMeasurementIndex].ts = savedTimestamp + millis(); //micros();
//     TickType_t stopTime = xTaskGetTickCount();
//     // Serial.println("Time to save: " + String(stopTime - startTime));
//     if (currentMeasurementIndex >= (MaxMeasurements - 1))
//     {
//       bufferIndex = 0;
//       bufferIndex += snprintf(buffer, bufferSize, "[");
//       for (int i = 0; i < numMeasurements; i++)
//       {
//         bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "{\"ts\":%llu,\"values\":{\"temp\":%g, \"humidity\": %g, \"cond\": %g, \"flow\": %g, \"watt\": %g, \"Temperature\": %g}}", measurement[i].ts, measurement[i].temperature,measurement[i].humidity, measurement[i].ecValue, measurement[i].flowRate, measurement[i].AcsValueF, measurement[i].DS18B20_1);
//         // bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "{\"ts\": %llu, \"values\": {\"temp\": %g, \"humidity\": %g}}", measurement[i].ts, measurement[i].temperature, measurement[i].humidity);
//         if (i < numMeasurements - 1)
//         {
//           bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, ",");
//         }
//       }
//       bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "]");
//       Serial.println("BufferIndex size: " + String(bufferIndex));
//       if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(3000)) == pdTRUE)
//       {
//         logMeasurement(buffer);
//         xSemaphoreGive(fileMutex);
//       }
//       else
//       {
//         Serial.println("sendArrayTask: logMeasurement could not take fileMutex");
//       }
//       currentMeasurementIndex = 0;
//       temperatureCount = 0;
//       humidityCount = 0;
//       ecValueCount = 0;
//       flowRateCount = 0;
//       acsValueFCount = 0;
//       ds18b20Count = 0;
//       if (measurementQueue != NULL)
//       {
//         if (xQueueSend(measurementQueue, &buffer, portMAX_DELAY))
//         {
//           Serial.println("Successfully posted buffer to queue");
//         vTaskDelay(pdMS_TO_TICKS(100));
//         }
//       }
//       TickType_t endTime = xTaskGetTickCount();
//       Serial.println("FormArray duration: " + String(endTime - startTime));
//       vTaskDelay(60000*2 / portTICK_PERIOD_MS);
//     }
//     else
//     {
//       currentMeasurementIndex++;
//     }
//     vTaskDelay(10 / portTICK_PERIOD_MS);
//     // Monitor stack and heap usage
//     // UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
//     // size_t freeHeap = xPortGetFreeHeapSize();
//     // Serial.print("MeasuringTask stack high water mark: ");
//     // Serial.println(highWaterMark);
//     // Serial.print("Free heap size MeasuringTask: ");
//     // Serial.println(freeHeap);
//   }
//   Serial.println("Measure and form task has ended.");
// }

/*

enum TaskStatus
{
  T1_INIT,
  T2_INIT,
  T3_INIT,
  T4_INIT,
  T5_INIT,
  TaskCreatingDone
};
TaskStatus taskStatus = T1_INIT;

//In setup()
switch (taskStatus)
  {
  case T1_INIT:
    if ((xReturned = xTaskCreate(Measuring, "Measuring", 8192, NULL, 2, &Task1)) != pdPASS)
    {
      printf("task Not created\n");
      return;
    }
    taskStatus = T2_INIT;
    break;

  case T2_INIT:
    if ((xReturned = xTaskCreate(DisplayMeasurements, "Display Measurements", 2048, NULL, 0, &Task2)) != pdPASS)
    {
      printf("Display Measurements task Not created\n");
      return;
    }
    taskStatus = T3_INIT;
    break;

  case T3_INIT:
    if ((xReturned = xTaskCreate(sendArray, "Send Array", 12288, NULL, 3, &Task3)) != pdPASS)
    {
      printf("Send Array task Not created\n");
      return;
    }
    taskStatus = T4_INIT;
    break;

  case T4_INIT:
    if ((xReturned = xTaskCreate(BluetoothListen, "Listen to Bluetooth", 2048, NULL, 0, &Task4)) != pdPASS)
    {
      printf(" Listen to Bluetoothtask Not created\n");
      return;
    }
    taskStatus = T5_INIT;
    break;

  case T5_INIT:
    if ((xReturned = xTaskCreate(Counting, "Count pulses", 2048, NULL, 2, &Task5)) != pdPASS)
    {
      printf(" Count pulses task Not created\n");
      return;
    }
    taskStatus = TaskCreatingDone;
    break;

  case TaskCreatingDone:
    Serial.println("Created tasks succesfully.");
    break;
  }
  */