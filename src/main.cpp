#include "config.h"
#include <driver/adc.h>

#define DEBUG_MODE //for RTOS task debug
//#define DEBUG_MODE_2 //for X debug
//#define DEBUG_MODE_3 //for X debug
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
String definiedGSMType;
uint64_t savedTimestamp;

//Define the GSM used. Current settings allow for SIM800 or SIM808
int GSMType = 0;

/*      MQ-7 CO sensor                  */
int Pin_MQ7 = 14; 
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
int DS18B20_PIN = 26; //27;
/*      Setup Flowsensor                */
volatile float flowRate = 0.00;
int flowSensorPin = 36; 
float flowSensorCalibration = 21.00;
float frequency1 = 0.0;

volatile float flowRate2 = 0.00;
int flowSensor2Pin = 27; //26;
float flowSensorCalibration2 = 7.50; 
float frequency2 = 0.0;

#define PCNT_INPUT_SIG_IO1 flowSensorPin  // Pulse Input GPIO for PCNT_UNIT_0
#define PCNT_INPUT_SIG_IO2 flowSensor2Pin // Pulse Input GPIO for PCNT_UNIT_1
#define PCNT_UNIT1 PCNT_UNIT_0
#define PCNT_UNIT2 PCNT_UNIT_1

/*      Flowsensor Temperature        */
int NTC_PIN = 12; // ADC2_5 
uint16_t nominalResistance  = 50000;
/// The resistance value of the serial resistor used in the conductivity sensor circuit.
int serialResistance      = 90700; //3230; (met typefout 997000 wat automatisch veranderd werdt naar 55032 is de temperatuur 19.91) (En schijnbaar kan 90700 ook niet, dit is verbeterd naar 25164 en nu is de temp 38.26 gaden)
uint16_t bCoefficient     = 3950;
int TEMPERATURENOMINAL = 25;
int NUMSAMPLES = 100; 
//#define VERBOSE_SENSOR_ENABLED 
float temp_flow;                   // Global temperature reading

/*      Bluetooth                       */
BluetoothSerial SerialBT;
String message = "";
char incomingChar;
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

/*      SD card                         */
int CS_PIN = 5;
SemaphoreHandle_t fileMutex = NULL; // Handler for the log.txt file

/*      Switch screen                   */
int buttonbigOled = 13; // Pin connected to the button
extern bool buttonBigPressed;
U8G2_WITH_HVLINE_SPEED_OPTIMIZATION

/*          Conductivity sensor             */
int EC_PIN = 39; // 4;

/*          Current sensor                  */
int CurrentPin = 35; //33

/*          Voltage sensor                  */
int voltPin = 33; //35

/*          pH sensor                       */
ESP_PH ph;
int PH_PIN = 34;


float t, phvalue, stroom, Volt, DS18B20_1, DS18B20_2, DS18B20_3, DS18B20_4, DS18B20_5, humidity, ecValue, ppmCO, ppmH;
float* ds18b20Sensors[] = {&DS18B20_1, &DS18B20_2, &DS18B20_3, &DS18B20_4, &DS18B20_5};

/*          Test for Array of JSON Objects         */
// Define the queue handle
QueueHandle_t measurementQueue; //= nullptr // Define the queue handle
const int queueLength = 2;  //5    // was 10, werkte goed maar met gaten in graph

// Ctrl + d for multiple cursors
int currentMeasurementIndex = 0;

int h2Amount = 5;
int coAmount = 5;
int flowRateAmount = 10;
int flowRate2Amount = 10;
int temperatureAmount = 5;
int humidityAmount = 4;
int phValueAmount = 2;
int ecValueAmount = 2;
int ds18b20Amount = 8;
int voltAmount = 60;
int powerAmount = 60;
int acsAmount = 60;
int TempFlowAmount = 5;

const int numMeasurements = std::max({temperatureAmount, phValueAmount, humidityAmount, ecValueAmount, flowRateAmount, flowRate2Amount, acsAmount, ds18b20Amount, h2Amount, coAmount, voltAmount, TempFlowAmount, powerAmount});
const int totMeasurements = temperatureAmount + phValueAmount + humidityAmount + ecValueAmount + flowRateAmount + flowRate2Amount + acsAmount + ds18b20Amount + h2Amount + coAmount + voltAmount + TempFlowAmount + powerAmount;

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

const int bufferSize = 12288; //7168 te klein voor 2x numMesurements //6144; //8192 Waarschijnlijk te groot
char jsonBuffer[bufferSize];
int bufferIndex = 0;

/*          Test for Array of JSON Objects         */
/*
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

*/

void sendArray(void *parameter)
{
  Serial.println("Now running sendArray task.");
  char receivedBuffer[bufferSize];
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
  unsigned long previousTime = 0;

  if (measurementQueue == NULL)
  {
    Serial.println("measurementQueue is NULL. Exiting sendArray task.");
    vTaskDelete(NULL);
  }

  for (;;)
  {
    if (xQueueReceive(measurementQueue, &receivedBuffer, xTicksToWait) == pdPASS)
    {
      int queueSize = uxQueueMessagesWaiting(measurementQueue);
      Serial.println("Amount in queue (sendArray): " + String(queueSize));

      unsigned long currentTime = millis();
      unsigned long timeBetweenUsage = currentTime - previousTime;
      previousTime = currentTime;

      post_http2(receivedBuffer);
      Serial.println("Time between usage: " + String(timeBetweenUsage) + " ms");

      memset(receivedBuffer, 0, sizeof(receivedBuffer));

      #ifdef DEBUG_MODE
        UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
        size_t freeHeap = xPortGetFreeHeapSize();
        Serial.print("SendArray stack high water mark: ");
        Serial.println(highWaterMark);
        Serial.print("Free heap size SendArray: ");
        Serial.println(freeHeap);
      #endif
    }
  }
}


void Measuring(void *parameter)
{
  vTaskDelay(10 / portTICK_PERIOD_MS);
  Serial.println("Inside Measuring task.");
  Serial.println("MaxMeasurements: " + String(numMeasurements));
  static int temperatureCount = 0, phValueCount = 0, humidityCount = 0, ecValueCount = 0, flowRateCount2 = 0, 
  flowRateCount = 0, acsValueFCount = 0, ds18b20Count = 0, voltCount = 0, coCount = 0, h2Count = 0, FTCount =0, powerCount = 0;

  TickType_t measureTime, stopTime, endTimeDHTtemp, endTimepH, endTimeDHThum, endTimeEC, endTimeFlow1, endTimeFlow2, endTimeStroom, endTimeDS18B20, endTimeH2, 
  endTimeCO, endTimeVolt, endTimeFlowTemp, endTimePower;

  TickType_t startTimeMeasurement, startTime, startDHTtempTime, startDHThumTime, startECtime, startFlowRateTime, startFlowRate2Time, 
  startStroomTime, startpHTime, startDS18B20Time, startH2time, startCOtime, startVoltTime, startFlowTempTime, startPowerTime;
      #ifdef DEBUG_MODE
  Serial.println("dht22_tempInterval: " + String(dht22_tempInterval));
  Serial.println("dht22_humInterval: " + String(dht22_humInterval));
  Serial.println("phValueInterval: " + String(phValueInterval));
  Serial.println("ecValueInterval: " + String(ecValueInterval));
  Serial.println("flowRateInterval: " + String(flowRateInterval));
  Serial.println("flowRate2Interval: " + String(flowRate2Interval));
  Serial.println("acsValueFInterval: " + String(acsValueFInterval));
  Serial.println("ds18b20Interval: " + String(ds18b20Interval));
  Serial.println("voltInterval: " + String(voltInterval));
  Serial.println("coInterval: " + String(coInterval));
  Serial.println("h2Interval: " + String(h2Interval));
  #endif

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
    
    for (int i = 0; i < numMeasurements*2; i++)
    {
      //JsonObject measurement = measurementsArray.createNestedObject();
      JsonObject measurement = measurementsArray.add<JsonObject>();
      measurement["ts"] = savedTimestamp + millis();
      JsonObject values = measurement["values"].to<JsonObject>();
      
      if (i % dht22_tempInterval == 0)
        {
          startDHTtempTime = xTaskGetTickCount();
          t = dht_sensor.readTemperature();
          ss.str("");
          ss << std::fixed << std::setprecision(2) << t;
          values["T_g"] = isnan(t) || isinf(t) ? "0.0" : ss.str();
          endTimeDHTtemp = xTaskGetTickCount();
        }

      if (i % ds18b20Interval == 0)
      {
        startDS18B20Time = xTaskGetTickCount();
        sensors.requestTemperatures();
        int numberOfDevices = sensors.getDeviceCount();
        for (int j = 0; j < numberOfDevices && j < 5; j++)
        {
          if (sensors.getAddress(tempDeviceAddress, j))
          {
            float tempC = sensors.getTempC(tempDeviceAddress);
            ss.str("");
            ss << std::fixed << std::setprecision(3) << tempC;
            values["T" + String(j+1)] = isnan(tempC) || isinf(tempC) ? "0.0" : ss.str();        
            *ds18b20Sensors[j] = tempC; // save tempC to corresponding sensor variable    
          }          
        }
        endTimeDS18B20 = xTaskGetTickCount();
      }

      if (i % phValueInterval == 0)
      {
        startpHTime = xTaskGetTickCount();  
        phvalue = pH();
        ss.str("");
        ss << std::fixed << std::setprecision(3) << phvalue;
        values["pH"] = isnan(phvalue) || isinf(phvalue) ? "0.00" : ss.str();
        endTimepH = xTaskGetTickCount();
      }

      if (i % acsValueFInterval == 0)
      {
        startStroomTime = xTaskGetTickCount();
        stroom = CurrentSensor_724();
        vTaskDelay(100);
        ss.str("");
        ss << std::fixed << std::setprecision(3) << stroom;
        values["A"] = isnan(stroom) || isinf(stroom) ? "0.00": ss.str();
        endTimeStroom = xTaskGetTickCount();
      }

      if (i % dht22_humInterval == 0)
      {
        startDHThumTime = xTaskGetTickCount();
        humidity = dht_sensor.readHumidity();
        ss.str("");
        ss << std::fixed << std::setprecision(3) << humidity;
        values["Hum"] = isnan(humidity) || isinf(humidity) ? "0.00" : ss.str();
        endTimeDHThum = xTaskGetTickCount();
      }

      if (i % ecValueInterval == 0)
      {
        startECtime = xTaskGetTickCount();
        ecValue = Cond();
        ss.str("");
        ss << std::fixed << std::setprecision(3) << ecValue;
        values["Cond"] = isnan(ecValue) || isinf(ecValue) ? "0.00": ss.str();
        endTimeEC = xTaskGetTickCount();
      }

      if (i % flowRateInterval == 0)
      {
        startFlowRateTime = xTaskGetTickCount();
        //flowRate = flowSens();
        ss.str("");
        ss << std::fixed << std::setprecision(2) << flowRate; 
        values["Flow1"] = isnan(flowRate) || isinf(flowRate) ? "0.00": ss.str();
        endTimeFlow1 = xTaskGetTickCount();
      }

      if (i % flowRate2Interval == 0)
      {
        startFlowRate2Time = xTaskGetTickCount();
        ss.str("");
        ss << std::fixed << std::setprecision(2) << flowRate2;
        values["Flow2"] = isnan(flowRate2) || isinf(flowRate2) ? "0.00": ss.str();
        endTimeFlow2 = xTaskGetTickCount();
      }

      if (i % FlowTempinterval == 0)
      {
        startFlowTempTime = xTaskGetTickCount();
        temp_flow = Read_NTC();   // Read temperature
        Serial.println("TempFlow1: " + String(temp_flow));
        ss.str("");
        ss << std::fixed << std::setprecision(1) << temp_flow;
        values["FT"] = isnan(temp_flow) || isinf(temp_flow) ? "0.00": ss.str();
        endTimeFlowTemp = xTaskGetTickCount();
      }
      
      if (i % coInterval == 0)
      {
        startCOtime = xTaskGetTickCount();
        MQ7.update();
        ppmCO = MQ7.readSensor();
        ss.str("");
        ss << std::fixed << std::setprecision(3) << ppmCO;
        values["CO"] = isnan(ppmCO) || isinf(ppmCO) ? "0.00": ss.str();
        endTimeCO = xTaskGetTickCount();
      }

      if (i % h2Interval == 0)
      {
        startH2time = xTaskGetTickCount();
        MQ8.update();
        ppmH = MQ8.readSensor();
        ss.str("");
        ss << std::fixed << std::setprecision(3) << ppmH; 
        values["H2"] = isnan(ppmH) || isinf(ppmH) ? "0.00": ss.str();
        endTimeH2 = xTaskGetTickCount();
      }

      if (i % voltInterval == 0)
      {
        startVoltTime = xTaskGetTickCount();
        Volt = readVoltage();
        ss.str("");
        ss << std::fixed << std::setprecision(3) << Volt;
        values["V"] = isnan(Volt) || isinf(Volt) ? "0.00": ss.str();      
        endTimeVolt = xTaskGetTickCount();
      }
      
      if (i % powerInterval == 0)
      {
        startPowerTime = xTaskGetTickCount();
        float power = Volt * stroom;
        ss.str("");
        ss << std::fixed << std::setprecision(3) << power;
        values["P"] = isnan(power) || isinf(power) ? "0.00": ss.str();
        endTimePower = xTaskGetTickCount();
      }
      
      currentMeasurementIndex++;
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    
    stopTime = xTaskGetTickCount();
    measureTime = stopTime - startTimeMeasurement;

    if (currentMeasurementIndex >= (numMeasurements - 1)) // MaxMeasurements -1
    {    
      // printCMD();
      //#ifdef DEBUG_MODE
      Serial.println("Measurement duration: " + String(measureTime));
      Serial.println("All of the following durations are singular, to obtain the total time you need to multiply by the number of measurements");
      Serial.println("Temp duration:   " + String(endTimeDHTtemp - startDHTtempTime) + "| Humi duration: " + String(endTimeDHThum- startDHThumTime) + "| DS18B20 duration:  " + String(endTimeDS18B20 - startDS18B20Time) + "| Power duration:  " + String(endTimePower - startPowerTime));
      Serial.println("pH duration:     " + String(endTimepH - startpHTime)            + "| EC duration:   " + String(endTimeEC - startECtime)   + "| Flowrate duration: " + String(endTimeFlow1 - startFlowRateTime) + "| Flowrate2 duration: " + String(endTimeFlow2 - startFlowRate2Time));
      Serial.println("Stroom duration: " + String(endTimeStroom - startStroomTime)    + "| Volt duration: " + String(endTimeVolt - startVoltTime)    + "| MQ8 H2 duration:   " + String(endTimeH2 - startH2time)          + "| MQ7 Co duration:    " + String(endTimeCO - startCOtime));
      Serial.println("FlowTemp duration: " + String(endTimeFlowTemp - startFlowTempTime));
      Serial.println();
      //#endif
      /* Add linebreaks and whitespaces to the end of the JSON document */
      // serializeJsonPretty(doc, jsonBuffer);

      /* Minifies the JSON document e.g. no linebreaks and no whitespaces */
      ArduinoJson::serializeJson(doc2, jsonBuffer);
      #ifdef DEBUG_MODE
      Serial.println("jsonBuffer created in Measuring task and sent to queue: ");
      Serial.println(jsonBuffer);
      #endif
      if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(3000)) == pdTRUE)
      {
        if (doc2 != nullptr)
        {
          //logMeasurement((&doc2)->as<String>().c_str());
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
          Serial.println("DataSize of buffer: " + String(sizeof(jsonBuffer)));
          int queueSize = uxQueueMessagesWaiting(measurementQueue);
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
      #ifdef DEBUG_MODE
    // Monitor stack and heap usage
    UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
    size_t freeHeap = xPortGetFreeHeapSize();
    Serial.print("MeasuringTask stack high water mark: ");
    Serial.println(highWaterMark);
    Serial.print("Free heap size MeasuringTask: ");
    Serial.println(freeHeap);
    #endif
  }
  Serial.println("Measuring task has ended.");
}

void DisplayMeasurements(void *parameter)
{
  vTaskDelay(15 / portTICK_PERIOD_MS);
  Serial.println("Inside Display Measurements task.");
    String RX =               "GSM_RX_PIN : " + String(GSM_RX_PIN);
    String TX =               "GSM_TX_PIN : " + String(GSM_TX_PIN);
    String definiedGSMTypeDis="GSM: " + String(definiedGSMType);
    String Pin_MQ7Dis =       "Pin_MQ7    : " + String(Pin_MQ7);
    String Pin_MQ8Dis =       "Pin_MQ8    : " + String(Pin_MQ8);
    String DHT_SENSOR_PINDis ="DHT22 PIN  : " + String(DHT_SENSOR_PIN);
    String DS18B20_PINDis =   "DS18B20_PIN: " + String(DS18B20_PIN);
    String flowSensorPinDis = "flow Pin   : " + String(flowSensorPin);
    String flowSensor2PinDis ="flow2 Pin  : " + String(flowSensor2Pin);
    String NTC_PINDis =       "NTC_PIN    : " + String(NTC_PIN);
    String EC_PINDis =        "EC_PIN     : " + String(EC_PIN);
    String CurrentPinDis =    "CurrentPin : " + String(CurrentPin);
    String PH_PINDis =        "PH_PIN     : " + String(PH_PIN);
    String voltPinDis =       "voltPin    : " + String(voltPin);

    String h2AmountDis =        "h2     : " + String(h2Amount);
    String coAmountDis =        "co     : " + String(coAmount);
    String flowRateAmountDis =  "flowR  : " + String(flowRateAmount);
    String flowRate2AmountDis = "flowR2 : " + String(flowRate2Amount);
    String temperatureAmountDis="temp   : " + String(temperatureAmount);
    String humidityAmountDis =  "hum    : " + String(humidityAmount);
    String phValueAmountDis =   "phV    : " + String(phValueAmount);
    String ecValueAmountDis =   "ec     : " + String(ecValueAmount);
    String ds18b20AmountDis =   "ds18b20: " + String(ds18b20Amount);
    String voltAmountDis =      "volt   : " + String(voltAmount);
    String acsAmountDis =       "Current: " + String(acsAmount);
    String powerAmountDis =     "Power  : " + String(powerAmount);
    String TempFlowAmountDis =  "Temp   : " + String(TempFlowAmount);
    int i = 1;
  for (;;)
  {
    String flowDis = "Flow: " + String(flowRate) + " L/min";
    String flowDis2 = "Flow2: " + String(flowRate2) + " L/min";
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
        bigOled.setFont(u8g2_font_micro_mr); // Was u8g2_font_ncenB08_tr
        bigOled.setDisplayRotation(U8G2_R1);
        bigOled.drawStr(0, 8, "Pinout registerd.");
        bigOled.drawStr(0, 16, RX.c_str());
        bigOled.drawStr(0, 24, TX.c_str());
        bigOled.drawStr(0, 32, mobileNumber.c_str());
        bigOled.drawStr(0, 40, voltPinDis.c_str());
        bigOled.drawStr(0, 48, definiedGSMTypeDis.c_str());
        bigOled.drawStr(0, 56, Pin_MQ7Dis.c_str());
        bigOled.drawStr(0, 64, Pin_MQ8Dis.c_str());
        bigOled.drawStr(0, 72, DHT_SENSOR_PINDis.c_str());
        bigOled.drawStr(0, 80, DS18B20_PINDis.c_str());
        bigOled.drawStr(0, 88, flowSensorPinDis.c_str());
        bigOled.drawStr(0, 96, flowSensor2PinDis.c_str());
        bigOled.drawStr(0, 104, NTC_PINDis.c_str());
        bigOled.drawStr(0, 112, EC_PINDis.c_str());
        bigOled.drawStr(0, 120, CurrentPinDis.c_str());
        bigOled.drawStr(0, 128, PH_PINDis.c_str());
      } while (bigOled.nextPage());
    }

    if (stateBigOled == 5)
    {
      bigOled.firstPage();
      do
      {
        bigOled.setFont(u8g2_font_micro_mr); // Was u8g2_font_ncenB08_tr
        bigOled.setDisplayRotation(U8G2_R1);
        bigOled.drawStr(0, 8, "Measuring amount: ");
        bigOled.drawStr(0, 16, h2AmountDis.c_str());
        bigOled.drawStr(0, 24, coAmountDis.c_str());
        bigOled.drawStr(0, 32, flowRateAmountDis.c_str());
        bigOled.drawStr(0, 40, flowRate2AmountDis.c_str());
        bigOled.drawStr(0, 48, temperatureAmountDis.c_str());
        bigOled.drawStr(0, 56, humidityAmountDis.c_str());
        bigOled.drawStr(0, 64, phValueAmountDis.c_str());
        bigOled.drawStr(0, 72, ecValueAmountDis.c_str());
        bigOled.drawStr(0, 80, ds18b20AmountDis.c_str());
        bigOled.drawStr(0, 88, voltAmountDis.c_str());
        bigOled.drawStr(0, 96, acsAmountDis.c_str());
        bigOled.drawStr(0, 104, powerAmountDis.c_str());
        bigOled.drawStr(0, 112, TempFlowAmountDis.c_str());
      } while (bigOled.nextPage());
    }  

    if (stateBigOled == 6)
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
    if(i!=0){        
     #ifdef DEBUG_MODE
        UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
        size_t freeHeap = xPortGetFreeHeapSize();
        Serial.print("Display stack high water mark: ");
        Serial.println(highWaterMark);
        Serial.print("Free heap size Display: ");
        Serial.println(freeHeap);
    #endif
    i = 0;
    }

  }
  Serial.println("Display task has ended.");
}

void BluetoothListen(void *parameter)
{
  vTaskDelay(20 / portTICK_PERIOD_MS);
  Serial.println("Inside Bluetooth task.");
  int i = 0;
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
          }
          else if (message == "2")
          {
            Serial.println("Sending log.txt");
            sendFileOverBluetooth("/log.txt");
          } 
          else if (message == "3")
          {
            size_t freeHeapBefore = esp_get_free_heap_size();
            Serial.println("Free heap before sending file: " + String(freeHeapBefore) + " bytes");

            sendLargeFileOverBluetooth("/log.txt");

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
            sendLargeFileOverBluetooth("/log.txt");
          }
          else if (message == "a")
          {
            Serial.println("Sending Test1_100.txt");
            sendLargeFileOverBluetooth("/Tetst1_100.txt");
          } 
          else if (message == "b")
          {
            Serial.println("Sending Test2_1k.txt");
            sendLargeFileOverBluetooth("/Test2_1k.txt");
          } 
          else if (message == "c")
          {
            Serial.println("Sending Test3_10k.txt");
            sendLargeFileOverBluetooth("/Test3_10k.txt");
          } 
           else if (message == "7")
          {
            Serial.println("Sending Test4_100k.txt");
            sendLargeFileOverBluetooth("/Test4_100k.txt");
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
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if(i==5){
     #ifdef DEBUG_MODE
        UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
        size_t freeHeap = xPortGetFreeHeapSize();
        Serial.print("Bluetooth stack high water mark: ");
        Serial.println(highWaterMark);
        Serial.print("Free heap size Bluetooth: ");
        Serial.println(freeHeap);
      #endif
    i = 0;
    }
    i++;
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
      frequency1 = (float)pulses1 / (elapsed_time / 1000.0); // Frequency in Hz
      flowRate = frequency1 / flowSensorCalibration;
      // Update last count for unit 1
      last_count1 = count1;      
      #ifdef DEBUG_MODE_2
      Serial.printf("Frequency on GPIO %d: %.2f Hz\n", PCNT_INPUT_SIG_IO1, frequency1);
      Serial.println("flowRate: " + String(flowRate));
      #endif
    }

    // Calculate frequency for PCNT_UNIT2
    if (elapsed_time > 0)
    {
      int16_t pulses2 = count2 - last_count2;
      frequency2 = (float)pulses2 / (elapsed_time / 1000.0); // Frequency in Hz
      flowRate2 = frequency2 / flowSensorCalibration2;
      // Update last count for unit 2
      last_count2 = count2;
      #ifdef DEBUG_MODE_2
      Serial.printf("Frequency on GPIO %d: %.2f Hz\n", PCNT_INPUT_SIG_IO2, frequency2);
      Serial.println("flowRate2: " + String(flowRate2));
      #endif
    }
    // Update last time
    last_time = current_time;

    vTaskDelay(1100 / portTICK_PERIOD_MS); // 1000
  }
  Serial.println("Counting task has ended.");
}

TaskHandle_t Task1, Task2, Task3, Task4, Task5 = NULL;

float Read_NTC2()
{
    uint8_t i;
    uint16_t sample;
    float average = 0;

    // Take N samples in a row, with a slight delay
    for (i = 0; i < NUMSAMPLES; i++)
    {
        sample = analogRead(NTC_PIN);
        average += sample;
        delay(1);
    }
    average /= NUMSAMPLES;
    float Vout = (average / 4095.0) * 3.3; // Adjust to match actual ADC reference voltage
    // Calculate the thermistor resistance
    float resistance = (serialResistance * (5.0 / Vout));
    // Calculate temperature using Steinhart-Hart equation
    float steinhart = log(resistance / nominalResistance); // ln(R/R0)
    steinhart /= bCoefficient; // 1/B * ln(R/R0)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart; // Invert
    steinhart -= 273.15; // Convert to Celsius

    #ifdef DEBUG_MODE_2
    Serial.printf("Calculated Voltage (Vout): %.2f mV\n", Vout * 1000);
    Serial.printf("Calculated Thermistor Resistance: %.2f Ohms\n", resistance);
    Serial.printf("Calculated Temperature: %.3f °C\n", steinhart);
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

  if (pinName == "temperatureAmount" || pinName == "phValueAmount" || pinName == "humidityAmount" || pinName == "ecValueAmount" || pinName == "flowRateAmount" || pinName == "flowRate2Amount" || pinName == "ds18b20Amount" || pinName == "acsAmount" || pinName == "h2Amount" || pinName == "voltAmount" || pinName == "powerAmount" || pinName == "TempFlowAmount" )
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
    else if (pinName == "powerAmount")
    {
      powerAmount = pinValue;
    }
    else if (pinName == "TempFlowAmount")
    {
      TempFlowAmount = pinValue;
    }
    else if (pinName == "flowSensorCalibration")
    {
      flowSensorCalibration = pinValue;
    }
    else if (pinName == "flowSensorCalibration2")
    {
      flowSensorCalibration2 = pinValue;
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

  if (pinName == "GSM_type")
    {
      definiedGSMType = pinValueStr;
      if (pinValueStr == "SIM800")
      {
        GSMType = 1;
      }
      else if (pinValueStr == "SIM808")
      {
        GSMType = 2;
      }
       else if (pinValueStr == "GA6")
      {
        GSMType = 3;
      }
       else if (pinValueStr == "A9G")
      {
        GSMType = 4;
      }
      else
      {
        Serial.println("Invalid GSM type: " + pinValueStr);
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

  Serial.print("Phone Number: "); Serial.println(mobileNumber); 
  Serial.print("Definied GSM Type: "); Serial.println(definiedGSMType);
}

void setup()
{
  Serial.begin(115200); // Initialize Serial for debug output
  vTaskDelay(100 / portTICK_PERIOD_MS);
  #ifdef DEBUG_MODE_2
  Serial.println("All values before setup:");
  printVariables(); 
  #endif
  SD_init();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  //#define SPI_BUFFER_SIZE 512
  //uint8_t spi_buffer[SPI_BUFFER_SIZE];
  SPIClass spi = SPIClass(VSPI); //HSPI
  //spi.begin(18, 19, 23, 5); // Adjust pins as needed
  //if (SD.begin(CS_PIN, spi, 80000000))
  
  if (SD.begin(CS_PIN, spi, 80000000))
  {
    Serial.println("Reading config file.");
    //read_configuration();
    //Serial.println("All values after setup:");
    //printVariables(); 
    //Serial.println("");
  }
  GSMType = 2;
  File root;
  root = SD.open("/");
  printDirectory(root, 0);
  vTaskDelay(100 / portTICK_PERIOD_MS);
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
  stateBigOled = 1;
  gsmSerial.begin(115200, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN, false); // 38400 Initialize gsmSerial with appropriate RX/TX pins
  vTaskDelay(1000 / portTICK_PERIOD_MS);                            // Give some time for the serial communication to establish
  gsmSerial.println("AT");
  vTaskDelay(100 / portTICK_PERIOD_MS);
  gsmSerial.println("AT+IPR=115200");
  vTaskDelay(100 / portTICK_PERIOD_MS);
  gsmSerial.println("AT&W");
  vTaskDelay(100 / portTICK_PERIOD_MS);
  gsmSerial.println("AT+CSQ");
  vTaskDelay(100 / portTICK_PERIOD_MS);
  //initialize_gsm();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  gsmSerial.println("AT+CCID=?"); 
  vTaskDelay(100 / portTICK_PERIOD_MS);
  gsmSerial.println("AT+CCID"); 
  //readGsmResponse();
  //gsmSerial.println("AT&V"); //Show saved GSM settings
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  nvs_flash_init();
  //getTime();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  savedTimestamp = getSavedTimestamp();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  mq7_init(MQ7);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  mq8_init(MQ8);
  vTaskDelay(100 / portTICK_PERIOD_MS);
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
  SerialBT.begin(921600);
  
  /* Flow sensor */
  pcnt_example_init(PCNT_UNIT1, PCNT_INPUT_SIG_IO1);
  pcnt_example_init(PCNT_UNIT2, PCNT_INPUT_SIG_IO2);

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
  
  vTaskDelay(100 / portTICK_PERIOD_MS);
  //xTaskCreatePinnedToCore(sendArray, "Send Array", 9216 , NULL, 3, &Task1, 0); // 8192 (Stack overflow)
  xTaskCreatePinnedToCore(sendArray, "Send Array", 15360 , NULL, 3, &Task1, 0); 
  xTaskCreatePinnedToCore(Measuring, "Measuring", 6144, NULL, 2, &Task2, 1); //6144
  xTaskCreatePinnedToCore(DisplayMeasurements, "Display Measurements", 3072, NULL, 0, &Task3, 0);
  xTaskCreatePinnedToCore(BluetoothListen, "Listen to Bluetooth", 5120, NULL, 0, &Task4, 0); //9216 probs too low //5120 //3072 Stack overflow with new functions for " one go" 
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
