#include "config.h"

/*      GSM Module Setup     */
HardwareSerial gsmSerial(2); // Use UART2
#define GSM_RX_PIN 17        // 16
#define GSM_TX_PIN 16        // 17
#define GSM_RST_PIN 4       // Not connected
String apn = "data.lycamobile.nl";
String apn_User = "lmnl";
String apn_Pass = "plus";
char httpapi[] = "http://jrbubuntu.ddns.net:5000/api/telemetry"; // Not yet tested as String
// char httpapi[] = "http://145.131.6.212/api/v1/HR/gl3soo07qchjimbsdwln/telemetry";
String mobileNumber = "+31614504288";

extern time_t timestamp; // Remove extern
uint64_t savedTimestamp;

/*      MQ-7 CO sensor                  */
#define Pin_MQ7 14 // 35
MQUnifiedsensor MQ7("ESP32", 5, 12, Pin_MQ7, "MQ-7");
/*      MQ-8 H2 sensor                   */
#define Pin_MQ8 32
MQUnifiedsensor MQ8("ESP32", 5, 12, Pin_MQ8, "MQ-8");

/*      DHT22 - Temperature and Humidity */
#include "DHT.h"
#define DHT_SENSOR_PIN 25
#define DHT_SENSOR_TYPE DHT22
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

/*      Setup Temperature sensor        */
const int DS18B20_PIN = 27;

/*      Setup Flowsensor                */
volatile float flowRate, flowRate2, flowRate3 = 0.00;
const int flowSensorPin = 36; // 14
const float flowSensorCalibration = 21.00;

const int flowSensor2Pin = 26;
const float flowSensorCalibration2 = 7.50;
// const int flowSensor3Pin = 14;
const float flowSensorCalibration3 = 11.0;

#define PCNT_INPUT_SIG_IO1 flowSensorPin  // Pulse Input GPIO for PCNT_UNIT_0
#define PCNT_INPUT_SIG_IO2 flowSensor2Pin // Pulse Input GPIO for PCNT_UNIT_1
// #define PCNT_INPUT_SIG_IO3 flowSensor3Pin // Pulse Input GPIO for PCNT_UNIT_2
#define PCNT_UNIT1 PCNT_UNIT_0
#define PCNT_UNIT2 PCNT_UNIT_1
// #define PCNT_UNIT3 PCNT_UNIT_2

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
const int queueLength = 5;      // was 10, werkte goed maar met gaten in graph

// Ctrl + d for multiple cursors
int currentMeasurementIndex = 0;

int h2Amount = 2;
int coAmount = 2;
int flowRateAmount = 5;
int flowRate2Amount = 5;
int temperatureAmount = 4;
int humidityAmount = 4;
int phValueAmount = 2;
int ecValueAmount = 2;
int ds18b20Amount = 8;
int voltAmount = 50;
int acsAmount = 50;

const int numMeasurements = std::max({temperatureAmount, phValueAmount, humidityAmount, ecValueAmount, flowRateAmount, flowRate2Amount, acsAmount, ds18b20Amount, h2Amount, coAmount, voltAmount});
const int totMeasurements = temperatureAmount + phValueAmount + humidityAmount + ecValueAmount + flowRateAmount + flowRate2Amount + acsAmount + ds18b20Amount + h2Amount + coAmount + voltAmount;
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
const int h2Interval = numMeasurements / h2Amount;
const int coInterval = numMeasurements / coAmount;

String intervals = "{\"Intervals\":{\"numMeasurements\":" + String(numMeasurements) + ",\"temperatureInterval\":" + String(dht22_tempInterval) + ",\"phValueInterval\":" + String(phValueInterval) + ",\"humidityInterval\":" + String(dht22_humInterval) + ",\"ecValueInterval\":" + String(ecValueInterval) + ",\"flowRateInterval\":" + String(flowRateInterval) + ",\"flowRate2Interval\":" + String(flowRate2Interval) + ",\"acsValueFInterval\":" + String(acsValueFInterval) + ",\"ds18b20Interval\":" + String(ds18b20Interval) + ",\"voltInterval\":" + String(voltInterval) + ",\"h2Interval\":" + String(h2Interval) + ",\"coInterval\":" + String(coInterval) + "},";
const int bufferSize = 6144; // 5120;
char jsonBuffer[bufferSize];
int bufferIndex = 0;

int GSM_RX_PIN2, GSM_TX_PIN2, GSM_RST_PIN2, Pin_MQ72, Pin_MQ82, DHT_SENSOR_PIN2, DS18B20_PIN2, flowSensorPin2, flowSensor2Pin2;

/*          Test for Array of JSON Objects         */
void sendArray(void *parameter)
{
  Serial.println("Now running sendArray task.");
  char receivedBuffer[bufferSize];
  memset(receivedBuffer, 0, sizeof(receivedBuffer));
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
          post_http2(receivedBuffer);
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

/*          Backup Measuring Task         */
/*
void Measuring(void *parameter)
{
  vTaskDelay(10 / portTICK_PERIOD_MS);
  Serial.println("Inside Measuring task.");
  Serial.println("MaxMeasurements: " + String(MaxMeasurements));
  memset(&measurement, 0, sizeof(measurement));
  static int temperatureCount = 0, phValueCount = 0, humidityCount = 0, ecValueCount = 0, flowRateCount2 = 0, flowRateCount = 0;
  static int acsValueFCount = 0, ds18b20Count = 0, voltCount = 0, coCount = 0, h2Count = 0;
  unsigned long start_time, end_time, duration_temperature, duration_phValue, duration_humidity, duration_ecValue;
  unsigned long duration_flowRate, duration_flowRate2, duration_acsValueF, duration_ds18b20, duration_h2, duration_volt;
  duration_temperature = duration_phValue = duration_humidity = duration_ecValue = duration_flowRate = duration_acsValueF = duration_ds18b20 = duration_h2 = duration_volt = 0;
  TickType_t startTimeFormArray, measureTime, stopTime, startTimeMeasurement, startTime, endTimeDHTtemp, endTimepH, endTimeDHThum, endTimeEC, endTimeFlow1, endTimeFlow2, endTimeACS712, endTimeDS18B20, endTimeH2, endTimeCO, endTimeVolt;

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

  for (;;)
  {
    startTimeMeasurement = xTaskGetTickCount();

    if (currentMeasurementIndex % dht22_tempInterval == 0)
    {
      startTime = xTaskGetTickCount();
      measurement[currentMeasurementIndex].temperature = dht_sensor.readTemperature();
      if (isnan(measurement[currentMeasurementIndex].temperature) || isinf(measurement[currentMeasurementIndex].temperature))
      {
        measurement[currentMeasurementIndex].temperature = 0;
      }
      temperatureCount++;
      endTimeDHTtemp = xTaskGetTickCount() - startTime;
    }

    if (currentMeasurementIndex % phValueInterval == 0)
    {
      startTime = xTaskGetTickCount();
      measurement[currentMeasurementIndex].phValue = pH();
      if (isnan(measurement[currentMeasurementIndex].phValue) || isinf(measurement[currentMeasurementIndex].phValue))
      {
        measurement[currentMeasurementIndex].phValue = 0;
      }
      phValueCount++;
      endTimepH = xTaskGetTickCount() - startTime;
    }

    if (currentMeasurementIndex % dht22_humInterval == 0)
    {
      startTime = xTaskGetTickCount();
      measurement[currentMeasurementIndex].humidity = dht_sensor.readHumidity();
      if (isnan(measurement[currentMeasurementIndex].humidity) || isinf(measurement[currentMeasurementIndex].humidity))
      {
        measurement[currentMeasurementIndex].humidity = 0;
      }
      humidityCount++;
      endTimeDHThum = xTaskGetTickCount() - startTime;
    }

    if (currentMeasurementIndex % ecValueInterval == 0)
    {
      startTime = xTaskGetTickCount();
      measurement[currentMeasurementIndex].ecValue = Cond();
      if (isnan(measurement[currentMeasurementIndex].ecValue) || isinf(measurement[currentMeasurementIndex].ecValue))
      {
        measurement[currentMeasurementIndex].ecValue = 0;
      }
      ecValueCount++;
      endTimeEC = xTaskGetTickCount() - startTime;
    }

    if (currentMeasurementIndex % flowRateInterval == 0)
    {
      startTime = xTaskGetTickCount();
      measurement[currentMeasurementIndex].flowRate = flowRate;
      if (isnan(measurement[currentMeasurementIndex].flowRate) || isinf(measurement[currentMeasurementIndex].flowRate))
      {
        measurement[currentMeasurementIndex].flowRate = 0;
      }
      flowRateCount++;
      endTimeFlow1 = xTaskGetTickCount() - startTime;
    }

    if (currentMeasurementIndex % flowRate2Interval == 0)
    {
      startTime = xTaskGetTickCount();
      measurement[currentMeasurementIndex].flowRate2 = flowRate2;
      if (isnan(measurement[currentMeasurementIndex].flowRate2) || isinf(measurement[currentMeasurementIndex].flowRate2))
      {
        measurement[currentMeasurementIndex].flowRate2 = 0;
      }
      flowRateCount2++;
      endTimeFlow2 = xTaskGetTickCount() - startTime;
    }

    if (currentMeasurementIndex % acsValueFInterval == 0)
    {
      startTime = xTaskGetTickCount();
      measurement[currentMeasurementIndex].AcsValueF = CurrentSensor_quick();
      if (isnan(measurement[currentMeasurementIndex].AcsValueF) || isinf(measurement[currentMeasurementIndex].AcsValueF))
      {
        measurement[currentMeasurementIndex].AcsValueF = 0;
      }
      acsValueFCount++;
      endTimeACS712 = xTaskGetTickCount() - startTime;
    }

    if (currentMeasurementIndex % ds18b20Interval == 0)
    {
      startTime = xTaskGetTickCount();
      AllDS18B20Sensors(measurement[currentMeasurementIndex]);
      ds18b20Count++;
      endTimeDS18B20 = xTaskGetTickCount() - startTime;
    }

    if (currentMeasurementIndex % h2Interval == 0)
    {
      startTime = xTaskGetTickCount();
      MQ8.update();
      measurement[currentMeasurementIndex].ppmH = MQ8.readSensor();
      if (isnan(measurement[currentMeasurementIndex].ppmH) || isinf(measurement[currentMeasurementIndex].ppmH))
      {
        measurement[currentMeasurementIndex].ppmH = 0;
      }
      h2Count++;
      endTimeH2 = xTaskGetTickCount() - startTime;
    }

    if (currentMeasurementIndex % coInterval == 0)
    {
      startTime = xTaskGetTickCount();
      MQ7.update();
      measurement[currentMeasurementIndex].ppmCO = MQ7.readSensor();
      if (isnan(measurement[currentMeasurementIndex].ppmCO) || isinf(measurement[currentMeasurementIndex].ppmCO))
      {
        measurement[currentMeasurementIndex].ppmCO = 0;
      }
      coCount++;
      endTimeCO = xTaskGetTickCount() - startTime;
      }

    if (currentMeasurementIndex % voltInterval == 0)
    {
      startTime = xTaskGetTickCount();
      measurement[currentMeasurementIndex].Volt = readVoltage(); // 27.22; // Placeholder for voltage
      if (isnan(measurement[currentMeasurementIndex].Volt) || isinf(measurement[currentMeasurementIndex].Volt))
      {
        measurement[currentMeasurementIndex].Volt = 0;
      }
      voltCount++;
      endTimeVolt = xTaskGetTickCount() - startTime;
    }

    measurement[currentMeasurementIndex].ts = savedTimestamp + millis();
    stopTime = xTaskGetTickCount();
    measureTime = stopTime - startTimeMeasurement;
    vTaskDelay(50 / portTICK_PERIOD_MS);
    if (currentMeasurementIndex >= (numMeasurements-1)) //MaxMeasurements -1
    {
      startTimeFormArray = xTaskGetTickCount();
      StaticJsonDocument<bufferSize> doc;
      // Add the intervals objects
      JsonObject intervals = doc.createNestedObject("intervals"); //Intervals (capital)
      intervals["numMeasurements"] = numMeasurements;
      intervals["TInt"] = dht22_tempInterval;
      intervals["pHInt"] = phValueInterval;
      intervals["AInt"] = acsValueFInterval;
      intervals["VInt"] = voltInterval;
      intervals["TDSInt"] = ds18b20Interval;
      intervals["HumInt"] = dht22_humInterval;
      intervals["CondInt"] = ecValueInterval;
      intervals["Flow1Int"] = flowRateInterval;
      intervals["Flow2Int"] = flowRate2Interval;
      intervals["COInt"] = coInterval;
      intervals["H2Int"] = h2Interval;

      JsonObject values = doc.createNestedObject("values");
      JsonArray ts = values.createNestedArray("ts");
      for (int i = 0; i < numMeasurements; i++)
      {
          ts.add(measurement[i].ts);
      }

      JsonArray tempGas = values.createNestedArray("T_g");
      int i = 0;
      for (int j = 0; j < numMeasurements; j += dht22_tempInterval)
      {
          tempGas.add(measurement[j].temperature);
          i++;
      }

      JsonArray zuurtegraad = values.createNestedArray("pH");
      i = 0;
      for (int j = 0; j < numMeasurements; j += phValueInterval)
      {
          zuurtegraad.add(measurement[j].phValue);
          i++;
      }

      JsonArray acs712 = values.createNestedArray("A");
      i = 0;
      for (int j = 0; j < numMeasurements; j += acsValueFInterval)
      {
          acs712.add(measurement[j].AcsValueF);
          i++;
      }

      JsonArray Spanning = values.createNestedArray("V");
      i = 0;
      for (int j = 0; j < numMeasurements; j += voltInterval)
      {
          Spanning.add(measurement[j].Volt);
          i++;
      }

      JsonArray ds18b20 = values.createNestedArray("T1");
      i = 0;
      for (int j = 0; j < numMeasurements; j += ds18b20Interval)
      {
          ds18b20.add(measurement[j].DS18B20_1);
          i++;
      }

      JsonArray ds18b202 = values.createNestedArray("T2");
      i = 0;
      for (int j = 0; j < numMeasurements; j += ds18b20Interval)
      {
          ds18b202.add(measurement[j].DS18B20_2);
          i++;
      }

      JsonArray ds18b203 = values.createNestedArray("T3");
      i = 0;
      for (int j = 0; j < numMeasurements; j += ds18b20Interval)
      {
          ds18b203.add(measurement[j].DS18B20_3);
          i++;
      }

      JsonArray ds18b204 = values.createNestedArray("T4");
      i = 0;
      for (int j = 0; j < numMeasurements; j += ds18b20Interval)
      {
          ds18b204.add(measurement[j].DS18B20_4);
          i++;
      }

      JsonArray ds18b205 = values.createNestedArray("T5");
      i = 0;
      for (int j = 0; j < numMeasurements; j += ds18b20Interval)
      {
          ds18b205.add(measurement[j].DS18B20_5);
          i++;
      }

      JsonArray humi = values.createNestedArray("Hum");
      i = 0;
      for (int j = 0; j < numMeasurements; j += dht22_humInterval)
      {
          humi.add(measurement[j].humidity);
          i++;
      }

      JsonArray ec = values.createNestedArray("Cond");
      i = 0;
      for (int j = 0; j < numMeasurements; j += ecValueInterval)
      {
          ec.add(measurement[j].ecValue);
          i++;
      }

      JsonArray flowRate = values.createNestedArray("Flow1");
      i = 0;
      for (int j = 0; j < numMeasurements; j += flowRateInterval)
      {
          flowRate.add(measurement[j].flowRate);
          i++;
      }

      JsonArray flowRate2 = values.createNestedArray("Flow2");
      i = 0;
      for (int j = 0; j < numMeasurements; j += flowRate2Interval)
      {
          flowRate2.add(measurement[j].flowRate2);
          i++;
      }

      JsonArray ppmCO = values.createNestedArray("CO");
      i = 0;
      for (int j = 0; j < numMeasurements; j += coInterval)
      {
          ppmCO.add(measurement[j].ppmCO);
          i++;
      }

      JsonArray ppmH = values.createNestedArray("H2");
      i = 0;
      for (int j = 0; j < numMeasurements; j += h2Interval)
      {
          ppmH.add(measurement[j].ppmH);
          i++;
      }

      if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(3000)) == pdTRUE)
      {
       if (doc != nullptr)
      {
       //logMeasurement((&doc)->as<String>().c_str());
       xSemaphoreGive(fileMutex);
      }
      }
      else
      {
       Serial.println("sendArrayTask: logMeusurement could not take fileMutex");
      }

      currentMeasurementIndex = 0;
      temperatureCount = 0;
      phValueCount = 0;
      humidityCount = 0;
      ecValueCount = 0;
      flowRateCount = 0;
      flowRateCount2 = 0;
      acsValueFCount = 0;
      ds18b20Count = 0;
      h2Count = 0;
      voltCount = 0;
      // printCMD();

      TickType_t endTimeFormArray = xTaskGetTickCount();
      Serial.println("FormArray duration: " + String(endTimeFormArray - startTimeFormArray));
      Serial.println("Measurement duration: " + String(measureTime));
      Serial.println("DHT22 Humidity duration: " + String(endTimeDHTtemp));
      Serial.println("pH duration: " + String(endTimepH));
      Serial.println("DHT22 Humidity duration: " + String(endTimeDHThum));
      Serial.println("ASC712 duration: " + String(endTimeEC));
      Serial.println("Flowrate duration: " + String(endTimeFlow1));
      Serial.println("Flowrate2 duration: " + String(endTimeFlow2));
      Serial.println("DS18B20 duration: " + String(endTimeDS18B20));
      Serial.println("MQ8 H2 duration: " + String(endTimeH2));
      Serial.println("MQ7 Co duration: " + String(endTimeCO));
      Serial.println("Volt duration: " + String(endTimeVolt));
      // printBufferInChunks(buffer, bufferIndex);
      // Add linebreaks and whitespaces to the end of the JSON document
      //serializeJsonPretty(doc, jsonBuffer);

      // Minifies the JSON document e.g. no linebreaks and no whitespaces
      serializeJson(doc, jsonBuffer);
      Serial.println("jsonBuffer created in Measuring task and sent to queue: ");
      Serial.println(jsonBuffer);

      // Send an item
      if (measurementQueue != NULL)
      {
        if (xQueueSend(measurementQueue, &jsonBuffer, portMAX_DELAY) == pdPASS)
        {
          Serial.println("Successfully posted buffer to queue");
          int queueSize = uxQueueMessagesWaiting(measurementQueue);
          Serial.println("Amount in queue (sendArray): " + String(queueSize));
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        else
        {
          Serial.println("Failed to post buffer to queue, deleting this buffer.");
          delete &doc;
        }
      }
    }
    else
    {
      currentMeasurementIndex++;
    }
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
*/

float t, phvalue, stroom, Volt, DS18B20_1, DS18B20_2, DS18B20_3, DS18B20_4, DS18B20_5, humidity, ecValue, ppmCO, ppmH;

void Measuring(void *parameter)
{
  vTaskDelay(10 / portTICK_PERIOD_MS);
  Serial.println("Inside Measuring task.");
  Serial.println("MaxMeasurements: " + String(numMeasurements));
  static int temperatureCount = 0, phValueCount = 0, humidityCount = 0, ecValueCount = 0, flowRateCount2 = 0, flowRateCount = 0, acsValueFCount = 0, ds18b20Count = 0, voltCount = 0, coCount = 0, h2Count = 0;
  unsigned long start_time, end_time, duration_temperature, duration_phValue, duration_humidity, duration_ecValue, duration_flowRate, duration_flowRate2, duration_acsValueF, duration_ds18b20, duration_h2, duration_volt;
  duration_temperature = duration_phValue = duration_humidity = duration_ecValue = duration_flowRate = duration_acsValueF = duration_ds18b20 = duration_h2 = duration_volt = 0;
  TickType_t measureTime, stopTime, endTimeDHTtemp, endTimepH, endTimeDHThum, endTimeEC, endTimeFlow1, endTimeFlow2, endTimeStroom, endTimeDS18B20, endTimeH2, endTimeCO, endTimeVolt;
  TickType_t startTimeMeasurement, startTime, startDHTTtempTime, startDHThumTime, startECValueTime, startFlowRateTime, startFlowRate2Time, startStroomTime, startpHTime, startDS18B20Time, startH2Time, startCOTime, startVoltTime;
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
    JsonArray Values = doc2["Values"].to<JsonArray>();
    for (int i = 0; i < numMeasurements; i++)
    {
      JsonObject obj = Values.add<JsonObject>();
      obj["ts"] = savedTimestamp + millis();
      if (currentMeasurementIndex % dht22_tempInterval == 0)
      {
        startDHTTtempTime = xTaskGetTickCount();
        t = dht_sensor.readTemperature();
        if (isnan(t) || isinf(t))
        {
          obj["T_g"] = 0;
        }
        else
        {
          obj["T_g"] = t;
        }
        temperatureCount++;
        endTimeDHTtemp = xTaskGetTickCount() - startTime;
      }
      if (currentMeasurementIndex % phValueInterval == 0)
      {
        startpHTime = xTaskGetTickCount();
        phvalue = pH();
        if (isnan(phvalue) || isinf(phvalue))
        {
          obj["pH"] = 0;
        }
        else
        {
          obj["pH"] = phvalue;
        }
        phValueCount++;
        endTimepH = xTaskGetTickCount() - startTime;
      }
      if (currentMeasurementIndex % acsValueFInterval == 0)
      {
        startStroomTime = xTaskGetTickCount();
        stroom = CurrentSensor_quick();
        if (isnan(stroom) || isinf(stroom))
        {
          obj["A"] = 0;
        }
        else
        {
          obj["A"] = stroom;
        }
        acsValueFCount++;
        endTimeStroom = xTaskGetTickCount() - startTime;
      }
      if (currentMeasurementIndex % voltInterval == 0)
      {
        startVoltTime = xTaskGetTickCount();
        Volt = readVoltage();
        if (isnan(Volt) || isinf(Volt))
        {
          obj["V"] = 0;
        }
        else
        {
          obj["V"] = Volt;
        }
        voltCount++;
        endTimeVolt = xTaskGetTickCount() - startTime;
      }
      if (currentMeasurementIndex % ds18b20Interval == 0)
      {
        startDS18B20Time = xTaskGetTickCount();
        sensors.requestTemperatures(); // Send the command to get temperatures
        int numberOfDevices = sensors.getDeviceCount();
        for (int i = 0; i < numberOfDevices; i++)
        {
          // Search the wire for address
          if (sensors.getAddress(tempDeviceAddress, i))
          {
            // Print the data
            float tempC = sensors.getTempC(tempDeviceAddress);
            // Assign readings to variables Tempc1 and Tempc2
            if (i == 0)
            {
              if (isnan(tempC) || isinf(tempC))
              {
                tempC = 0.0;
              }
              else
              {
                obj["T1"] = tempC;
              }
            }
            else if (i == 1)
            {
              if (isnan(tempC) || isinf(tempC))
              {
                tempC = 0.0;
              }
              else
              {
                obj["T2"] = tempC;
              }
            }
            else if (i == 2)
            {
              if (isnan(tempC) || isinf(tempC))
              {
                tempC = 0.0;
              }
              else
              {
                obj["T3"] = tempC;
              }
            }
            else if (i == 3)
            {
              if (isnan(tempC) || isinf(tempC))
              {
                tempC = 0.0;
              }
              else
              {
                obj["T4"] = tempC;
              }
            }
            else if (i == 4)
            {
              if (isnan(tempC) || isinf(tempC))
              {
                tempC = 0.0;
              }
              else
              {
                obj["T5"] = tempC;
              }
            }
          }
        }
        ds18b20Count++;
        endTimeDS18B20 = xTaskGetTickCount() - startTime;
      }
      if (currentMeasurementIndex % dht22_humInterval == 0)
      {
        startDHThumTime = xTaskGetTickCount();
        humidity = dht_sensor.readHumidity();
        if (isnan(humidity) || isinf(humidity))
        {
          obj["Hum"] = 0;
        }
        else
        {
          obj["Hum"] = humidity;
        }
        humidityCount++;
        endTimeDHThum = xTaskGetTickCount() - startTime;
      }
      if (currentMeasurementIndex % ecValueInterval == 0)
      {
        startECValueTime = xTaskGetTickCount();
        ecValue = Cond();
        if (isnan(ecValue) || isinf(ecValue))
        {
          obj["Cond"] = 0;
        }
        else
        {
          obj["Cond"] = ecValue;
        }
        ecValueCount++;
        endTimeEC = xTaskGetTickCount() - startTime;
      }
      if (currentMeasurementIndex % flowRateInterval == 0)
      {
        startFlowRateTime = xTaskGetTickCount();
        if (isnan(flowRate) || isinf(flowRate))
        {
          obj["Flow1"] = 0;
        }
        else
        {
          obj["Flow1"] = flowRate;
        }
        flowRateCount++;
        endTimeFlow1 = xTaskGetTickCount() - startTime;
      }
      if (currentMeasurementIndex % flowRate2Interval == 0)
      {
        startFlowRate2Time = xTaskGetTickCount();
        if (isnan(flowRate2) || isinf(flowRate2))
        {
          obj["Flow2"] = 0;
        }
        else
        {
          obj["Flow2"] = flowRate2;
        }
        flowRateCount2++;
        endTimeFlow2 = xTaskGetTickCount() - startTime;
      }
      if (currentMeasurementIndex % coInterval == 0)
      {
        startCOTime = xTaskGetTickCount();
        MQ7.update();
        ppmCO = MQ7.readSensor();
        if (isnan(ppmCO) || isinf(ppmCO))
        {
          obj["CO"] = 0;
        }
        else
        {
          obj["CO"] = ppmCO;
        }
        coCount++;
        endTimeCO = xTaskGetTickCount() - startTime;
      }
      if (currentMeasurementIndex % h2Interval == 0)
      {
        startH2Time = xTaskGetTickCount();
        MQ8.update();
        ppmH = MQ8.readSensor();
        if (isnan(ppmH) || isinf(ppmH))
        {
          obj["H2"] = 0;
        }
        else
        {
          obj["H2"] = ppmH;
        }
        h2Count++;
        endTimeH2 = xTaskGetTickCount() - startTime;
      }
      currentMeasurementIndex++;
    }
    stopTime = xTaskGetTickCount();
    measureTime = stopTime - startTimeMeasurement;
    vTaskDelay(50 / portTICK_PERIOD_MS);

    if (currentMeasurementIndex >= (numMeasurements - 1)) // MaxMeasurements -1
    {
      if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(3000)) == pdTRUE)
      {
        if (doc2 != nullptr)
        {
          // logMeasurement((&doc)->as<String>().c_str());
          xSemaphoreGive(fileMutex);
        }
      }
      else
      {
        Serial.println("Measuring Task: logMeasurement could not take fileMutex");
      }

      currentMeasurementIndex = 0, temperatureCount = 0, phValueCount = 0, humidityCount = 0, ecValueCount = 0, flowRateCount = 0, flowRateCount2 = 0, acsValueFCount = 0, ds18b20Count = 0, h2Count = 0, voltCount = 0;
      // printCMD();

      Serial.println("Measurement duration: " + String(measureTime));
      Serial.println("All of the following durations are singular, to obtain the total time you need to multiply by the number of measurements");
      Serial.println("Temp duration:   " + String(endTimeDHTtemp - startDHTTtempTime) + "| Humi duration: " + String(endTimeDHThum- startDHThumTime) + "| DS18B20 duration:  " + String(endTimeDS18B20 - startDS18B20Time));
      Serial.println("pH duration:     " + String(endTimepH - startpHTime)            + "| EC duration:   " + String(endTimeEC - startECValueTime)   + "| Flowrate duration: " + String(endTimeFlow1 - startFlowRateTime) + "| Flowrate2 duration: " + String(endTimeFlow2 - startFlowRate2Time));
      Serial.println("Stroom duration: " + String(endTimeStroom - startStroomTime)    + "| Volt duration: " + String(endTimeVolt - startVoltTime)    + "| MQ8 H2 duration:   " + String(endTimeH2 - startH2Time)          + "| MQ7 Co duration:    " + String(endTimeCO - startCOTime));
      Serial.println();
      /* Add linebreaks and whitespaces to the end of the JSON document */
      // serializeJsonPretty(doc, jsonBuffer);

      /* Minifies the JSON document e.g. no linebreaks and no whitespaces */
      ArduinoJson::serializeJson(doc2, jsonBuffer);
      Serial.println("jsonBuffer created in Measuring task and sent to queue: ");
      Serial.println(jsonBuffer);

      // Send an item
      if (measurementQueue != NULL)
      {
        if (xQueueSend(measurementQueue, &jsonBuffer, portMAX_DELAY) == pdPASS)
        {
          Serial.println("Successfully posted buffer to queue");
          int queueSize = uxQueueMessagesWaiting(measurementQueue);
          Serial.println("Amount in queue (MeauringTask): " + String(queueSize));
          vTaskDelay(pdMS_TO_TICKS(10));
          memset(jsonBuffer, 0, sizeof(jsonBuffer));
        }
        else
        {
          Serial.println("Failed to post buffer to queue, deleting this buffer.");
          memset(jsonBuffer, 0, sizeof(jsonBuffer));
          //delete &doc2;
        }
      }
    }

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
        bigOled.drawStr(0, 120, flowDis3.c_str());
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
        bigOled.drawStr(0, 128, flowDis3.c_str());
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
      float frequency2 = (float)pulses2 / (elapsed_time / 1000.0); // Frequency in Hz
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
      GSM_RX_PIN2 = pinValue;
    }
    else if (pinName == "GSM_TX_PIN")
    {
      GSM_TX_PIN2 = pinValue;
    }
    else if (pinName == "GSM_RST_PIN")
    {
      GSM_RST_PIN2 = pinValue;
    }
    else if (pinName == "Pin_MQ7")
    {
      Pin_MQ72 = pinValue;
    }
    else if (pinName == "Pin_MQ8")
    {
      Pin_MQ82 = pinValue;
    }
    else if (pinName == "DHT_SENSOR_PIN")
    {
      DHT_SENSOR_PIN2 = pinValue;
    }
    else if (pinName == "DS18B20_PIN")
    {
      DS18B20_PIN2 = pinValue;
    }
    else if (pinName == "flowSensorPin")
    {
      flowSensorPin2 = pinValue;
    }
    else if (pinName == "flowSensor2Pin")
    {
      flowSensor2Pin2 = pinValue;
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
  // Serial.println(flowSensorPin);
  Serial.print("flowSensor2Pin: ");
  // Serial.println(flowSensor2Pin);
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

void setup()
{
  Serial.begin(115200); // Initialize Serial for debug output
  vTaskDelay(1000 / portTICK_PERIOD_MS);
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
  readGsmResponse();
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
  // pcnt_example_init(PCNT_UNIT3, PCNT_INPUT_SIG_IO3);

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

  /*
   char buftest[bufferSize];
   measurementQueue = xQueueCreate(queueLength, sizeof(buftest));
   if (measurementQueue == NULL)
   {
     Serial.println("measurementQueue could not be created.");
     while (1)
     {
     }
   }
   else {
     Serial.println("measurementQueue created.");
   }
 */
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
  analogReadResolution(ADC_ATTENDB_MAX); // 12
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
  xTaskCreatePinnedToCore(sendArray, "Send Array", 7168, NULL, 3, &Task1, 0); // 8192
  xTaskCreatePinnedToCore(Measuring, "Measuring", 4096, NULL, 2, &Task2, 1);
  xTaskCreatePinnedToCore(DisplayMeasurements, "Display Measurements", 2048, NULL, 0, &Task3, 0);
  xTaskCreatePinnedToCore(BluetoothListen, "Listen to Bluetooth", 5120, NULL, 0, &Task4, 0);
  xTaskCreatePinnedToCore(Counting, "Count pulses", 1024, NULL, 1, &Task5, 1);

  free_size = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  Serial.println("Free heap size: " + String(free_size) + ", largest free block: " + String(largest_free_block));

  SD_init();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  if (SD.begin(CS_PIN))
  {
    // read_configuration();
  }
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