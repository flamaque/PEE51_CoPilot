#include <Arduino.h>
#include <SD.h> //Changed uint8_t max_files=5 to uint8_t max_files=2
#include <ArduinoJson.h>
#include <sstream>
#include <iomanip>

JsonDocument doc2;
JsonArray measurementsArray = doc2.to<JsonArray>();
JsonObject measurement = measurementsArray.add<JsonObject>();
const int bufferSize = 6144; //6144; //8192 Waarschijnlijk te groot
char jsonBuffer[bufferSize];

int CS_PIN = 5;

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root)
  {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory())
  {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file)
  {
    if (file.isDirectory())
    {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels)
      {
        listDir(fs, file.name(), levels - 1);
      }
    }
    else
    {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void readFile(fs::FS &fs, const char *path)
{
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available())
  {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    Serial.println("Message appended");
  }
  else
  {
    Serial.println("Append failed");
  }
  file.close();
}

void SD_init()
{
  if (!SD.begin(CS_PIN))
  {
    Serial.println("SD Card initialization failed!");
    Serial.println("Log file not created and configuration not read.");
    Serial.println("Restart ESP32!");
    return; //Continuing for now, production version should halt indefinitely
    
  }
  Serial.println("SD Card initialized.");
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    // Light up RED LED
    // return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC)
  {
    Serial.println("MMC");
  }
  else if (cardType == CARD_SD)
  {
    Serial.println("SDSC");
  }
  else if (cardType == CARD_SDHC)
  {
    Serial.println("SDHC");
  }
  else
  {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  File file = SD.open("/log.txt");
  if (!file)
  {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/log.txt", "Reading ID, Date, Hour, Temperature \r\n");
  }
  else
  {
    Serial.println("File already exists");
  }
  file.close();

  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}

void logMeasurement(String s)
{
  File dataFile = SD.open("/log.txt", FILE_APPEND);
  if (dataFile)
  {
    dataFile.println("22-07-2024, 11:23:45");
    dataFile.println(s);
    dataFile.println("");
    dataFile.close();
    Serial.println("Data written to file.");
  }
  else
  {
    Serial.println("Error opening file for writing.");
  }
}

void setup()
{
  Serial.begin(115200); // Initialize Serial for debug output
  SD_init();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  if (SD.begin(CS_PIN))
  {
    //Serial.println("Reading config file.");
    //read_configuration();
    //Serial.println("All values after setup:");
    //printVariables(); 
  }
  else
  {
    Serial.println("Failed to read config file.");
  }
}
  
void loop()
{
  TickType_t startTimeMeasurement = xTaskGetTickCount();
  for (int i = 0; i < 3; i++)
  {
  measurement["ts"] = 1724935018451 + millis();
  JsonObject values = measurement["values"].to<JsonObject>();
  values["temperature"] = 23.5;
  values["temperature2"] = 25.67;
  values["temperature3"] = 22.33;
  values["phValue"] = 8.0;
  values["humidity"] = 62.0;
  values["ecValue"] = 12.33;
  values["flowRate"] = 4.47;
  values["flowRate2"] = 9.12;
  values["stroom"] = 32.44;
  values["flowRateTemp"] = 87.13;
  delay(5000);
  }

  ArduinoJson::serializeJson(doc2, jsonBuffer);
  Serial.println("jsonBuffer created: ");
  Serial.println(jsonBuffer);

  if (doc2 != nullptr)
  {
    logMeasurement((&doc2)->as<String>().c_str());
  }
  else 
  {
    Serial.println("doc2 is null, nothing to log.");
  }

  Serial.println("End of write test.");
  while(1){}
}