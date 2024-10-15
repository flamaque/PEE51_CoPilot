#include "config.h"
#include <algorithm>
#define ARDUINOJSON_STRING_LENGTH_SIZE 2      //Max characters 65,635
#define ARDUINOJSON_SLOT_ID_SIZE 2            //Max-nodes 65,635
#define ARDUINOJSON_USE_LONG_LONG 0           //Store jsonVariant as long
#define ARDUINOJSON_USE_DOUBLE 0              //Store floating point NOT as double 

/*        Flor sensor temperature     */
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

  #ifdef DEBUG_MODE_3  
  Serial.print("1 sample analog reading "); 
  Serial.println(sample);
  Serial.printf("Average analog reading: %.2f\n", average);
  #endif
 
  // convert the value to resistance
  float resistance = 4095 / average - 1;
  resistance = serialResistance * resistance;

  #ifdef DEBUG_MODE_3
  Serial.printf("Thermistor resistance: %.2f\n", resistance);
  #endif
 
 //resistance  / nominalResistance = 10 graden
 //nominalResistance / resistance = 43.25 graden
  float steinhart;
  steinhart = nominalResistance / (resistance - nominalResistance);     // (R/Ro)
  //steinhart = resistance  / nominalResistance;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= bCoefficient;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
 
  #ifdef DEBUG_MODE_3
  Serial.printf("Temperature: %.3f *C\n", steinhart);
  #endif
  
  return steinhart;
}

/* voltage sensor   */
float voltage = 0.00;

float readVoltage()
{
  const int numSamples = 100;
  float adc_voltage_sum = 0.0;
  float R1 = 1000.0;
  float R2 = 10000.0;
  // Read ADC value multiple times to average
  for (int i = 0; i < numSamples; i++)
  {
    int adc = analogRead(voltPin);
    adc_voltage_sum += adc * (3.3 / 4095.0);
     vTaskDelay(2 / portTICK_PERIOD_MS); // Small delay to allow for better averaging
  }
  
  float adc_voltage = adc_voltage_sum / numSamples; // Average the ADC voltage
  // Serial.println("ADC voltage: " + String(adc_voltage));
  //printf("R1: %f, R2: %f, sampling: %d, ADC voltage: %f\n", R1, R2, numSamples, adc_voltage);
  voltage = (adc_voltage * R2) / (R1 + R2);  // Calculate the sensor voltage

  return voltage;
}

/*      DS18B20 sensor            */
OneWire oneWire(DS18B20_PIN);                                               // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);                                        // Pass our oneWire reference to Dallas Temperature.
int numberOfDevices;                                                        // Number of temperature devices found
DeviceAddress tempDeviceAddress;                                            // We'll use this variable to store a found device address
// DS18B20 Find and print Address
void printDS18B20Address()
{
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();
  // Loop through each device, print out address
  for (int i = 0; i < numberOfDevices; i++)
  {
    // Search the wire for address
    if (sensors.getAddress(tempDeviceAddress, i))
    {
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      // Print the address
      for (uint8_t j = 0; j < 8; j++)
      {
        if (tempDeviceAddress[j] < 16)
          Serial.print("0");
        Serial.print(tempDeviceAddress[j], HEX);
      }
      Serial.println();
    }
    else
    {
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
      Serial.println();
    }
  }
  sensors.setResolution(11); // Set the resolution to 11 bits for 0.5°C resolution
}

/*
// Loop through each device, print out DS18B20 temperature data
void AllDS18B20Sensors()
{
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
        if(isnan(tempC)||isinf(tempC)){
          tempC = 0.0;
        } 
        else{
          values["T1"] = tempC;
        }
      }
      else if (i == 1)
      {
        if(isnan(tempC)||isinf(tempC)){
          tempC = 0.0;
        } 
        else{
          values["T2"] = tempC;
        }
      }
      else if (i == 2)
      {
        if(isnan(tempC)||isinf(tempC)){
          tempC = 0.0;
        } 
        else{
          values["T3"] = tempC;
          }        
      }
      else if (i == 3)
      {
        if(isnan(tempC)||isinf(tempC)){
          tempC = 0.0;
        }
        else{
          values["T4"] = tempC;
        }
      }
      else if (i == 4)
      {
        if(isnan(tempC)||isinf(tempC)){
          tempC = 0.0;
        }
        else{
        values["T5"] = tempC;
      }
      }
    }
  }
}
*/

/*              Setup Flowsensor    */
void pcnt_example_init(pcnt_unit_t unit, int pulse_gpio_num)
{
  /* Prepare configuration for the PCNT unit */
  pcnt_config_t pcnt_config = {
      // Set PCNT input signal GPIO
      .pulse_gpio_num = pulse_gpio_num,
      // No control GPIO needed
      .ctrl_gpio_num = PCNT_PIN_NOT_USED,
      // What to do on the positive / negative edge of pulse input?
      .pos_mode = PCNT_COUNT_INC, // Count up on the positive edge
      .neg_mode = PCNT_COUNT_DIS, // Ignore negative edge
      // Set the maximum and minimum limit values to watch
      .counter_h_lim = 0,
      .counter_l_lim = 0,
      .unit = unit,
      .channel = PCNT_CHANNEL_0,
  };
  /* Initialize PCNT unit */
  pcnt_unit_config(&pcnt_config);

  /* Configure and enable the input filter */
  pcnt_set_filter_value(unit, 100);
  pcnt_filter_enable(unit);

  /* Initialize PCNT's counter */
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);

  /* Everything is set up, now go to counting */
  pcnt_counter_resume(unit);
}

/*      MQ-7 MQ-8 sensor            */
float RatioMQ7CleanAir = 27.5;
float RatioMQ8CleanAir = 70.0;
void mq7_init(MQUnifiedsensor &MQ7)
{
  // CO
  MQ7.setRegressionMethod(1); //_PPM =  a*ratio^b
  // MQ7.setA(521853); MQ7.setB(-3.821); // Configurate the ecuation values to get Benzene concentration
  MQ7.setA(99.042);
  MQ7.setB(-1.518); // Configure the equation to calculate CO concentration value
  MQ7.init();
  vTaskDelay(5 / portTICK_PERIOD_MS);

  float calcR0 = 0;
  for (int i = 1; i <= 10; i++)
  {
    MQ7.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ7.calibrate(RatioMQ7CleanAir);
    Serial.print(".");
  }
  MQ7.setR0(calcR0 / 10);
  Serial.println("Done calculating R0 for MQ7!.");
  /*
    //If the RL value is different from 10K please assign your RL value with the following method:
    MQ7.setRL(9.87);
  */
  if (isinf(calcR0))
  {
    Serial.println("MQ7 Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
  } // while(1);
  if (calcR0 == 0)
  {
    Serial.println("MQ7 Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
  } // while(1);
  /*****************************  MQ CAlibration ********************************************/
  Serial.println("MQ7 initialized!");
  MQ7.serialDebug(true);
}

void mq8_init(MQUnifiedsensor &MQ8)
{
  // Hydrogen
  MQ8.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ8.setA(976.97);
  MQ8.setB(-0.688); // Configure the equation to to calculate H2 concentration
  MQ8.init();
  Serial.print("Calibrating MQ8 please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++)
  {
    MQ8.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ8.calibrate(RatioMQ8CleanAir);
    Serial.print(".");
  }
  MQ8.setR0(calcR0 / 10);
  Serial.println("R0 for MQ8 calculation done!.");

  if (isinf(calcR0))
  {
    Serial.println("MQ8 Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
  }
  if (calcR0 == 0)
  {
    Serial.println("MQ8 Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
  }
  /*****************************  MQ CAlibration ********************************************/
  Serial.println("MQ8 initialized!");
  MQ8.serialDebug(true);
}

/*      Display Setup               */
U8G2_SSD1306_128X64_NONAME_1_HW_I2C bigOled(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/22, /* data=*/21);

// For GSMSerial output on OLED
#define U8LOG_WIDTH 12 // 25
#define U8LOG_HEIGHT 6 // 8+
uint8_t u8log_buffer[U8LOG_WIDTH * U8LOG_HEIGHT];
U8G2LOG u8g2log;


void init_displays(){
    // Serial.println("Display hight: " + String(bigOled.getDisplayHeight()) + "Display width: " + String(bigOled.getDisplayWidth()));
    bigOled.setI2CAddress(0x3C * 2);
    bigOled.setBusClock(400000);
    bigOled.begin();
    bigOled.clearBuffer();
    bigOled.setFont(u8g2_font_6x12_mf); // set the font for the terminal window
    bigOled.setDisplayRotation(U8G2_R1);
    u8g2log.begin(bigOled, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
    u8g2log.setLineHeightOffset(2); // set extra space between lines in pixel, this can be negative
    u8g2log.setRedrawMode(1);       // 0: Update screen with newline, 1: Update screen for every char
    Serial.println("Displays initialized!"); // Serial.println("Big Display height: " + bigOled.getDisplayHeight() + " Big Display Width: "  + bigOled.getDisplayHeight());
  }

/*      Switching screens           */
volatile bool buttonBigPressed, buttonDebugPressed = false;

/*              Setup Currentsensor    */
// extern int CurrentPin;
float CurrentSensor_724()
{
  float current_voltage, current = 0.0;

  float R1 = 3300.0; //1000.0;
  float R2 = 6800.0; //2000.0;
  float RatioVolDiv = (R1 + R2) / R2;
  const int numSamples = 100;
  long adc_voltage_sum = 0;

  // Read ADC value multiple times to average
  for (int i = 0; i < numSamples; i++)
  {
    int adc = analogReadMilliVolts(CurrentPin);
    adc_voltage_sum += adc; //3.3
    vTaskDelay(2 / portTICK_PERIOD_MS);// Small delay to allow for better averaging
  }
  //Serial.println("ADC voltage raw: " + String(analogReadRaw(CurrentPin)));
  //Serial.println("ADC mV Quick: " + String(analogReadMilliVolts(CurrentPin)));
  // Average the ADC voltage
  float adc_voltage = (adc_voltage_sum / numSamples);// * (3300 / 4095.0);
  //Serial.println("ADC voltage Quick: " + String(adc_voltage));
  // Serial.println("ADC voltage: " + String(adc_voltage));

  // Calculate the sensor voltage
  current_voltage = adc_voltage * RatioVolDiv;
  //Serial.println("Current voltage Quick: " + String(current_voltage));

  // Measure this value when no current is flowing to calibrate zeroCurrentVoltage
  //float zeroCurrentVoltage = 0.48; // Use the previously measured value or measure again
  
  //float sensitivity = 0.066; // Change this value based on your specific ACS712 model

  float zeroCurrentVoltage = 2500;//2500; //Or 1.58V after voltage divider
  float sensitivity = 40; //0.040; //ACS724 sensitivity Change this value based on your specific ACS712 model

  // Calculate the current
  current = (current_voltage - zeroCurrentVoltage) / sensitivity;
  // Serial.println("Current: " + String(current));
  //printf("R1: %f, R2: %f, sampling: %d, ADC voltage: %f\n", R1, R2, numSamples, adc_voltage);

  return current;
}

/*      Conductivity Sensor   */
DFRobot_ESP_EC ec;
volatile float voltage_cond, temperature_cond = 25; // variable for storing the potentiometer value
// extern int EC_PIN;
float ecValueFloat = 0;
float Cond()
{
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) // time interval: 1s
  {
    timepoint = millis();
    voltage_cond = analogRead(EC_PIN) / 4095.0 * 3300;
    // Serial.println("voltage: " + String(voltage_cond, 4));
    // temperature = readTemperature();  // read your temperature sensor to execute temperature compensation
    // Serial.println("voltage_cond: " String(temperature_cond, 1));
    // Serial.println("^C");

    ecValueFloat = ec.readEC(voltage_cond, temperature_cond); // convert voltage to EC with temperature compensation
                                                              // Serial.println("EC: " + String(measurement.ecValue, 4) + " ms/cm");
  }
  ec.calibration(voltage_cond, temperature_cond); // calibration process by Serail CMD

  // Serial.print(ecValue,2);  Serial.println("ms/cm");
  return ecValueFloat;
}

/*      pH Sensor             */
#define ESPADC 4096.0   // the esp Analog Digital Conversion value
#define ESPVOLTAGE 3300 // the esp voltage supply value
// extern int PH_PIN;
float voltage_pH, phValue;
float temperature_pH = 20.0; // Fixed temperature value kan vervangen worden wanneer temp sensor gebruikt wordt

// wanneer je inf melding krijgt moet je caliberen lees hieronder om de code te laten werken is er een calibratie proces nodig type enterph
// doe ph sensor in 4ph en type calph
// doe ph sensor in 7ph en type calph
// type endcalph om compleet te maken.

float pH()
{
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) // time interval: 1s
  {
    timepoint = millis();
    voltage_pH = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE; // read the voltage
    // Serial.print("voltage_pH:");
    // Serial.println(voltage_pH, 4);

    phValue = ph.readPH(voltage_pH, temperature_pH); // convert voltage to pH with fixed temperature
                                                     // Serial.print("pH:");
    // Serial.println(phValue, 4);
  }
  ph.calibration(voltage_pH, temperature_pH); // calibration process by Serial CMD
  return phValue;
}

/*      SD card       */
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
  //spi.begin();
  if (!SD.begin(CS_PIN,spi,80000000)) {
  Serial.println("Card Mount Failed");
  return;
}
  if (SD.begin(CS_PIN, spi, 25000000))
  {
    Serial.println("Reading config file.");
    // read_configuration();
    // Serial.println("All values after setup:");
    // printVariables();
    // Serial.println("");
    File root;
    root = SD.open("/");
    printDirectory(root, 0);
    root.close();
  }
  else {
    Serial.println("SD initialization failed.");
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
    Serial.println("Log.txt file already exists");
  }
  file.close();

  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}

void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void deleteFile(fs::FS &fs, const char * path){
  Serial.printf("Deleting file: %s\n", path);
  if(fs.remove(path)){
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}
void renameFile(fs::FS &fs, const char * path1, const char * path2){
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}
/*
void createDir(fs::FS &fs, const char * path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path){
  Serial.printf("Removing Dir: %s\n", path);
  if(fs.rmdir(path)){
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}
*/

/*      Bluetooth Setup   */
void logMeasurement(String s)
{
  File dataFile = SD.open("/log.txt", FILE_APPEND);
  if (dataFile)
  {
    dataFile.println(datetime_gsm);
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


void sendFileOverBluetooth(const char *path)
{
  Serial.printf("Reading file: %s\n", path);
  File file = SD.open(path, FILE_READ);
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }
  while (file.available())
  {
    SerialBT.write(file.read());
  }
  file.close();
  Serial.println("File sent over Bluetooth");
}

unsigned long button_time = 0;
unsigned long last_button_time = 0;
void buttonInterrupt_bigOled()
{
  button_time = millis();
  if (button_time - last_button_time > 500)
  {
    last_button_time = button_time;
    buttonBigPressed = true; // Set button press flag
  }
}

unsigned long button_time2 = 0;
unsigned long last_button_time2 = 0;
void buttonInterrupt_debug()
{
  button_time2 = millis();
  if (button_time2 - last_button_time2 > 500)
  {
    last_button_time2 = button_time2;
    buttonDebugPressed = true; // Set button press flag
  }
}

