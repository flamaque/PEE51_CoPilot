#include "config.h"
extern HardwareSerial gsmSerial;
volatile uint8_t stateBigOled = 1;
String response, longitude, latitude, date, time_gsm, jsonPayload, datetime_gsm = "";
bool Posting = false;
#define ARDUINOJSON_STRING_LENGTH_SIZE 2      //Max characters 65,635
#define ARDUINOJSON_SLOT_ID_SIZE 2            //Max-nodes 65,635
#define ARDUINOJSON_USE_LONG_LONG 0           //Store jsonVariant as long
#define ARDUINOJSON_USE_DOUBLE 0              //Store floating point NOT as double 

void sendCmd(const char* cmd)
{
    //serialSIM800.listen();
    gsmSerial.flush();
    delay(500);
    gsmSerial.write(cmd);
    gsmSerial.flush();
}

void readGsmResponse()
{
    char c;
    response = "";
    unsigned long startTime = millis();
    unsigned long lastReadTime = millis();
    const unsigned long readTimeout = 500; // Time to wait for new data in milliseconds
       
    while (1){
        if (gsmSerial.available() > 0)
        {
            uint8_t byteFromSerial = gsmSerial.read();
            c = char(byteFromSerial);
            response += c;
            //printf("%c", c);
            //Serial.write(byteFromSerial);
            if (stateBigOled == 1)
            {            
                u8g2log.print(c);                
            }
            lastReadTime = millis(); // Update last read time
        }
        if (millis() - lastReadTime > readTimeout){
            Serial.println("readGsmResponse timeout");
            break;
        }
    }        
    printf("(printf) Response: %s\n", response.c_str());            
}
char command[20];

void readGsmResponse2() {
    char c;
    response = "";
    bool skipEcho = true;  // Flag to skip the echoed command
    unsigned long startTime = millis();
    unsigned long lastReadTime = millis();
    const unsigned long readTimeout = 1000; // Time to wait for new data in milliseconds

    while (millis() - startTime < 10000) { // Overall timeout after 10 seconds
        while (gsmSerial.available() > 0) {
            uint8_t byteFromSerial = gsmSerial.read();
            c = char(byteFromSerial);

            if (skipEcho && c == '\n') {
                skipEcho = false;  // Stop skipping after encountering the newline character
                continue;
            }

            if (!skipEcho) {
                response += c;
                Serial.write(byteFromSerial);
                if (stateBigOled == 1) {
                    u8g2log.print(c);
                }
            }
            lastReadTime = millis(); // Update last read time
        }
        // Check if there's been no data read for the readTimeout duration
        if (millis() - lastReadTime > readTimeout) {
            break;
        }
    }
}

String readGsmResponse3()
{
    char c;
    response = "";
    unsigned long startTime = millis();
    unsigned long lastReadTime = millis();
    const unsigned long readTimeout = 2000; // Time to wait for new data in milliseconds

    while (millis() - startTime < 10000)
    {
        while (gsmSerial.available() > 0)
        {
            uint8_t byteFromSerial = gsmSerial.read();
            c = char(byteFromSerial);
            response += c;
            Serial.write(byteFromSerial);
            if (stateBigOled == 1)
            {
                if (Posting == false)
                {
                    u8g2log.print(c);
                }
            }
            lastReadTime = millis();
        }

        // Check if there's been no data read for the readTimeout duration
        if (millis() - lastReadTime > readTimeout)
        {
            break;
        }

        // Check if "+CIPGSMLOC:" is found in the response
        size_t foundIndex = response.indexOf("+CIPGSMLOC:");
        if (foundIndex != std::string::npos)
        {
            // Remove everything before "+CIPGSMLOC:"
            response = response.substring(foundIndex);
        }
    }

    // Remove any trailing characters after the newline
    int newlineIndex = response.indexOf("\r\n");
    if (newlineIndex != -1)
    {
        response = response.substring(0, newlineIndex + 2);
    }
    return response;
}

String readGsmResponse4() {
    String response = "";
    unsigned long startTime = millis();
    while (millis() - startTime < 2000) {  // Increased timeout to 2 seconds
        if (gsmSerial.available()) {
            char c = gsmSerial.read();
            response += c;
            if (response.endsWith("\r\n")) {
                response.trim();
                if (response.startsWith("+CCLK:")) {
                    int startIndex = response.indexOf('"');
                    int endIndex = response.lastIndexOf('"');
                    if (startIndex != -1 && endIndex != -1 && endIndex > startIndex) {
                        return response.substring(startIndex + 1, endIndex);
                    }
                } else if (response.indexOf(',') != -1 && response.indexOf(':') != -1) {
                    // Direct timestamp format without +CCLK: prefix
                    return response;
                }
            }
        }
    }
    return "Error: Invalid time";
}

String readGsmResponse5() {
    String response = "";
    unsigned long startTime = millis();
    while (millis() - startTime < 2000) {  // Increased timeout to 2 seconds
        if (gsmSerial.available()) {
            char c = gsmSerial.read();
            response += c;
            if (response.endsWith("\r\n")) {
                response.trim();
                if (response.startsWith("+CCLK:") || response.startsWith("AT+CCLK?+CCLK:")) {
                    return response; // Return the entire response string
                }
            }
        }
    }
    Serial.println("Error invalid time: " + response);
    return "Error: Invalid time";
}

String date_getTime="";
uint64_t unixTimestamp;

void getTime()
{
    bool validResponse = false;
    while (!validResponse)
    { 
        // Setup GPRS
        // gsmSerial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\""); // Sets the mode to GPRS
        // readGsmResponse();
        // vTaskDelay(500 / portTICK_PERIOD_MS);

        gsmSerial.println("AT+SAPBR=3,1,\"APN\"," + apn); // Set APN parameters
        readGsmResponse();
        vTaskDelay(500 / portTICK_PERIOD_MS);

        gsmSerial.println("AT+SAPBR=3,1,\"USER\"," + apn_User); // Set APN username
        readGsmResponse();
        vTaskDelay(500 / portTICK_PERIOD_MS);

        gsmSerial.println("AT+SAPBR=3,1,\"PWD\"," + apn_Pass); // Set APN password
        readGsmResponse();
        vTaskDelay(500 / portTICK_PERIOD_MS);

        gsmSerial.println("AT+SAPBR=1,1"); // Open the carrier with previously defined parameters "start command"
        readGsmResponse();
        vTaskDelay(500 / portTICK_PERIOD_MS);

        gsmSerial.println("AT+SAPBR=2,1"); // Query the status of previously opened GPRS carrier
        readGsmResponse();

        // vTaskDelay(2000 / portTICK_PERIOD_MS);
        
        while (!validResponse)
        {
            if (GSMType ==1){
            printf("SIM800 requesting datetime...\n");
            gsmSerial.println("AT+CIPGSMLOC=2,1");
            String timeTest = readGsmResponse3();
            Serial.println("timeTest= " + timeTest);
            vTaskDelay(500 / portTICK_PERIOD_MS);

            gsmSerial.println("AT+CIPGSMLOC=2,1");
            vTaskDelay(500 / portTICK_PERIOD_MS);
            String timeTestChar = response;
            Serial.println("timeTestChar= " + timeTestChar);
            vTaskDelay(500 / portTICK_PERIOD_MS);

            gsmSerial.println("AT+CIPGSMLOC=2,1");
            String resp = readGsmResponse3();
            vTaskDelay(500 / portTICK_PERIOD_MS);

            int startIndex = timeTestChar.indexOf("+CIPGSMLOC: ");
            if (startIndex == -1) 
            {
                printf("Error: Invalid time, trying again...\n");
                printf("Response: %s\n", timeTestChar.c_str());
                break; // Exit the inner loop and retry GPRS setup
            }

            int endIndex = timeTestChar.indexOf("\r\n", startIndex);
            String data = timeTestChar.substring(startIndex + 12, endIndex);

            // Split the response string by commas
            int firstComma = data.indexOf(',');
            int secondComma = data.indexOf(',', firstComma + 1);

            if (firstComma == -1 || secondComma == -1)
            {
                Serial.println("Error: Malformed response line 244, trying again...");
                printf("(printf) Malformed response: %s\n", data.c_str());
                printf("(printf) resp: %s\n", timeTestChar.c_str());
                printf("firstComma: %d, secondComma: %d, \n", firstComma, secondComma);
                break; // Exit the inner loop and retry GPRS setup
            }

            date_getTime = data.substring(firstComma + 1, secondComma);
            time_gsm = data.substring(secondComma + 1);

            // Check if the date and time are valid
            if (date_getTime.toDouble() == 0.000000 || time_gsm.toDouble() == 0.000000)
            {
                Serial.println("Error: Invalid date/time, trying again...");
                Serial.println("Parsed Date: " + date_getTime);
                Serial.println("Parsed Time: " + time_gsm);
                break; // Exit the inner loop and retry GPRS setup
            }
        }
            else if (GSMType ==2){
                printf("SIM808 requesting datetime...\n");
                gsmSerial.println("AT+CLTS=1"); //+CLTS: OK
                vTaskDelay(500 / portTICK_PERIOD_MS);
                readGsmResponse();
                gsmSerial.println("AT+CLTS=?"); //+CLTS: "yy/MM/dd,hh:mm:ss+/-zz"
                vTaskDelay(500 / portTICK_PERIOD_MS);
                readGsmResponse();
                gsmSerial.println("AT+CLTS?");  //+CLTS: 1
                vTaskDelay(500 / portTICK_PERIOD_MS);
                readGsmResponse();
                gsmSerial.println("AT+CCLK?"); //+CCLK: "04/01/01,00:43:35+08"
                vTaskDelay(10 / portTICK_PERIOD_MS);
                String response = readGsmResponse5();
                Serial.println("Response: " + response); // Print the entire response for debugging

                // Find the index of the "+CCLK: "
                int startIndex = response.indexOf("+CCLK: \"");
                if (startIndex == -1) {
                    printf("Error: Invalid time, trying again...\n");
                    printf("Response: %s\n", response.c_str());
                    break; // Exit the inner loop and retry GPRS setup
                }

                // Adjust the start index to capture the actual date/time string
                startIndex += 8; // Move to the start of the date/time string

                // Find the end index of the closing quote
                int endIndex = response.indexOf("\"", startIndex);
                if (endIndex == -1) {
                    Serial.println("Error: Malformed response, trying again...");
                    printf("(printf) Malformed response: %s\n", response.c_str());
                    break; // Exit the inner loop and retry GPRS setup
                }

                // Extract the date/time string
                String data = response.substring(startIndex, endIndex);
                int commaIndex = data.indexOf(','); // Split the response string by comma
                if (commaIndex == -1) {
                    Serial.println("Error: Malformed response on line 296, trying again...");
                    printf("(printf) Malformed response: %s\n", data.c_str());
                    printf("(printf) resp: %s\n", response.c_str());
                    break; // Exit the inner loop and retry GPRS setup
                }

                date_getTime = data.substring(0, commaIndex);
                String time_gsm = data.substring(commaIndex + 1);

                // Remove the timezone offset from time_gsm
                int plusIndex = time_gsm.indexOf('+');
                if (plusIndex != -1) {
                    time_gsm = time_gsm.substring(0, plusIndex);
                }

                // Check if the date and time are valid
                if (date_getTime.length() != 8 || time_gsm.length() != 8) {
                    Serial.println("Error: Invalid date/time format, trying again...");
                    Serial.println("Parsed Date: " + date_getTime);
                    Serial.println("Parsed Time: " + time_gsm);
                    break; // Exit the inner loop and retry GPRS setup
                }

                Serial.println("Parsed Date: " + date_getTime);
                Serial.println("Parsed Time: " + time_gsm);
                unixTimestamp = convertToUnixTimestamp(date_getTime, time_gsm);
                Serial.println("Timestamp before comparison: " + String(unixTimestamp));

                if (unixTimestamp <= 1609459200000ULL || unixTimestamp >= 2524608000000ULL) 
                { // 1609459200 is 2021-01-01, 2524608000 is 2050-01-01
                    Serial.println("Error: Invalid timestamp, trying again...");
                    Serial.println("Timestamp: " + String(unixTimestamp));
                    break; // Exit the inner loop and retry GPRS setup
                } 
            }

            // If all validations pass, set validResponse to true
            validResponse = true;            
        }
    }

    datetime_gsm = date_getTime + " " + time_gsm;
    Serial.println("Valid datetime received. " + datetime_gsm);
    Serial.println("Date: " + date_getTime);
    Serial.println("Time: " + time_gsm);
    saveTimestamp(unixTimestamp);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void saveTimestamp(uint64_t timestamp_ms)
{
    nvs_flash_init();
    nvs_handle_t my_handle;
    nvs_open("storage", NVS_READWRITE, &my_handle);
    nvs_set_u64(my_handle, "timestamp_ms", timestamp_ms);
    nvs_commit(my_handle);
    nvs_close(my_handle);
}

uint64_t getSavedTimestamp()
{
    uint64_t timestamp_ms = 0;
    nvs_flash_init();
    nvs_handle_t my_handle;
    nvs_open("storage", NVS_READWRITE, &my_handle);
    nvs_get_u64(my_handle, "timestamp_ms", &timestamp_ms);
    nvs_close(my_handle);
    return timestamp_ms;
}

uint64_t convertToUnixTimestamp(String date, String time)
{
    uint64_t timestamp_ms = 0;
    int year, month, day = 0;

    // Extract year, month, day from date
    if (GSMType == 1) {
        year = date.substring(0, 4).toInt();
        // Ensure correct substring positions based on your date format "DD/MM/YY"
        month = date.substring(3, 5).toInt();
        day = date.substring(0, 2).toInt(); // Extract day from the start
    }
    else if (GSMType == 2) {
        String yearStr = date.substring(0, 2);
        if (yearStr.toInt() > 70) {
            year = (yearStr.toInt() + 1900);
        }
        else {
            year = (yearStr.toInt() + 2000);
        }
        // Ensure correct substring positions based on your date format "YY/MM/DD"
        month = date.substring(3, 5).toInt();
        day = date.substring(6, 8).toInt(); // Extract day from the start
    }

    // Extract hour, minute, second from time
    int hour = time.substring(0, 2).toInt();
    int minute = time.substring(3, 5).toInt();
    int second = time.substring(6, 8).toInt();

    // Create a tm struct
    struct tm t;
    memset(&t, 0, sizeof(t)); // Initialize to zero
    t.tm_year = year - 1900;   // tm_year is years since 1900
    t.tm_mon = month - 1;      // tm_mon is 0-11
    t.tm_mday = day;
    t.tm_hour = hour;
    t.tm_min = minute;
    t.tm_sec = second;
    t.tm_isdst = -1; // Not set by default

    // Convert to time_t (UNIX timestamp)
    time_t timestamp = mktime(&t);
    if (timestamp == -1) {
        Serial.println("Failed to convert time using mktime.");
        return -1;
    }
    Serial.println("Data in convertToUnixTimestamp: ");
    Serial.println("Parsed day: " + String(day));
    Serial.println("Parsed month: " + String(month));
    Serial.println("Parsed year: " + String(year));
    Serial.println("Parsed Date: " + date);
    Serial.println("Parsed hour: " + String(hour));
    Serial.println("Parsed minute: " + String(minute));
    Serial.println("Parsed second: " + String(second));
    Serial.println("Parsed Time: " + time);
    // Print the intermediate timestamp
    Serial.println("Intermediate timestamp: " + String(timestamp));

    // Calculate the timestamp in milliseconds
    timestamp_ms = timestamp * 1000LL; // No milliseconds provided in time string

    // Print the final timestamp in milliseconds
    Serial.println("Final timestamp with milliseconds: " + String(timestamp_ms));
    return timestamp_ms;
}

void post_http2(const char* jsonPayload)
{
    TickType_t startTime = xTaskGetTickCount();
    int dataSize = strlen(jsonPayload);
    Serial.println("Buffer in post_http: " + String(jsonPayload));
    Serial.println("Data size: " + String(dataSize));
    Serial.println();

    if (dataSize <= 0)
    {
        Serial.println("No data to post.");
        return;
    }
    sendCmd(HTTP_INIT);
    //snprintf(command, sizeof(command), "AT+HTTPINIT=?");
    //gsmSerial.println(command);
    //gsmSerial.println("AT+HTTPINIT=?");
    readGsmResponse();
    sendCmd(HTTP_INIT2);
    //snprintf(command, sizeof(command), "AT+HTTPINIT");
    //gsmSerial.println(command);
    //gsmSerial.println("AT+HTTPINIT");
    readGsmResponse();
    TickType_t EndTime1 = xTaskGetTickCount();
    printf("httpinit=? and httpinit: %d ms \n", EndTime1 - startTime);

    Serial.println("Post http data...");
/*
    const int maxRetries = 5;
    int retryCount = 0;
    bool httpInitSuccess = false;

    while (!httpInitSuccess && retryCount < maxRetries)
    {
        gsmSerial.println("AT+HTTPINIT=?");
        readGsmResponse2();

        gsmSerial.println("AT+HTTPINIT");
        readGsmResponse2();

        if (response.indexOf("ERROR") == -1)
        {
            httpInitSuccess = true;
        }
        else
        {
            Serial.println("AT+HTTPINIT failed. Retrying...");
            Serial.println("Response of GSM: " + response);
            retryCount++;
            vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait for 2 seconds before retrying
        }
    }

    if (!httpInitSuccess)
    {
        Serial.println("AT+HTTPINIT failed after multiple retries. Aborting HTTP request.");
        Serial.println("Continuing for now.");
        //return;
    } */

    vTaskDelay(10 / portTICK_PERIOD_MS);
    snprintf(command, sizeof(command), "AT+HTTPPARA=\"CID\",1");
    gsmSerial.println(command);
    //gsmSerial.println("AT+HTTPPARA=\"CID\",1");
    readGsmResponse();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+HTTPPARA=\"URL\", " + String(httpapi));
    readGsmResponse();
    TickType_t EndTime2 = xTaskGetTickCount();
    printf("httppara=cid and -url: %d ms \n", EndTime2 - startTime);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
    readGsmResponse();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    String httpDataCommand = "AT+HTTPDATA=" + String(dataSize) + ",20000";
    vTaskDelay(50 / portTICK_PERIOD_MS);
    TickType_t EndTime3 = xTaskGetTickCount();
    printf("httppara=content and httpdata: %d ms \n", EndTime3 - startTime);

    gsmSerial.println(httpDataCommand);
    readGsmResponse();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    TickType_t endTimeBeforejsonInjection = xTaskGetTickCount();
    printf("Time for post_http before json injection: %d ms \n", endTimeBeforejsonInjection - startTime);

    gsmSerial.println(jsonPayload);
    readGsmResponse();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    TickType_t endTimeHalf = xTaskGetTickCount();
    printf("Time for post_http before httpaction=1: %d ms \n", endTimeHalf - startTime);

    gsmSerial.println("AT+HTTPACTION=1");
    readGsmResponse();
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+HTTPREAD");
    String response = readGsmResponse3();
        
        if (response.indexOf("+HTTPREAD: 1,200,0") != -1) {
            Serial.println("correct");
            // Continue with normal operation
        } else if (response.indexOf("+HTTPREAD: 1,400,0") != -1 || response.indexOf("+HTTPREAD: 1,601,0") != -1) {
            Serial.println("error");
            
            // Send SMS
            //gsmSerial.println("AT+CMGF=1"); // Set SMS text mode
            //delay(100);
            //gsmSerial.println("AT+CMGS=\"+31614504283\""); // Set recipient number
            //delay(100);
            //gsmSerial.println("HTTP Error: " + response); // SMS content
            //delay(100);
            //gsmSerial.write(26); // Ctrl+Z to send SMS
            
            // Wait for SMS to be sent
            //delay(5000);
        }
    vTaskDelay(50 / portTICK_PERIOD_MS);

    TickType_t endTime = xTaskGetTickCount();
    printf("Time for post_http: %d ms \n", endTime - startTime);
}

enum GsmState
{
    GSM_INIT,
    GSM_SET_CONTYPE,
    GSM_SET_APN,
    GSM_SET_USER,
    GSM_SET_PASS,
    GSM_ACTIVATE_GPRS,
    GSM_QUERY_STATUS,
    GSM_HTTP_INIT,
    GSM_HTTP_PARA_CID,
    GSM_HTTP_PARA_URL,
    GSM_HTTP_PARA_CONTENT,
    GSM_CONFIG_DONE
};

GsmState gsmState = GSM_INIT;
unsigned long previousMillis = 0;
const long interval1 = 5000;
const long interval2 = 2000;
const long interval3 = 3000;

void initialize_gsm()
{
    unsigned long currentMillis = millis();

    switch (gsmState)
    {
    case GSM_INIT:
        Serial.println("Configure APN settings.");
        gsmSerial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\",\"IP\"");
        previousMillis = currentMillis;
        gsmState = GSM_SET_CONTYPE;
        break;

    case GSM_SET_CONTYPE:
        if (currentMillis - previousMillis >= interval1)
        {
            gsmSerial.println("AT+SAPBR=3,1,\"APN\"," + apn);
            previousMillis = currentMillis;
            gsmState = GSM_SET_APN;
        }
        break;

       
    case GSM_SET_APN:
        if (currentMillis - previousMillis >= interval2)
        {
            gsmSerial.println("AT+SAPBR=3,1,\"USER\"," + apn_User);
            previousMillis = currentMillis;
            gsmState = GSM_SET_USER;
        }
        break;
    case GSM_SET_USER:
        if (currentMillis - previousMillis >= interval2)
        {
            gsmSerial.println("AT+SAPBR=3,1,\"PWD\"," + apn_Pass);
            previousMillis = currentMillis;
            gsmState = GSM_SET_PASS;
        }
        break;
    case GSM_SET_PASS:
        if (currentMillis - previousMillis >= interval2)
        {
            Serial.println("APN settings configured.");
            Serial.println("Configure GPRS settings.");
            gsmSerial.println("AT+SAPBR=1,1");
            previousMillis = currentMillis;
            gsmState = GSM_ACTIVATE_GPRS;
        }
        break;
    case GSM_ACTIVATE_GPRS:
        if (currentMillis - previousMillis >= interval3)
        {
            gsmSerial.println("AT+SAPBR=2,1");
            previousMillis = currentMillis;
            gsmState = GSM_QUERY_STATUS;
        }
        break;
    case GSM_QUERY_STATUS:
        if (currentMillis - previousMillis >= interval3)
        {
            Serial.println("GPRS settings configured.");
            gsmSerial.println("AT+HTTPINIT"); // Initialize HTTP service
            previousMillis = currentMillis;
            gsmState = GSM_HTTP_INIT;
        }
        break;

    case GSM_HTTP_INIT:
        if (currentMillis - previousMillis >= interval2)
        {
            gsmSerial.println("AT+HTTPPARA=\"CID\",1"); // Define carrier profile
            previousMillis = currentMillis;
            gsmState = GSM_HTTP_PARA_CID;
        }
        break;

    case GSM_HTTP_PARA_CID:
        if (currentMillis - previousMillis >= interval2)
        {
            gsmSerial.println("AT+HTTPPARA=\"URL\", " + String(httpapi)); // Set the URL
            previousMillis = currentMillis;
            gsmState = GSM_HTTP_PARA_URL;
        }
        break;

    case GSM_HTTP_PARA_URL:
        if (currentMillis - previousMillis >= interval2)
        {
            gsmSerial.println("AT+HTTPPARA=\"CONTENT\",\"application/json\""); // Set the content type
            previousMillis = currentMillis;
            gsmState = GSM_HTTP_PARA_CONTENT;
        }
        break;

    case GSM_HTTP_PARA_CONTENT:
        if (currentMillis - previousMillis >= interval2)
        {
            Serial.println("HTTP settings configured.");
            gsmState = GSM_CONFIG_DONE;
        }
        break;

    case GSM_CONFIG_DONE:
    Serial.println("GSM configuration done.");
    gsmSerial.println("AT+CFUN?"); 
    readGsmResponse();
    gsmSerial.println("AT+CSQ"); 
    readGsmResponse();
        gsmSerial.println("AT+CIFSR"); 
    readGsmResponse();
        // Initialization done
        break;
    }
}

/*
void initialize_gsm2()
{
    Serial.println("Configure APN settings.");

    gsmSerial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\", \"IP\""); // Sets the mode to GPRS
    vTaskDelay(500 / portTICK_PERIOD_MS);
    readGsmResponse();

    gsmSerial.println("AT+SAPBR=3,1,\"APN\"," + apn); // Set APN parameters
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    readGsmResponse();

    gsmSerial.println("AT+SAPBR=3,1,\"USER\"," + apn_User); // Set APN username
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    readGsmResponse();

    gsmSerial.println("AT+SAPBR=3,1,\"PWD\"," + apn_Pass); // Set APN password
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    readGsmResponse();

    Serial.println("APN settings configured.");
    Serial.println("Configure GPRS settings.");

    gsmSerial.println("AT+SAPBR=1,1"); // Open the carrier with previously defined parameters "start command"
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    readGsmResponse();

    gsmSerial.println("AT+SAPBR=2,1"); // Query the status of previously opened GPRS carrier
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    readGsmResponse();

    Serial.println("GPRS settings configured.");
}
*/

/*
void GA6_init()
{
    Serial.println("GA6_init.");
    Serial.println("Initializing IoT-GA6.");
    // Set APN (Access Point Name) details
    //To set TCP function
    //gsmSerial.println("AT+CGDCONT=1,\"IP\",\"data.lycamobile.nl\"");
    //To set http or FTP
    gsmSerial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\", IP");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+SAPBR=3,1,\"APN\",\"data.lycamobile.nl\"");

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+SAPBR=3,1,\"USER\",\"lmnl\"");

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+SAPBR=3,1,\"PWD\",\"plus\"");

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    Serial.println("APN settings configured.");

    Serial.println("Activating GPRS (PDP) context.");
    gsmSerial.println("AT+CGACT=1,1");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Return current stat of PDD context.");
    gsmSerial.println("AT+CGDCONT?");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Bringing up wireless connection.");
    gsmSerial.println("AT+CIICR");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Check if there's an IP.");
    gsmSerial.println("AT+CIFSR");

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    Serial.println("Check the registration status.");
    gsmSerial.println("AT+CREG?");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Check attach status.");
    gsmSerial.println("AT+CGACT?");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Attach to network.");
    gsmSerial.println(" b");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Wait for attach.");
    vTaskDelay(30000 / portTICK_PERIOD_MS);
    Serial.println("Start task and set the APN.");
    //gsmSerial.println("AT+CSTT=\"data.lycamobile.nl\", \"lmnl\", \"plus\"");
    //
    //vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Send AT+CIPSTATUS?. Return STATE: IP START."); // Check if IP stack is intitalized.
    gsmSerial.println("AT+CIPSTATUS");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Bring up the wireless connection.");
    gsmSerial.println("AT+CIICR");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Wait for bringup.");
    vTaskDelay(30000 / portTICK_PERIOD_MS);
    Serial.println("Get the local IP address.");
    gsmSerial.println("AT+CIFSR");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Get the status of the IP connection.");
    gsmSerial.println("AT+CIPSTATUS");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("APN settings configured.");
}
*/
