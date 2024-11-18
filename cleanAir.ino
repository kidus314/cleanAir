#include <Wire.h>
#include <sps30.h>
#include <Adafruit_AHT10.h>
#include <TinyGPSPlus.h>

// Instances for sensors and GPS module
Adafruit_AHT10 aht10;
TinyGPSPlus gps;

// Serial configurations
#define myserial Serial2 // Serial port for GSM communication
#define gpsSerial Serial3 // Serial port for GPS module communication

// Unique sensor identifier
String sensorID = "00001";

/**
 * @brief Initializes the SPS30 sensor.
 *        Configures the I2C communication and starts the measurement process.
 *        Retries until successful initialization.
 */
void initializeSPS30() {
    sensirion_i2c_init();
    while (sps30_probe() != 0) {
        Serial.println("SPS30 sensor probing failed. Trying again...");
        delay(500);
    }
    Serial.println("SPS30 sensor initialized successfully.");
    int16_t ret = sps30_start_measurement();
    if (ret < 0) {
        Serial.println("Error starting SPS30 measurement.");
    } else {
        Serial.println("SPS30 measurement started successfully.");
    }
}

/**
 * @brief Reads PM2.5 data from the SPS30 sensor.
 * @return float - Mass concentration of PM2.5 in µg/m³, or -1.0 in case of an error.
 */
float readPM2_5() {
    struct sps30_measurement measurement;
    uint16_t data_ready;
    int16_t ret = sps30_read_data_ready(&data_ready);
    if (ret < 0 || !data_ready) {
        Serial.println("Data not ready or error reading data-ready flag.");
        return -1.0;
    }
    ret = sps30_read_measurement(&measurement);
    if (ret < 0) {
        Serial.println("Error reading SPS30 measurement.");
        return -1.0;
    }
    return measurement.mc_2p5;
}

/**
 * @brief Reads temperature and humidity data from the AHT10 sensor.
 * @param temperature - Reference to store the temperature value (in °C).
 * @param humidity - Reference to store the humidity value (in %).
 */
void readAHT10(float &temperature, float &humidity) {
    sensors_event_t hum, temp;
    aht10.getEvent(&hum, &temp);
    humidity = hum.relative_humidity;
    temperature = temp.temperature;
}

/**
 * @brief Initializes the GSM module.
 *        Configures the module for GPRS communication and prepares for HTTP requests.
 */
void initGSM() {
    myserial.println("AT");
    ShowSerialData();
    delay(2000);
    myserial.println("ATI");
    ShowSerialData();
    delay(2000);
    myserial.println("AT+CSQ");
    ShowSerialData();
    delay(2000);
    myserial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
    ShowSerialData();
    delay(3000);
    myserial.println("AT+SAPBR=3,1,\"APN\",\"etc.com\""); // Replace with your APN
    ShowSerialData();
    delay(3000);
    myserial.println("AT+SAPBR=1,1");
    ShowSerialData();
    delay(3000);
    myserial.println("AT+SAPBR=2,1");
    ShowSerialData();
    delay(3000);
    myserial.println("AT+HTTPSSL=1");
    ShowSerialData();
    delay(3000);
    myserial.println("AT+HTTPINIT");
    ShowSerialData();
    delay(3000);
    myserial.println("AT+HTTPPARA=\"CID\",1");
    ShowSerialData();
    delay(3000);
    myserial.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
    ShowSerialData();
    delay(3000);
}

/**
 * @brief Sends the collected sensor data to the server via HTTP POST.
 * @param data - JSON-formatted string containing the sensor data.
 */
void sendDataToServer(String data) {
    myserial.println("AT+HTTPSSL=1");
    ShowSerialData();
    delay(3000);
    myserial.println("AT+HTTPPARA=\"URL\",\"https://cleanair-a01ff-default-rtdb.firebaseio.com/sensorData/0000001.json?auth=aKnIqpWGOZzphw7WklPDaC9xtKLrPG84XJBL5Zvf\"");
    ShowSerialData();
    delay(3000);
    myserial.println("AT+HTTPDATA=" + String(data.length()) + ",100000");
    ShowSerialData();
    delay(3000);
    myserial.println(data);
    ShowSerialData();
    delay(3000);
    myserial.println("AT+HTTPACTION=1");
    ShowSerialData();
    delay(5000);
    myserial.println("AT+HTTPREAD");
    ShowSerialData();
    delay(3000);
}

/**
 * @brief Displays data received from the GSM module on the Serial Monitor.
 *        Helps in debugging communication with the GSM module.
 */
void ShowSerialData() {
    while (myserial.available() != 0) {
        Serial.write(myserial.read());
    }
    delay(500);
}

/**
 * @brief Retrieves GPS data including location, date, and time.
 *        Uses placeholders for invalid data.
 * @return String - JSON-formatted string with latitude, longitude, date, and time.
 */
String readGPS() {
    String gpsData = "";
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
    if (gps.location.isValid()) {
        gpsData += "\"latitude\":\"" + String(gps.location.lat(), 6) + "\",";
        gpsData += "\"longitude\":\"" + String(gps.location.lng(), 6) + "\",";
    } else {
        gpsData += "\"latitude\":\"INVALID\",\"longitude\":\"INVALID\",";
    }
    if (gps.date.isValid()) {
        gpsData += "\"date\":\"" + String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year()) + "\",";
    } else {
        gpsData += "\"date\":\"INVALID\",";
    }
    if (gps.time.isValid()) {
        gpsData += "\"time\":\"" + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + "\"";
    } else {
        gpsData += "\"time\":\"INVALID\"";
    }
    return gpsData;
}

/**
 * @brief Arduino setup function.
 *        Initializes the Serial Monitor, I2C bus, sensors, GPS, and GSM module.
 */
void setup() {
    Serial.begin(9600);
    delay(2000);
    Wire.begin();
    initializeSPS30();
    delay(3000);
    if (!aht10.begin()) {
        Serial.println("Failed to initialize AHT10 sensor!");
    } else {
        Serial.println("AHT10 sensor initialized successfully.");
    }
    gpsSerial.begin(9600);
    myserial.begin(9600);
    Serial.println("Initializing GSM...");
    initGSM();
}

/**
 * @brief Arduino loop function.
 *        Periodically reads sensor data, constructs a JSON payload, and sends it to the server.
 */
void loop() {
    float pm2_5 = readPM2_5();
    float temperature, humidity;
    readAHT10(temperature, humidity);

    char timestamp[25];
    if (gps.date.isValid() && gps.time.isValid()) {
        snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                 gps.date.year(), gps.date.month(), gps.date.day(),
                 gps.time.hour(), gps.time.minute(), gps.time.second());
    } else {
        snprintf(timestamp, sizeof(timestamp), "2024-11-16T00:00:00Z");
    }

    String longitude = gps.location.isValid() ? String(gps.location.lng(), 6) : "INVALID";
    String latitude = gps.location.isValid() ? String(gps.location.lat(), 6) : "INVALID";

    String sendToServer = "{\"sensorId\":\"" + sensorID + "\","
                          "\"humidity\":\"" + String(humidity) + "\","
                          "\"temperature\":\"" + String(temperature) + "\","
                          "\"pm25\":\"" + String(pm2_5) + "\","
                          "\"createdAt\":\"" + String(timestamp) + "\","
                          "\"longitude\":\"" + longitude + "\","
                          "\"latitude\":\"" + latitude + "\"}";

    Serial.println(sendToServer);
    sendDataToServer(sendToServer);

    delay(5000);
}
