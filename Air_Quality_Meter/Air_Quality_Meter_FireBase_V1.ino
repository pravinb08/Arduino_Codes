#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// Provide the token generation process info.
#include "addons/TokenHelper.h"

// Replace with your Wi-Fi credentials
#define WIFI_SSID "Mindnics"
#define WIFI_PASSWORD "Mindnics"

// Replace with your Firebase project credentials
#define API_KEY "AIzaSyD_SjcPDxzmQB5uvb6zmIlk5Ufw4NEp8fE"
#define DATABASE_URL "https://air-quality-meter-ksrct-default-rtdb.firebaseio.com/"

// NTP setup
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 0); // 19800 sec offset = GMT+5:30 (IST)

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Sensor variables
float temperature = 0.0;
float humidity = 0.0;
float coConcentration = 0.0;
int co2Concentration = 0;
float pm25 = 0.0;
float pm10 = 0.0;

// Send data to Firebase with timestamp path
void sendToFirebase() {
  timeClient.update();  // Sync time

  time_t rawTime = timeClient.getEpochTime();
  struct tm * timeinfo = localtime(&rawTime);

  char buffer[30];
  sprintf(buffer, "%04d-%02d-%02d_%02d:%02d:%02d",
          timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
          timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

  String timeStamp = String(buffer); // Formatted time
  String datePath = "/Environment_Readings/" + timeStamp;

  // Send values to Firebase
  Firebase.RTDB.setFloat(&fbdo, datePath + "/Temperature_(C)", temperature);
  Firebase.RTDB.setFloat(&fbdo, datePath + "/Humidity:", humidity);
  Firebase.RTDB.setFloat(&fbdo, datePath + "/Carbon_monoxide_(CO)", coConcentration);
  Firebase.RTDB.setInt(&fbdo, datePath + "/Carbon_dioxide_(CO2)", co2Concentration);
  Firebase.RTDB.setFloat(&fbdo, datePath + "/Particulate_matter_(PM_2_5)", pm25);
  Firebase.RTDB.setFloat(&fbdo, datePath + "/Particulate_matter_(PM_10)", pm10);

  if (fbdo.httpCode() == 200) {
    Serial.println("✅ Data sent to Firebase at: " + timeStamp);
  } else {
    Serial.print("❌ Firebase error: ");
    Serial.println(fbdo.errorReason());
  }
}

// Helpers to extract values
int extractIntValue(String data, String label) {
  int startIndex = data.indexOf(label);
  if (startIndex == -1) return 0;
  startIndex += label.length();
  int endIndex = data.indexOf(" |", startIndex);
  if (endIndex == -1) endIndex = data.length();
  return data.substring(startIndex, endIndex).toInt();
}

float extractFloatValue(String data, String label) {
  int startIndex = data.indexOf(label);
  if (startIndex == -1) return 0.0;
  startIndex += label.length();
  int endIndex = data.indexOf(" |", startIndex);
  if (endIndex == -1) endIndex = data.length();
  return data.substring(startIndex, endIndex).toFloat();
}

// Parse incoming sensor string
void parseSensorData(String data) {
  temperature = extractFloatValue(data, "Temperature : ");
  humidity = extractFloatValue(data, "Humidity : ");
  coConcentration = extractFloatValue(data, "CO Concentration: ");
  co2Concentration = extractIntValue(data, "CO2 Concentration : ");
  pm25 = extractFloatValue(data, "PM2.5: ");
  pm10 = extractFloatValue(data, "PM10: ");
}

// Setup function
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // UART RX=16, TX=17

  // Start Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Connected to Wi-Fi");

  // Start NTP
  timeClient.begin();

  // Firebase config
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = "RR4P9M3kmNIurTbHd1NJ3niifosdJIsvAhCOSIZK";

  auth.user.email = "";
  auth.user.password = "";

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  timeClient.update();
  timeClient.update();
  timeClient.update();
  timeClient.update();
  timeClient.update();
}

// Main loop
void loop() {
  if (Serial2.available()) {
    String receivedMessage = Serial2.readStringUntil('\n');
    Serial.println("📩 Received from Arduino: " + receivedMessage);

    parseSensorData(receivedMessage);

    Serial.print("      🌡 Temperature: "); Serial.println(temperature);
    Serial.print("      💧 Humidity: "); Serial.println(humidity);
    Serial.print("      🛢 CO Concentration: "); Serial.println(coConcentration);
    Serial.print("      🏭 CO2 Concentration: "); Serial.println(co2Concentration);
    Serial.print("      🌫 PM2.5: "); Serial.println(pm25);
    Serial.print("      🌫 PM10: "); Serial.println(pm10);

    sendToFirebase();
  }
  delay(1000);
}
