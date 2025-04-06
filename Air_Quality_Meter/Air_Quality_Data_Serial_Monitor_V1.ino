#include <SoftwareSerial.h>

//SDS Sensor Interface
#define SDS_RX 4                           // Sensor TX -> Arduino RX (D4)
#define SDS_TX 5                           // Sensor RX -> Arduino TX (D5) (optional)
SoftwareSerial sdsSerial(SDS_RX, SDS_TX);  // SoftwareSerial for SDS011 sensor
int pm10 = 0;
int pm25 = 0;
void SDS_Data();

//CO2  Sensor Interface
class CO2Sensor {
private:
  int pin;
  float calibrationFactor;
  int baseCO2Level;

public:
  CO2Sensor(int analogPin, float factor, int baseLevel) {
    pin = analogPin;
    calibrationFactor = factor;
    baseCO2Level = baseLevel;
  }

  float getCO2Level() {
    int sensorValue = analogRead(pin);
    float voltage = sensorValue * (5.0 / 1023.0);                         // Convert ADC value to voltage
    float co2_ppm = (voltage * calibrationFactor * 2000) + baseCO2Level;  // Approximate CO2 calculation
    return co2_ppm;
  }
};
CO2Sensor co2Sensor(A0, 0.99, 100);
float co2Level = 0;
void Co2_Data();

//CO Sensor Interface
#define RX_PIN 2
#define TX_PIN 3
SoftwareSerial coSensor(RX_PIN, TX_PIN);
byte packet[9];
int CO_concentration = 0;
unsigned char checksum;
void CO_data();
unsigned char calculateChecksum(byte *data);

void update_all_Data();


void setup() {
  Serial.begin(115200);  // Open Serial Monitor

  Serial.println("MG-811 CO2 Sensor Test...");
  delay(500);
  //dmd.begin();
  /*sdsSerial.begin(9600);  // Start communication with SDS011
  Serial.println("SDS011 PM Sensor Initialized...");
  delay(500);*/

  /*coSensor.begin(9600);  // Start communication with Winsen ZE16B-CO carbon monoxide sensor
  Serial.println("ZE16B-CO Sensor Initialized");
  delay(500);
  */
}

void loop() {

  update_all_Data();

  Serial.print("CO2 Concentration: ");
  Serial.print(co2Level);
  Serial.print(" ppm");

  Serial.print(" | PM2.5: ");
  Serial.print(pm25 / 10.0);  // Convert to µg/m³
  Serial.print(" µg/m³,| PM10: ");
  Serial.print(pm10 / 10.0);  // Convert to µg/m³
  Serial.print(" µg/m³");

  Serial.print(" | CO Concentration: ");
  Serial.print(CO_concentration);
  Serial.println(" ppm");

  delay(100);
}

void update_all_Data() {

  Co2_Data();

  sdsSerial.begin(9600);
  delay(200);
  SDS_Data();
  delay(200);
  sdsSerial.end();
  delay(200);

  coSensor.begin(9600);
  delay(200);
  CO_data();
  delay(200);
  coSensor.end();
  delay(200);
}
void SDS_Data() {
  if (sdsSerial.available() >= 10) {  // SDS011 sends 10-byte data frames
    byte buffer[10];

    // Read 10 bytes
    for (int i = 0; i < 10; i++) {
      buffer[i] = sdsSerial.read();
    }

    // Validate data frame
    if (buffer[0] == 0xAA && buffer[1] == 0xC0 && buffer[9] == 0xAB) {
      pm25 = (buffer[3] << 8) | buffer[2];  // PM2.5 value
      pm10 = (buffer[5] << 8) | buffer[4];  // PM10 value

      /*Serial.print("PM2.5: ");
      Serial.print(pm25 / 10.0);  // Convert to µg/m³
      Serial.print(" µg/m³, PM10: ");
      Serial.print(pm10 / 10.0);  // Convert to µg/m³
      Serial.println(" µg/m³");*/
    } else {
      Serial.println("Invalid Data Frame.");
    }
  }
}
void Co2_Data() {
  co2Level = co2Sensor.getCO2Level();
  /*Serial.print("CO2 Concentration: ");
  Serial.print(co2Level);
  Serial.println(" ppm");*/
}
void CO_data() {
  // Check if data is available from the sensor
  if (coSensor.available() >= 9) {
    // Read the 9-byte packet
    for (int i = 0; i < 9; i++) {
      packet[i] = coSensor.read();
    }

    // Verify the start byte
    if (packet[0] == 0xFF) {
      // Calculate the checksum
      checksum = calculateChecksum(packet);

      // Verify the checksum
      if (checksum == packet[8]) {
        // Extract concentration value
        CO_concentration = (packet[4] << 8) | packet[5];

        // Print the concentration value
        //Serial.print("CO Concentration: ");
        //Serial.print(CO_concentration);
        //Serial.println(" ppm");
      } else {
        Serial.println("Checksum mismatch! Invalid data.");
      }
    } else {
      Serial.println("Invalid start byte! Invalid data.");
    }
  }
}
unsigned char calculateChecksum(byte *data) {
  unsigned char sum = 0;
  for (int i = 1; i < 8; i++) {
    sum += data[i];
  }
  return (~sum) + 1;
}
