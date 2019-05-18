#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LoRaWan.h>

#define uS_TO_S_FACTOR 1000000
#define SLEEP_TIME 5

Adafruit_BME280 bme;
LoRaWanClass ttn;

const int soilTempSensorPin = 13;
const int soilMoisSensorPin = 14;

const int ttnTxPin = 17;
const int ttnRxPin = 16;

char *DevAddr = "2601120E";
char *DevEUI = "00A377F70AE88716";
char *AppEUI = "70B3D57ED001BF9F";
char *NwkSKey = "EE43A22A89DF29F1B01D0AA314857769";
char *AppSKey = "54EAB647769D39908A0030A13A10090D";

char buffer[256];

float R1 = 10000;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

void setup() {
  // ==== Wakeup time settting ====
  
  esp_sleep_enable_timer_wakeup(SLEEP_TIME * uS_TO_S_FACTOR);
  
  // ==============================
  
  // ==== Serial link initialization ====
  
  Serial.begin(9600);

  Serial.println("========aMazeGarden data logger========");
  
  // ====================================

  // ==== Meteo sensor initialization ====
  
  bool status = bme.begin();

  if (!status) {
    Serial.println("No sensor found");
  }
  
  // =====================================

  // ==== LoRaWan ABP setup ====
  
  lora.init(ttnTxPin, ttnRxPin);

  memset(buffer, 0, 256);
  lora.getVersion(buffer, 256, 1);
  Serial.print(buffer); 
  
  memset(buffer, 0, 256);
  lora.getId(buffer, 256, 1);
  Serial.print(buffer);

  lora.setId("2601120E", "00A377F70AE88716", "70B3D57ED001BF9F");
  lora.setKey("EE43A22A89DF29F1B01D0AA314857769", "54EAB647769D39908A0030A13A10090D", NULL);
  
  lora.setDeciveMode(LWABP);
  lora.setDataRate(DR0, EU868);
  
  lora.setChannel(0, 868.1);
  lora.setChannel(1, 868.3);
  lora.setChannel(2, 868.5);
  
  lora.setReceiceWindowFirst(0, 868.1);
  lora.setReceiceWindowSecond(869.5, DR3);
  
  lora.setDutyCycle(false);
  lora.setJoinDutyCycle(false);
  
  lora.setPower(14);
  // ===================================

  // ==== Reading sensors ====

  // Soil temperature reading.
  int soilTempSensorReadingAnalog = analogRead(soilTempSensorPin);
  int R2 = R1 * (4096.0 / (float)soilTempSensorReadingAnalog - 1.0);
  int logR2 = log(R2);
  int Temp = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  int soilTemperature = Temp - 273.15;

  Serial.print("Soil temperature: ");
  Serial.println(soilTemperature);

  // Soil moisture reading
  int soilMoistureReading = analogRead(soilMoisSensorPin);
  int soilMoisture = map(soilMoistureReading, 0, 4096, 0, 100);

  Serial.print("Soil moisture: ");
  Serial.println(soilMoisture);

  // BME280 reading
  uint32_t airTemperature = bme.readTemperature() * 100;
  uint32_t airHumidity = bme.readHumidity() * 100;
  uint32_t airPressure = bme.readPressure() * 100;

  Serial.print("Air temperature: ");
  Serial.println(airTemperature);
  Serial.print("Air humidity: ");
  Serial.println(airHumidity);
  Serial.print("Air pressure: ");
  Serial.println(airPressure);

  // =========================

  // ==== Constructing payload ====

  char payload[6];

  payload[0] = highByte(airTemperature);
  payload[1] = lowByte(airTemperature);
  payload[2] = highByte(airHumidity);
  payload[3] = lowByte(airHumidity);
  payload[4] = highByte(airPressure);
  payload[5] = lowByte(airPressure);


  // ==============================

  // ==== Sending payload ====

  lora.transferPacket("payload");

  // =========================

  // ==== Going into deep sleep ====

  esp_deep_sleep_start();

  // ===============================
}

void loop() {
  
}