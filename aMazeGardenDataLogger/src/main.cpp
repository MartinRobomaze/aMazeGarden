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

const int ttnTxPin = 26;
const int ttnRxPin = 27;

char *DevEUI = "00B6CCF440F52B10";
char *AppEUI = "70B3D57ED001BF9F";
char *AppKey = "B5982228595B10BAF453AF61A2B1FF23";

char buffer[256];
\
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

  // ==== LoRaWan OTAA activation ====
  
  ttn.init(ttnTxPin, ttnRxPin);

  ttn.getVersion(buffer, 256, 1);
  Serial.print(buffer); 
  
  ttn.getId(buffer, 256, 1);
  Serial.print(buffer);
  
  lora.setId(NULL, DevEUI, AppEUI);
  lora.setKey(NULL, NULL, AppKey);
  
  ttn.setDeciveMode(LWOTAA);
  ttn.setDataRate(DR0, EU868);
  
  ttn.setChannel(0, 868.1);
  ttn.setChannel(1, 868.3);
  ttn.setChannel(2, 868.5);
  
  ttn.setReceiceWindowFirst(0, 868.1);
  ttn.setReceiceWindowSecond(869.5, DR3);
  
  ttn.setDutyCycle(false);
  ttn.setJoinDutyCycle(false);
  
  ttn.setPower(14);
  
  while(!ttn.setOTAAJoin(JOIN));
  
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

  byte payload[10];

  payload[0] = highByte(airTemperature);
  payload[1] = lowByte(airTemperature);
  payload[2] = highByte(airHumidity);
  payload[3] = lowByte(airHumidity);
  payload[4] = highByte(airPressure);
  payload[5] = lowByte(airPressure);
  payload[6] = highByte(soilMoisture);
  payload[7] = lowByte(soilMoisture);
  payload[8] = highByte(soilTemperature);
  payload[9] = lowByte(soilTemperature);

  // ==============================

  // ==== Sending payload ====

  ttn.transferPacket(payload, 10);

  // =========================

  // ==== Going into deep sleep ====

  esp_deep_sleep_start();

  // ===============================
}

void loop() {
  
}