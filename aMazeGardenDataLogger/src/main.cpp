#include <rn2xx3.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <HardwareSerial.h>

#define RESET 14

#define DHTPIN 21
#define DHTTYPE    DHT22 

DHT_Unified dht(DHTPIN, DHTTYPE);
HardwareSerial mySerial(1);

rn2xx3 myLora(mySerial);

const int sleepTime = 10000 * 1000;

const int txPin = 27;
const int rxPin = 26;

const int soilMoistureSensorPin = 35;
const int soilTemperatureSensorPin = 32 ;

float R1 = 10000;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

const char *appEui = "70B3D57ED001BF9F";
const char *devEui = "0004A30B00E8F02C";
const char *appKey = "D4930E64F5BAA8FD1D3A1A8672EC93E9";

void initializeRadio();
void led_on();
void led_off();

void setup() {
  esp_sleep_enable_timer_wakeup(sleepTime);
  delay(10);
  pinMode(2, OUTPUT);
  led_on();

  Serial.begin(57600);
  mySerial.begin(57600, SERIAL_8N1, rxPin, txPin);

  dht.begin();
  delay(1000); 
  
  Serial.println("Startup");

  initializeRadio();

  led_off();
  delay(500);
  led_on();

  sensors_event_t event;
  dht.temperature().getEvent(&event);

  int airTemperature = event.temperature;
  dht.humidity().getEvent(&event);
  int airHumidity = event.relative_humidity;
  int soilMoisture = map(analogRead(soilMoistureSensorPin), 0, 4095, 1, 100);
  int Vo = analogRead(soilTemperatureSensorPin);

  float R2 = R1 * (4096.0 / (float)Vo - 1.0);
  float logR2 = log(R2);
  float T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  int soilTemperature = T - 273.15;

  Serial.println(airHumidity);

  char payload[4];
  payload[0] = char(airTemperature);
  payload[1] = char(airHumidity);
  payload[2] = char(soilMoisture);
  payload[3] = char(soilTemperature);

  int sent;
  do {
    Serial.println("TXing");
    sent = myLora.txCnf(payload);
  } while (sent == TX_FAIL);

  led_off();

  esp_deep_sleep_start();
}

void loop() {}

void led_on()
{
  digitalWrite(2, 0);
}

void led_off()
{
  digitalWrite(2, 1);
}

void initializeRadio()
{
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);

  delay(100); 
  mySerial.flush();

  delay(100); 
  String hweui = myLora.deveui();
  Serial.println(hweui);
    while(hweui.length() != 16)
  {
    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    Serial.println(hweui);
    delay(10000);
    hweui = myLora.hweui();
  }

  Serial.println("RN2xx3 firmware version:");
  Serial.println(myLora.sysver());

  Serial.println("Trying to join TTN");
  bool join_result = false;
  
  join_result = myLora.initOTAA(appEui, appKey, devEui);

  while(!join_result)
  {
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(60000); 
    join_result = myLora.init();
  }
  Serial.println("Successfully joined TTN");
}