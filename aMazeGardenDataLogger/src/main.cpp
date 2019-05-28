#include <rn2xx3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <HardwareSerial.h>

#define RESET 14
HardwareSerial mySerial(1);

rn2xx3 myLora(mySerial);
Adafruit_BME280 bme;

const int sleepTime = 10000 * 1000;

const int txPin = 27;
const int rxPin = 26;

const char *appEui = "70B3D57ED001BF9F";
const char *appKey = "D4930E64F5BAA8FD1D3A1A8672EC93E9";

void initializeRadio();
void led_on();
void led_off();

void setup() {
  esp_sleep_enable_timer_wakeup(sleepTime);
  pinMode(2, OUTPUT);
  led_on();

  Serial.begin(57600);
  mySerial.begin(57600, SERIAL_8N1, rxPin, txPin);

  bool status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

  delay(1000); 
  
  Serial.println("Startup");

  initializeRadio();

  led_off();
  delay(500);
  led_on();

  int airTemperature = bme.readTemperature();
  int airHumidity = bme.readHumidity();
  int soilMoisture = 10;
  int soilTemperature = 18;

  char payload[4];

  payload[0] = char(airTemperature);
  payload[1] = char(airHumidity);
  payload[2] = char(soilMoisture);
  payload[3] = char(soilTemperature);

  Serial.println("TXing");

  myLora.tx(payload);

  led_off();

  esp_deep_sleep_start();
}

void loop() {}

void led_on()
{
  digitalWrite(2, 1);
}

void led_off()
{
  digitalWrite(2, 0);
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
  
  join_result = myLora.initOTAA(appEui, appKey);

  while(!join_result)
  {
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(60000); 
    join_result = myLora.init();
  }
  Serial.println("Successfully joined TTN");
}