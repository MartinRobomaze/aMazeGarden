#include <Arduino.h>
#include <rn2xx3.h>
#include <BasicStepperDriver.h>
#include <MultiDriver.h>
#include <SyncDriver.h>

#define RESET 14

const int txPin = 27;
const int rxPin = 26;

const char *appEui = "70B3D57ED001BF9F";
const char *appKey = "D4930E64F5BAA8FD1D3A1A8672EC93E9";

const int stepsPerRevolution = 200;
const int rpm = 200;
const int microsteps = 1;

const int dirPinA = 19;
const int stepPinA = 18;
const int dirPinB = 17;
const int stepPinB = 16;

const int distanceX = 20;
const int distanceY = 20;

const int pumpPin = 14;

HardwareSerial mySerial(1);

rn2xx3 myLora(mySerial);

BasicStepperDriver motorA(dirPinA, stepPinA, stepsPerRevolution);
BasicStepperDriver motorB(dirPinB, stepPinB, stepsPerRevolution);

SyncDriver controller(motorA, motorB);

void initializeRadio();
void water(char *response);
void led_on();
void led_off();
void forward(float distance);
void rotate(int rotateAngle);

void setup() {
  motorA.begin(rpm, microsteps);
  motorB.begin(rpm, microsteps);

  Serial.begin(57600);
  mySerial.begin(57600, SERIAL_8N1, rxPin, txPin);

  pinMode(pumpPin, OUTPUT);

  led_on();
  initializeRadio();
  led_off();
}

void loop() {
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);
  
  Serial.println("TXing");
  switch (myLora.txCnf("!")) {
    case TX_FAIL: {
      Serial.println("TX unsuccessful or not acknowledged");
      break;
    }
    case TX_SUCCESS: {
      Serial.println("TX successful and acknowledged");
      break;
    }
    case TX_WITH_RX: {
      Serial.println("TX with RX");
      String received = myLora.getRx();
      char receivedCh[6];

      received.toCharArray(receivedCh, 6);

      water(receivedCh);
      break;
    }
    default: {
      Serial.println("Unknown response from TX function");
    }
  }
}

void water(char *response) {
  long long number = strtoll(response, NULL, 16);

  long long wateredSoilMoisture = number >> 16;
  long long plantPosX = number >> 8 & 0xFF;
  long long plantPosY = number & 0xFF;

  rotate(90);
  forward(plantPosX * distanceX);
  rotate(-90);
  forward(plantPosY * distanceY);

  if (plantPosX == 0) {
    rotate(-90);
    digitalWrite(pumpPin, HIGH);
    delay(100 * wateredSoilMoisture);
    digitalWrite(pumpPin, LOW);
    rotate(90);
  }

  else {
    rotate(90);
    digitalWrite(pumpPin, HIGH);
    delay(100 * wateredSoilMoisture);
    digitalWrite(pumpPin, LOW);
    rotate(-90);
  }

  forward(-(plantPosY * distanceY));
  rotate(-90);
  forward(-(plantPosX * distanceX));
  rotate(90);
}

void forward(float distance) {
  int wheelCircumference = PI * 5.6;

  float revolutions = distance / wheelCircumference;
  int degrees = revolutions * 360;
  Serial.println(degrees);

  controller.rotate(degrees, degrees);
}

void rotate(int rotateAngle) {
  float wheelCircumference = PI * 5.6;

  float distance = ((PI * 20) / 360) * rotateAngle;
  float revolutions = distance / wheelCircumference;
  float degrees = revolutions * 360;
  Serial.println(degrees);

  controller.rotate(degrees, -degrees);
}

void initializeRadio() {
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

void led_on()
{
  digitalWrite(2, 1);
}

void led_off()
{
  digitalWrite(2, 0);
}