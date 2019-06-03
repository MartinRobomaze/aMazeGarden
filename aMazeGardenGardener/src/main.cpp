#include <Arduino.h>
#include <rn2xx3.h>
#include <BasicStepperDriver.h>
#include <Servo.h>

#define RESET 14

const int txPin = 27;
const int rxPin = 26;

const char *appEui = "70B3D57ED001BF9F";
const char *devEui = "00F6701E23C21152";
const char *appKey = "F6F881E158707E20B3BD5EAEF79DE0D1";

const int stepsPerRevolution = 200;
const int rpm = 60;
const int microsteps = 1;

const int dirPinA = 19;
const int stepPinA = 18;
const int slp = 22;

const int distanceX = 20;
const int distanceY = 10;

const int pumpPin = 23;

const int servoPin = 16;

HardwareSerial mySerial(1);

rn2xx3 myLora(mySerial);

BasicStepperDriver motors(stepsPerRevolution, dirPinA, stepPinA);

Servo servo;

void initializeRadio();
void water(String response);
void led_on();
void led_off();
void forward(float distance);
unsigned int hexToDec(String hexString);

void setup() {
  delay(10);
  pinMode(2, OUTPUT);
  led_on();

  Serial.begin(57600);
  mySerial.begin(57600, SERIAL_8N1, rxPin, txPin);

  motors.begin(rpm, microsteps);

  delay(1000); 
  
  Serial.println("Startup");

  led_off();
  delay(500);
  led_on();
}

void loop() {
  initializeRadio();
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
      

      water(received);
      break;
    }
    default: {
      Serial.println("Unknown response from TX function");
    }
  }
}

void water(String response) {
  Serial.println(response);
  String responseY = response.substring(4, 6);
  int plantPosY = hexToDec(responseY);

  Serial.println(responseY);

  if (plantPosY == 0) {
    Serial.println("x");
    digitalWrite(pumpPin, HIGH);
    delay(5000);
    digitalWrite(pumpPin, LOW);
  }

  else {
    digitalWrite(slp, HIGH);
    motors.rotate(221);
    digitalWrite(pumpPin, HIGH);
    delay(5000);
    digitalWrite(pumpPin, LOW);
    motors.rotate(-221);
    digitalWrite(slp, LOW);
  }
}

void forward(float distance) {
  int wheelCircumference = PI * 5.6;

  float revolutions = distance / wheelCircumference;
  int degrees = revolutions * 360;
  Serial.println(degrees);

  motors.rotate(degrees);
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

void led_on()
{
  digitalWrite(2, 1);
}

void led_off()
{
  digitalWrite(2, 0);
}

unsigned int hexToDec(String hexString) {
  
  unsigned int decValue = 0;
  int nextInt;
  
  for (int i = 0; i < hexString.length(); i++) {
    
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    
    decValue = (decValue * 16) + nextInt;
  }
  
  return decValue;
}