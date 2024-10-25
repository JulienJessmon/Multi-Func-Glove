#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

const int flexPin1 = 15;
const int flexPin2 = 2;
const int flexPin3 = 4;
const int flexPin4 = 12;
const int flexPin5 = 27;


void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT");

  pinMode(flexPin1, INPUT);
  pinMode(flexPin2, INPUT);
  pinMode(flexPin3, INPUT);
  pinMode(flexPin4, INPUT);
  pinMode(flexPin5, INPUT);
}

void loop() {
  int flexValue1 = analogRead(flexPin1);
  int flexValue2 = analogRead(flexPin2);
  int flexValue3 = analogRead(flexPin3);
  int flexValue4 = analogRead(flexPin4);
  int flexValue5 = analogRead(flexPin5);

  String sensorData = String(flexValue1) + "," + String(flexValue2) + "," + String(flexValue3) + "," + String(flexValue4) + "," + String(flexValue5);
  SerialBT.println(sensorData);

  delay(2000);
}
