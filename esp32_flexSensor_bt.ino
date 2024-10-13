#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

const int flexPin1 = 15;
const int flexPin2 = 2;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT");
  Serial.println("Ready to pair");
}

void loop() {
  int flexValue1 = analogRead(flexPin1);
  int flexValue2 = analogRead(flexPin2);

  String sensorData = String(flexValue1) + "," + String(flexValue2);

  SerialBT.println(sensorData);
  Serial.println(sensorData);

  delay(3000);
}
