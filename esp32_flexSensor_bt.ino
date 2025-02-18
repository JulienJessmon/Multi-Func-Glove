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


"""

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "BluetoothSerial.h"

// Bluetooth and MPU6050 Objects
BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);       // USB Serial for debugging
  SerialBT.begin("ESP32_BT"); // Initialize Bluetooth with name
  Serial.println("Bluetooth Started. Pair with 'ESP32_BT'.");

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Configure Accelerometer
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void loop() {
  // Read Accelerometer Data
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Extract X and Y Acceleration Values
  float xAccel = accel.acceleration.x;
  float yAccel = accel.acceleration.z;

  // Send via Bluetooth with a delimiter for better parsing
  String data = "X:" + String(xAccel) + "|Y:" + String(yAccel) + "\n";
  SerialBT.print(data);    // Use SerialBT for Bluetooth data
  SerialBT.flush();        // Ensure all data is sent
  Serial.print(data);

  delay(50);  // Adjust for smoother movement
}

"""

