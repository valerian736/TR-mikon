#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

const float crashThreshold = 20.0;
bool crashDetected = false;
bool crashNotified = false;

SoftwareSerial sim800l(4, 3); 

float prevAx = 0, prevAy = 0, prevAz = 0;
float lat = 0.0, lon = 0.0; // Placeholder for GPS coordinates

void sendMessage(String message);
void updateSerial();

void setup() {
  pinMode(11, OUTPUT);
  sim800l.begin(9600);
  Serial.begin(9600);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  attachInterrupt(digitalPinToInterrupt(2), []() {
    crashDetected = false;
    crashNotified = false;
  }, RISING);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float deltaAx = ax - prevAx;
  float deltaAy = ay - prevAy;
  float deltaAz = az - prevAz;

  float vSum = sqrt(deltaAx * deltaAx + deltaAy * deltaAy + deltaAz * deltaAz);

  prevAx = ax;
  prevAy = ay;
  prevAz = az;

  if (vSum > crashThreshold) {
    crashDetected = true;
    if (!crashNotified) {
      String message = "Crash Detected!\n lat = " + String(lat) + " lon = " + String(lon);
      sendMessage(message);
      crashNotified = true;
    }
  } else {
    Serial.print(" a: "); Serial.print(vSum);
    Serial.print(" | X: "); Serial.print(ax);
    Serial.print(" Y: "); Serial.print(ay);
    Serial.print(" Z: "); Serial.println(az);
  }

  if (crashDetected) {
    tone(11, 3000, 3000);
  } else {
    noTone(11);
  }

  delay(100);
}

void sendMessage(String message) {
  delay(1000);
  sim800l.println("AT");
  updateSerial();

  sim800l.println("AT+CMEE=1");
  updateSerial();

  sim800l.println("AT+CMGF=1");
  updateSerial();

  sim800l.println("AT+CMGS=\"+628996970405\"");  // Replace with real number
  updateSerial();

  sim800l.print(message);
  updateSerial();

  sim800l.write(26);  // CTRL+Z to send
  updateSerial();
}

void updateSerial() {
  delay(500);
  while (Serial.available()) {
    sim800l.write(Serial.read());
  }
  while (sim800l.available()) {
    Serial.write(sim800l.read());
  }
}
