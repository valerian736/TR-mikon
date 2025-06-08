#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <stdio.h>

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
const float crashT = 5.0;
const int timeT = 500;
bool crashDetected = false;
bool crashNotified = false;
bool gpsConnected = false;
unsigned long crashDetectionTime = 0;
unsigned long lastGPSCheck = 0;

SoftwareSerial sim800l(4, 3);
SoftwareSerial gpsSerial(8, 9);

float prevAx = 0, prevAy = 0, prevAz = 0;
double lat = 0, lon = 0;

void sendMessage(String message);
void updateSerial();
void updateGPS();

PROGMEM const char message[] = "Format str from flash\nlong = %15ld\nFlash string = %10S\n";

ISR(INT0_vect)
{
  crashDetected = false;
}

void setup()
{
  pinMode(11, OUTPUT);
  sim800l.begin(9600);
  
  // Try different baud rates for GPS
  gpsSerial.begin(9600);
  Serial.begin(9600);
  
  // Give GPS module time to initialize
  Serial.println("Initializing GPS...");
  delay(2000);

  PORTD |= (1 << PD2);
  EICRA |= (1 << ISC01);
  EIMSK |= (1 << INT0);
  sei();

  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
      delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
  
  Serial.println("System initialized. Waiting for GPS fix...");
}

void loop()
{
  // Update GPS more frequently
  updateGPS();

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

  // Print debug info every 2 seconds to avoid spam
  if (millis() - lastGPSCheck > 2000) {
    Serial.print("Accel: ");
    Serial.print(vSum);
    Serial.print(" | X: ");
    Serial.print(ax);
    Serial.print(" Y: ");
    Serial.print(ay);
    Serial.print(" Z: ");
    Serial.print(az);
    Serial.print(" | GPS: ");
    Serial.print(gpsConnected ? "Connected" : "Searching...");
    Serial.print(" | Sats: ");
    Serial.print(gps.satellites.value());
    Serial.print(" | Lat: ");
    Serial.print(lat, 6);
    Serial.print(" Lon: ");
    Serial.println(lon, 6);
    
    lastGPSCheck = millis();
  }

  if (vSum > crashT)
  {
    if (crashDetectionTime == 0)
    {
      crashDetectionTime = millis();
    }
    unsigned long timeSinceCrash = millis() - crashDetectionTime;
    if (timeSinceCrash > timeT)
    {
      crashDetected = true;
      Serial.println("Crash detected, sending message...");
      
      // Only send message if we have valid GPS coordinates
      if (gpsConnected && lat != 0 && lon != 0) {
        sendMessage("Crash Detected! Location: https://www.google.com/maps/search/?api=1&query=" + String(lat, 6) + "," + String(lon, 6));
      } else {
        sendMessage("Crash Detected! GPS location not available.");
      }
      
      crashDetectionTime = 0;
    }
  }

  if (crashDetected)
  {
    tone(11, 1000, 3000);
  }
  else
  {
    noTone(11);
    crashNotified = false;
  }
  
  delay(100); // Small delay to prevent overwhelming the system
}

void sendMessage(String message)
{
  sim800l.println("AT");
  updateSerial();
  sim800l.println("AT+CMEE=1");
  updateSerial();
  sim800l.println("AT+CMGF=1");
  updateSerial();
  sim800l.println("AT+CMGS=\"+628996970405\"");
  updateSerial();
  sim800l.print(message);
  updateSerial();
  sim800l.write(26);
  updateSerial();
  Serial.println("Message sent");
}

void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    sim800l.write(Serial.read());
  }
  while (sim800l.available())
  {
    Serial.write(sim800l.read());
  }
}

void updateGPS() {
  // Process all available GPS data
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    if (gps.encode(c)) {
      // New sentence processed
      if (gps.location.isValid()) {
        gpsConnected = true;
        lat = gps.location.lat();
        lon = gps.location.lng();
      }
    }
  }
  
  // Check if GPS data is stale (no updates for 5 seconds)
  if (gps.location.age() > 5000) {
    gpsConnected = false;
  }
}
