#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <stdio.h>
// int uart_putchar(char c, FILE *stream) { return Serial.write(c); }
// PROGMEM const char str1[] = "Format str from flash\nlong = %15ld\nFlash string = %10S\n";
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
const float crashT = 20.0;
const int timeT = 500;
bool crashDetected = false;
bool crashNotified = false;
unsigned long crashDetectionTime = 0;

SoftwareSerial sim800l(4, 3);
SoftwareSerial gpsSerial(5, 6);

float prevAx = 0, prevAy = 0, prevAz = 0;
float lat = 0.0, lon = 0.0;

void sendMessage(String message);
void updateSerial();

PROGMEM const char message[] = "Format str from flash\nlong = %15ld\nFlash string = %10S\n";

ISR(INT0_vect)
{
  crashDetected = false;
}

void setup()
{

  pinMode(11, OUTPUT);
  sim800l.begin(9600);
  gpsSerial.begin(9600);
  Serial.begin(9600);

  // attachInterrupt(digitalPinToInterrupt(2), []()
  //                 { crashDetected = false; }, FALLING);

  PORTD |= (1 << PD2);
  EICRA |= (1 << ISC01);
  EIMSK |= (1 << INT0);

  // TCCR1A = 0;
  // TCCR1B = 0;
  // TCCR1B |= (1 << WGM12);
  // TCCR1B |= (1 << CS12);
  // OCR1A = 625;
  // TIMSK1 |= (1 << OCIE1A);
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
}

void loop()
{

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

  Serial.print(" a: ");
  Serial.print(vSum);
  Serial.print(" | X: ");
  Serial.print(ax);
  Serial.print(" Y: ");
  Serial.print(ay);
  Serial.print(" Z: ");
  Serial.println(az);

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
      sendMessage("Crash Detected! https://www.google.com/maps/search/?api=1&query=" + String(lat) + "," + String(lon));

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
