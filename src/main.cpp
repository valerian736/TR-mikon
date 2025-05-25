#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;
bool crashDetected = false;
const int crashThreshold = 20; // Adjust this value based on your needs
MPU6050 mpu6050;

void setup()
{
  pinMode(11, OUTPUT);
  Serial.begin(9600);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);

  attachInterrupt(digitalPinToInterrupt(2), []() {
    crashDetected = false; // Reset crash detection on interrupt
  }, RISING); // Change to FALLING if needed
}

void loop()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  long totalAcceleration = (float)a.acceleration.x * a.acceleration.x + (float)a.acceleration.y * a.acceleration.y;
  long acceleration = sqrt(totalAcceleration);
  if (acceleration > crashThreshold)
  {
    Serial.println("Crash detected! \t Total acceleration: " + String(acceleration));
    crashDetected = true;

  }
  else{
    Serial.print("Acceleration: " + String(acceleration));
    Serial.print(" \t X: " + String(a.acceleration.x) + " \t Y: " + String(a.acceleration.y) + " \t Z: " + String(a.acceleration.z));
    Serial.println();
  }

  if (crashDetected)
  {
    tone(11, 30000); 
  }
  else
  {
    noTone(11); 
  }

  

  delay(100);
}
