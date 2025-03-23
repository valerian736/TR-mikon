#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const int crashThreshold = 20000; // Adjust this value based on your needs

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection())
  {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;
  }
  Serial.println("MPU6050 connection successful");
  mpu.setXAccelOffset(782);
  mpu.setYAccelOffset(-831);
  mpu.setZAccelOffset(1413);
  mpu.setXGyroOffset(166);
  mpu.setYGyroOffset(-54);
  mpu.setZGyroOffset(-70);
}

void loop()
{
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  long totalAcceleration = (long)ax * ax + (long)ay * ay;
  long acceleration = sqrt(totalAcceleration);
  if (acceleration > crashThreshold)
  {
    Serial.println("Crash detected! \t Total acceleration: " + String(totalAcceleration));
  }
  Serial.println("Acceleration: " + String(acceleration));
  delay(100);
}