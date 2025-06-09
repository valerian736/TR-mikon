#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
static const int RXPin = 8, TXPin = 9;

double lat, lon;
bool gpsConnected;
volatile bool crashDetected;
// The TinyGPS++ object
TinyGPSPlus gps;

float AccX, AccY, AccZ;
float prevAx, prevAy, prevAz = 0;
unsigned long lastDetectedCrash = 0;
unsigned long messageTimer = 0;
int aThreshold = 5;
int tThreshold = 500;

bool buttonPressed = false;

// The serial connection to the GPS device
SoftwareSerial gpsSerial(RXPin, TXPin);
SoftwareSerial gsmSerial(4, 3);

ISR(INT0_vect)
{
  buttonPressed = true;
  digitalWrite(13, buttonPressed);
  crashDetected = false;
  Serial.println("pressed");
}


void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    gsmSerial.write(Serial.read());
  }
  while (gsmSerial.available())
  {
    Serial.write(gsmSerial.read());
  }
}

void sendMessage(String message)
{
  gsmSerial.println("AT"); // Once the handshake test is successful, it will back to OK
  updateSerial();
  gsmSerial.println("AT+CMEE=2");
  updateSerial();
  gsmSerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  gsmSerial.println("AT+CMGS=\"+628996970405\"");
  updateSerial();
  gsmSerial.print(message); // text content
  updateSerial();
  gsmSerial.write(26);
}


void updateGPS()
{
  // Process all available GPS data
  while (gpsSerial.available() > 0)
  {
    gpsConnected = true;
    char c = gpsSerial.read();
    if (gps.encode(c))
    {

      if (gps.location.isUpdated())
      {

        lat = gps.location.lat();
        lon = gps.location.lng();
      }
    }
  }

  if (gps.location.age() > 5000)
  {
    gpsConnected = false;
  }
}

void update_acc(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x18);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  AccX = ((float)AccXLSB / 2048) * 9.8;
  AccY = ((float)AccYLSB / 2048) * 9.8;
  AccZ = ((float)AccZLSB / 2048) * 9.8;
}

void mpu_begin()
{
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void setup()
{

  Serial.begin(9600);
  gpsSerial.begin(9600);
  gsmSerial.begin(9600);
  pinMode(11, OUTPUT);
  PORTD |= (1 << PD2);
  EICRA |= (1 << ISC01);
  EIMSK |= (1 << INT0);
  sei();
  mpu_begin();
}

void loop()
{
  buttonPressed = false;
  gpsSerial.listen();
  updateGPS();

  update_acc();
  
  float deltaAx = AccX - prevAx;
  float deltaAy = AccY - prevAy;
  float deltaAz = AccZ - prevAz;
  float aSum = sqrt(deltaAx * deltaAx + deltaAy * deltaAy + deltaAz * deltaAz);
  Serial.println("Acc: " + String(aSum)+ " lon: " + String(lon) + " lat: " + String(lat) + " gps status: " + String(gpsConnected));
  prevAx = AccX;
  prevAy = AccY;
  prevAz = AccZ;

  if (aSum > aThreshold)
  {
    if (lastDetectedCrash == 0)
    {
      lastDetectedCrash = millis();
    }
    unsigned long timeSinceCrash = millis() - lastDetectedCrash;
    if (timeSinceCrash > tThreshold)
    {
      crashDetected = true;
      Serial.println("Crash detected, sending message...");
      messageTimer = millis();

      if (lat != 0 && lon != 0)
      {

        gsmSerial.listen();
        sendMessage("Crash Detected! time: " + String(gps.time.hour() + 7) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + " Location: https://www.google.com/maps/search/?api=1&query=" + String(lat, 6) + "," + String(lon, 6));

        // Serial.println("Crash Detected! Location: https://www.google.com/maps/search/?api=1&query=" + String(lat, 6) + "," + String(lon, 6));
      }
      else
      {
        gsmSerial.listen();
        sendMessage("Crash Detected! GPS location not available.");
        // Serial.println("Crash Detected! GPS location not available.");
      }

      lastDetectedCrash = 0;
    }
  }

  if (crashDetected)
  {
    tone(11, 1000);
  }
  else
  {
    noTone(11);
  }
}
