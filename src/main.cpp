#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
static const int RXPin = 8, TXPin = 9;

double lat, lon;
bool gpsConnected;
volatile bool crashDetected;

TinyGPSPlus gps;

float AccX, AccY, AccZ;
float prevAx, prevAy, prevAz = 0;
unsigned long lastDetectedCrash = 0;
unsigned long messageTimer = 0;
int aThreshold = 5;
int tThreshold = 500;

bool buttonPressed = false;


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
  gsmSerial.println("AT"); 
  updateSerial();
  gsmSerial.println("AT+CMEE=2");
  updateSerial();
  gsmSerial.println("AT+CMGF=1"); 
  updateSerial();
  gsmSerial.println("AT+CMGS=\"+628996970405\"");
  updateSerial();
  gsmSerial.print(message); 
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
  Wire.write(0x1A);//CONFIG
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);//ACCEL_CONFIG
  Wire.write(0x18); // 16g
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);//ACCEL_XOUT_H 
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6); // ngirim 2 part H, L abis itu dijadiin jadi
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  //2 ^ 16 = 65536/32 -> 32 dari total range +- 16g
  AccX = ((float)AccXLSB / 2048) * 9.8;
  AccY = ((float)AccYLSB / 2048) * 9.8;
  AccZ = ((float)AccZLSB / 2048) * 9.8;
}

void mpu_begin()
{
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); //PWR_MGMT_1
  Wire.write(0x00);// writes value to register
  Wire.endTransmission();
}

void setup()
{

  Serial.begin(9600);
  gpsSerial.begin(9600);
  gsmSerial.begin(9600);
  pinMode(11, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
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
    tone(11, 2000);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    noTone(11);
    digitalWrite(LED_BUILTIN, LOW);
  }
}
