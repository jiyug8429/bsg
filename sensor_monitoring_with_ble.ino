/*
  Optical Heart Rate Detection (PBA Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30105_Breakout

  This is a demo to show the reading of heart rate or beats per minute (BPM) using
  a Penpheral Beat Amplitude (PBA) algorithm.

  It is best to attach the sensor to your finger using a rubber band or other tightening
  device. Humans are generally bad at applying constant pressure to a thing. When you
  press your finger against the sensor it varies enough to cause the blood in your
  finger to flow differently which causes the sensor readings to go wonky.

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected

  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/

#include <Wire.h>
#include <SoftwareSerial.h> // 블루투스 시리얼 통신 라이브러리 추가
#include <DHT11.h>
#include "MAX30105.h"
#include "heartRate.h"

#define BT_RXD 3
#define BT_TXD 2
SoftwareSerial bluetooth(BT_TXD, BT_RXD);    
DHT11 dht11(5); //온습도 디지털핀 
//압력센서
int FSRsensor = A0;
//심박센서 
MAX30105 particleSensor;



////////////심박수//////////

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg, irValue;



void setup()
{
  Serial.begin(9600);
  bluetooth.begin(9600);     
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

}

void readHeartRate(){
  irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  
}

void loop()
{
  //////////심박수 읽기//////////////////
  readHeartRate();
  
  if(bluetooth.available()){
    char c = bluetooth.read();
    Serial.println(c);
    if(c=='b'){
      bluetooth.println(beatAvg);
    }
  }
  //////////////온습도 읽기/////////////////
  int temperature = 0;
  int humidity = 0;
  int result = dht11.readTemperatureHumidity(temperature, humidity);
  int pressure = analogRead(FSRsensor);  



  if (Serial.available()) {
    if(temperature!=0){
      bluetooth.print(beatAvg);
      bluetooth.print(",");
      bluetooth.print(temperature);
      bluetooth.print(",");
      bluetooth.print(humidity);
      bluetooth.print(",");
      bluetooth.print(pressure);
    }
  }

  Serial.print(beatsPerMinute);
  Serial.print(beatAvg);
  Serial.print(",");
  Serial.print(temperature);
  Serial.print(",");
  Serial.print(humidity);
  Serial.print(",");
  Serial.print(pressure);

  
  if (irValue < 50000) Serial.print(" No finger?");

  Serial.println();

  delay(10);
}
