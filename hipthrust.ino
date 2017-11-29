#include <I2Cdev.h>
#include <I2Cdev.h>
#include <FastLED.h>
#include <Wire.h>
#include <math.h>
#define PI 3.14159265
#define NUM_LEDS 144
#define DATA_PIN 6

int offsets[] = {-1524,-89,1450,-9,-32,25};

double accX, accY, accZ;
uint8_t i2cData[14];

CRGB leds[NUM_LEDS];  
uint8_t max_bright = 64; 

const int OuterWindowSize = 60;
const int InnerWindowSize = 20;
const int DiffArraySize = 30;
const int ColorSmoothArraySize = 5;
const int cycle = 5;

int DiffArray[DiffArraySize];
int DiffIndex = 0;
int DiffMax = 0;
int DiffMin = 0;
int OuterWindowArray[OuterWindowSize];      // the readings from the analog input
int OuterWindowIndex = 0;              // the index of the current reading
int OuterWindowAvg = 0;
long OuterWindowTotal = 0;
int OuterWindow = 0;

int ColorSmoothArray[ColorSmoothArraySize];
int ColorSmoothIndex = 0;


int InnerWindowArray[InnerWindowSize];      // the readings from the analog input
int InnerWindowIndex = 0;              // the index of the current reading
int InnerWindowAvg = 0;
long InnerWindowTotal = 0;
int InnerWindow = 0;
int diff = 0;
int reading = 0;
int initvalue = 16000;
int color = 0;
int ColorTotal = 0;
int diffpercent;
int lastColor = 0;
int brightness = 0; 
int lastBrightness = 0;
void setup() {
  Serial.begin(115200);
  initFastLED();
  initMPUraw();

  for (int thisReading = 0; thisReading < OuterWindowSize; thisReading++) {
  OuterWindowArray[thisReading] = initvalue;
  OuterWindowTotal = OuterWindowTotal + OuterWindowArray[thisReading];
  }
  //Serial.print("OuterWindowTotal: "); Serial.println(OuterWindowTotal);
  
  for (int thisReading = 0; thisReading < InnerWindowSize; thisReading++) {
  InnerWindowArray[thisReading] = initvalue;
  InnerWindowTotal = InnerWindowTotal + InnerWindowArray[thisReading];
  }
  
}


int smoother (int readvalue, int smoothindex, int smoothingarray[], int total) {
  total = total - smoothingarray[smoothindex];
  smoothingarray[smoothindex] = readvalue;
  total = total + smoothingarray[smoothindex];
  return abs(total);
  }

 int findmax (int maxarray[], int maxsize) {
  int currentmax = 0;
  for ( int i = 0; i < maxsize; i++ ) {
    if (maxarray[i] > currentmax) {
      currentmax = maxarray[i];
    }
  }
  return currentmax;
 }
 int findmin (int minarray[], int maxsize) {
  int currentmin = minarray[0];
  for ( int i = 0; i < maxsize; i++ ) {
    if (minarray[i] < currentmin) {
      currentmin = minarray[i];
    }
  }
  return currentmin;
 }

 
void loop() {
  getData();
  reading = abs(accZ);
  OuterWindowTotal = smoother(reading, OuterWindowIndex, OuterWindowArray, OuterWindowTotal);
  InnerWindowTotal = smoother(reading, InnerWindowIndex, InnerWindowArray, InnerWindowTotal);
  OuterWindowIndex = OuterWindowIndex + 1;
  InnerWindowIndex = InnerWindowIndex + 1;
  if (OuterWindowIndex >= OuterWindowSize) {OuterWindowIndex = 0;}
  if (InnerWindowIndex >= InnerWindowSize) {InnerWindowIndex = 0;}
  OuterWindow = OuterWindowTotal / OuterWindowSize;
  InnerWindow = InnerWindowTotal / InnerWindowSize; 
  diff = abs(InnerWindow - OuterWindow);
  DiffArray[DiffIndex] = diff;
  DiffIndex = DiffIndex + 1;
  if (DiffIndex >= DiffArraySize) { DiffIndex = 0;}

  DiffMax = findmax(DiffArray, DiffArraySize);
  DiffMin = findmin(DiffArray, DiffArraySize);
  diffpercent = (float(diff - DiffMin)/float(DiffMax - DiffMin)) * 100;
  
  color = map(diffpercent,0,100,5,240);
  
  if (abs(color - lastColor) > 30) {
    if (color > lastColor) {
      color = lastColor + 1;
    }
    else {
      color = lastColor - 1;
    }
  }
   Serial.print(color);Serial.print(", ");Serial.println(lastColor);
  lastColor = color;


brightness = map(diff,DiffMin,DiffMax,35,220);
  if (abs(brightness - lastBrightness) > 30) {
    if (brightness > lastBrightness) {
      brightness = lastBrightness + 1;
    }
    else {
      brightness = lastBrightness - 1;
    }
  }

lastBrightness = brightness;

  //ColorTotal = smoother(color, ColorSmoothIndex, ColorSmoothArray, ColorTotal);
  //color = ColorTotal / ColorSmoothArraySize;
  
  Serial.print(color); Serial.print(", ");
  Serial.print(DiffMin); Serial.print(", ");
  Serial.print(DiffMax); Serial.print(", "); 
  Serial.print(diff); Serial.print(", ");
  Serial.println( float(diff)/float(DiffMax) );
 // Serial.print(InnerWindowTotal);Serial.print(", "); Serial.print(OuterWindowTotal);Serial.print(", "); Serial.println(diff);
  //Serial.println(diff);



if (InnerWindow > OuterWindow) {
  InnerWindow = InnerWindow + (diff/3);
  OuterWindow = OuterWindow - (diff/3);
}
else {
  OuterWindow = OuterWindow - (diff/3);
  InnerWindow = InnerWindow + (diff/3);
}
  delay(cycle);
  
  fill_solid(leds,NUM_LEDS,CHSV( color, 187, brightness));
  FastLED.show();

  }

void initFastLED(){
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(max_bright);
  set_max_power_in_volts_and_milliamps(5, 500);    
}



void initMPUraw(){
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz   
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }
  
  delay(100); // Wait for sensor to stabilize
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];
}


void getData() {

  uint32_t  timer2 = micros();
  while (i2cRead(0x3B, i2cData, 14)) {if (timer2 > 5000) {break;}}
  
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);

}

const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}