/*   Moktarino's Fancy Dan Dancin' Belt
 *   
 *   This project entails making a motion-reactive display in the form of a belt.
 *   The motion data is provided by a MPU6050 IMU (GY-521 board in my case) and 
 *   displayed on a neopixel array.  The current effect, with proper positioning, will
 *   allow a dancer to roll a colored gradient around their waist by hip motion alone.
 *   
 *   I'm looking forward to adding more effects, another IMU, and other dimensions
 *   to make this another way a dancer can express complex movements visually.
 *   
 *   Much of this code (i2c read/write functions, complementary filter) I cribbed from 
 *   Kristian Lauszus, whose work with the MPU6050 can be found here: https://github.com/TKJElectronics
 *   
 *   The formula for calculating the location of the peak pixel came courtesy of
 *   redditor /u/benargee.  Thanks!
 *   
 *   Please please please contact me with improvement suggestions or ideas for effects, 
 *   especially with tips on how to implement them in code!
 *   
 *   Happy dancing!
 *   
 *   Moktarino@gmail.com
 *     
 */


#include <I2Cdev.h>
#include <FastLED.h>
#include <Wire.h>
#include <math.h>
#define PI 3.14159265
#define NUM_LEDS 144
#define DATA_PIN 6

int offsets[] = {-1524,-89,1450,-9,-32,25};

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double gyroXangle, gyroYangle;
double compAngleX, compAngleY;
double compAngles[2];

int peakLED, currentLED;
uint32_t timer;
uint8_t i2cData[14];

CRGB leds[NUM_LEDS];  
uint8_t max_bright = 64; 

int getNextLEDPosition(double x, double y, int num_leds, int currentLED) {
  int scaler;
  int peakLED, returnLED;
  
  peakLED = (int) (round(atan2 (x,y) * num_leds / (2 * PI))) + num_leds/2;

  // If the current LED is 0-36 and the peak LED is 108-144, add 144 to the current LED (144-pixel strip).
  // If the peak LED is 0-36 and the current LED is 108-144, add 144 to the peak LED.
  // This helps with the math later by avoiding negative numbers while keeping everything in order.
  if ((peakLED > (num_leds/4)*3) && (currentLED < (num_leds/4)*1)) {
    currentLED += num_leds;
  }
  if ((peakLED < (num_leds/4)*1) && (currentLED > (num_leds/4)*3)) {
    peakLED += num_leds;
  }
  
  //Instead of jumping right to the peak LED, we'll move toward it in 1 to 3 pixel steps
  // depending on how far apart the two are.  The steps max at 3 when the distance is more
  // than 10 pixels.
  scaler = map(abs(peakLED - currentLED),0,10,1,3);
  // If the peak LED is ahead of the current LED, add the scaler.
  if ((peakLED - currentLED) > 0) {returnLED = currentLED + scaler;}
  // if the peak LED is behind the current LED, subtract the scaler.
  else {returnLED = currentLED - scaler;}

  // Divide down to get the returnLED within the range of 0 to num_leds
  if (returnLED > num_leds) {
    returnLED -= (num_leds * (returnLED/num_leds));
  }
  
  return returnLED;
  }



void setup() {
  Serial.begin(115200);
  initFastLED();
  initMPUraw();
}

void loop() {
  movingBand();
  FastLED.show();
  delay(5);
}

void movingBand(){
  getCompAngles(&*compAngles);
  peakLED = getNextLEDPosition(compAngles[0], compAngles[1], NUM_LEDS-1, currentLED);
  currentLED = peakLED;
  fill_solid(leds,NUM_LEDS,CRGB::Black);
  drawGradient(NUM_LEDS, leds, currentLED,5);
  Serial.println(currentLED);
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
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();
}

void getCompAngles(double compAngles[]) {
  uint32_t  timer2 = micros();
  while (i2cRead(0x3B, i2cData, 14)) {if (timer2 > 5000) {break;}}
  
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  double gyroXrate = gyroX / 131.0;
  double gyroYrate = gyroY / 131.0;

  gyroXangle += gyroXrate * dt;
  gyroYangle += gyroYrate * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  compAngles[0] = compAngleX;
  compAngles[1] = compAngleY;

}
  


void  drawGradient(int num_leds, CRGB leds[], int peakLED, int distance){
    int beginning, ending, overlap;

    beginning = peakLED - distance;
    ending = peakLED + distance;

    if (ending >= num_leds) {
      overlap = ending - num_leds;
      ending = num_leds - 1;
    }

    if (beginning <= 1) {
      overlap = abs(beginning);
      beginning = 1;
      
    }
   fill_gradient(leds,beginning,CHSV(0,255,255),ending,CHSV(96,255,255),SHORTEST_HUES); 

  }




// Following code courtesy of Kristian Lauszus, TKJ Electronics (2012)
// https://github.com/TKJElectronics/KalmanFilter/blob/master/examples/MPU6050/I2C.ino

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
