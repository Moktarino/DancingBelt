#include <I2Cdev.h>
#include <FastLED.h>
#include <Wire.h>
#include <math.h>
#define PI 3.14159265
#define NUM_LEDS 144
#define DATA_PIN 6

int offsets[] = {-1524,-89,1450,-9,-32,25};

double accX, accY, accZ;



int frequency = 100; // Set reading frequency [Hz] - readings per second.

double multiplier = 9.81; // Set multiplier to convert accelerometer data to acceleration m/s^2
double acc_threshold = 0.2; // Set threshold to ignore small noise in acceleration
int xAcc_Filtered = 0; // initialise filtered value to 0
int acceleration_Filtered=0;
int velocity_Filtered =0;


// coefficient to apply filtering, adjust these to give better result
double alpha = 0.15;
double beta = 0.95;
double gamma = 0.9;


// Initialise variables for calculation
long current_time = millis();
long t0 = millis();
long previous_time = 0;
int previous_velocity =0;
int current_velocity = 0;
int total_displacement = 0;



void Setup() {

  Serial.begin(115200);
  initFastLED();
  initMPUraw();
  
//// Initialise accelerometer reading
for {i=0; i < 5; i++){
 getData()
 gx = accZ;
 xAcc_Filtered = gx - ((1-alpha)*xAcc_Filtered + alpha*gx); // apply high pass filter to the accelerometer output
}

init_accx = (floor(xAcc_Filtered*100)*multiplier/100); // accelerometer data is multiplied to get 1g=10m/s^2

}



void Loop() {

current_time = millis() - t0;

getData();

gx = accZ;
xAcc_Filtered = gx - ((1-alpha)*xAcc_Filtered + alpha*gx); // apply high pass filter to the accelerometer output
acceleration = (floor(xAcc_Filtered*100)*multiplier/100 - init_accx); // convert to acceleration from accelerometer

accFilt = (1-beta)*accFilt + beta*acceleration;
// ignore small value acceleration due to noise
if( accFilt > -acc_threshold && accFilt < acc_threshold){
 accFilt = 0;
}

x=accFilt;

// Calculate velocity
current_velocity = previous_velocity + int(x, previous_time, current_time);
// low pass filter to filter out noise from calculated velocity
velocity_Filtered = (1-gamma)*velocity_Filtered + gamma*current_velocity;
// Integrate velocity to displacement


displacement = int(current_velocity, previous_time, current_time);
total_displacement = total_displacement + displacement;
previous_velocity = velocity_Filtered;
previous_time = current_time;



pause(1/frequency);


}



void getData() {
  uint32_t  timer2 = micros();
  while (i2cRead(0x3B, i2cData, 14)) {
    if (timer2 > 5000) {break;}
  }
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
}






