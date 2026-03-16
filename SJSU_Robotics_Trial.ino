#include <Wire.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
const int NUM_SAMPLES = 100;
int xSamples[NUM_SAMPLES];
int i = 0;
int sr_no = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Wake up the MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power Management register
  Wire.write(0);    // Wake up
  Wire.endTransmission(true);
  
  delay(400); // Stabilization time
}

void loop() {
    
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Start reading from ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); // Request 6 bytes (2 per axis)

  // Read high and low bytes for each axis
  int16_t xAccel = Wire.read() << 8 | Wire.read();
  int16_t yAccel = Wire.read() << 8 | Wire.read();
  int16_t zAccel = Wire.read() << 8 | Wire.read();

  // Output to Serial Monitor
  

  // Circular Buffer
  xSamples[i] = xAccel;
  i = (i+1) % NUM_SAMPLES;
  
  float mean, StdDev ;

  // Calculate statistics of lates 100 samples
  if(i == 0) {
    float sum = 0;
    for( int j = 0 ; j < 100 ; j++ )
        sum += xSamples[j];

    mean = sum / NUM_SAMPLES;

    float variance_num = 0;
    for( int j = 0 ; j < 100 ; j++)
        variance_num += pow(xSamples[j] - mean , 2);

    StdDev = sqrt(variance_num / NUM_SAMPLES);
  }

  Serial.print(++sr_no);
  Serial.print(" X: "); Serial.print(xAccel);
  Serial.print(" | Y: "); Serial.print(yAccel);
  Serial.print(" | Z: "); Serial.print(zAccel);
  Serial.print(" | Mean: "); Serial.print(mean);
  Serial.print(" | StdDev: "); Serial.print(StdDev);
  Serial.println(" ");

  delay(400); // Slow down for readability
}