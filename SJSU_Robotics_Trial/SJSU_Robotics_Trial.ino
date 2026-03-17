#include <Wire.h>
#include<Servo.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
const int NUM_SAMPLES = 100;
int ySamples[NUM_SAMPLES], zSamples[NUM_SAMPLES];
int i = 0;
int sr_no = 0;
float sumY = 0, sumZ = 0;
int lastAngleY = -100 , lastAngleZ = -100;

//servo motor variables
Servo servo1;
Servo servo2;

//Serial monitor delayed output traker
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Wake up the MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power Management register
  Wire.write(0);    // Wake up
  Wire.endTransmission(true);

  //Servo motor connections
  servo1.attach(9);
  servo2.attach(10);

  //test moverment
  servo1.write(0); delay(1000);
  servo2.write(0); delay(1000);
  servo1.write(90); delay(1000);
  servo2.write(90); delay(1000);
  servo1.write(180); delay(1000);
  servo2.write(180); delay(1000);
  
  delay(300); // Stabilization time
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

  // Circular Buffer
  //removing existing value from sum
  sumY -= ySamples[i];
  sumZ -= zSamples[i];

  //replacing current value with a new value
  ySamples[i] = yAccel;
  zSamples[i] = zAccel;

  //recalculating sum with new value
  sumY += ySamples[i];
  sumZ += zSamples[i];

  i = (i+1) % NUM_SAMPLES;  //index update
  
  float meanY , StdDev_Y , meanZ , StdDev_Z , variance_Y = 0 , variance_Z = 0;;

  meanY = sumY / NUM_SAMPLES;
  meanZ = sumZ / NUM_SAMPLES;


  // Only print to Serial Monitor once every 400ms for readability, without slowing down the code
  if (millis() - lastPrintTime > 400) {
    for( int j = 0 ; j < NUM_SAMPLES ; j++) {
    variance_Y += pow(ySamples[j] - meanY , 2);
    variance_Z += pow(zSamples[j] - meanZ , 2);
    }

    StdDev_Y = sqrt(variance_Y / NUM_SAMPLES);
    StdDev_Z = sqrt(variance_Z / NUM_SAMPLES);
    
    Serial.print(++sr_no);
    Serial.print(")  X: "); Serial.print(xAccel);
    Serial.print(" | Y: "); Serial.print(yAccel);
    Serial.print(" | Z: "); Serial.print(zAccel);
    Serial.print("   | Mean Y: "); Serial.print(meanY);
    Serial.print(" | StdDev Y: "); Serial.print(StdDev_Y);
    Serial.print(" | Mean Z: "); Serial.print(meanZ);
    Serial.print(" | StdDev Z: "); Serial.print(StdDev_Z);
    Serial.println(" ");
    lastPrintTime = millis();
  }


  //mapping mean of I2C data with degrees to move motors
  int angleY = map(meanY, -17000, 17000, 0, 180);
  int angleZ = map(meanZ, -17000, 17000, 0, 180);


  // Only update if the new target is more than 3 degrees away from the last position
  if (abs(angleY - lastAngleY) > 3) {
    servo1.write(constrain(angleY, 0, 180));
    lastAngleY = angleY;
  }
  if (abs(angleZ - lastAngleZ) > 3) {
    servo2.write(constrain(angleZ, 0, 180));
    lastAngleZ = angleZ;
  }

  delay(10); //breathing room delay for I2C 

}