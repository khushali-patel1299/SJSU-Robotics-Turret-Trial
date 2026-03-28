#include <Wire.h>
#include <Servo.h>

const int MPU_ADDR = 0x68; 
const int NUM_SAMPLES = 100;

class TurretAxis {
  private:
    int16_t samples[NUM_SAMPLES];
    int index = 0;
    long axisSum = 0;
    int lastAngle = -100;
    Servo motor;
    int motorPin;

  public:
    TurretAxis(int pin) : motorPin(pin) {}
    void init() { motor.attach(motorPin); motor.write(90); }
    void addSample(int16_t newVal) {
      axisSum -= samples[index];
      samples[index] = newVal;
      axisSum += samples[index];
      index = (index + 1) % NUM_SAMPLES;
    }
    float getMean() { return (float)axisSum / (float)NUM_SAMPLES; }
    float getStdDev() {
      float meanVal = getMean();
      float varianceSum = 0;
      for (int j = 0; j < NUM_SAMPLES; j++) {
        float diff = (float)samples[j] - meanVal;
        varianceSum += diff * diff;
      }
      return sqrt(varianceSum / (float)NUM_SAMPLES);
    }
    void commandMotor(int targetAngle) {
      int constrainedAngle = constrain(targetAngle, 0, 180);
      if (abs(constrainedAngle - lastAngle) > 3) {
        motor.write(constrainedAngle);
        lastAngle = constrainedAngle;
      }
    }
};

TurretAxis axisY(9);
TurretAxis axisZ(10);

int sr_no = 0;
unsigned long lastPrintTime = 0;

// Sample Rate Variables
unsigned long sampleCounter = 0;
unsigned long lastSampleTime = 0;
float currentSampleRate = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  axisY.init();
  axisZ.init();
  delay(500); 
}

void loop() {
  sampleCounter++; // Increment every loop

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t xAccel = Wire.read() << 8 | Wire.read();
  int16_t yAccel = Wire.read() << 8 | Wire.read();
  int16_t zAccel = Wire.read() << 8 | Wire.read();

  axisY.addSample(yAccel);
  axisZ.addSample(zAccel);

  float meanY = axisY.getMean();
  float meanZ = axisZ.getMean();
  int angleY = map((int)meanY, -17000, 17000, 0, 180);
  int angleZ = map((int)meanZ, -17000, 17000, 0, 180);

  axisY.commandMotor(angleY);
  axisZ.commandMotor(angleZ);

  // Calculate Sample Rate and Print every 1000ms (1 second)
  if (millis() - lastSampleTime >= 1000) {
    currentSampleRate = sampleCounter;
    sampleCounter = 0;
    lastSampleTime = millis();
  }

  if (millis() - lastPrintTime > 400) {
    Serial.print(++sr_no);
    Serial.print(") Hz: "); Serial.print(currentSampleRate); // SAMPLE RATE
    Serial.print(" | X: "); Serial.print(xAccel);
    Serial.print(" | Mean Y: "); Serial.print(meanY);
    Serial.print(" | StdDev Y: "); Serial.print(axisY.getStdDev());
    Serial.print(" | Sample Rate (Hz): "); Serial.print(currentSampleRate);
    Serial.println();
    lastPrintTime = millis();
  }
  delay(10); 
}