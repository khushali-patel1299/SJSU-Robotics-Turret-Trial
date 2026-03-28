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
    float filteredAngle = 90.0;
    unsigned long lastMicros;
    bool isReversed;

  public:
    TurretAxis(int pin, bool reverse = false) : motorPin(pin), isReversed(reverse) {}

    void init() {
      motor.attach(motorPin);
      motor.write(90);
      lastMicros = micros();
    }

    void addSample(int16_t newVal) {
      axisSum -= samples[index];
      samples[index] = newVal;
      axisSum += samples[index];
      index = (index + 1) % NUM_SAMPLES;
    }

    float getMean() { return (float)axisSum / (float)NUM_SAMPLES; }
    
    float calculateStdDev() {
      float meanVal = getMean();
      float varSum = 0;
      for (int j = 0; j < NUM_SAMPLES; j++) {
        float diff = (float)samples[j] - meanVal;
        varSum += diff * diff;
      }
      return sqrt(varSum / (float)NUM_SAMPLES);
    }

    void updatePointing(float target, int16_t gyro) {
      unsigned long curMicros = micros();
      float dt = (curMicros - lastMicros) / 1000000.0;
      lastMicros = curMicros;

      float gyroRate = (float)gyro / 131.0;
      filteredAngle = 0.98 * (filteredAngle + gyroRate * dt) + 0.02 * target;

      int finalAngle = (int)constrain(filteredAngle, 0, 180);
      
      // APPLY REVERSE MAPPING FOR PIN 9
      if (isReversed) {
        finalAngle = 180 - finalAngle;
      }

      if (abs(finalAngle - lastAngle) > 1) {
        motor.write(finalAngle);
        lastAngle = finalAngle;
      }
    }
};

// Pin 9 is reversed as requested; Pin 10 remains standard
TurretAxis axisX(9, true); 
TurretAxis axisY(10, false); 

unsigned long lastPrint = 0;
unsigned long sampleCounter = 0;
unsigned long lastHzCheck = 0;
float currentHz = 0;
int sr_no = 0;

void setup() {
  Serial.begin(115200); 
  Wire.begin();
  Wire.setClock(400000); 
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);

  axisX.init();
  axisY.init();
}

void loop() {
  sampleCounter++; // For Sample Rate calculation

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t ax = (Wire.read() << 8 | Wire.read());
  int16_t ay = (Wire.read() << 8 | Wire.read());
  int16_t az = (Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read(); 
  int16_t gx = (Wire.read() << 8 | Wire.read());
  int16_t gy = (Wire.read() << 8 | Wire.read());
  int16_t gz = (Wire.read() << 8 | Wire.read());

  // Trig angles for vertical orientation
  float pitch = atan2(ax, az) * 180 / PI + 90; 
  float roll = atan2(ay, az) * 180 / PI + 90;

  axisX.addSample(ax);
  axisY.addSample(ay);

  axisX.updatePointing(pitch, gx);
  axisY.updatePointing(roll, gy);

  // Calculate Hz every second
  if (millis() - lastHzCheck >= 1000) {
    currentHz = sampleCounter;
    sampleCounter = 0;
    lastHzCheck = millis();
  }

  // SERIAL MONITOR: All required info per PDF
  if (millis() - lastPrint > 400) {
    Serial.print(++sr_no);
    Serial.print(") Hz: "); Serial.print(currentHz);
    Serial.print(" | RAW X:"); Serial.print(ax);
    Serial.print(" Y:"); Serial.print(ay);
    Serial.print(" Z:"); Serial.print(az);
    
    Serial.print(" || X-MEAN: "); Serial.print(axisX.getMean());
    Serial.print(" X-STD: "); Serial.print(axisX.calculateStdDev());
    
    Serial.print(" || Y-MEAN: "); Serial.print(axisY.getMean());
    Serial.print(" Y-STD: "); Serial.println(axisY.calculateStdDev());
    
    lastPrint = millis();
  }
}