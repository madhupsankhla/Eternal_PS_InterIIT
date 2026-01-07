#include <util/atomic.h>

#define ENCA 2
#define ENCB 3
#define PWM_PIN 9
#define DIR_PIN 8

// Calibrated counts per revolution
const float COUNTS_PER_REVOLUTION = 349.2;

volatile long encoderCount = 0;
long prevEncoderCount = 0;

float rpmFiltered = 0;
float rpmPrev = 0;  // For low-pass filter
float eintegral = 0;
float eprev = 0;  // For derivative term
int dir = 1;  // 1=forward, 0=backward

void onEncoderA() {
  // Determine direction and increment/decrement count
  if (digitalRead(ENCA) == digitalRead(ENCB)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup() {
  Serial.begin(115200);
  
  // Set pin modes BEFORE setting outputs
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);

  // Immediately set motor outputs to 0
  digitalWrite(DIR_PIN, LOW);
  analogWrite(PWM_PIN, 0);
  
  delay(1000);  // Wait to ensure motor is fully stopped

  attachInterrupt(digitalPinToInterrupt(ENCA), onEncoderA, CHANGE);
}

void loop() {

  // int pwr = 20;
  // int dir = 1; // 1=forward, 0=backward
  // setMotor(dir, pwr, PWM_PIN, DIR_PIN);

  // Calculate RPM using time between pulses
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  
  if (now - lastPrint >= 100) {
    lastPrint = now;
    
    long currentCount;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      currentCount = encoderCount;
    }
    
    // Calculate RPM using change in encoder count over time interval
    long deltaCount = currentCount - prevEncoderCount;
    float deltaTime = 0.1;  // 100ms = 0.1s
    
    // Convert to RPM: (counts/second) / (counts/revolution) * 60
    float rpm = (deltaCount / deltaTime) / COUNTS_PER_REVOLUTION * 60.0;
    
    prevEncoderCount = currentCount;
    
    // Apply low-pass filter (25 Hz cutoff)
    float v2Filt = 0.854 * rpm + 0.0728 * rpm + 0.0728 * rpmPrev;
    rpmPrev = rpm;
    
    // Apply exponential smoothing filter after low-pass
    float alpha = 0.3;
    rpmFiltered = alpha * v2Filt + (1 - alpha) * rpmFiltered;

    // Set a target rpm
    float targetRpm = 120;

    float kp = 0.5;
    float ki = 2.0;
    float kd = 0.001;  // Start with a small value

    float e = targetRpm - rpmFiltered;
    eintegral += e * 0.1;  // Integral term with dt=0.1s
    float dedt = (e - eprev) / 0.1;  // Derivative term
    float u = kp*e + ki*eintegral + kd*dedt;
    
    eprev = e;  // Save error for next iteration

    // Determine direction based on control signal
    if (u < 0) {
      dir = 0;  // Backward
    } else {
      dir = 1;  // Forward
    }

    int pwmVal = (int)fabs(u);
    if (pwmVal > 255) {
      pwmVal = 255;
    }

    setMotor(dir, pwmVal, PWM_PIN, DIR_PIN);

    // Output for Serial Plotter
    Serial.print("Target:");
    Serial.print(targetRpm, 2);
    Serial.print(",Filtered:");
    Serial.println(rpmFiltered, 2);
  }
}

void setMotor(int dir, int pwmVal, int pwm, int dirPin) {
  digitalWrite(DIR_PIN, dir);
  analogWrite(PWM_PIN, pwmVal);
}