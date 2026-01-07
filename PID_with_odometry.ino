#include <util/atomic.h>

// Motor 1 pins
#define ENCA1 2
#define ENCB1 4
#define PWM_PIN1 9
#define DIR_PIN1 8

// Motor 2 pins
#define ENCA2 3
#define ENCB2 5
#define PWM_PIN2 10
#define DIR_PIN2 7

// Calibrated counts per revolution
const float COUNTS_PER_REVOLUTION = 349.2;

// Wheel diameter in cm
const float WHEEL_DIAMETER_CM = 18.6;
const float WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * 3.14159265359;

// Motor 1 variables
volatile long encoderCount1 = 0;
long prevEncoderCount1 = 0;
float rpmFiltered1 = 0;
float rpmPrev1 = 0;
float eintegral1 = 0;
float eprev1 = 0;
float distanceCm1 = 0;  // Distance traveled in cm

// Motor 2 variables
volatile long encoderCount2 = 0;
long prevEncoderCount2 = 0;
float rpmFiltered2 = 0;
float rpmPrev2 = 0;
float eintegral2 = 0;
float eprev2 = 0;
float distanceCm2 = 0;  // Distance traveled in cm

// Target settings (configurable outside loop)
float targetRpm1 = 0;
float targetRpm2 = 0;
int targetDir1 = 1;  // 1=forward, 0=backward
int targetDir2 = 1;  // 1=forward, 0=backward

void onEncoderA1() {
  // Determine direction and increment/decrement count for motor 1
  if (digitalRead(ENCA1) == digitalRead(ENCB1)) {
    encoderCount1++;
  } else {
    encoderCount1--;
  }
}

void onEncoderA2() {
  // Determine direction and increment/decrement count for motor 2
  if (digitalRead(ENCA2) == digitalRead(ENCB2)) {
    encoderCount2++;
  } else {
    encoderCount2--;
  }
}

void setup() {
  Serial.begin(115200);
  
  // Set pin modes for Motor 1
  pinMode(PWM_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);

  // Set pin modes for Motor 2
  pinMode(PWM_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(ENCA2, INPUT_PULLUP);
  pinMode(ENCB2, INPUT_PULLUP);

  // Immediately set both motors to 0
  digitalWrite(DIR_PIN1, LOW);
  analogWrite(PWM_PIN1, 0);
  digitalWrite(DIR_PIN2, LOW);
  analogWrite(PWM_PIN2, 0);
  
  delay(1000);  // Wait to ensure motors are fully stopped

  attachInterrupt(digitalPinToInterrupt(ENCA1), onEncoderA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2), onEncoderA2, CHANGE);
  
  Serial.println("Arduino Ready");
}

void checkSerialCommands() {
  // Check if data is available from Raspberry Pi
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Parse the two RPM values separated by comma
    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
      float rpm1 = input.substring(0, commaIndex).toFloat();
      float rpm2 = input.substring(commaIndex + 1).toFloat();
      
      // Reset PID integral terms when stopping or changing direction
      if ((rpm1 == 0 && targetRpm1 != 0) || (rpm1 * targetRpm1 < 0)) {
        eintegral1 = 0;
      }
      if ((rpm2 == 0 && targetRpm2 != 0) || (rpm2 * targetRpm2 < 0)) {
        eintegral2 = 0;
      }
      
      // Update target RPM values
      targetRpm1 = rpm1;
      targetRpm2 = -rpm2;  // Invert the sign of motor 2's target RPM
      
      // Send back the current distances
      Serial.print(distanceCm1, 2);
      Serial.print(",");
      Serial.println(distanceCm2, 2);
    }
  }
}

void loop() {
  // Check for incoming serial commands from Raspberry Pi
  checkSerialCommands();
  
  // PID constants (same for both motors)
  float kp = 0.5;
  float ki = 2.0;
  float kd = 0.001;

  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  
  if (now - lastPrint >= 100) {
    lastPrint = now;
    
    // ========== Motor 1 Control ==========
    long currentCount1;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      currentCount1 = encoderCount1;
    }
    
    // Calculate RPM for motor 1
    long deltaCount1 = currentCount1 - prevEncoderCount1;
    float deltaTime = 0.1;  // 100ms = 0.1s
    float rpm1 = (deltaCount1 / deltaTime) / COUNTS_PER_REVOLUTION * 60.0;
    prevEncoderCount1 = currentCount1;
    
    // Apply low-pass filter
    float v2Filt1 = 0.854 * rpm1 + 0.0728 * rpm1 + 0.0728 * rpmPrev1;
    rpmPrev1 = rpm1;
    
    // Apply exponential smoothing filter
    float alpha = 0.3;
    rpmFiltered1 = alpha * v2Filt1 + (1 - alpha) * rpmFiltered1;

    // PID control for motor 1
    float e1 = targetRpm1 - rpmFiltered1;
    eintegral1 += e1 * 0.1;
    float dedt1 = (e1 - eprev1) / 0.1;
    float u1 = kp*e1 + ki*eintegral1 + kd*dedt1;
    eprev1 = e1;

    // Determine direction and PWM for motor 1
    int dir1;
    if (u1 < 0) {
      dir1 = !targetDir1;  // Reverse if control is negative
    } else {
      dir1 = targetDir1;
    }

    int pwmVal1 = (int)fabs(u1);
    if (pwmVal1 > 255) pwmVal1 = 255;
    setMotor(dir1, pwmVal1, PWM_PIN1, DIR_PIN1);

    // Calculate distance traveled for motor 1
    float revolutions1 = currentCount1 / COUNTS_PER_REVOLUTION;
    distanceCm1 = revolutions1 * WHEEL_CIRCUMFERENCE_CM;

    // ========== Motor 2 Control ==========
    long currentCount2;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      currentCount2 = encoderCount2;
    }
    
    // Calculate RPM for motor 2
    long deltaCount2 = currentCount2 - prevEncoderCount2;
    float rpm2 = (deltaCount2 / deltaTime) / COUNTS_PER_REVOLUTION * 60.0;
    prevEncoderCount2 = currentCount2;
    
    // Apply low-pass filter
    float v2Filt2 = 0.854 * rpm2 + 0.0728 * rpm2 + 0.0728 * rpmPrev2;
    rpmPrev2 = rpm2;
    
    // Apply exponential smoothing filter
    rpmFiltered2 = alpha * v2Filt2 + (1 - alpha) * rpmFiltered2;

    // PID control for motor 2
    float e2 = targetRpm2 - rpmFiltered2;
    eintegral2 += e2 * 0.1;
    float dedt2 = (e2 - eprev2) / 0.1;
    float u2 = kp*e2 + ki*eintegral2 + kd*dedt2;
    eprev2 = e2;

    // Determine direction and PWM for motor 2
    int dir2;
    if (u2 < 0) {
      dir2 = !targetDir2;  // Reverse if control is negative
    } else {
      dir2 = targetDir2;
    }

    int pwmVal2 = (int)fabs(u2);
    if (pwmVal2 > 255) pwmVal2 = 255;
    setMotor(dir2, pwmVal2, PWM_PIN2, DIR_PIN2);

    // Calculate distance traveled for motor 2
    float revolutions2 = currentCount2 / COUNTS_PER_REVOLUTION;
    distanceCm2 = revolutions2 * WHEEL_CIRCUMFERENCE_CM;

    // Output for Serial Plotter and Monitor
    Serial.print("Target1:");
    Serial.print(targetRpm1, 2);
    Serial.print(",Filtered1:");
    Serial.print(rpmFiltered1, 2);
    Serial.print(",Target2:");
    Serial.print(targetRpm2, 2);
    Serial.print(",Filtered2:");
    Serial.print(rpmFiltered2, 2);
    Serial.print(",Dist1(cm):");
    Serial.print(distanceCm1, 2);
    Serial.print(",Dist2(cm):");
    Serial.println(distanceCm2, 2);
  }
}

void setMotor(int dir, int pwmVal, int pwm, int dirPin) {
  digitalWrite(dirPin, dir);
  analogWrite(pwm, pwmVal);
}