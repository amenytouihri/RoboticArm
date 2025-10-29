/*
   7MRI0060 - Applied Medical Robotics Module
   September 2024
   Author: Harry Robertshaw
   Updated: PID control added and stop-at-target logic
   Purpose: 
     - Control two motors using an H-Bridge with PWM (speed control)
     - Read rotary encoders using interrupts
     - Calculate motor angles in degrees
     - Move motors to target angles using PID and stop when target reached
*/

// -------------------- ENCODER SETUP --------------------
#define ENC_A_A 2  // Motor A encoder Channel A
#define ENC_A_B 4  // Motor A encoder Channel B
#define ENC_B_A 3  // Motor B encoder Channel A
#define ENC_B_B 6  // Motor B encoder Channel B

const float PPR = 360.0;  // Pulses per revolution
const float GR  = 30.0;   // Gear ratio

volatile int counterA = 0;
int aLastState;
float positionA = 0.0;

volatile int counterB = 0;
int bLastState;
float positionB = 0.0;

// -------------------- MOTOR SETUP --------------------
// Motor A pins
const int mA_pinIn1 = 7;
const int mA_pinIn2 = 8;
const int mA_pinEnA = 5;

// Motor B pins
const int mB_pinIn3 = 9;
const int mB_pinIn4 = 10;
const int mB_pinEnB = 11;

// -------------------- PID SETUP --------------------
float Kp = 0.1;    
float Ki = 0.005;  
float Kd = 0.02;   

float targetA = 90.0;  
float targetB = 180.0; 

float prevErrorA = 0.0, integralA = 0.0;
float prevErrorB = 0.0, integralB = 0.0;
float threshold = 1.0; // degrees tolerance for stopping

void setup() {
  // Encoder setup
  pinMode(ENC_A_A, INPUT_PULLUP);
  pinMode(ENC_A_B, INPUT_PULLUP);
  pinMode(ENC_B_A, INPUT_PULLUP);
  pinMode(ENC_B_B, INPUT_PULLUP);

  aLastState = digitalRead(ENC_A_A);
  bLastState = digitalRead(ENC_B_A);

  attachInterrupt(digitalPinToInterrupt(ENC_A_A), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_A), updateEncoderB, CHANGE);

  Serial.begin(9600);

  // Motor setup
  pinMode(mA_pinIn1, OUTPUT);
  pinMode(mA_pinIn2, OUTPUT);
  pinMode(mA_pinEnA, OUTPUT);

  pinMode(mB_pinIn3, OUTPUT);
  pinMode(mB_pinIn4, OUTPUT);
  pinMode(mB_pinEnB, OUTPUT); 
}

void loop() {
  // -------------------- READ POSITIONS --------------------
  positionA = ((float)counterA / (PPR * GR)) * 360.0;
  positionB = ((float)counterB / (PPR * GR)) * 360.0;

  // -------------------- PID CONTROL MOTOR A --------------------
  float errorA = targetA - positionA;
  if (abs(errorA) > threshold) integralA += errorA;
  float derivativeA = errorA - prevErrorA;
  float outputA = Kp*errorA + Ki*integralA + Kd*derivativeA;
  int pwmA = (abs(errorA) < threshold) ? 0 : constrain(abs(outputA), 0, 255);

  if (pwmA > 0) {
    if (outputA > 0) {
      digitalWrite(mA_pinIn1, HIGH);
      digitalWrite(mA_pinIn2, LOW);
    } else {
      digitalWrite(mA_pinIn1, LOW);
      digitalWrite(mA_pinIn2, HIGH);
    }
  } else {
    digitalWrite(mA_pinIn1, LOW);
    digitalWrite(mA_pinIn2, LOW); // stop motor
  }

  analogWrite(mA_pinEnA, pwmA);
  prevErrorA = errorA;

  // -------------------- PID CONTROL MOTOR B --------------------
  float errorB = targetB - positionB;
  if (abs(errorB) > threshold) integralB += errorB;
  float derivativeB = errorB - prevErrorB;
  float outputB = Kp*errorB + Ki*integralB + Kd*derivativeB;
  int pwmB = (abs(errorB) < threshold) ? 0 : constrain(abs(outputB), 0, 255);

  if (pwmB > 0) {
    if (outputB > 0) {
      digitalWrite(mB_pinIn3, HIGH);
      digitalWrite(mB_pinIn4, LOW);
    } else {
      digitalWrite(mB_pinIn3, LOW);
      digitalWrite(mB_pinIn4, HIGH);
    }
  } else {
    digitalWrite(mB_pinIn3, LOW);
    digitalWrite(mB_pinIn4, LOW); // stop motor
  }

  analogWrite(mB_pinEnB, pwmB);
  prevErrorB = errorB;

  // -------------------- PRINT --------------------
  Serial.print("Motor A: "); Serial.print(positionA);
  Serial.print(" | PWM: "); Serial.print(pwmA);
  Serial.print(" | Error: "); Serial.print(errorA);
  Serial.print(" || Motor B: "); Serial.print(positionB);
  Serial.print(" | PWM: "); Serial.print(pwmB);
  Serial.print(" | Error: "); Serial.println(errorB);

  delay(200); // PID loop interval
}

// -------------------- ENCODER INTERRUPTS --------------------
void updateEncoderA() {
  int aState = digitalRead(ENC_A_A);
  int bState = digitalRead(ENC_A_B);
  if (aState != aLastState) {
    if (aState == bState) counterA++;
    else counterA--;
  }
  aLastState = aState;
}

void updateEncoderB() {
  int aState = digitalRead(ENC_B_A);
  int bState = digitalRead(ENC_B_B);
  if (aState != bLastState) {
    if (aState == bState) counterB++;
    else counterB--;
  }
  bLastState = aState;
}
