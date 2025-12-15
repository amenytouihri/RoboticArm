/*
   Dual-Motor Robotic Arm Prototype
   ------------------------------------------------------
   Purpose:
     - Controls two DC motors (Motor A & Motor B) using encoders
     - Moves Motor A to 90° and Motor B to 180°
     - Uses PID control for smooth, precise motion
     - Implements gradual ramping (virtual target stepping)
       to prevent abrupt starts/stops
     - Automatically stops when target position is reached
*/

// -------------------- ENCODER SETUP --------------------

// Encoder pin assignments (each encoder has two channels A & B)
#define ENC_A_A 2   // Motor A encoder Channel A
#define ENC_A_B 4   // Motor A encoder Channel B
#define ENC_B_A 3   // Motor B encoder Channel A
#define ENC_B_B 6   // Motor B encoder Channel B

// Encoder specs
const float PPR = 600.0;  // Pulses per revolution (encoder resolution)
const float GR  = 30.0;   // Gear ratio of motor gearbox

// Motor A encoder state
volatile long counterA = 0; // counts encoder ticks (updated by interrupt)
int aLastState;             // stores previous state of encoder A channel
float positionA = 0.0;      // current position in degrees

// Motor B encoder state
volatile long counterB = 0; // counts encoder ticks (updated by interrupt)
int bLastState;             // stores previous state of encoder A channel
float positionB = 0.0;      // current position in degrees


// -------------------- MOTOR SETUP --------------------

// Motor A driver pins
const int mA_pinIn1 = 7;   // Motor direction pin 1
const int mA_pinIn2 = 8;   // Motor direction pin 2
const int mA_pinEnA = 5;   // PWM enable pin for Motor A

// Motor B driver pins
const int mB_pinIn3 = 9;   // Motor direction pin 1
const int mB_pinIn4 = 10;  // Motor direction pin 2
const int mB_pinEnB = 11;  // PWM enable pin for Motor B


// -------------------- PID CONFIGURATION --------------------

// PID coefficients (tuning parameters)
float Kp = 3.0;    // Proportional gain: response to error magnitude
float Ki = 0.01;   // Integral gain: response to accumulated error
float Kd = 0.1;    // Derivative gain: response to error rate of change

float threshold = 1.0; // tolerance in degrees — when error < threshold, stop motor

// PID state variables (for both motors)
float prevErrorA = 0.0, integralA = 0.0;
float prevErrorB = 0.0, integralB = 0.0;


// -------------------- MOTION SMOOTHING --------------------

// Used to prevent sudden jumps in PWM output
int pwmA_current = 0;    // current PWM applied to Motor A
int pwmB_current = 0;    // current PWM applied to Motor B
int pwmRampStep = 10;    // maximum PWM change allowed per control loop iteration

// Virtual (intermediate) target positions for gradual movement
float virtualTargetA = 0.0;  // current intermediate target for Motor A
float virtualTargetB = 0.0;  // current intermediate target for Motor B

// Final target angles for each motor (in degrees)
float targetA = 90.0;
float targetB = 180.0;

// Step size (in degrees) for incrementing toward the target
float step = 1.5; // smaller = smoother but slower motion


// -------------------- SETUP --------------------
void setup() {
  Serial.begin(9600); // open serial monitor for debugging

  // --- Initialize encoders ---
  pinMode(ENC_A_A, INPUT_PULLUP);
  pinMode(ENC_A_B, INPUT_PULLUP);
  pinMode(ENC_B_A, INPUT_PULLUP);
  pinMode(ENC_B_B, INPUT_PULLUP);

  // Read initial states for encoders
  aLastState = digitalRead(ENC_A_A);
  bLastState = digitalRead(ENC_B_A);

  // Attach interrupts for encoder channel A signals
  // They trigger every time the encoder channel A changes (rising/falling)
  attachInterrupt(digitalPinToInterrupt(ENC_A_A), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_A), updateEncoderB, CHANGE);

  // --- Initialize motor driver pins as outputs ---
  pinMode(mA_pinIn1, OUTPUT);
  pinMode(mA_pinIn2, OUTPUT);
  pinMode(mA_pinEnA, OUTPUT);

  pinMode(mB_pinIn3, OUTPUT);
  pinMode(mB_pinIn4, OUTPUT);
  pinMode(mB_pinEnB, OUTPUT);
}


// -------------------- MAIN LOOP --------------------
void loop() {

  // --- Compute actual motor shaft positions (in degrees) ---
  // Formula: (encoder ticks / (pulses per rev * gear ratio)) * 360
  positionA = ((float)counterA / (PPR * GR)) * 360.0;
  positionB = ((float)counterB / (PPR * GR)) * 360.0;


  // --- Gradually move virtual targets toward final targets ---
  // This prevents sudden jumps and ensures smooth movement.
  if (abs(virtualTargetA - targetA) > 0.1) {
    if (virtualTargetA < targetA) virtualTargetA += step;
    else virtualTargetA -= step;
  }

  if (abs(virtualTargetB - targetB) > 0.1) {
    if (virtualTargetB < targetB) virtualTargetB += step;
    else virtualTargetB -= step;
  }


  // -------------------- PID CONTROL FOR MOTOR A --------------------
  float errorA = virtualTargetA - positionA;  // difference between target and actual angle
  integralA += errorA;                        // accumulate error (for steady-state correction)
  float derivativeA = errorA - prevErrorA;    // rate of change of error
  float outputA = Kp * errorA + Ki * integralA + Kd * derivativeA; // PID output
  outputA = constrain(outputA, -255, 255);    // limit PWM output range

  // Stop motor when error is within acceptable range
  if (abs(errorA) < threshold) outputA = 0;

  // Smoothly ramp PWM to avoid sudden changes
  pwmA_current += constrain(outputA - pwmA_current, -pwmRampStep, pwmRampStep);
  int pwmA = abs(pwmA_current); // always positive for PWM magnitude

  // Set motor A direction based on PID output sign
  if (pwmA > 0) {
    if (pwmA_current > 0) {
      digitalWrite(mA_pinIn1, HIGH);  // rotate forward
      digitalWrite(mA_pinIn2, LOW);
    } else {
      digitalWrite(mA_pinIn1, LOW);   // rotate backward
      digitalWrite(mA_pinIn2, HIGH);
    }
  } else {
    // Motor off (no movement)
    digitalWrite(mA_pinIn1, LOW);
    digitalWrite(mA_pinIn2, LOW);
  }

  // Apply PWM to enable pin
  analogWrite(mA_pinEnA, pwmA);

  // Store error for next derivative calculation
  prevErrorA = errorA;


  // -------------------- PID CONTROL FOR MOTOR B --------------------
  float errorB = virtualTargetB - positionB;
  integralB += errorB;
  float derivativeB = errorB - prevErrorB;
  float outputB = Kp * errorB + Ki * integralB + Kd * derivativeB;
  outputB = constrain(outputB, -255, 255);

  if (abs(errorB) < threshold) outputB = 0;

  pwmB_current += constrain(outputB - pwmB_current, -pwmRampStep, pwmRampStep);
  int pwmB = abs(pwmB_current);

  if (pwmB > 0) {
    if (pwmB_current > 0) {
      digitalWrite(mB_pinIn3, HIGH);  // forward direction
      digitalWrite(mB_pinIn4, LOW);
    } else {
      digitalWrite(mB_pinIn3, LOW);   // backward direction
      digitalWrite(mB_pinIn4, HIGH);
    }
  } else {
    digitalWrite(mB_pinIn3, LOW);
    digitalWrite(mB_pinIn4, LOW);
  }
  analogWrite(mB_pinEnB, pwmB);
  prevErrorB = errorB;


  // -------------------- SERIAL DEBUG OUTPUT --------------------
  Serial.print("A: "); Serial.print(positionA, 2);
  Serial.print("° | PWM: "); Serial.print(pwmA);
  Serial.print(" | Target: "); Serial.print(virtualTargetA, 2);
  Serial.print(" | Error: "); Serial.print(errorA, 2);
  Serial.print(" || B: "); Serial.print(positionB, 2);
  Serial.print("° | PWM: "); Serial.print(pwmB);
  Serial.print(" | Target: "); Serial.print(virtualTargetB, 2);
  Serial.print(" | Error: "); Serial.println(errorB, 2);

  delay(50); // short delay for control loop stability
}


// -------------------- ENCODER INTERRUPT FUNCTIONS --------------------

// Called automatically when Motor A encoder Channel A changes
void updateEncoderA() {
  int aState = digitalRead(ENC_A_A);
  int bState = digitalRead(ENC_A_B);

  // Determine rotation direction:
  // if channels A and B are the same, increment; else decrement
  if (aState != aLastState) {
    if (aState == bState) counterA++;
    else counterA--;
  }
  aLastState = aState; // update previous state
}

// Called automatically when Motor B encoder Channel A changes
void updateEncoderB() {
  int aState = digitalRead(ENC_B_A);
  int bState = digitalRead(ENC_B_B);

  if (aState != bLastState) {
    if (aState == bState) counterB++;
    else counterB--;
  }
  bLastState = aState;
}
