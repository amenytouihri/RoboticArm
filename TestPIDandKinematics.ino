/*
   ==============================================================
   Dual-Motor Robotic Arm Prototype + Serial Angle Input
   ==============================================================
   Purpose:
     - Controls two DC motors (Motor A & Motor B) using encoders
     - Uses PID control for smooth, precise motion
     - Gradually ramps targets (soft motion transitions)
     - Receives target angles (A,B) from Serial input (e.g. Python)
       → Format: "angleA,angleB\n"
     - Designed for a 2-DOF robotic arm joint control demo
*/

// -------------------- ENCODER SETUP --------------------
// Each encoder provides feedback on motor shaft rotation.
// Two channels (A & B) per encoder let us detect direction.

// Encoder pin assignments
#define ENC_A_A 2
#define ENC_A_B 4
#define ENC_B_A 3
#define ENC_B_B 6

// Encoder and gearbox parameters
const float PPR = 600.0;  // Pulses per revolution (encoder)
const float GR  = 30.0;   // Gear ratio (reduction gear)

// Encoder position counters (updated in interrupt routines)
volatile long counterA = 0;
volatile long counterB = 0;

// Used to detect encoder direction changes
int aLastState, bLastState;

// Computed angular positions (in degrees)
float positionA = 0.0;
float positionB = 0.0;


// -------------------- MOTOR SETUP --------------------
// Motor driver pin connections (e.g. L298N, TB6612, etc.)
const int mA_pinIn1 = 7;   // Direction pin 1 for Motor A
const int mA_pinIn2 = 8;   // Direction pin 2 for Motor A
const int mA_pinEnA = 5;   // PWM enable pin (speed control)

const int mB_pinIn3 = 9;   // Direction pin 1 for Motor B
const int mB_pinIn4 = 10;  // Direction pin 2 for Motor B
const int mB_pinEnB = 11;  // PWM enable pin (speed control)


// -------------------- PID CONFIGURATION --------------------
// PID constants control responsiveness and stability
float Kp = 3.0;     // Proportional term → how strongly error is corrected
float Ki = 0.01;    // Integral term → reduces steady-state error
float Kd = 0.1;     // Derivative term → dampens oscillation

float threshold = 1.0; // Dead zone: when error < 1°, motor stops

// PID memory variables (for each motor)
float prevErrorA = 0.0, integralA = 0.0;
float prevErrorB = 0.0, integralB = 0.0;


// -------------------- MOTION SMOOTHING --------------------
// Prevents jerky motor movement when changing targets

int pwmA_current = 0;    // Current PWM value for Motor A
int pwmB_current = 0;    // Current PWM value for Motor B
int pwmRampStep = 10;    // Maximum PWM change per loop iteration

float virtualTargetA = 0.0; // Gradually approaches targetA
float virtualTargetB = 0.0; // Gradually approaches targetB

// Actual target angles (updated from Serial commands)
float targetA = 90.0;    // Default target for Motor A
float targetB = 180.0;   // Default target for Motor B

// Step size for virtual target updates
float step = 1.5;        // Smaller = smoother motion


// -------------------- SERIAL INPUT VARIABLES --------------------
int ledPin = 13;         // LED feedback indicator
float angle1Value;       // Parsed angle for motor A
float angle2Value;       // Parsed angle for motor B
String data_new;         // For printing processed serial data


// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);  // Fast baud rate for real-time control
  Serial.setTimeout(10); // Short serial timeout
  pinMode(ledPin, OUTPUT);

  // --- Setup encoder pins ---
  pinMode(ENC_A_A, INPUT_PULLUP);
  pinMode(ENC_A_B, INPUT_PULLUP);
  pinMode(ENC_B_A, INPUT_PULLUP);
  pinMode(ENC_B_B, INPUT_PULLUP);

  // Read initial encoder states
  aLastState = digitalRead(ENC_A_A);
  bLastState = digitalRead(ENC_B_A);

  // Attach interrupts for encoder channel A on both motors
  attachInterrupt(digitalPinToInterrupt(ENC_A_A), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_A), updateEncoderB, CHANGE);

  // --- Setup motor driver pins ---
  pinMode(mA_pinIn1, OUTPUT);
  pinMode(mA_pinIn2, OUTPUT);
  pinMode(mA_pinEnA, OUTPUT);
  pinMode(mB_pinIn3, OUTPUT);
  pinMode(mB_pinIn4, OUTPUT);
  pinMode(mB_pinEnB, OUTPUT);
}


// -------------------- MAIN LOOP --------------------
void loop() {

  // ======================================================
  // 1 RECEIVE SERIAL INPUT: "angleA,angleB"
  // ======================================================
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // read until newline
    data.trim(); // remove any spaces or stray chars
    int commaIndex = data.indexOf(',');

    if (commaIndex > 0) {
      // Split string into two angle parts
      String angle1Str = data.substring(0, commaIndex);
      String angle2str = data.substring(commaIndex + 1);

      // Optional: add a small random scaling factor
      // (This seems like experimental jitter – can be removed safely)
      float r = (random(0, 100) / 1000.0) + 1; // 1.000–1.099
      angle1Value = angle1Str.toFloat() * r;
      angle2Value = angle2str.toFloat() * r;

      // Keep a readable record of received data
      data_new = String(angle1Value) + "," + String(angle2Value);

      // Update motor targets
      targetA = angle1Value;
      targetB = angle2Value;

      // Debug print
      Serial.print("Received angles: ");
      Serial.println(data);
      Serial.print("Applied with variation: ");
      Serial.println(data_new);

      // Blink LED for confirmation
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
    }
  }

  // ======================================================
  // 2️ COMPUTE ENCODER-BASED POSITIONS
  // ======================================================
  // Convert encoder counts into motor output shaft degrees.
  // Formula: (count / (PPR * GearRatio)) * 360°
  positionA = -((float)counterA / (PPR * GR)) * 360.0;
  positionB = ((float)counterB / (PPR * GR)) * 360.0;


  // ======================================================
  // 3️ UPDATE "VIRTUAL" TARGETS GRADUALLY
  // ======================================================
  // Instead of jumping directly to targetA/B,
  // we incrementally step toward it to reduce jerk.
  if (abs(virtualTargetA - targetA) > 0.1)
    virtualTargetA += (virtualTargetA < targetA ? step : -step);

  if (abs(virtualTargetB - targetB) > 0.1)
    virtualTargetB += (virtualTargetB < targetB ? step : -step);


  // ======================================================
  // 4️ PID CONTROL: MOTOR A
  // ======================================================
  float errorA = virtualTargetA - positionA; // Difference between desired and actual
  integralA += errorA;                       // Integrate error over time
  float derivativeA = errorA - prevErrorA;   // Rate of error change

  // PID output (control signal)
  float outputA = Kp * errorA + Ki * integralA + Kd * derivativeA;
  outputA = constrain(outputA, -255, 255);   // Limit PWM

  // Stop motor when close enough
  if (abs(errorA) < threshold) outputA = 0;

  // Gradual PWM ramping for smoothness
  pwmA_current += constrain(outputA - pwmA_current, -pwmRampStep, pwmRampStep);
  int pwmA = abs(pwmA_current);

  // Set direction and apply PWM
  if (pwmA > 0) {
    if (pwmA_current > 0) { digitalWrite(mA_pinIn1, HIGH); digitalWrite(mA_pinIn2, LOW); }
    else { digitalWrite(mA_pinIn1, LOW); digitalWrite(mA_pinIn2, HIGH); }
  } else { digitalWrite(mA_pinIn1, LOW); digitalWrite(mA_pinIn2, LOW); }

  analogWrite(mA_pinEnA, pwmA);  // Apply PWM to motor
  prevErrorA = errorA;


  // ======================================================
  // 5️ PID CONTROL: MOTOR B
  // ======================================================
  float errorB = virtualTargetB - positionB;
  integralB += errorB;
  float derivativeB = errorB - prevErrorB;

  float outputB = Kp * errorB + Ki * integralB + Kd * derivativeB;
  outputB = constrain(outputB, -255, 255);
  if (abs(errorB) < threshold) outputB = 0;

  pwmB_current += constrain(outputB - pwmB_current, -pwmRampStep, pwmRampStep);
  int pwmB = abs(pwmB_current);

  if (pwmB > 0) {
    if (pwmB_current > 0) { digitalWrite(mB_pinIn3, HIGH); digitalWrite(mB_pinIn4, LOW); }
    else { digitalWrite(mB_pinIn3, LOW); digitalWrite(mB_pinIn4, HIGH); }
  } else { digitalWrite(mB_pinIn3, LOW); digitalWrite(mB_pinIn4, LOW); }

  analogWrite(mB_pinEnB, pwmB);
  prevErrorB = errorB;


  // ======================================================
  // 6️ SERIAL DEBUG OUTPUT
  // ======================================================
  Serial.print("A: "); Serial.print(positionA, 2);
  Serial.print("° | PWM: "); Serial.print(pwmA);
  Serial.print(" | Target: "); Serial.print(virtualTargetA, 2);
  Serial.print(" | Error: "); Serial.print(errorA, 2);

  Serial.print(" || B: "); Serial.print(positionB, 2);
  Serial.print("° | PWM: "); Serial.print(pwmB);
  Serial.print(" | Target: "); Serial.print(virtualTargetB, 2);
  Serial.print(" | Error: "); Serial.println(errorB, 2);

  delay(50); // small delay for control loop stability
}


// ======================================================
// 7️ ENCODER INTERRUPT HANDLERS
// ======================================================
// These functions run automatically whenever the encoder A-channel
// detects a change, allowing us to count ticks in real-time.

void updateEncoderA() {
  int aState = digitalRead(ENC_A_A);
  int bState = digitalRead(ENC_A_B);
  if (aState != aLastState) {
    if (aState == bState) counterA++;  // CW rotation
    else counterA--;                   // CCW rotation
  }
  aLastState = aState;
}

void updateEncoderB() {
  int aState = digitalRead(ENC_B_A);
  int bState = digitalRead(ENC_B_B);
  if (aState != bLastState) {
    if (aState == bState) counterB++;  // CW rotation
    else counterB--;                   // CCW rotation
  }
  bLastState = aState;
}
