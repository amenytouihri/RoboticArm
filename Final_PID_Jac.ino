/*
  ==============================================================
  Dual-Motor Robotic Arm Controller
  Version: Virtual Target Enabled
  ==============================================================
  - Controls TWO DC motors (Motor A & Motor B)
  - Each motor has an encoder for position feedback
  - Uses PID control to reach desired angles
  - Uses a "virtual target" to avoid sudden jumps
  - Receives commands from a PC (e.g. Python GUI) via Serial:
        "angleA,angleB"
  - Designed for linked / parallel robotic arms
*/

///////////////////////////////////////////////////////////////
// ENCODER SETTINGS
///////////////////////////////////////////////////////////////

// Encoder pin definitions
// Each motor has a quadrature encoder (A + B channels)
#define ENC_A_A 2   // Encoder A, channel A (interrupt pin)
#define ENC_A_B 4   // Encoder A, channel B
#define ENC_B_A 3   // Encoder B, channel A (interrupt pin)
#define ENC_B_B 6   // Encoder B, channel B

// Encoder resolution
// CPR = Counts Per Revolution (measured experimentally)
const float CPR = 187.0;

// Conversion factor: encoder ticks → degrees
// one encoder tick corresponds to this many degrees of shaft rotation.
// CPR (Counts Per Revolution) = Number of encoder ticks generated for one full rotation
const float DEG_PER_TICK = 360.0 / CPR;

// Encoder counters (updated inside interrupts)
// volatile = tells compiler this can change at any time
volatile long counterA = 0;
volatile long counterB = 0;

// Encoder value at startup (used as "home" reference)
long homeCountA = 0;
long homeCountB = 0;

// Last encoder A-channel state (for direction detection)
int lastAState, lastBState;

// Current motor positions in degrees
// We ASSUME the physical arm is placed at 90° at startup
float positionA = 90.0;
float positionB = 90.0;

// Direction correction for each motor
// Use +1 or -1 depending on how the motor/encoder is mounted
const int DIR_SIGN_A = +1;
const int DIR_SIGN_B = -1;

// Zone where we enforce minimum PWM
// Prevents motors from stalling when error is still large
const float MOVE_ZONE = 3.0;

///////////////////////////////////////////////////////////////
// MOTOR SETTINGS
///////////////////////////////////////////////////////////////

// Motor A pins (H-bridge)
const int mA_in1 = 7;
const int mA_in2 = 8;
const int mA_pwm = 5;

// Motor B pins (H-bridge)
const int mB_in1 = 9;
const int mB_in2 = 10;
const int mB_pwm = 11;

///////////////////////////////////////////////////////////////
// PID SETTINGS
///////////////////////////////////////////////////////////////

// PID gains for Motor A
float KpA = 2.6455;
float KiA = 0.8;
float KdA = 0.1242;

// PID gains for Motor B
float KpB = 2.85;
float KiB = 0.6;
float KdB = 0.5;

// Small error range where motor is considered "close enough"
float deadzone = 0.5;

// PID memory terms
float prevErrorA = 0.0;
float integralA  = 0.0;
float prevErrorB = 0.0;
float integralB  = 0.0;

///////////////////////////////////////////////////////////////
// MOTION SMOOTHING (VIRTUAL TARGET SYSTEM)
///////////////////////////////////////////////////////////////

// Current PWM output values (for smooth ramping)
int pwmA_current = 0;
int pwmB_current = 0;

// PWM limits
const int MAX_PWM     = 100;
const int MIN_PWM     = 30;
const int pwmRampStep = 10;   // max PWM change per loop

// REAL target angles (from Serial commands)
float targetA  = 90.0;
float targetB  = 90.0;

// VIRTUAL target angles
// These slowly move toward the real target
float virtualA = 90.0;
float virtualB = 90.0;

// Control loop timing
const float LOOP_DT_SEC           = 0.05; // 50 ms loop
const float MAX_SPEED_DEG_PER_SEC = 10.0; // virtual motion speed

///////////////////////////////////////////////////////////////
// SYSTEM STATE
///////////////////////////////////////////////////////////////

// Prevents motors from moving until first Serial command arrives
bool firstCommandReceived = false;

// Onboard LED (optional debug)
const int ledPin = 13;

///////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////
void setup() {

  // Start serial communication
  Serial.begin(115200);
  Serial.setTimeout(10);

  pinMode(ledPin, OUTPUT);

  // Encoder inputs
  pinMode(ENC_A_A, INPUT_PULLUP);
  pinMode(ENC_A_B, INPUT_PULLUP);
  pinMode(ENC_B_A, INPUT_PULLUP);
  pinMode(ENC_B_B, INPUT_PULLUP);

  // Read initial encoder states
  lastAState = digitalRead(ENC_A_A);
  lastBState = digitalRead(ENC_B_A);

  // Attach interrupts to encoder A channels
  attachInterrupt(digitalPinToInterrupt(ENC_A_A), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_A), updateEncoderB, CHANGE);

  // Motor outputs
  pinMode(mA_in1, OUTPUT);
  pinMode(mA_in2, OUTPUT);
  pinMode(mA_pwm, OUTPUT);

  pinMode(mB_in1, OUTPUT);
  pinMode(mB_in2, OUTPUT);
  pinMode(mB_pwm, OUTPUT);

  delay(300); // let system settle

  // Capture encoder values as "home"
  homeCountA = counterA;
  homeCountB = counterB;

  // Initialize all angles at 90°
  positionA = positionB = 90.0;
  targetA   = targetB   = 90.0;
  virtualA  = virtualB  = 90.0;

  Serial.println("System ready.");
}

///////////////////////////////////////////////////////////////
// MAIN LOOP
///////////////////////////////////////////////////////////////
void loop() {

  ///////////////////////////////////////////////////////////////
  // 1) READ SERIAL COMMAND
  ///////////////////////////////////////////////////////////////
  // Expected format: "angleA,angleB"
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();
  /*  1) parses a serial command in the form angleA,angleB
      2)converts both values to numerical targets
      3)enables motion only after a valid command is received */ 
    int comma = data.indexOf(',');
    if (comma > 0) {
      targetA = data.substring(0, comma).toFloat();
      targetB = data.substring(comma + 1).toFloat();
      firstCommandReceived = true;
    }
  }

  // Safety: do nothing until first command arrives
  if (!firstCommandReceived) {
    analogWrite(mA_pwm, 0);
    analogWrite(mB_pwm, 0);
    return;
  }

  ///////////////////////////////////////////////////////////////
  //2) ENCODER → CURRENT POSITION
  ///////////////////////////////////////////////////////////////
  long rawA, rawB;
  // Local copies of encoder counters

  // Read encoder counts atomically
  noInterrupts();  //Temporarily pauses all interrupt service routines
  rawA = counterA; // Copies the current encoder counts
  rawB = counterB;
  interrupts(); //Restores normal interrupt operation
  // This was used bc "long" is multi-byte on Arduino ISRs could update it mid-read
  // Converts the raw encoder count of Motor into a real angular position in degrees
  //relative to a known starting position of 90°.
  positionA = 90.0 + DIR_SIGN_A * (rawA - homeCountA) * DEG_PER_TICK;
  positionB = 90.0 + DIR_SIGN_B * (rawB - homeCountB) * DEG_PER_TICK;

  ///////////////////////////////////////////////////////////////
  // 3) VIRTUAL TARGET RAMPING
  // Limit how fast the target angle can change per control loop
  // This prevents sudden jumps from IK and enforces a maximum angular speed
  ///////////////////////////////////////////////////////////////
  /*MAX_SPEED_DEG_PER_SEC = maximum allowed angular speed (degrees per second)
  LOOP_DT_SEC = duration of one control loop iteration (seconds)
  maxStep = maximum angle change allowed in one loop*/
  float maxStep = MAX_SPEED_DEG_PER_SEC * LOOP_DT_SEC;

  if (fabs(targetA - virtualA) > 0.01) {
    // Compute a bounded step toward the real target
    float step = constrain(targetA - virtualA, -maxStep, maxStep);
    // Move virtual target gradually
    virtualA += step;
  }

  if (fabs(targetB - virtualB) > 0.01) {
    float step = constrain(targetB - virtualB, -maxStep, maxStep);
    virtualB += step;
  }

  ///////////////////////////////////////////////////////////////
  // 4) PID CONTROL (USING VIRTUAL TARGETS)
  ///////////////////////////////////////////////////////////////
  /*runs the full PID control loop for Motor, 
  using the virtual target angle and the current measured position, 
  and directly drives the motor pins with smooth PWM output.  */
  controlMotor(
    virtualA, positionA,
    prevErrorA, integralA,
    pwmA_current,
    mA_in1, mA_in2, mA_pwm,
    KpA, KiA, KdA
  );

  controlMotor(
    virtualB, positionB,
    prevErrorB, integralB,
    pwmB_current,
    mB_in1, mB_in2, mB_pwm,
    KpB, KiB, KdB
  );

  ///////////////////////////////////////////////////////////////
  // 5) DEBUG OUTPUT
  ///////////////////////////////////////////////////////////////
  Serial.print("A: ");
  Serial.print(positionA, 2);
  Serial.print(" | PWM: ");
  Serial.print(pwmA_current);
  Serial.print(" | VTarget: ");
  Serial.print(virtualA, 2);
  Serial.print(" | Error: ");
  Serial.print(virtualA - positionA, 2);

  Serial.print(" || B: ");
  Serial.print(positionB, 2);
  Serial.print(" | PWM: ");
  Serial.print(pwmB_current);
  Serial.print(" | VTarget: ");
  Serial.print(virtualB, 2);
  Serial.print(" | Error: ");
  Serial.println(virtualB - positionB, 2);

  delay(50); // loop timing
}

///////////////////////////////////////////////////////////////
// PID MOTOR CONTROL FUNCTION
///////////////////////////////////////////////////////////////
/*
one complete closed-loop controller for a single motor:
  compares desired angle vs measured angle
  computes a PID control output
  smooths the PWM
  sets motor direction and speed via an H-bridge
*/ 
void controlMotor(
  float target, float position,
  float &prevError, float &integral,
  int &pwmCurrent,
  int in1, int in2, int pwmPin,
  float Kp, float Ki, float Kd
) {
  float error = target - position;
  float derivative = error - prevError;

  // Deadzone: stop motor if close enough
  if (fabs(error) < deadzone) {
    analogWrite(pwmPin, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    pwmCurrent = 0;
    prevError = error;
    return;
  }

  /* Integral term with anti-windup
  accumulates the control error over time to eliminate steady-state error, 
  while preventing the integral term from growing too large, 
  which could destabilize the system.
  */ 
  integral += error * LOOP_DT_SEC;
  integral = constrain(integral, -20.0, 20.0);

  // PID equation
  float output = Kp * error + Ki * integral + Kd * derivative;
  output = constrain(output, -MAX_PWM, MAX_PWM);

  /* Smooth PWM ramping
  This block prevents sudden jumps in motor power 
  by limiting how fast the PWM value is allowed to change from one control loop to the next.
  pwmCurrent += updates PWM slowly over multiple iterations
  providing smooth acceleration = protecting the mechanical system*/ 
  int desired = (int)output;
  pwmCurrent += constrain(desired - pwmCurrent, -pwmRampStep, pwmRampStep);

  /*ensures the motor receives enough power to actually move when it is far from the target
  fabs(error) >= MOVE_ZONE = the joint is still far from the target
  pwmValue < MIN_PWM = commanded power is too low to overcome friction
  => force PWM up to a minimum value that guarantees motion*/ 
  int pwmValue = abs(pwmCurrent);
  if (fabs(error) >= MOVE_ZONE && pwmValue < MIN_PWM) pwmValue = MIN_PWM;
  pwmValue = constrain(pwmValue, 0, MAX_PWM);

  // Set motor direction
  if (pwmCurrent > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  analogWrite(pwmPin, pwmValue);
  prevError = error;
}

///////////////////////////////////////////////////////////////
// ENCODER INTERRUPT SERVICE ROUTINES
///////////////////////////////////////////////////////////////
// Reads quadrature encoder channels to update position and direction
void updateEncoderA() {
  // Read encoder channels
  int a = digitalRead(ENC_A_A);
  int b = digitalRead(ENC_A_B);
  // Process only on state change of channel A
  if (a != lastAState) {
    // Determine direction using channel B
    counterA += (a == b) ? 1 : -1;
    lastAState = a;
  }
}

void updateEncoderB() {
  int a = digitalRead(ENC_B_A);
  int b = digitalRead(ENC_B_B);
  if (a != lastBState) {
    counterB += (a == b) ? 1 : -1;
    lastBState = a;
  }
}
