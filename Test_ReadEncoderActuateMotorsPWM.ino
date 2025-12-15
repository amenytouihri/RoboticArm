/*
   Purpose: 
     - Control two motors using an H-Bridge with PWM (speed control)
     - Read a rotary encoder using interrupts
     - Calculate the motor's angle in degrees
     - Demonstrate basic concepts in robotics and motor control
*/

// -------------------- ENCODER SETUP --------------------

// Define the digital pins where the rotary encoder is connected
// Rotary encoders give two signals, A and B, that allow us to detect movement and direction
#define outputA 2  // Channel A pin
#define outputB 4  // Channel B pin

// Encoder and motor characteristics
const float PPR = 360.0;  // Pulses per revolution of the encoder
const float GR  = 30.0;   // Gear ratio (motor shaft rotates 30 times for one output revolution)

// Variables used to track encoder position
volatile int counter = 0;          // Stores current encoder count (must be 'volatile' because it changes in the interrupt)
int aLastState;                     // Stores previous state of encoder channel A
float positionInDegrees_m2 = 0.0;  // Stores calculated motor angle in degrees

// -------------------- MOTOR SETUP --------------------

// Motor A pins
const int mA_pinIn1 = 7;       // Direction control pin 1
const int mA_pinIn2 = 8;       // Direction control pin 2
const int mA_pinEnA = 5;       // Enable pin (controls speed using PWM)

// Motor B pins
const int mB_pinIn3 = 9;       // Direction control pin 3
const int mB_pinIn4 = 10;      // Direction control pin 4
const int mB_pinEnB = 11;      // Enable pin (PWM speed control)

void setup() {
  // -------------------- ENCODER INITIALIZATION --------------------
  // Set the encoder pins as inputs with pull-up resistors
  // INPUT_PULLUP ensures the pin reads HIGH when not connected
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);

  // Read initial state of encoder channel A
  aLastState = digitalRead(outputA);

  // Attach an interrupt to channel A
  // This means whenever the voltage on channel A changes (HIGH to LOW or LOW to HIGH),
  // the function updateEncoder() will automatically run
  attachInterrupt(digitalPinToInterrupt(outputA), updateEncoder, CHANGE);

  // Start serial communication at 9600 baud
  // This allows us to print messages to the Serial Monitor on the computer
  Serial.begin(9600); //number of signal changes (symbols) per second

  // -------------------- MOTOR INITIALIZATION --------------------
  // Set motor pins as outputs
  // OUTPUT mode means we can send signals to control motors
  pinMode(mA_pinIn1, OUTPUT);
  pinMode(mA_pinIn2, OUTPUT);
  pinMode(mA_pinEnA, OUTPUT);

  pinMode(mB_pinIn3, OUTPUT);
  pinMode(mB_pinIn4, OUTPUT);
  pinMode(mB_pinEnB, OUTPUT); 
}

void loop() {
  // -------------------- ENCODER READING --------------------
  // Convert encoder counts to motor angle in degrees
  // Formula explanation:
  // - counter / (PPR * GR) gives fraction of one motor rotation
  // - multiply by 360 to get degrees
  positionInDegrees = ((float)counter / (PPR * GR)) * 360.0;

  // Print motor angle to Serial Monitor
  Serial.print("Position in Degrees: ");
  Serial.println(positionInDegrees);

  // If the motor angle exceeds ±360°, reset the counter
  // This prevents numbers from becoming too large and keeps the angle readable
  if (positionInDegrees > 360.0 || positionInDegrees < -360.0) {
    noInterrupts();   // Temporarily stop interrupts so we can safely reset counter
    counter = 0;      // Reset encoder count
    interrupts();     // Turn interrupts back on
    positionInDegrees = 0.0; // Reset calculated angle
    Serial.println("Position reset to 0 degrees.");
  }

  // -------------------- MOTOR CONTROL --------------------
  // MOTOR A - clockwise rotation at 75% speed
  analogWrite(mA_pinEnA, 190);  // analogWrite controls motor speed (0-255)
  digitalWrite(mA_pinIn1, HIGH); // Set direction: In1 HIGH
  digitalWrite(mA_pinIn2, LOW);  // Set direction: In2 LOW
  delay(2000);                    // Run for 2 seconds

  // MOTOR A - counterclockwise rotation at 50% speed
  analogWrite(mA_pinEnA, 127);   // ~50% speed
  digitalWrite(mA_pinIn1, LOW);  // Reverse direction
  digitalWrite(mA_pinIn2, HIGH);
  delay(2000);

  // MOTOR B - clockwise rotation at 75% speed
  analogWrite(mB_pinEnB, 190);  
  digitalWrite(mB_pinIn3, HIGH); 
  digitalWrite(mB_pinIn4, LOW);  
  delay(2000);

  // MOTOR B - counterclockwise rotation at 50% speed
  analogWrite(mB_pinEnB, 127);  
  digitalWrite(mB_pinIn3, LOW);  
  digitalWrite(mB_pinIn4, HIGH); 
  delay(2000);

  // STOP BOTH MOTORS
  digitalWrite(mA_pinEnA, LOW);  // Disable Motor A
  digitalWrite(mB_pinEnB, LOW);  // Disable Motor B
  delay(1000);                    // Wait 1 second before repeating loop
}

// -------------------- ENCODER INTERRUPT --------------------
// This function automatically runs whenever channel A changes state
void updateEncoder() {
  // Read current states of encoder channels
  int aState = digitalRead(outputA); // Current state of channel A
  int bState = digitalRead(outputB); // Current state of channel B

  // Check if channel A state has changed since last reading
  if (aState != aLastState) {
    // Determine rotation direction:
    // - If channel A matches channel B, the motor is rotating clockwise
    // - Otherwise, it is rotating counterclockwise
    if (aState == bState) {
      counter++; // Clockwise rotation: increase count
    } else {
      counter--; // Counterclockwise rotation: decrease count
    }
  }

  // Save current state for next interrupt
  aLastState = aState;
}
