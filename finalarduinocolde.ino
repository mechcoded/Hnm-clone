/ --- Global Variables ---
volatile int encoderPos = 0;
int lastEncoded = 0;
double xh = 0;                   // Displacement in meters
double rs = 0.077;               // Sector pulley radius [m]
double rh = 0.07;               // Handle radius [m]
double rp = 0.01;                // Motor pulley radius [m]
double force = 0;

int pwmPin = 9;                  // Motor PWM pin
int dirPin = 8;                  // Motor direction pin
int encoderPinA = 2;             // Rotary encoder A
int encoderPinB = 3;             // Rotary encoder B

// --- Constants for feedback ---
double x_wall = 0.02;            // Wall position in meters
double k_wall = 1200;            // Wall stiffness [N/m]

void setup() {
  Serial.begin(9600);

  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
}

void loop() {
  // --- Position Calculation ---
  xh = encoderPos * 2.0 * PI * rs / 1024.0;  // Adjust based on your encoder resolution

  // --- Force Feedback Calculation ---
  if (xh > x_wall) {
    force = -k_wall * (xh - x_wall);  // Strong virtual wall force
  } else {
    force = 0;
  }

  // --- Torque and PWM Control ---
  double Tp = rp / rs * rh * force;  // Torque at motor pulley
  double duty = sqrt(min(abs(Tp) / 0.015, 1.0));  // Normalize torque into duty

  if (duty > 1) duty = 1;
  if (duty < 0) duty = 0;

  int output = (int)(duty * 255);

  if (force < 0) {
    digitalWrite(dirPin, HIGH);  // CW
  } else {
    digitalWrite(dirPin, LOW);   // CCW
  }

  analogWrite(pwmPin, output);  // Apply PWM

  // --- Debug (optional) ---
  Serial.print("xh (m): ");
  Serial.print(xh, 4);
  Serial.print(" | Force: ");
  Serial.print(force, 2);
  Serial.print(" | PWM: ");
  Serial.println(output);

  delay(5);  // Stability delay
}

void updateEncoder() {
  int MSB = digitalRead(encoderPinA); // Most significant bit
  int LSB = digitalRead(encoderPinB); // Least significant bit

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPos++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPos--;

  lastEncoded = encoded;
}

