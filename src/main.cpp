/*
  PS5 Controlled Smart Car

  Features:
  - Differential drive motor control (throttle + steering mixing)
  - Deadzone filtering for triggers and joystick
  - Non-blocking RGB + buzzer feedback system
  - Honk feature
  - Connection state detection

  Architecture:
  loop():
    1. Read controller input
    2. Update feedback system (non-blocking)
    3. Compute drive command
    4. Map motor outputs
    5. Write PWM to motors
    6. Handle honk override

  Design Philosophy:
  - No delay() anywhere
  - All time-based behavior uses millis()
  - Motors, Bluetooth, and feedback run concurrently
*/

#include <Arduino.h>
#include <ps5Controller.h>

// ======================== PIN DEFINITIONS ========================
// Motor driver pins (H-Bridge inputs)
#define M1A 5
#define M1B 18
#define M2A 19
#define M2B 21

// Feedback pins
#define BUZ 33
#define RED 25
#define GREEN 26
#define BLUE 27

// ======================== PWM CONFIGURATION ========================
// Using ESP32 LEDC hardware PWM for motor speed control
int PWM_FREQ = 10000;       // 10kHz PWM
int PWM_RESOLUTION = 8;     // 8-bit resolution (0–255)

int CH_M1A = 0;
int CH_M1B = 1;
int CH_M2A = 2;
int CH_M2B = 3;

// Used for throttled serial printing
unsigned long lastPrintedMs = 0;
bool lastConnectedState = false;

// Deadzone thresholds to ignore small joystick noise
int leftStickDeadzone = 10;
int triggerDeadzone = 6;

int boostIntervalMs = 50; // interval of boost buzzer sound
int reverseIntervalMs = 400; // interval of reverse buzzer sound

// ======================== FEEDBACK STATE MACHINE ========================
/*
  Stores all timing and state variables for:
  - Non-blocking buzzer pattern playback
  - Non-blocking RGB blink playback

  This struct acts as a small state machine.
*/
struct feedbackState { 
  bool lastConnected = false;

  // ---- Buzzer state ----
  bool buzzerActive = false;
  bool buzzerOn = false;
  unsigned long buzzerLastMs = 0;
  int buzzerStep = 0;
  int buzzerLen = 0;
  int buzzerPattern[4];   // Alternating ON/OFF durations (max 2 beeps)

  // ---- RGB state ----
  bool rgbActive = false;
  bool rgbOn = false;
  unsigned long rgbLastMs = 0;
  unsigned long rgbIntervalMs = 0;
  int rgbBlinksLeft = 0;
  int r = 0;
  int g = 0;
  int b = 0;
};

feedbackState feedback;

// ======================== CONTROLLER & DRIVE STRUCTURES ========================

enum BuzzerState {
  PATTERN,
  BOOST,
  REVERSE,
  HORN,
  STANDBY
};

BuzzerState buzzerState;
BuzzerState lastBuzzerState;
unsigned long buzzerModeStartMs;

enum StopMode {
  COAST,   // Motor free-spins when stopped
  BRAKE    // Motor actively brakes when stopped
};

/*
  Raw input from controller.
*/
struct ControllerInput {
  bool connected;
  int lx;              // Left stick X (steering)
  int ly;
  unsigned int r2;     // Right trigger (forward throttle)
  unsigned int l2;     // Left trigger (reverse throttle)
  bool honk;
  bool boost;
};

ControllerInput controller;

/*
  Processed drive intent.
*/
struct DriveCommand {
  int throttle;
  int steering;
  int left;
  int right;
};

DriveCommand driveCommand;

/*
  Final PWM output for one motor.
*/
struct MotorOut {
  int pwmA;
  int pwmB;
};

MotorOut leftMotor;
MotorOut rightMotor;

// ======================== RGB CONTROL ========================
// Sets RGB LED pins HIGH or LOW
void setRGB(int r, int g, int b) {
  digitalWrite(RED, r ? HIGH : LOW);
  digitalWrite(GREEN, g ? HIGH : LOW);
  digitalWrite(BLUE, b ? HIGH : LOW);
}

/*
  Prepares an RGB blink pattern (non-blocking).
  Does NOT block — only sets initial state.
*/
void startRGBBlink (int r, int g, int b, int intervalMs, int blinkCount) { 
  feedback.r = r; feedback.g = g; feedback.b = b;
  feedback.rgbIntervalMs = intervalMs;

  feedback.rgbBlinksLeft = blinkCount;
  feedback.rgbActive = (blinkCount > 0);

  feedback.rgbOn = false;
  feedback.rgbLastMs = millis();

  setRGB(0, 0, 0);
}

/*
  Advances RGB blink pattern using millis().
*/
void updateRGB(unsigned long nowMs) {
  if (!feedback.rgbActive) return;

  if (nowMs - feedback.rgbLastMs >= feedback.rgbIntervalMs) {
    feedback.rgbLastMs = nowMs;

    feedback.rgbOn = !feedback.rgbOn;

    if (feedback.rgbOn) {
      // Turn ON
      setRGB(feedback.r, feedback.g, feedback.b);
    } else {
      // Turn OFF
      setRGB(0, 0, 0);

      feedback.rgbBlinksLeft--;
      if (feedback.rgbBlinksLeft <= 0) {
        feedback.rgbActive = false;
      }
    }
  }
}


// ======================== BUZZER STATE MACHINE ========================
/*
  buzzerPattern[] stores alternating ON and OFF durations in ms.
  buzzerStep tracks which duration is currently active.
  This forms a simple state machine driven by millis().
*/

void startBeepPattern(int beepCount) {
  if (beepCount < 1) return;
  if (beepCount > 2) beepCount = 2;

  feedback.buzzerStep = 0;
  feedback.buzzerLen = beepCount * 2;

  // Build timing pattern: ON, OFF, ON, OFF...
  for (int i = 0; i < beepCount; i++) {
    feedback.buzzerPattern[i * 2] = 120;
    feedback.buzzerPattern[i * 2 + 1] = 120;
  }

  feedback.buzzerActive = true;
  feedback.buzzerOn = true;
  feedback.buzzerLastMs = millis();
  digitalWrite(BUZ, HIGH);
}

/*
  Advances buzzer pattern using millis().
  No blocking delays.
*/
void updateBuzzer(unsigned long nowMs) {
  if (!feedback.buzzerActive) return;

  // If we've reached the end of the pattern, stop and force OFF
  if (feedback.buzzerStep >= feedback.buzzerLen) {
    feedback.buzzerActive = false;
    digitalWrite(BUZ, LOW);
    return;
  }

  int dur = feedback.buzzerPattern[feedback.buzzerStep];

  // Only change state when the duration has passed
  if (nowMs - feedback.buzzerLastMs >= (unsigned long)dur) {
    feedback.buzzerLastMs = nowMs;

    // Move to next step in the pattern
    feedback.buzzerStep++;

    // If that was the last step, stop and force OFF
    if (feedback.buzzerStep >= feedback.buzzerLen) {
      feedback.buzzerActive = false;
      digitalWrite(BUZ, LOW);
      return;
    }

    // Toggle ON/OFF for the next step
    feedback.buzzerOn = !feedback.buzzerOn;
    digitalWrite(BUZ, feedback.buzzerOn ? HIGH : LOW);
  }
}

/*
  Handles connection state changes and triggers:
  - 1 beep + green blink on connect
  - 2 beeps + red blink on disconnect
*/
void handleFeedback(bool connected) {
  unsigned long now = millis();

  if (connected != feedback.lastConnected) {
    feedback.lastConnected = connected;
    if (connected) {
      startBeepPattern(1);
      startRGBBlink(0, 1, 0, 120, 1);
    } else {
      startBeepPattern(2);
      startRGBBlink(1, 0, 0, 120, 2);
    }
  }
  updateRGB(now);
}

void decideBuzzerState(feedbackState &fb, BuzzerState &buzstate, const ControllerInput c) {
  if (fb.buzzerActive && fb.buzzerStep < fb.buzzerLen) {
    buzstate = PATTERN;
  } else if (c.honk) {
    buzstate = HORN;
  } else if (c.boost) {
    buzstate = BOOST;
  } else if ((int)c.r2 - (int)c.l2 < -20) { // Reverse threshold
    buzstate = REVERSE;
  } else {
    buzstate = STANDBY;
  }
}

void updateBuzzerSystem(BuzzerState buzstate) {
  unsigned long now = millis();
  switch (buzstate) {
    case PATTERN:
      updateBuzzer(now);
      break;
    case HORN:
      digitalWrite(BUZ, HIGH);
      break;
    case BOOST:
      if (now - buzzerModeStartMs >= boostIntervalMs) {
        digitalWrite(BUZ, !digitalRead(BUZ)); // Toggle buzzer state
        buzzerModeStartMs = now;
      } 
      break;
    case REVERSE:
      if (now - buzzerModeStartMs >= reverseIntervalMs) {
        digitalWrite(BUZ, !digitalRead(BUZ)); // Toggle buzzer state
        buzzerModeStartMs = now;
      } 
      break;
    case STANDBY:
      digitalWrite(BUZ, LOW);
      break;
  }
}

// ======================== MOTOR CONTROL ========================

MotorOut mapMotor(int cmd, enum StopMode stop) {
  MotorOut m;

  int magnitude = abs(cmd);
  if (magnitude > 255) magnitude = 255;

  if (cmd > 0) {
    m.pwmA = magnitude;
    m.pwmB = 0;
  } else if (cmd < 0) {
    m.pwmA = 0;
    m.pwmB = magnitude;
  } else if (cmd == 0 && stop == COAST) {
    m.pwmA = 0;
    m.pwmB = 0;
  } else {
    m.pwmA = 255;
    m.pwmB = 255;
  }
  return m;
}

// Eliminates small joystick noise near zero
int applyDeadzone(int value, int deadzone) {
  if (abs(value) < deadzone) {
    return 0;
  }
  return value;
}

/*
  Differential drive mixing:
    left  = throttle + steering
    right = throttle - steering
*/
void computeDriveCommand(const ControllerInput c, DriveCommand &d) {
  int r2 = c.r2;
  int l2 = c.l2;

  int throttle = r2 - l2;
  int steering = c.lx;

  d.throttle = applyDeadzone(throttle, triggerDeadzone);
  d.steering = applyDeadzone(steering, leftStickDeadzone);

  d.left = d.throttle + d.steering;
  d.right = d.throttle - d.steering;

  if (d.left > 255) d.left = 255;
  if (d.left < -255) d.left = -255;

  if (d.right > 255) d.right = 255;
  if (d.right < -255) d.right = -255;
}

// Reads controller safely
void readController(ControllerInput &c) {
  c.connected = ps5.isConnected();
  if (c.connected) {
    c.lx = ps5.LStickX();
    c.ly = ps5.LStickY();
    c.l2 = ps5.L2Value();
    c.r2 = ps5.R2Value();
    c.honk = ps5.R1();
    c.boost = ps5.Circle();
  } else {
    c.lx = 0;
    c.ly = 0;
    c.l2 = 0;
    c.r2 = 0;
    c.honk = false;
    c.boost = false;
  }
}

// Sends PWM values to ESP32 LEDC channels
void writeMotorOutputs(const MotorOut left, const MotorOut right) {
  ledcWrite(CH_M1A, left.pwmA);
  ledcWrite(CH_M1B, left.pwmB);
  ledcWrite(CH_M2A, right.pwmA);
  ledcWrite(CH_M2B, right.pwmB);
}

// ======================== SETUP ========================

void setup() {

  Serial.begin(115200);
  Serial.println(esp_reset_reason());
  ps5.begin("E8:47:3A:BC:DD:DF");
  Serial.println("Ready");

  pinMode(BUZ, OUTPUT);
  digitalWrite(BUZ, LOW);

  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  setRGB(0, 0, 0);

  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);

  ledcSetup(CH_M1A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_M1B, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_M2A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_M2B, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(M1A, CH_M1A);
  ledcAttachPin(M1B, CH_M1B);
  ledcAttachPin(M2A, CH_M2A);
  ledcAttachPin(M2B, CH_M2B);
}

// ======================== MAIN LOOP ========================
// Non-blocking control loop

void loop() {

  readController(controller);

  decideBuzzerState(feedback, buzzerState, controller);

  if (buzzerState != lastBuzzerState) {
    lastBuzzerState = buzzerState;
    buzzerModeStartMs = millis();
    digitalWrite(BUZ, LOW); // Ensure buzzer is off when changing state
  }

  updateBuzzerSystem(buzzerState);

  handleFeedback(controller.connected);

  computeDriveCommand(controller, driveCommand);

  leftMotor = mapMotor(driveCommand.left, COAST);
  rightMotor = mapMotor(driveCommand.right, COAST);

  writeMotorOutputs(leftMotor, rightMotor);
  
}
