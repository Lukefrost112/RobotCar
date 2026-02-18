#include <Arduino.h>
#include <ps5Controller.h>

#define M1A 5
#define M1B 18
#define M2A 19
#define M2B 21

#define BUZ 33
#define RED 25
#define GREEN 26
#define BLUE 27

int PWM_FREQ = 10000;
int PWM_RESOLUTION = 8;
int CH_M1A = 0;
int CH_M1B = 1;
int CH_M2A = 2;
int CH_M2B = 3;

unsigned long lastPrintedMs = 0;
bool lastConnectedState = false;

int leftStickDeadzone = 10;
int triggerDeadzone = 6;


struct feedbackState { 
  bool lastConnected = false;

  bool buzzerActive = false;
  bool buzzerOn = false;
  unsigned long buzzerLastMs = 0;
  int buzzerStep = 0;
  int buzzerLen = 0;
  int buzzerPattern[4];

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

enum StopMode {
  COAST,
  BRAKE
};

struct ControllerInput {
  bool connected;
  int lx;
  int ly;
  unsigned int r2;
  unsigned int l2;
};

ControllerInput controller;

struct DriveCommand {
  int throttle;
  int steering;
  int left;
  int right;
};

DriveCommand driveCommand;

struct MotorOut {
  int pwmA;
  int pwmB;
};

MotorOut leftMotor;
MotorOut rightMotor;

void startBeepPattern(int beepCount);
void updateBuzzer(unsigned long nowMs);

void setRGB(int r, int g, int b) {
  digitalWrite(RED, r ? HIGH : LOW);
  digitalWrite(GREEN, g ? HIGH : LOW);
  digitalWrite(BLUE, b ? HIGH : LOW);
}


void startRGBBlink (int r, int g, int b, int intervalMs, int blinkCount) { 
  feedback.r = r; feedback.g = g; feedback.b = b;
  feedback.rgbIntervalMs = intervalMs;

  feedback.rgbBlinksLeft = blinkCount;
  feedback.rgbActive = (blinkCount > 0);

  feedback.rgbOn = false;
  feedback.rgbLastMs = millis();

  setRGB(0, 0, 0);
}

void updateRGB(unsigned long nowMs) {
  if (!feedback.rgbActive) return;

  if (nowMs - feedback.rgbLastMs >= (unsigned long) feedback.rgbIntervalMs) {
    feedback.rgbLastMs = nowMs;

    feedback.rgbOn = !feedback.rgbOn;
    
    if(feedback.rgbOn) {
      setRGB(feedback.r, feedback.g, feedback.b);
    } else {
      setRGB(0, 0, 0);
    }

    feedback.rgbBlinksLeft--;
    if (feedback.rgbBlinksLeft <= 0) {
      feedback.rgbActive = false;
    }
  }
}

void handleFeedback(bool connected) {
  unsigned long now = millis();

  if (connected != feedback.lastConnected) {
    feedback.lastConnected = connected;
    if (connected) {
      startBeepPattern(1);
      startRGBBlink(0, 1, 0, 1, 120);
    } else {
      startBeepPattern(2);
      startRGBBlink(1, 0, 0, 2, 120);
    }
  }

  updateBuzzer(now);
  updateRGB(now);
}

void startBeepPattern(int beepCount) {
  if (beepCount < 1) return;
  if (beepCount > 2) beepCount = 2;

  feedback.buzzerStep = 0;
  feedback.buzzerLen = beepCount * 2;

  for (int i = 0; i < beepCount; i++) {
    feedback.buzzerPattern[i * 2] = 120;
    feedback.buzzerPattern[i * 2 + 1] = 120;
  }

  feedback.buzzerActive = true;
  feedback.buzzerOn = true;
  feedback.buzzerLastMs = millis();
  digitalWrite(BUZ, HIGH);
}

void updateBuzzer(unsigned long nowMs) {
  if (!feedback.buzzerActive) return;

  // duration for the current step
  int dur = feedback.buzzerPattern[feedback.buzzerStep];

  // check if the current step is done

  if (nowMs - feedback.buzzerLastMs >= (unsigned long) dur) {
    feedback.buzzerStep++;
    feedback.buzzerLastMs = nowMs;
  }

  // checks if the pattern is done
  if (feedback.buzzerStep >= feedback.buzzerLen) {
    feedback.buzzerActive = false;
    digitalWrite(BUZ, LOW);
    return;
  }

  // toggle the buzzer on and off
  feedback.buzzerOn = !feedback.buzzerOn;
  digitalWrite(BUZ, feedback.buzzerOn ? HIGH : LOW);

}

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

int applyDeadzone(int value, int deadzone) {
  if (abs(value) < deadzone) {
    return 0;
  }
  return value;
}

void computeDriveCommand(const ControllerInput c, DriveCommand &d) {
  
  // Applying trigger deadzone
  int r2 = (c.r2 < triggerDeadzone) ? 0 : c.r2;
  int l2 = (c.l2 < triggerDeadzone) ? 0 : c.l2;

  int throttle = r2 - l2;
  int steering = c.lx;

  d.throttle = applyDeadzone(throttle, triggerDeadzone);
  d.steering = applyDeadzone(steering, leftStickDeadzone);

  // Computing left and right motor commands
  d.left = d.throttle + d.steering;
  d.right = d.throttle - d.steering;

  // Clamping the motor commands to the range [-255, 255]
  if (d.left > 255) d.left = 255;
  if (d.left < -255) d.left = -255;

  if (d.right > 255) d.right = 255;
  if (d.right < -255) d.right = -255;
}

void printDriveCommand(const DriveCommand d, const ControllerInput c) {

  unsigned long now = millis();

  if (c.connected != lastConnectedState) {
    lastConnectedState = c.connected;

    Serial.print("Status: ");
    Serial.println(c.connected ? "Connected" : "Disconnected");

    lastPrintedMs = now;
    return;
  }

  if (!c.connected) return;

  if (now - lastPrintedMs >= 100) {

    lastPrintedMs = now;

    Serial.print("Throttle: ");
    Serial.print(d.throttle);
    Serial.print(" Steering: ");
    Serial.print(d.steering);
    Serial.print(" Left Motor: ");
    Serial.print(d.left);
    Serial.print(" Right Motor: ");
    Serial.println(d.right);
    
  }

}

void printMotorOut(const MotorOut &m, const char* name, const ControllerInput &c) {
  
  unsigned long now = millis();

  if (c.connected != lastConnectedState) {
    lastConnectedState = c.connected;

    Serial.print("Status: ");
    Serial.println(c.connected ? "Connected" : "Disconnected");

    lastPrintedMs = now;
    return;
  }

  if (!c.connected) return;

  if (now - lastPrintedMs >= 100) {

    lastPrintedMs = now;

    Serial.print(name);
    Serial.print(" - PWM A: ");
    Serial.print(m.pwmA);
    Serial.print(" PWM B: ");
    Serial.println(m.pwmB);
    
  }

}

void readController(ControllerInput &c) {
  c.connected = ps5.isConnected();
  if (c.connected) {
    c.lx = ps5.LStickX();
  
  c.ly = ps5.LStickY();
  
  c.l2 = ps5.L2Value();
  
  c.r2 = ps5.R2Value();
  } else {
    c.lx = 0;
    c.ly = 0;
    c.l2 = 0;
    c.r2 = 0;
  }
  
}

void printController(const ControllerInput c) {
  unsigned long now = millis();

  if (c.connected != lastConnectedState) {
    lastConnectedState = c.connected;

    Serial.print("Status: ");
    Serial.println(c.connected ? "Connected" : "Disconnected");

    lastPrintedMs = now;
    return;
  }

  if (!c.connected) return;

  if (now - lastPrintedMs >= 100) {

    lastPrintedMs = now;

    Serial.print("R2: ");
    Serial.print(c.r2);
    Serial.print(" ");
    Serial.print("L2: ");
    Serial.print(c.l2);
    Serial.print(" ");
    Serial.print("Left Stick X: ");
    Serial.println(c.lx);
    
  }

}



void writeMotorOutputs(const MotorOut left, const MotorOut right) {
  ledcWrite(CH_M1A, left.pwmA);
  ledcWrite(CH_M1B, left.pwmB);
  ledcWrite(CH_M2A, right.pwmA);
  ledcWrite(CH_M2B, right.pwmB);
}

void setup() {
  // put your setup code here, to run once:


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

void loop() {
  readController(controller);
  handleFeedback(controller.connected);
  //printController(controller);
  computeDriveCommand(controller, driveCommand);
  //printDriveCommand(driveCommand, controller);

  leftMotor = mapMotor(driveCommand.left, COAST);
  rightMotor = mapMotor(driveCommand.right, COAST);

  printMotorOut(leftMotor, "Left Motor", controller);
  printMotorOut(rightMotor, "Right Motor", controller);

  // Set motor outputs
  writeMotorOutputs(leftMotor, rightMotor);

  if (ps5.Circle()) {
    digitalWrite(BUZ, HIGH);
  } else {
    digitalWrite(BUZ, LOW);
  }

}

