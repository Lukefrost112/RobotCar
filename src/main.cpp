#include <Arduino.h>
#include <ps5Controller.h>

int applyDeadzone(int value, int deadzone);

unsigned long lastPrintedMs = 0;
bool lastConnectedState = false;

int leftStickDeadzone = 10;
int triggerDeadzone = 6;

int r = 255;
int g = 0;
int b = 0;

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

int applyDeadzone(int value, int deadzone) {
  if (abs(value) < deadzone) {
    return 0;
  }
  return value;
}

void computeDriveCommand(const ControllerInput c, DriveCommand &d) {
  // Getting raw values from the controller

  int r2 = (c.r2 < triggerDeadzone) ? 0 : c.r2;
  int l2 = (c.l2 < triggerDeadzone) ? 0 : c.l2;

  int throttle = r2 - l2;
  int steering = c.lx;

  // Applying deadzones
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

void nextRainbowColor() {
  if (r > 0 && b == 0) {
    r--;
    g++;
  }
  if (g > 0 && r == 0) {
    g--;
    b++;
  }
  if (b > 0 && g == 0) {
    r++;
    b--;
  }
}

void setControllerLight(int r, int g, int b, ControllerInput c) {
  unsigned long now = millis();
  if (now - lastPrintedMs < 100) return;
  if (!c.connected) return;

  ps5.setLed(r, g, b);
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  ps5.begin("E8:47:3A:BC:DD:DF");
  Serial.println("Ready");

}

void loop() {
  readController(controller);
  //printController(controller);
  computeDriveCommand(controller, driveCommand);
  printDriveCommand(driveCommand, controller);


}

