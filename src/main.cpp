#include <Arduino.h>
#include <ps5Controller.h>

unsigned long lastPrintedMs = 0;
bool lastConnectedState = false;

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

void readController(ControllerInput &c) {
  c.connected = ps5.isConnected();
  
  c.lx = ps5.LStickX();
  
  c.ly = ps5.LStickY();
  
  c.l2 = ps5.L2Value();
  
  c.r2 = ps5.R2Value();
  
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
  printController(controller);
}

