#include <Arduino.h>
#include <ps5Controller.h>

unsigned long lastPrintedMs = 0;

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

  if (now - lastPrintedMs >= 200) {
    Serial.print("Status: ");
    Serial.println(c.connected ? "Connected" : "Disconnected");
    if (c.connected) {
      Serial.print("R2: ");
      Serial.print(c.r2);
      Serial.print("L2: ");
      Serial.print(c.l2);
      Serial.print("Left Stick X: ");
      Serial.println(c.lx);
    }
  }

  lastPrintedMs = now;
}

void setControllerLight(int r, int g, int b) {

}

void setup() {
  // put your setup code here, to run once:
  ps5.begin("1a:2b:3c:01:01:01");
  Serial.println("Ready");

}

void loop() {
  // put your main code here, to run repeatedly:
}

