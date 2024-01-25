#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define PIN_IN1  16
#define PIN_IN2  17
#define PIN_IN3  5
#define PIN_IN4  18
#define PIN_ENA_LEFT  19
#define PIN_ENA_RIGHT  4

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32_BT_Car"); // Bluetooth device name

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  if (SerialBT.available()) {
    char receivedChar = SerialBT.read();
    switch (receivedChar) {
      case 'F': // Forward
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
        digitalWrite(PIN_IN3, HIGH);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENA_LEFT, 255);
        analogWrite(PIN_ENA_RIGHT, 255);
        break;
      case 'G': // Backward
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, HIGH);
        analogWrite(PIN_ENA_LEFT, 255);
        analogWrite(PIN_ENA_RIGHT, 255);
        break;
      case 'R': // Front Right Turn
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
        digitalWrite(PIN_IN3, HIGH);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENA_LEFT, 255);
        analogWrite(PIN_ENA_RIGHT, 170);
        break;
      case 'L': // Front Left Turn
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
        digitalWrite(PIN_IN3, HIGH);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENA_LEFT, 170);
        analogWrite(PIN_ENA_RIGHT, 255);
        break;
      case 'S': // Stop
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENA_LEFT, 0);
        analogWrite(PIN_ENA_RIGHT, 0);
        break;
      // Add more cases as needed
    }
  }
}
