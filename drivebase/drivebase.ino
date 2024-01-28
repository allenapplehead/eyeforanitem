#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// Front motors
#define PIN_IN1  16
#define PIN_IN2  17
#define PIN_IN3  5
#define PIN_IN4  18
#define PIN_ENA_LEFT_F  19
#define PIN_ENA_RIGHT_F  4

// Back motors
#define PIN_IN5  25
#define PIN_IN6  26
#define PIN_IN7  27
#define PIN_IN8  14
#define PIN_ENA_LEFT_B  33
#define PIN_ENA_RIGHT_B  12

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32_BT_Car"); // Bluetooth device name

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  pinMode(PIN_IN5, OUTPUT);
  pinMode(PIN_IN6, OUTPUT);
  pinMode(PIN_IN7, OUTPUT);
  pinMode(PIN_IN8, OUTPUT);


  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  if (SerialBT.available()) {
    char receivedChar = SerialBT.read();
    switch (receivedChar) {
      case 'F': // Forward
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, HIGH);
        analogWrite(PIN_ENA_LEFT_F, 255);
        analogWrite(PIN_ENA_RIGHT_F, 255);

        digitalWrite(PIN_IN5, HIGH);
        digitalWrite(PIN_IN6, LOW);
        digitalWrite(PIN_IN7, LOW);
        digitalWrite(PIN_IN8, HIGH);
        analogWrite(PIN_ENA_LEFT_B, 255);
        analogWrite(PIN_ENA_RIGHT_B, 255);
        break;
      case 'G': // Backward
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
        digitalWrite(PIN_IN3, HIGH);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENA_LEFT_F, 255);
        analogWrite(PIN_ENA_RIGHT_F, 255);

        digitalWrite(PIN_IN5, LOW);
        digitalWrite(PIN_IN6, HIGH);
        digitalWrite(PIN_IN7, HIGH);
        digitalWrite(PIN_IN8, LOW);
        analogWrite(PIN_ENA_LEFT_B, 255);
        analogWrite(PIN_ENA_RIGHT_B, 255);
        break;
      case 'R': // Front Right Turn
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
        digitalWrite(PIN_IN3, HIGH);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENA_LEFT_F, 255);
        analogWrite(PIN_ENA_RIGHT_F, 0);

        digitalWrite(PIN_IN5, HIGH);
        digitalWrite(PIN_IN6, LOW);
        digitalWrite(PIN_IN7, HIGH);
        digitalWrite(PIN_IN8, LOW);
        analogWrite(PIN_ENA_LEFT_B, 255);
        analogWrite(PIN_ENA_RIGHT_B, 0);
        break;
      case 'L': // Front Left Turn
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, HIGH);
        analogWrite(PIN_ENA_LEFT_F, 0);
        analogWrite(PIN_ENA_RIGHT_F, 255);

        digitalWrite(PIN_IN5, LOW);
        digitalWrite(PIN_IN6, HIGH);
        digitalWrite(PIN_IN7, LOW);
        digitalWrite(PIN_IN8, HIGH);
        analogWrite(PIN_ENA_LEFT_B, 0);
        analogWrite(PIN_ENA_RIGHT_B, 255);
        break;
      case 'S': // Stop
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENA_LEFT_F, 0);
        analogWrite(PIN_ENA_RIGHT_F, 0);

        digitalWrite(PIN_IN5, LOW);
        digitalWrite(PIN_IN6, LOW);
        digitalWrite(PIN_IN7, LOW);
        digitalWrite(PIN_IN8, LOW);
        analogWrite(PIN_ENA_LEFT_B, 0);
        analogWrite(PIN_ENA_RIGHT_B, 0);
        break;
      // Add more cases as needed
    }
  }
}
