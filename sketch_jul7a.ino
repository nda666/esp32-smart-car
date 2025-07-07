#include <Arduino.h>
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

const int enPin = 25;  // ENA semua motor gabung ke sini (PWM)
const int pwmChannel = 0;
const int pwmFreq = 1000;
const int pwmResolution = 8;

// Motor KANAN ATAS
const int RKA_MAJU = 5;
const int RKA_MUNDUR = 23;

// Motor KANAN BAWAH
const int RKB_MAJU = 13;
const int RKB_MUNDUR = 12;

// Motor KIRI ATAS
const int LKA_MAJU = 14;
const int LKA_MUNDUR = 27;

// Motor KIRI BAWAH
const int LKB_MAJU = 17;
const int LKB_MUNDUR = 16;

// Buzzer & LED
const int buzPin = 2;
const int ledPin = 33;

int valSpeed = 255;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32Robot");
  Serial.println("Bluetooth nyambung");

  // Set semua pin motor
  pinMode(RKA_MAJU, OUTPUT);
  pinMode(RKA_MUNDUR, OUTPUT);
  pinMode(RKB_MAJU, OUTPUT);
  pinMode(RKB_MUNDUR, OUTPUT);
  pinMode(LKA_MAJU, OUTPUT);
  pinMode(LKA_MUNDUR, OUTPUT);
  pinMode(LKB_MAJU, OUTPUT);
  pinMode(LKB_MUNDUR, OUTPUT);

  pinMode(buzPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // PWM Setup
  ledcAttach(enPin, pwmFreq, pwmResolution);  // ✅ pin, freq, resolution
  ledcWrite(enPin, valSpeed);                 // ✅ write ke pin, bukan channel

  stopAll();
}

void loop() {
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    Serial.println(cmd);

    switch (cmd) {
      case 'F':
        forward();
        SerialBT.print("Forward");
        break;
      case 'B':
        backward();
        SerialBT.print("Backward");
        break;
      case 'R':
        right();
        SerialBT.print("Turn Right");
        break;
      case 'L':
        left();
        SerialBT.print("Turn Left");
        break;
      case 'G':
        forwardLeft();
        SerialBT.print("Forward Left");
        break;
      case 'H':
        forwardRight();
        SerialBT.print("Forward Right");
        break;
      case 'I':
        backwardLeft();
        SerialBT.print("Backward Left");
        break;
      case 'J':
        backwardRight();
        SerialBT.print("Backward Right");
        break;
      case 'S':
        stopAll();
        // SerialBT.println("Stop");
        break;
      case 'Y':
        honk();
        SerialBT.print("Honk");
        break;
      case 'U':
        digitalWrite(ledPin, HIGH);
        SerialBT.print("Headlight ON");
        break;
      case 'u':
        digitalWrite(ledPin, LOW);
        SerialBT.print("Headlight OFF");
        break;
      case 'Z':
        setSpeed();
        SerialBT.print("Speed set: " + String((valSpeed * 100) / 255) + "%");
        break;
      default:
        break;
    }
  }
}

void setMotor(int majuPin, int mundurPin, bool maju) {
  digitalWrite(majuPin, maju ? HIGH : LOW);
  digitalWrite(mundurPin, !maju ? HIGH : LOW);
}

void analogMotor(int majuPin, int mundurPin, bool maju, int speedVal) {
  int pinAktif = maju ? majuPin : mundurPin;
  int pinNonAktif = maju ? mundurPin : majuPin;

  digitalWrite(pinNonAktif, LOW);  // Pastikan lawannya LOW
  ledcAttach(pinAktif, 1000, 8);   // Attach pin ke PWM (auto channel)
  ledcWrite(pinAktif, speedVal);   // Tulis duty cycle langsung ke pin
}


void forward() {
  setMotor(RKA_MAJU, RKA_MUNDUR, true);
  setMotor(RKB_MAJU, RKB_MUNDUR, true);
  setMotor(LKA_MAJU, LKA_MUNDUR, true);
  setMotor(LKB_MAJU, LKB_MUNDUR, true);
}

void backward() {
  setMotor(RKA_MAJU, RKA_MUNDUR, false);
  setMotor(RKB_MAJU, RKB_MUNDUR, false);
  setMotor(LKA_MAJU, LKA_MUNDUR, false);
  setMotor(LKB_MAJU, LKB_MUNDUR, false);
}

void right() {
  setMotor(RKA_MAJU, RKA_MUNDUR, false);  // kanan mundur
  setMotor(RKB_MAJU, RKB_MUNDUR, false);
  setMotor(LKA_MAJU, LKA_MUNDUR, true);  // kiri maju
  setMotor(LKB_MAJU, LKB_MUNDUR, true);
}

void left() {
  setMotor(RKA_MAJU, RKA_MUNDUR, true);  // kanan maju
  setMotor(RKB_MAJU, RKB_MUNDUR, true);
  setMotor(LKA_MAJU, LKA_MUNDUR, false);  // kiri mundur
  setMotor(LKB_MAJU, LKB_MUNDUR, false);
}

// void forwardLeft() {
//   setMotor(RKA_MAJU, RKA_MUNDUR, true);
//   setMotor(RKB_MAJU, RKB_MUNDUR, true);
//   setMotor(LKA_MAJU, LKA_MUNDUR, false); // kiri stop
//   setMotor(LKB_MAJU, LKB_MUNDUR, false);
// }

// void forwardRight() {
//   setMotor(RKA_MAJU, RKA_MUNDUR, false); // kanan stop
//   setMotor(RKB_MAJU, RKB_MUNDUR, false);
//   setMotor(LKA_MAJU, LKA_MUNDUR, true);
//   setMotor(LKB_MAJU, LKB_MUNDUR, true);
// }

// void backwardLeft() {
//   setMotor(RKA_MAJU, RKA_MUNDUR, false);
//   setMotor(RKB_MAJU, RKB_MUNDUR, false);
//   setMotor(LKA_MAJU, LKA_MUNDUR, false); // kiri stop
//   setMotor(LKB_MAJU, LKB_MUNDUR, false);
// }

// void backwardRight() {
//   setMotor(RKA_MAJU, RKA_MUNDUR, false); // kanan stop
//   setMotor(RKB_MAJU, RKB_MUNDUR, false);
//   setMotor(LKA_MAJU, LKA_MUNDUR, false);
//   setMotor(LKB_MAJU, LKB_MUNDUR, false);
// }

void stopAll() {
  setMotor(RKA_MAJU, RKA_MUNDUR, false);
  setMotor(RKB_MAJU, RKB_MUNDUR, false);
  setMotor(LKA_MAJU, LKA_MUNDUR, false);
  setMotor(LKB_MAJU, LKB_MUNDUR, false);
}

void honk() {
  digitalWrite(buzPin, HIGH);
  delay(200);
  digitalWrite(buzPin, LOW);
  delay(80);
  digitalWrite(buzPin, HIGH);
  delay(300);
  digitalWrite(buzPin, LOW);
}

void forwardLeft() {
  // Kanan full, kiri pelan
  setMotor(RKA_MAJU, RKA_MUNDUR, true);
  setMotor(RKB_MAJU, RKB_MUNDUR, true);
  analogMotor(LKA_MAJU, LKA_MUNDUR, true, valSpeed / 2);
  analogMotor(LKB_MAJU, LKB_MUNDUR, true, valSpeed / 2);
}

void forwardRight() {
  // Kiri full, kanan pelan
  analogMotor(RKA_MAJU, RKA_MUNDUR, true, valSpeed / 2);
  analogMotor(RKB_MAJU, RKB_MUNDUR, true, valSpeed / 2);
  setMotor(LKA_MAJU, LKA_MUNDUR, true);
  setMotor(LKB_MAJU, LKB_MUNDUR, true);
}

void backwardLeft() {
  // Kanan full, kiri pelan (mundur)
  setMotor(RKA_MAJU, RKA_MUNDUR, false);
  setMotor(RKB_MAJU, RKB_MUNDUR, false);
  analogMotor(LKA_MAJU, LKA_MUNDUR, false, valSpeed / 2);
  analogMotor(LKB_MAJU, LKB_MUNDUR, false, valSpeed / 2);
}

void backwardRight() {
  // Kiri full, kanan pelan (mundur)
  analogMotor(RKA_MAJU, RKA_MUNDUR, false, valSpeed / 2);
  analogMotor(RKB_MAJU, RKB_MUNDUR, false, valSpeed / 2);
  setMotor(LKA_MAJU, LKA_MUNDUR, false);
  setMotor(LKB_MAJU, LKB_MUNDUR, false);
}

void setSpeed() {
  // Urutan naik: 65 → 130 → 195 → 255 → balik lagi ke 65
  if (valSpeed == 65) valSpeed = 130;
  else if (valSpeed == 130) valSpeed = 195;
  else if (valSpeed == 195) valSpeed = 255;
  else valSpeed = 65;

  ledcWrite(enPin, valSpeed);
}
