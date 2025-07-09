#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

const unsigned char melody[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x1f, 0x80, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x31, 0xe0, 0x00, 0x00, 0x00, 0xc7, 0x80, 0x00, 0x00, 0x03, 0x9e, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x60, 0x70, 0x00, 0x00, 0x01, 0x81, 0xc0, 0x00, 0x00, 0x07, 0x07, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x40, 0x38, 0x00, 0x00, 0x01, 0x80, 0xe0, 0x00, 0x00, 0x06, 0x03, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x60, 0x18, 0x00, 0x00, 0x01, 0x80, 0x60, 0x00, 0x00, 0x06, 0x01, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x60, 0x1c, 0x00, 0x00, 0x01, 0x80, 0x70, 0x00, 0x00, 0x06, 0x01, 0xc0, 0x00, 0x00, 0x00,
  0x00, 0x60, 0x0c, 0x01, 0xf0, 0x01, 0x80, 0x10, 0x07, 0xc0, 0x06, 0x00, 0x40, 0x1f, 0x00, 0x00,
  0x00, 0x70, 0x0e, 0x07, 0x1c, 0x01, 0xc0, 0x18, 0x1c, 0x70, 0x07, 0x00, 0x60, 0x71, 0xc0, 0x00,
  0x00, 0x30, 0x02, 0x0c, 0x06, 0x00, 0xc0, 0x08, 0x30, 0x18, 0x03, 0x00, 0x20, 0xc0, 0x60, 0x00,
  0x00, 0x30, 0x02, 0x18, 0x03, 0x00, 0xc0, 0x08, 0x60, 0x0c, 0x03, 0x00, 0x21, 0x80, 0x30, 0x00,
  0x00, 0x38, 0x03, 0x30, 0x01, 0x00, 0xe0, 0x0c, 0xc0, 0x04, 0x03, 0x80, 0x33, 0x00, 0x10, 0x00,
  0x00, 0x18, 0x01, 0x20, 0x01, 0x80, 0x60, 0x04, 0x80, 0x06, 0x01, 0x80, 0x12, 0x00, 0x18, 0x00,
  0x00, 0x1c, 0x01, 0x60, 0x20, 0x80, 0x70, 0x05, 0x80, 0x82, 0x00, 0xc0, 0x16, 0x02, 0x08, 0x00,
  0x00, 0x0c, 0x3d, 0xc0, 0x20, 0x80, 0x30, 0xf7, 0x00, 0x82, 0x00, 0xc1, 0xdc, 0x02, 0x08, 0x00,
  0x00, 0x0e, 0x66, 0x40, 0x60, 0x80, 0x39, 0x99, 0x01, 0x82, 0x00, 0xe2, 0x64, 0x06, 0x08, 0x00,
  0x00, 0x3f, 0xc2, 0x40, 0x60, 0x80, 0xff, 0x09, 0x01, 0x82, 0x01, 0xfc, 0x24, 0x06, 0x08, 0x00,
  0x00, 0x63, 0xc2, 0x60, 0xc0, 0x81, 0xcf, 0x09, 0x83, 0x02, 0x07, 0x1c, 0x26, 0x0c, 0x08, 0x00,
  0x00, 0x63, 0x62, 0x71, 0xc0, 0x81, 0x8d, 0x89, 0xc7, 0x02, 0x06, 0x16, 0x27, 0x1c, 0x08, 0x00,
  0x00, 0x43, 0x71, 0x3f, 0x80, 0x81, 0x0d, 0xc4, 0xfe, 0x02, 0x04, 0x33, 0x13, 0xf8, 0x08, 0x00,
  0x00, 0x63, 0xe1, 0x1d, 0x01, 0x81, 0x8f, 0x84, 0x7c, 0x06, 0x06, 0x3e, 0x11, 0xf0, 0x18, 0x00,
  0x00, 0x21, 0xe3, 0x00, 0x00, 0xc0, 0xc7, 0x8c, 0x00, 0x03, 0x03, 0x1f, 0x30, 0x00, 0x0c, 0x00,
  0x00, 0x21, 0xff, 0x00, 0x00, 0xc0, 0xc6, 0xfc, 0x00, 0x03, 0x03, 0x1b, 0xf0, 0x00, 0x0c, 0x00,
  0x00, 0x33, 0x1e, 0x00, 0x00, 0x60, 0xcc, 0x78, 0x00, 0x01, 0x83, 0x39, 0xe0, 0x00, 0x06, 0x00,
  0x00, 0x3e, 0x00, 0x00, 0x00, 0x60, 0xfc, 0x00, 0x00, 0x01, 0x83, 0xf0, 0x00, 0x00, 0x06, 0x00,
  0x00, 0x6c, 0x00, 0x00, 0x00, 0x31, 0xb0, 0x00, 0x00, 0x00, 0xc6, 0x60, 0x00, 0x00, 0x03, 0x00,
  0x00, 0x60, 0x0f, 0xff, 0x00, 0x31, 0x80, 0x3f, 0xfc, 0x00, 0xc6, 0x00, 0xff, 0xf0, 0x03, 0x00,
  0x00, 0x60, 0x3f, 0xff, 0xc0, 0x11, 0x80, 0xff, 0xff, 0x00, 0x46, 0x03, 0xff, 0xfc, 0x01, 0x00,
  0x00, 0x60, 0x70, 0x00, 0xf0, 0x11, 0x81, 0xc0, 0x03, 0xc0, 0x46, 0x07, 0x00, 0x0f, 0x01, 0x00,
  0x00, 0x61, 0x80, 0x00, 0x38, 0x11, 0x86, 0x00, 0x00, 0xe0, 0x46, 0x18, 0x00, 0x01, 0x81, 0x00,
  0x00, 0x61, 0x00, 0x00, 0x0c, 0x11, 0x84, 0x00, 0x00, 0x30, 0x46, 0x10, 0x00, 0x00, 0xc1, 0x00,
  0x00, 0x62, 0x30, 0x00, 0xe6, 0x11, 0x88, 0xc0, 0x03, 0x98, 0x46, 0x23, 0x00, 0x0e, 0x61, 0x00,
  0x00, 0x62, 0x70, 0x00, 0xe6, 0x11, 0x88, 0xc0, 0x03, 0x98, 0x46, 0x23, 0x00, 0x0e, 0x61, 0x00,
  0x00, 0x72, 0x30, 0x00, 0xe6, 0x31, 0xc8, 0xc0, 0x03, 0x98, 0xc7, 0x23, 0x00, 0x0e, 0x63, 0x00,
  0x00, 0x32, 0x30, 0xa0, 0xc6, 0x20, 0xc8, 0xc2, 0xc1, 0x18, 0x83, 0x23, 0x09, 0x04, 0x62, 0x00,
  0x00, 0x3a, 0x00, 0x60, 0x06, 0x60, 0xe8, 0x01, 0x80, 0x19, 0x83, 0xa0, 0x06, 0x00, 0x66, 0x00,
  0x00, 0x1d, 0x00, 0x00, 0x0c, 0xc0, 0x74, 0x00, 0x00, 0x33, 0x00, 0xd0, 0x00, 0x00, 0xcc, 0x00,
  0x00, 0x0f, 0x87, 0x38, 0x1f, 0x80, 0x3e, 0x1c, 0x70, 0x7e, 0x00, 0x78, 0x71, 0xc1, 0xf8, 0x00,
  0x00, 0x07, 0xf8, 0xe7, 0xfe, 0x00, 0x1f, 0xe3, 0x9f, 0xfc, 0x00, 0x3f, 0xce, 0x7f, 0xf0, 0x00,
  0x00, 0x00, 0x78, 0x42, 0x30, 0x00, 0x01, 0xa1, 0x08, 0xe0, 0x00, 0x02, 0x84, 0x23, 0x80, 0x00,
  0x00, 0x00, 0x58, 0x42, 0x18, 0x00, 0x01, 0x21, 0x08, 0x60, 0x00, 0x04, 0x84, 0x21, 0x80, 0x00,
  0x00, 0x00, 0xf0, 0xc3, 0x18, 0x00, 0x03, 0xc3, 0x0c, 0x60, 0x00, 0x0d, 0x0c, 0x31, 0xc0, 0x00,
  0x00, 0x00, 0x61, 0x21, 0xf0, 0x00, 0x01, 0x84, 0x87, 0xe0, 0x00, 0x06, 0x1a, 0x1f, 0x80, 0x00,
  0x00, 0x00, 0x23, 0x30, 0x70, 0x00, 0x00, 0x8c, 0xc0, 0xc0, 0x00, 0x02, 0x1b, 0x03, 0x00, 0x00,
  0x00, 0x00, 0x37, 0x18, 0x7c, 0x00, 0x00, 0xdc, 0x61, 0xf0, 0x00, 0x03, 0x31, 0x87, 0xc0, 0x00,
  0x00, 0x00, 0x1c, 0x0f, 0xd6, 0x00, 0x00, 0x78, 0x3f, 0x58, 0x00, 0x01, 0xe0, 0xfd, 0x60, 0x00,
  0x00, 0x00, 0x10, 0x00, 0x13, 0x00, 0x00, 0x40, 0x00, 0x4c, 0x00, 0x01, 0x00, 0x01, 0x30, 0x00,
  0x00, 0x00, 0x10, 0x00, 0x11, 0x00, 0x00, 0x40, 0x00, 0x44, 0x00, 0x01, 0x00, 0x01, 0x10, 0x00,
  0x00, 0x00, 0x10, 0x00, 0x11, 0x00, 0x00, 0x40, 0x00, 0x44, 0x00, 0x01, 0x00, 0x01, 0x10, 0x00,
  0x00, 0x00, 0x08, 0x60, 0x13, 0x00, 0x00, 0x21, 0x80, 0x4c, 0x00, 0x00, 0x86, 0x01, 0x10, 0x00,
  0x00, 0x00, 0x18, 0xe0, 0x16, 0x00, 0x00, 0x63, 0x80, 0x58, 0x00, 0x01, 0x8e, 0x01, 0x30, 0x00,
  0x00, 0x00, 0x10, 0x80, 0x1c, 0x00, 0x00, 0x42, 0x00, 0x70, 0x00, 0x01, 0x08, 0x01, 0xc0, 0x00,
  0x00, 0x00, 0x10, 0x80, 0x10, 0x00, 0x00, 0x42, 0x00, 0x40, 0x00, 0x01, 0x08, 0x01, 0x00, 0x00,
  0x00, 0x00, 0x18, 0x80, 0x70, 0x00, 0x00, 0x62, 0x00, 0xc0, 0x00, 0x01, 0x88, 0x03, 0x00, 0x00,
  0x00, 0x00, 0x0f, 0xff, 0xe0, 0x00, 0x00, 0x3f, 0xff, 0x80, 0x00, 0x00, 0xff, 0xfe, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
BluetoothSerial SerialBT;

Servo servoX;
Servo servoY;

const int SERVO_X = 19;
const int SERVO_Y = 18;
// Published values for SG90 servos; adjust if needed
int minUs = 1000;
int maxUs = 2000;

unsigned long lastServoXMove = 0;
unsigned long lastServoYMove = 0;
const unsigned long SERVO_IDLE_TIMEOUT = 55555555555000;  // 3 detik

bool servoXActive = false;
bool servoYActive = false;

int angleX = 90;
int angleY = 90;

int stepSize = 5;  // seberapa besar gerakan tiap command

const int enPin = 25;  // ENA semua motor gabung ke sini (PWM)
const int pwmChannel = 0;
const int pwmFreq = 1000;
const int pwmResolution = 8;

// Motor KANAN ATAS
const int RA_FW = 5;
const int RA_BW = 23;

// Motor KANAN BAWAH
const int RB_FW = 13;
const int RB_BW = 12;

// Motor KIRI ATAS
const int LA_FW = 14;
const int LA_BW = 27;

// Motor KIRI BAWAH
const int LB_FW = 17;
const int LB_BW = 16;

// Buzzer & LED
const int buzPin = 2;
const int ledPin = 33;

bool isStop = false;

int valSpeed = 255;

const int motorPins[] = {
  5, 23,   // Motor Kanan Atas
  13, 12,  // Motor Kanan Bawah
  14, 27,  // Motor Kiri Atas
  17, 16,  // Motor Kiri Bawah
  SERVO_X, SERVO_Y
};
const int numMotorPins = sizeof(motorPins) / sizeof(motorPins[0]);

int screenMenu = 0;
ESP32PWM pwm;

void setup() {
  Serial.begin(115200);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("OLED gagal nyala"));
    while (1)
      ;
  }

  SerialBT.begin("ESP32Robot");
  Serial.println("Bluetooth nyambung");

  showMainMenu();
  servoX.attach(SERVO_X);
  servoY.attach(SERVO_Y);

  servoX.write(angleX);
  servoY.write(angleY);

  // Set semua pin motor
  pinMode(RA_FW, OUTPUT);
  pinMode(RA_BW, OUTPUT);
  pinMode(RB_FW, OUTPUT);
  pinMode(RB_BW, OUTPUT);
  pinMode(LA_FW, OUTPUT);
  pinMode(LA_BW, OUTPUT);
  pinMode(LB_FW, OUTPUT);
  pinMode(LB_BW, OUTPUT);

  pinMode(buzPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // PWM Setup
  ledcAttach(enPin, pwmFreq, pwmResolution);  // ✅ pin, freq, resolution
  ledcWrite(enPin, valSpeed);                 // ✅ write ke pin, bukan channel

  stopAll();
}

void loop() {
  static String input = "";



  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    Serial.println(cmd);

    if (cmd == '#') {
      input = "#";
    } else if (input.startsWith("#")) {
      if (isDigit(cmd)) {
        input += cmd;  // tambah angka
      } else {
        // Kalau bukan angka, proses input
        int newSpeed = input.substring(1).toInt();
        valSpeed = constrain(newSpeed, 0, 255);
        ledcWrite(enPin, valSpeed);
        Serial.print("Speed set: ");
        Serial.print((valSpeed * 100) / 255);
        Serial.println("%");
        input = "";  // reset input
      }
      // continue; // skip switch(cmd)
    }

    if (cmd == '@') {
      input = "@";
    } else if (input.startsWith("@")) {
      if (isDigit(cmd)) {
        input += cmd;  // tambah angka
      } else {
        int rawInput = input.substring(1).toInt();  // Ambil angka setelah "@"
        rawInput = constrain(rawInput, 0, 255);     // Batasi input 0-255
        stepSize = (rawInput * 60) / 255;           // Hitung nilai PWM maksimal 60

        ledcWrite(enPin, stepSize);

        Serial.print("Speed set: ");
        Serial.print((stepSize * 100) / 60);  // Hitung % dari 60
        Serial.println("%");

        input = "";  // reset
      }
    }

    isStop = (cmd == 'S');

    switch (cmd) {
      case 'F':
        forward();
        Serial.print("Forward");
        break;
      case 'B':
        backward();
        Serial.print("Backward");
        break;
      case 'R':
        right();
        Serial.print("Turn Right");
        break;
      case 'L':
        left();
        Serial.print("Turn Left");
        break;
      case 'G':
        forwardLeft();
        Serial.print("Forward Left");
        break;
      case 'H':
        forwardRight();
        Serial.print("Forward Right");
        break;
      case 'I':
        backwardLeft();
        Serial.print("Backward Left");
        break;
      case 'J':
        backwardRight();
        Serial.print("Backward Right");
        break;
      case 'S':
        stopAll();
        stopServo();
        // Serial.println("Stop");
        break;
      case 'Y':
        honk();
        Serial.print("Honk");
        break;
      // case 'U':
      //   digitalWrite(ledPin, HIGH);
      //   Serial.print("Headlight ON");
      //   break;
      // case 'u':
      //   digitalWrite(ledPin, LOW);
      //   Serial.print("Headlight OFF");
      //   break;
      case 'Z':
        setSpeed();
        Serial.print("Speed set: " + String((valSpeed * 100) / 255) + "%");
        break;
      case 'A':  // analog atas
        angleY = constrain(angleY - stepSize, 0, 180);
        servoY.write(angleY);
        Serial.println("Servo Y ↑ : " + String(angleY));
        break;

      case 'E':  // analog bawah
        angleY = constrain(angleY + stepSize, 0, 180);
        servoY.write(angleY);
        Serial.println("Servo Y ↓ : " + String(angleY));
        break;

      case 'U':  // analog kiri
        angleX = constrain(angleX - stepSize, 0, 180);
        servoX.write(angleX);
        Serial.println("Servo X ← : " + String(angleX));
        break;

      case 'C':  // analog kanan
        angleX = constrain(angleX + stepSize, 0, 180);
        servoX.write(angleX);
        Serial.println("Servo X → : " + String(angleX));
        break;

      case 'M':  // kanan atas
        angleX = constrain(angleX + stepSize, 0, 180);
        angleY = constrain(angleY - stepSize, 0, 180);
        servoX.write(angleX);
        servoY.write(angleY);
        Serial.println("Servo ↗ : X=" + String(angleX) + ", Y=" + String(angleY));
        break;

      case 'D':  // kanan bawah
        angleX = constrain(angleX + stepSize, 0, 180);
        angleY = constrain(angleY + stepSize, 0, 180);
        servoX.write(angleX);
        servoY.write(angleY);
        Serial.println("Servo ↘ : X=" + String(angleX) + ", Y=" + String(angleY));
        break;

      case 'V':  // kiri bawah
        angleX = constrain(angleX - stepSize, 0, 180);
        angleY = constrain(angleY + stepSize, 0, 180);
        servoX.write(angleX);
        servoY.write(angleY);
        Serial.println("Servo ↙ : X=" + String(angleX) + ", Y=" + String(angleY));
        break;

      case 'T':  // kiri atas
        angleX = constrain(angleX - stepSize, 0, 180);
        angleY = constrain(angleY - stepSize, 0, 180);
        servoX.write(angleX);
        servoY.write(angleY);
        Serial.println("Servo ↖ : X=" + String(angleX) + ", Y=" + String(angleY));
        break;
      case 'O':  // kiri atas
        angleX = 90;
        angleY = 90;
        servoX.write(angleX);
        servoY.write(angleY);
        Serial.println("Servo ↖ : X=" + String(90) + ", Y=" + String(90));
        break;
      case 'X':
        if (screenMenu == 4) {
          screenMenu = 0;
        } else {
          screenMenu = screenMenu + 1;
        }
        break;
      default:
        break;
    }
  }

  switch (screenMenu) {
    case 0:
      showMainMenu();
      break;
    case 1:
      showMotorMappingMenu();
      break;
    case 2:
      showServoMappingMenu();
      break;
    case 3:
      showPinStates();
      break;
    case 4:
      showAboutMenu();
      break;
  }

  // Detach servo X jika idle
  if (servoXActive && millis() - lastServoXMove > SERVO_IDLE_TIMEOUT) {
    // servoX.detach();

    servoXActive = false;
    SerialBT.println("Servo X auto-detached");
  }

  // Detach servo Y jika idle
  if (servoYActive && millis() - lastServoYMove > SERVO_IDLE_TIMEOUT) {
    // servoY.detach();

    servoYActive = false;
    SerialBT.println("Servo Y auto-detached");
  }
}

void showMainMenu() {
  display.clearDisplay();
  display.drawBitmap(0, 0, melody, 128, 64, SSD1306_WHITE);
  display.display();
}

void showMotorMappingMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  int x = 0;
  int y = 0;

  display.setCursor(x, y);
  display.println("F: MT UP   | B: MT DN");
  display.setCursor(x, y += 10);
  display.println("G: MT UL   | H: MT UR");
  display.setCursor(x, y += 10);
  display.println("I: MT DL   | J: MT DR");
  display.setCursor(x, y += 10);
  display.println("R: MT RIGHT| L: MT LEFT");
  display.setCursor(x, y += 10);
  display.println("S: STOP    | Y: HORN");

  display.display();
}

void showServoMappingMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  int x = 0;
  int y = 0;

  display.setCursor(x, y);
  display.println("A: SV UP   | E: SV DN");
  display.setCursor(x, y += 10);
  display.println("U: SV LEFT | C: SV RIGHT");
  display.setCursor(x, y += 10);
  display.println("M: SV UR   | D: SV DR");
  display.setCursor(x, y += 10);
  display.println("T: SV UL   | V: SV DL");
  display.setCursor(x, y += 10);
  display.println("O: SV CNTR | X: MENU");

  display.display();
}

void showAboutMenu() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Baris 1: Judul
  String title = "AVELIN";
  display.setTextSize(2);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  int xPos = (128 - w) / 2;
  int yPos = 5;
  display.setCursor(xPos, yPos);
  display.println(title);

  // Baris 2: Deskripsi
  String desc = "Smart Car";
  display.setTextSize(1);
  display.getTextBounds(desc, 0, 0, &x1, &y1, &w, &h);
  xPos = (128 - w) / 2;
  yPos += 20;  // jarak setelah title
  display.setCursor(xPos, yPos);
  display.println(desc);

  // Baris 3: Versi
  String version = "Versi 0.0.1";
  display.getTextBounds(version, 0, 0, &x1, &y1, &w, &h);
  xPos = (128 - w) / 2;
  yPos += 12;  // jarak setelah desc
  display.setCursor(xPos, yPos);
  display.println(version);

  display.display();
}


void showPinStates() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  for (int i = 0; i < numMotorPins; i += 2) {
    display.setCursor(0, (i / 2) * 10);

    // Pin kiri
    int pin1 = motorPins[i];
    int state1 = digitalRead(pin1);
    display.print(pin1 == 5 ? String(" ") + pin1 : String(pin1));
    display.print(": ");
    display.print(state1 == HIGH ? "High" : "Low");

    // Spacer
    display.print("  |  ");

    // Pin kanan (kalau masih ada)
    if (i + 1 < numMotorPins) {
      int pin2 = motorPins[i + 1];
      int state2 = digitalRead(pin2);
      display.print(pin2);
      display.print(": ");
      display.print(state2 == HIGH ? "High" : "Low");
    }

    display.println();
  }

  display.display();
}


void setMotor(int m1, int m2, int m3, int m4, int s1 = -1, int s2 = -1, int s3 = -1, int s4 = -1) {
  // Motor 1: Kanan Atas
  digitalWrite(RA_FW, m1 == 1);
  digitalWrite(RA_BW, m1 == -1);

  // Motor 2: Kanan Bawah
  digitalWrite(RB_FW, m2 == 1);
  digitalWrite(RB_BW, m2 == -1);

  // Motor 3: Kiri Atas
  digitalWrite(LA_FW, m3 == 1);
  digitalWrite(LA_BW, m3 == -1);

  // Motor 4: Kiri Bawah
  digitalWrite(LB_FW, m4 == 1);
  digitalWrite(LB_BW, m4 == -1);
}



void forward() {
  setMotor(1, 1, 1, 1);
  showStatus("↑ Maju");
  Serial.println("Arah: Maju");
}

void backward() {
  setMotor(-1, -1, -1, -1);
  showStatus("↓ Mundur");
  Serial.println("Arah: Mundur");
}

void right() {
  setMotor(0, 0, 1, 1);
  showStatus("→ Kanan");
  Serial.println("Arah: Kanan");
}

void left() {
  setMotor(1, 1, 0, 0);
  showStatus("← Kiri");
  Serial.println("Arah: Kiri");
}
// void forwardLeft() {
//   setMotor(RA_FW, RA_BW, true);
//   setMotor(RB_FW, RB_BW, true);
//   setMotor(LA_FW, LA_BW, false); // kiri stop
//   setMotor(LB_FW, LB_BW, false);
// }

// void forwardRight() {
//   setMotor(RA_FW, RA_BW, false); // kanan stop
//   setMotor(RB_FW, RB_BW, false);
//   setMotor(LA_FW, LA_BW, true);
//   setMotor(LB_FW, LB_BW, true);
// }

// void backwardLeft() {
//   setMotor(RA_FW, RA_BW, false);
//   setMotor(RB_FW, RB_BW, false);
//   setMotor(LA_FW, LA_BW, false); // kiri stop
//   setMotor(LB_FW, LB_BW, false);
// }

// void backwardRight() {
//   setMotor(RA_FW, RA_BW, false); // kanan stop
//   setMotor(RB_FW, RB_BW, false);
//   setMotor(LA_FW, LA_BW, false);
//   setMotor(LB_FW, LB_BW, false);
// }

void ensureServoAttached(Servo &servo, int pin, bool &isActive, unsigned long &lastMove) {
  if (!isActive) {
    // servo.attach(pin);
  }
  isActive = true;
  lastMove = millis();
}

void moveServoUntilInput(
  Servo &servo, int &angle, int delta, int pin, bool &active, unsigned long &lastMove) {
  ensureServoAttached(servo, pin, active, lastMove);

  while (!SerialBT.available()) {
    angle = constrain(angle + delta, 0, 180);
    servo.write(angle);
    lastMove = millis();
    SerialBT.print("xxxx");
    delay(10);
  }
  while (SerialBT.available()) SerialBT.read();  // Kosongkan buffer
}

void moveDoubleServoUntilInput(
  Servo &servoX, int angle1, int deltaX, int pinX, bool &activeX, unsigned long &lastMoveX,
  Servo &servoY, int angle2, int deltaY, int pinY, bool &activeY, unsigned long &lastMoveY) {
  ensureServoAttached(servoX, pinX, activeX, lastMoveX);
  ensureServoAttached(servoY, pinY, activeY, lastMoveY);

  while (!SerialBT.available()) {
    angleX = constrain(angle1 + deltaX, 0, 180);
    angleY = constrain(angle2 + deltaY, 0, 180);

    servoX.write(angleX);
    servoY.write(angleY);

    lastMoveX = millis();
    lastMoveY = millis();

    SerialBT.print("xxxx");
    delay(10);
  }

  while (SerialBT.available()) SerialBT.read();  // Kosongkan buffer
}


void stopServo() {
  // servoX.detach();
  // servoY.detach();
  // servoX.write(90);
  // servoY.write(90);
}

void stopAll() {
  setMotor(0, 0, 0, 0);
  // stopServo();
  showStatus("Stop Motor");
  Serial.println("Stop Motor");
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
  setMotor(1, 1, 1, 1,
           valSpeed, valSpeed, valSpeed / 2, valSpeed / 2);
  Serial.println("Arah: Forward Left");
  showStatus("↖ Maju Kiri");
}

void forwardRight() {
  setMotor(1, 1, 1, 1,
           valSpeed / 2, valSpeed / 2, valSpeed, valSpeed);
  Serial.println("Arah: Forward Right");
  showStatus("↗ Maju Kanan");
}

void backwardLeft() {
  setMotor(-1, -1, -1, -1,
           valSpeed, valSpeed, valSpeed / 2, valSpeed / 2);
  Serial.println("Arah: Backward Left");
  showStatus("↙ Mundur Kiri");
}

void backwardRight() {
  setMotor(-1, -1, -1, -1,
           valSpeed / 2, valSpeed / 2, valSpeed, valSpeed);
  Serial.println("Arah: Backward Right");
  showStatus("↘ Mundur Kanan");
}

void setSpeed() {
  // Urutan naik: 65 → 130 → 195 → 255 → balik lagi ke 65
  if (valSpeed == 65) valSpeed = 130;
  else if (valSpeed == 130) valSpeed = 195;
  else if (valSpeed == 195) valSpeed = 255;
  else valSpeed = 65;

  ledcWrite(enPin, valSpeed);
}

void showStatus(String text) {
  // display.clearDisplay();
  // display.setTextSize(2);  // Ukuran teks (1~3)
  // display.setTextColor(SSD1306_WHITE);
  // display.setCursor(0, 0);
  // display.println(text);
  // display.display();
}
