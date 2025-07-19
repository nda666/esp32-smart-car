#include <EEPROM.h>  // misal 64 byte, tergantung kebutuhan

#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_mac.h"
#include <Ps3Controller.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

#define EEPROM_SIZE 64
#define EEPROM_ADDR_HARD 0  // alamat untuk hardTurnSpeed (float = 4 byte)
#define EEPROM_ADDR_SOFT 4  // alamat untuk softTurnSpeed
#define EEPROM_ADDR_BT_MODE 8

#define BT_JOYSTICK_MAC "41:42:D7:D9:7C:C3"

int player = 0;
int joystickBattery = 0;

HardwareSerial SerialUART(2);

const unsigned char melody[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xfd, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x55, 0x75, 0x75, 0x75, 0x11, 0x10, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0xb3, 0xff, 0xff, 0xfe, 0xbb, 0xba, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x55, 0x55, 0xd5, 0xfd, 0xdd, 0x55, 0x55, 0x55, 0x40, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xae, 0xee, 0xbf, 0xff, 0xff, 0xfe, 0xee, 0xee, 0xee, 0xe0, 0x00,
  0x00, 0x00, 0x01, 0x55, 0x15, 0x55, 0x55, 0x77, 0xf7, 0x7f, 0x55, 0x55, 0x55, 0x55, 0x55, 0x00,
  0x00, 0x00, 0x3b, 0xff, 0xfa, 0xbb, 0xbb, 0xff, 0xff, 0xff, 0xfe, 0xbb, 0xbb, 0xbb, 0xbb, 0xa0,
  0x00, 0x01, 0x55, 0xfd, 0x5d, 0x55, 0x55, 0xdf, 0xf5, 0x5d, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
  0x00, 0x03, 0xff, 0xff, 0xff, 0xfb, 0xfe, 0xfa, 0xff, 0xff, 0xfe, 0xaa, 0xaa, 0xee, 0xee, 0xee,
  0x00, 0x07, 0xf7, 0x77, 0x57, 0xf7, 0x77, 0x57, 0x77, 0x7f, 0x75, 0xf7, 0x75, 0x55, 0x55, 0x55,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xbf, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3b, 0xbb, 0xbb,
  0x00, 0x07, 0xd5, 0x5f, 0xdf, 0xf5, 0x5d, 0xdd, 0xfd, 0xdf, 0xd5, 0x55, 0xdd, 0x55, 0x55, 0x55,
  0x00, 0x03, 0xff, 0xff, 0xfe, 0x0b, 0xff, 0xdf, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xee, 0xee, 0xee,
  0x00, 0x15, 0x77, 0x7f, 0xf5, 0x75, 0x77, 0xd7, 0x77, 0x57, 0x57, 0xd5, 0x75, 0x55, 0x55, 0x55,
  0x00, 0x3a, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xa2, 0x22, 0xbf, 0xff, 0xbb, 0xbb, 0xbb, 0xbb,
  0x01, 0x55, 0x7f, 0xd5, 0xd5, 0xff, 0x55, 0xd5, 0x5d, 0xdf, 0xd5, 0x7d, 0x55, 0x55, 0x55, 0x55,
  0x02, 0xee, 0xbf, 0xff, 0xff, 0xff, 0x8e, 0xab, 0xff, 0xff, 0xfe, 0xff, 0xae, 0xee, 0xee, 0xee,
  0x15, 0x55, 0x57, 0x57, 0xff, 0x75, 0x55, 0x55, 0x55, 0x55, 0x55, 0x75, 0x55, 0x55, 0x55, 0x55,
  0x3b, 0xbb, 0xb3, 0xff, 0xff, 0xaf, 0xeb, 0xbb, 0xbf, 0xff, 0xff, 0xff, 0xeb, 0xbb, 0xbb, 0xbb,
  0x55, 0x55, 0x55, 0xfd, 0xfd, 0x55, 0x55, 0x55, 0x55, 0x55, 0xd5, 0xfd, 0xd5, 0x55, 0x55, 0x55,
  0xee, 0xee, 0xeb, 0xff, 0xef, 0xfe, 0xee, 0xee, 0xee, 0xea, 0xff, 0xff, 0xe2, 0xee, 0xee, 0xee,
  0x55, 0x55, 0x57, 0xf5, 0x57, 0x77, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
  0xbb, 0xbb, 0xbb, 0xfa, 0xff, 0xbf, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb,
  0x55, 0x55, 0x5d, 0xd5, 0xdd, 0xd5, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
  0xee, 0xec, 0xfe, 0xef, 0xef, 0xee, 0xee, 0xee, 0x8f, 0xff, 0xff, 0xea, 0xee, 0xee, 0xae, 0xee,
  0x55, 0x57, 0xf7, 0x55, 0x57, 0xd5, 0x55, 0x57, 0x7f, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
  0xbb, 0xaf, 0xff, 0xff, 0xff, 0xbb, 0xbb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf3, 0xbb, 0xb8, 0x3b,
  0x55, 0x55, 0x55, 0x7f, 0xdd, 0x55, 0x57, 0xff, 0xff, 0xff, 0xff, 0xd7, 0xfd, 0x55, 0x54, 0x05,
  0xee, 0xee, 0xae, 0xff, 0xf8, 0xea, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xae, 0xe8, 0x00,
  0x55, 0x55, 0x55, 0x7f, 0x75, 0x57, 0xff, 0xff, 0xff, 0xff, 0xff, 0x55, 0x7f, 0xd5, 0x54, 0x00,
  0xbb, 0xbb, 0xbb, 0x7f, 0xab, 0xbf, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x3f, 0xeb, 0xb8, 0x00,
  0x55, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x55, 0x5f, 0xf5, 0x50, 0x00,
  0xee, 0xee, 0xea, 0x78, 0xef, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x0f, 0xf6, 0xe0, 0x00,
  0x55, 0x55, 0x55, 0x55, 0x5f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xd5, 0x5f, 0xf5, 0x50, 0x00,
  0xbb, 0xbb, 0xb3, 0xbb, 0xbf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x3f, 0xf3, 0x80, 0x00,
  0x55, 0x55, 0x55, 0x55, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xff, 0xf5, 0x00, 0x00,
  0xee, 0xee, 0xee, 0xee, 0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xee, 0x00, 0x00,
  0x55, 0x55, 0x55, 0x55, 0xf5, 0x57, 0xff, 0xff, 0xf5, 0x5f, 0xff, 0xff, 0xff, 0xd4, 0x00, 0x00,
  0xbb, 0xbb, 0xbb, 0xbb, 0xf0, 0x03, 0xff, 0xff, 0xef, 0xff, 0xff, 0xff, 0xff, 0xa0, 0x00, 0x00,
  0x55, 0x55, 0x55, 0x57, 0xf5, 0x55, 0xff, 0xff, 0xd7, 0x5f, 0xff, 0xff, 0xf5, 0x55, 0xd4, 0x00,
  0xee, 0xee, 0xee, 0xe7, 0xf0, 0x03, 0xff, 0xff, 0xfa, 0xff, 0xff, 0xff, 0xef, 0xdf, 0xfe, 0x00,
  0x55, 0x55, 0x55, 0x57, 0xf5, 0x57, 0xff, 0xff, 0xff, 0xf5, 0xff, 0xff, 0xdf, 0xdf, 0xff, 0x00,
  0xbb, 0xb0, 0xbb, 0xb7, 0xf8, 0x03, 0xff, 0xff, 0xff, 0xe3, 0xff, 0xff, 0x1f, 0xff, 0xff, 0x80,
  0x55, 0x50, 0x55, 0x57, 0xfd, 0x5f, 0xff, 0xff, 0xd5, 0x5f, 0xff, 0xd4, 0x1f, 0xff, 0xff, 0xc0,
  0xee, 0x80, 0x2e, 0xeb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x2e, 0x1f, 0xff, 0xff, 0xc0,
  0x55, 0x00, 0x15, 0x55, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xd5, 0x55, 0x5f, 0xff, 0xff, 0x40,
  0xb8, 0x00, 0x03, 0xba, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xfa, 0xbb, 0xba, 0x3f, 0xff, 0xff, 0x80,
  0x50, 0x00, 0x00, 0x55, 0x5f, 0xff, 0xfd, 0x55, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xfd, 0x00,
  0x80, 0x00, 0x00, 0x02, 0xea, 0xff, 0xea, 0xee, 0xe8, 0xff, 0xae, 0xeb, 0xff, 0xff, 0xf8, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x15, 0x55, 0x55, 0x55, 0x55, 0x7f, 0xd5, 0x57, 0xff, 0xff, 0x50, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0xbb, 0xbb, 0xbb, 0xaf, 0xf3, 0xaf, 0xff, 0xfa, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x05, 0x55, 0x55, 0x55, 0x55, 0x57, 0xf5, 0x5f, 0xff, 0xd0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0e, 0xee, 0xee, 0xee, 0xea, 0xaf, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x15, 0x55, 0x55, 0x55, 0x57, 0xf5, 0xff, 0xf7, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xbb, 0xbb, 0xba, 0xff, 0xfe, 0xff, 0xef, 0xf8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0x55, 0x55, 0x57, 0xff, 0xff, 0x7f, 0xf7, 0xc0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0xee, 0x9f, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x77, 0x55, 0x77, 0xff, 0xff, 0xd7, 0xf0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0x57, 0xff, 0xf7, 0xf0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xef, 0xff, 0xfb, 0xe0, 0x00, 0x00, 0x00, 0x00
};


struct Setting {
  float hardTurnSpeed = 0.2;
  float softTurnSpeed = 0.6;
} setting;

float hardTurnOptions[] = { 0.0, 0.1, 0.2 };
float softTurnOptions[] = { 0.2, 0.3, 0.4 };
int settingMenuIndex = 0;  // 0 = hardTurnSpeed, 1 = softTurnSpeed


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
BluetoothSerial SerialBT;

Servo servoX;
Servo servoY;

const int SERVO_X = 19;
const int SERVO_Y = 18;
// Published values for SG90 servos; adjust if needed
int minUs = 1000;
int maxUs = 2000;

bool moveUSSensor = false;
unsigned long isUSSensorMoving = false;

unsigned long lastServoXMove = 0;
unsigned long lastServoYMove = 0;
const unsigned long SERVO_IDLE_TIMEOUT = 55555555555000;  // 3 detik

bool servoXActive = false;
bool servoYActive = false;

int angleX = 90;
int angleY = 90;

int stepSize = 5;  // seberapa besar gerakan tiap command

const int RM_PWM = 25;
const int LM_PWM = 26;

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

int MT_PWM_SPEED = 255;

// Buzzer & LED
const int buzPin = 2;
const int ledPin = 33;

const int motorPins[] = {
  5, 23,   // Motor Kanan Atas
  13, 12,  // Motor Kanan Bawah
  14, 27,  // Motor Kiri Atas
  17, 16,  // Motor Kiri Bawah
  SERVO_X, SERVO_Y
};
const int numMotorPins = sizeof(motorPins) / sizeof(motorPins[0]);

int screenMenu = 0;

unsigned long lastSpeedUpdate = 0;
int pendingServoX = -1;

unsigned long lastStepUpdate = 0;
int pendingServoY = -1;

const unsigned long throttleDelay = 200;

int rightDistance = -1;
int leftDistance = -1;

int btMode = 0; // 0 = host, 1 = client

void setup() {
  Serial.begin(115200);
  SerialUART.begin(9600, SERIAL_8N1, 35, 4);  // RX = 35, TX = 4
  // SerialUART.begin(9600, SERIAL_8N1, 3, 1);
  EEPROM.begin(EEPROM_SIZE);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("OLED gagal nyala"));
    while (1)
      ;
  }
  loadSetting();  // baca dari EEPROM saat startup

  if (btMode == 0) {
    SerialBT.begin("ESP32Robot");
    Serial.println("BT HOST MODE");
  } else {
    Serial.println("Mencari joystick");
    Ps3.attach(notifyPs3);
    Ps3.attachOnConnect(onConnectPs3);
    Ps3.begin(BT_JOYSTICK_MAC);
  }

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
  ledcAttach(RM_PWM, pwmFreq, pwmResolution);  // ✅ pin, freq, resolution
  ledcAttach(LM_PWM, pwmFreq, pwmResolution);  // ✅ pin, freq, resolution

  ledcWrite(RM_PWM, MT_PWM_SPEED);  // ✅ write ke pin, bukan channel
  ledcWrite(LM_PWM, MT_PWM_SPEED);  // ✅ write ke pin, bukan channel

  stopAll();
}

void onConnectPs3() {
  Serial.println("PS3 Controller Connected!");
}

void notifyPs3()
{
    //--- Digital cross/square/triangle/circle button events ---
    if( Ps3.event.button_down.cross )
        Serial.println("Started pressing the cross button");
    if( Ps3.event.button_up.cross )
        Serial.println("Released the cross button");

    if( Ps3.event.button_down.square )
        Serial.println("Started pressing the square button");
    if( Ps3.event.button_up.square )
        Serial.println("Released the square button");

    if( Ps3.event.button_down.triangle )
        Serial.println("Started pressing the triangle button");
    if( Ps3.event.button_up.triangle )
        Serial.println("Released the triangle button");

    if( Ps3.event.button_down.circle )
        Serial.println("Started pressing the circle button");
    if( Ps3.event.button_up.circle )
        Serial.println("Released the circle button");

    //--------------- Digital D-pad button events --------------
    if( Ps3.event.button_down.up )
        Serial.println("Started pressing the up button");
    if( Ps3.event.button_up.up )
        Serial.println("Released the up button");

    if( Ps3.event.button_down.right )
        Serial.println("Started pressing the right button");
    if( Ps3.event.button_up.right )
        Serial.println("Released the right button");

    if( Ps3.event.button_down.down )
        Serial.println("Started pressing the down button");
    if( Ps3.event.button_up.down )
        Serial.println("Released the down button");

    if( Ps3.event.button_down.left )
        Serial.println("Started pressing the left button");
    if( Ps3.event.button_up.left )
        Serial.println("Released the left button");

    //------------- Digital shoulder button events -------------
    if( Ps3.event.button_down.l1 )
        Serial.println("Started pressing the left shoulder button");
    if( Ps3.event.button_up.l1 )
        Serial.println("Released the left shoulder button");

    if( Ps3.event.button_down.r1 )
        Serial.println("Started pressing the right shoulder button");
    if( Ps3.event.button_up.r1 )
        Serial.println("Released the right shoulder button");

    //-------------- Digital trigger button events -------------
    if( Ps3.event.button_down.l2 )
        Serial.println("Started pressing the left trigger button");
    if( Ps3.event.button_up.l2 )
        Serial.println("Released the left trigger button");

    if( Ps3.event.button_down.r2 )
        Serial.println("Started pressing the right trigger button");
    if( Ps3.event.button_up.r2 )
        Serial.println("Released the right trigger button");

    //--------------- Digital stick button events --------------
    if( Ps3.event.button_down.l3 )
        Serial.println("Started pressing the left stick button");
    if( Ps3.event.button_up.l3 )
        Serial.println("Released the left stick button");

    if( Ps3.event.button_down.r3 )
        Serial.println("Started pressing the right stick button");
    if( Ps3.event.button_up.r3 )
        Serial.println("Released the right stick button");

    //---------- Digital select/start/ps button events ---------
    if( Ps3.event.button_down.select )
        Serial.println("Started pressing the select button");
    if( Ps3.event.button_up.select )
        Serial.println("Released the select button");

    if( Ps3.event.button_down.start )
        Serial.println("Started pressing the start button");
    if( Ps3.event.button_up.start )
        Serial.println("Released the start button");

    if( Ps3.event.button_down.ps )
        Serial.println("Started pressing the Playstation button");
    if( Ps3.event.button_up.ps )
        Serial.println("Released the Playstation button");


    //---------------- Analog stick value events ---------------
   if( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2 ){
       Serial.print("Moved the left stick:");
       Serial.print(" x="); Serial.print(Ps3.data.analog.stick.lx, DEC);
       Serial.print(" y="); Serial.print(Ps3.data.analog.stick.ly, DEC);
       Serial.println();
    }

   if( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 ){
       Serial.print("Moved the right stick:");
       Serial.print(" x="); Serial.print(Ps3.data.analog.stick.rx, DEC);
       Serial.print(" y="); Serial.print(Ps3.data.analog.stick.ry, DEC);
       Serial.println();
   }

   //--------------- Analog D-pad button events ----------------
   if( abs(Ps3.event.analog_changed.button.up) ){
       Serial.print("Pressing the up button: ");
       Serial.println(Ps3.data.analog.button.up, DEC);
   }

   if( abs(Ps3.event.analog_changed.button.right) ){
       Serial.print("Pressing the right button: ");
       Serial.println(Ps3.data.analog.button.right, DEC);
   }

   if( abs(Ps3.event.analog_changed.button.down) ){
       Serial.print("Pressing the down button: ");
       Serial.println(Ps3.data.analog.button.down, DEC);
   }

   if( abs(Ps3.event.analog_changed.button.left) ){
       Serial.print("Pressing the left button: ");
       Serial.println(Ps3.data.analog.button.left, DEC);
   }

   //---------- Analog shoulder/trigger button events ----------
   if( abs(Ps3.event.analog_changed.button.l1)){
       Serial.print("Pressing the left shoulder button: ");
       Serial.println(Ps3.data.analog.button.l1, DEC);
   }

   if( abs(Ps3.event.analog_changed.button.r1) ){
       Serial.print("Pressing the right shoulder button: ");
       Serial.println(Ps3.data.analog.button.r1, DEC);
   }

   if( abs(Ps3.event.analog_changed.button.l2) ){
       Serial.print("Pressing the left trigger button: ");
       Serial.println(Ps3.data.analog.button.l2, DEC);
   }

   if( abs(Ps3.event.analog_changed.button.r2) ){
       Serial.print("Pressing the right trigger button: ");
       Serial.println(Ps3.data.analog.button.r2, DEC);
   }

   //---- Analog cross/square/triangle/circle button events ----
   if( abs(Ps3.event.analog_changed.button.triangle)){
       Serial.print("Pressing the triangle button: ");
       Serial.println(Ps3.data.analog.button.triangle, DEC);
   }

   if( abs(Ps3.event.analog_changed.button.circle) ){
       Serial.print("Pressing the circle button: ");
       Serial.println(Ps3.data.analog.button.circle, DEC);
   }

   if( abs(Ps3.event.analog_changed.button.cross) ){
       Serial.print("Pressing the cross button: ");
       Serial.println(Ps3.data.analog.button.cross, DEC);
   }

   if( abs(Ps3.event.analog_changed.button.square) ){
       Serial.print("Pressing the square button: ");
       Serial.println(Ps3.data.analog.button.square, DEC);
   }

   //---------------------- Battery events ---------------------
    if( joystickBattery != Ps3.data.status.battery ){
        joystickBattery = Ps3.data.status.battery;
        Serial.print("The controller battery is ");
        if( joystickBattery == ps3_status_battery_charging )      Serial.println("charging");
        else if( joystickBattery == ps3_status_battery_full )     Serial.println("FULL");
        else if( joystickBattery == ps3_status_battery_high )     Serial.println("HIGH");
        else if( joystickBattery == ps3_status_battery_low)       Serial.println("LOW");
        else if( joystickBattery == ps3_status_battery_dying )    Serial.println("DYING");
        else if( joystickBattery == ps3_status_battery_shutdown ) Serial.println("SHUTDOWN");
        else Serial.println("UNDEFINED");
    }

}


int getUSSensorVal() {
  if (SerialUART.available()) {
    String inputEsp32Cam = SerialUART.readStringUntil('\n');

    if (inputEsp32Cam.startsWith("DIS: ")) {
      String angkaStr = inputEsp32Cam.substring(5);  // mulai dari index 5
      int angka = angkaStr.toInt();
      return angka;  // ubah ke integer
    }
  }
  return -1;
}

int getLatestUSVal() {
  String lastLine = "";
  while (SerialUART.available()) {
    lastLine = SerialUART.readStringUntil('\n');
    lastLine.trim();
  }

  if (lastLine.startsWith("DIS: ")) {
    return lastLine.substring(5).toInt();
  }

  return -1;
}
void loop() {

if(Ps3.isConnected()){
  Ps3.setPlayer(player);
}
  int angka = getUSSensorVal();  // ubah ke integer
  if (angka <= 10 && angka >= 0 && !moveUSSensor &&  millis() - lastServoXMove >= 500) {
    moveUSSensor = true;
    angleX = 0; 
    angleY = 90;
    servoX.write(angleX);     
    servoY.write(angleY);
    Serial.print("MOVE SERVO: ");
    Serial.println(angleX);
    lastServoXMove = millis();
  }


  if (moveUSSensor && millis() - lastServoXMove >= 500) {
    angleX += 180;
    angleY = 90;
    if (angleX > 180) {
       leftDistance = getLatestUSVal();
      angleX = 90;
      servoX.write(angleX);
      servoY.write(angleY);
      Serial.print("MOVE SERVO: ");
      Serial.println(90);
      SerialUART.print("MOVE SERVO: ");
      SerialUART.println(90);
      // Selesai kanan kiri
      moveUSSensor = false;

    } else {

      rightDistance = getLatestUSVal();
      servoX.write(angleX);
      servoY.write(angleY);
      Serial.print("MOVE SERVO: ");
      Serial.println(angleX);
      SerialUART.print("MOVE SERVO: ");
      SerialUART.println(angleX);
     
    }

    lastServoXMove = millis();
  }

  bool btAvaliable = SerialBT.available();
  if (screenMenu == 4 && btAvaliable) {
    char cmd = SerialBT.read();
    handleSettingInput(cmd);
  }

  if (pendingServoX != -1 && millis() - lastSpeedUpdate > throttleDelay) {
    // MT_PWM_SPEED = pendingServoX;
    // ledcWrite(RM_PWM, MT_PWM_SPEED);
    // ledcWrite(LM_PWM, MT_PWM_SPEED);
    servoX.write(pendingServoX);
    Serial.print("Servo X set: ");
    Serial.println(pendingServoX);
    lastSpeedUpdate = millis();
    pendingServoX = -1;  // reset
  }

  if (pendingServoY != -1 && millis() - lastSpeedUpdate > throttleDelay) {
    servoY.write(pendingServoY);
    Serial.print("Servo Y set: ");
    Serial.println(pendingServoY);
    lastStepUpdate = millis();
    pendingServoY = -1;
  }

  if (btAvaliable) {
    // char cmd = SerialBT.read();
    String input = SerialBT.readStringUntil('\n');
    input.trim();  // Hilangkan spasi/enter kalau ada
    char cmd = input.charAt(0);

    if (input.startsWith("#")) {
      int speed = input.substring(1).toInt();
      speed = constrain(speed, 0, 255);
      pendingServoX = speed;  // simpan nilai terakhir yang masuk
      lastSpeedUpdate = millis();
    } else if (input.startsWith("@")) {
      int raw = input.substring(1).toInt();
      raw = constrain(raw, 0, 255);
      pendingServoY = raw;
      lastStepUpdate = millis();
    }

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
      case 'Z':
        setSpeed();
        Serial.print("Speed set: " + String((MT_PWM_SPEED * 100) / 255) + "%");
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

      // case 'M':  // kanan atas
      //   angleX = constrain(angleX + stepSize, 0, 180);
      //   angleY = constrain(angleY - stepSize, 0, 180);
      //   servoX.write(angleX);
      //   servoY.write(angleY);
      //   Serial.println("Servo ↗ : X=" + String(angleX) + ", Y=" + String(angleY));
      //   break;

      // case 'D':  // kanan bawah
      //   angleX = constrain(angleX + stepSize, 0, 180);
      //   angleY = constrain(angleY + stepSize, 0, 180);
      //   servoX.write(angleX);
      //   servoY.write(angleY);
      //   Serial.println("Servo ↘ : X=" + String(angleX) + ", Y=" + String(angleY));
      //   break;

      // case 'V':  // kiri bawah
      //   angleX = constrain(angleX - stepSize, 0, 180);
      //   angleY = constrain(angleY + stepSize, 0, 180);
      //   servoX.write(angleX);
      //   servoY.write(angleY);
      //   Serial.println("Servo ↙ : X=" + String(angleX) + ", Y=" + String(angleY));
      //   break;

      // case 'T':  // kiri atas
      //   angleX = constrain(angleX - stepSize, 0, 180);
      //   angleY = constrain(angleY - stepSize, 0, 180);
      //   servoX.write(angleX);
      //   servoY.write(angleY);
      //   Serial.println("Servo ↖ : X=" + String(angleX) + ", Y=" + String(angleY));
      //   break;
      case 'O':  // kiri atas
        angleX = 90;
        angleY = 90;
        servoX.write(angleX);
        servoY.write(angleY);
        Serial.println("Servo ↖ : X=" + String(90) + ", Y=" + String(90));
        break;
      case 'K':
        if (screenMenu == 5) {
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
      showDirectionMenu();
      break;
    case 2:
      showMotorMappingMenu();
      break;
    case 3:
      showPinStates();
      break;
    case 4:
      showSettingsScreen();
      break;
    case 5:
      showAboutMenu();
      break;
  }
}

void handleSettingInput(char input) {

  static int optionIndex = 0;  // index dari value yg dipilih
  switch (input) {
    case 'F':
      settingMenuIndex = (settingMenuIndex - 1 + 2) % 2;
      break;
    case 'B':
      settingMenuIndex = (settingMenuIndex + 1) % 2;
      break;
    case 'L':
      if (settingMenuIndex == 0) {
        optionIndex = (optionIndex - 1 + 3) % 3;
        setting.hardTurnSpeed = hardTurnOptions[optionIndex];
      } else {
        optionIndex = (optionIndex - 1 + 3) % 3;
        setting.softTurnSpeed = softTurnOptions[optionIndex];
      }
      saveSetting();
      break;
    case 'R':
      if (settingMenuIndex == 0) {
        optionIndex = (optionIndex + 1) % 3;
        setting.hardTurnSpeed = hardTurnOptions[optionIndex];
      } else {
        optionIndex = (optionIndex + 1) % 3;
        setting.softTurnSpeed = softTurnOptions[optionIndex];
      }
      saveSetting();
      break;
    case 'K':
      if (screenMenu == 5) {
        screenMenu = 0;
      } else {
        screenMenu = screenMenu + 1;
      }
      break;
    case 'X':  // OK
      saveSetting();
      break;
  }
}

void showDirectionMenu() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  int x = 0;
  int y = 0;

  display.setCursor(x, y);
  display.print("LEFT: ");
  display.print(leftDistance);
  display.setCursor(x, y += 15);
  display.print("RIGHT: ");
  display.print(rightDistance);

  display.display();
}

void showMainMenu() {
  display.clearDisplay();

  // Tampilkan teks BT Mode di atas (y = 0)
  display.setTextSize(1);              // Teks kecil
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);             // Teks paling atas
  display.print("BT Mode: ");
  display.println(btMode == 0 ? "Host" : "Client");

  // Gambar melody di bawah teks, misalnya y = 10
  display.drawBitmap(0, 12, melody, 128, 52, SSD1306_WHITE);
  // 52 = 64 - 12, sisa tinggi layar setelah header teks

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


void showSettingsScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println("SETTING");

  display.setCursor(0, 16);
  display.print(settingMenuIndex == 0 ? "> " : "  ");
  display.print("HardTurn: ");
  display.println(setting.hardTurnSpeed, 1);

  display.setCursor(0, 30);
  display.print(settingMenuIndex == 1 ? "> " : "  ");
  display.print("SoftTurn: ");
  display.println(setting.softTurnSpeed, 1);

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


void setMotor(int m1, int m2, int m3, int m4, int rightSpeed = MT_PWM_SPEED, int leftSpeed = MT_PWM_SPEED) {
  ledcWrite(RM_PWM, rightSpeed);
  ledcWrite(LM_PWM, leftSpeed);

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
  // setMotor(0, 0, 1, 1);
  setMotor(1, 1, 1, 1,  // semua motor maju
           MT_PWM_SPEED * setting.hardTurnSpeed, MT_PWM_SPEED);
  showStatus("→ Kanan");
  Serial.println("Arah: Kanan");
}

void left() {
  // setMotor(1, 1, 0, 0);
  setMotor(1, 1, 1, 1,  // semua motor maju
           MT_PWM_SPEED, MT_PWM_SPEED * setting.hardTurnSpeed);
  showStatus("← Kiri");
  Serial.println("Arah: Kiri");
}

void forwardLeft() {
  setMotor(1, 1, 1, 1,
           MT_PWM_SPEED, MT_PWM_SPEED * setting.softTurnSpeed);
  Serial.println("Arah: Forward Left");
  showStatus("↖ Maju Kiri");
}

void forwardRight() {
  setMotor(1, 1, 1, 1,
           MT_PWM_SPEED * setting.softTurnSpeed, MT_PWM_SPEED);
  Serial.println("Arah: Forward Right");
  showStatus("↗ Maju Kanan");
}

void backwardLeft() {
  setMotor(-1, -1, -1, -1,
           MT_PWM_SPEED, MT_PWM_SPEED * setting.softTurnSpeed);
  Serial.println("Arah: Mundur Kiri");
  showStatus("↙ Mundur Kiri");
}


void backwardRight() {
  setMotor(-1, -1, -1, -1,
           MT_PWM_SPEED * setting.softTurnSpeed, MT_PWM_SPEED);
  Serial.println("Arah: Mundur Kanan");
  showStatus("↘ Mundur Kanan");
}

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

void setSpeed() {
  // Urutan naik: 65 → 130 → 195 → 255 → balik lagi ke 65
  if (MT_PWM_SPEED == 65) MT_PWM_SPEED = 130;
  else if (MT_PWM_SPEED == 130) MT_PWM_SPEED = 195;
  else if (MT_PWM_SPEED == 195) MT_PWM_SPEED = 255;
  else MT_PWM_SPEED = 65;

  ledcWrite(RM_PWM, MT_PWM_SPEED);
}


void saveSetting() {
  EEPROM.put(EEPROM_ADDR_HARD, setting.hardTurnSpeed);
  EEPROM.put(EEPROM_ADDR_SOFT, setting.softTurnSpeed);
  EEPROM.commit();  // WAJIB agar tersimpan
  Serial.println("Setting disimpan!");
}

void loadSetting() {
  EEPROM.get(EEPROM_ADDR_HARD, setting.hardTurnSpeed);
  EEPROM.get(EEPROM_ADDR_SOFT, setting.softTurnSpeed);  
  EEPROM.get(EEPROM_ADDR_BT_MODE, btMode);

  // validasi jika datanya aneh
  if (setting.hardTurnSpeed < 0.1 || setting.hardTurnSpeed > 0.3) setting.hardTurnSpeed = 0.2;
  if (setting.softTurnSpeed < 0.5 || setting.softTurnSpeed > 0.7) setting.softTurnSpeed = 0.6;  
  if (btMode != 0 && btMode != 1) btMode = 0;  // default ke host kalau aneh
 
  btMode = !btMode;
  EEPROM.put(EEPROM_ADDR_BT_MODE, btMode);
  EEPROM.commit();

  Serial.println("Setting dimuat:");
  Serial.println(setting.hardTurnSpeed);
  Serial.println(setting.softTurnSpeed);
}

void showStatus(String text) {
  // display.clearDisplay();
  // display.setTextSize(2);  // Ukuran teks (1~3)
  // display.setTextColor(SSD1306_WHITE);
  // display.setCursor(0, 0);
  // display.println(text);
  // display.display();
}
