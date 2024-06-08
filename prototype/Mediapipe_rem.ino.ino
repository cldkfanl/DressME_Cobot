#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include <Servo.h>

int basePin = 4;
int shoulderPin = 5;
int upperarmPin = 6;
int forearmPin = 7;

const int servoInitAngle = 90;

int baseAngle = 0;
int shoulderAngle = 0;
int upperarmAngle = 0;
int forearmAngle = 0;

Servo base;
Servo shoulder;
Servo upperarm;
Servo forearm;

int baseStatus = 0;
int shoulderStatus = 0;
int upperarmStatus = 0;
int forearmStatus = 0;

int servoParallelControl(int thePos, Servo theServo, int speed) {
  int startPos = theServo.read();
  int newPos = startPos;

  if (startPos < (thePos)) {
    newPos = newPos + 1;
    theServo.write(newPos);
    delay(speed);
    return 0;
  } else if (newPos > (thePos)) {
    newPos = newPos - 1;
    theServo.write(newPos);
    delay(speed);
    return 0;
  } else {
    return 1;
  }
}


void setup() {
  Serial.begin(115200);

  base.attach(basePin);
  shoulder.attach(shoulderPin);
  upperarm.attach(upperarmPin);
  forearm.attach(forearmPin);

  base.write(servoInitAngle);
  shoulder.write(servoInitAngle);
  upperarm.write(servoInitAngle);
  forearm.write(servoInitAngle);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  // Clear the buffer
  Serial.println("oled found!");
  display.clearDisplay();
  display.display();
}

void loop() {
  int baseAngle, shoulderAngle, upperarmAngle, forearmAngle;

  while (Serial.available() > 0) {
    //10,30,40,50d
    //90,90,90,90d
    //45,75,75,75d

    baseAngle = Serial.parseInt();
    shoulderAngle = Serial.parseInt();
    upperarmAngle = Serial.parseInt();
    forearmAngle = Serial.parseInt();

    if (Serial.read() == 'd') {
      Serial.println("received signal");
    }

    String baseStr = " base     : " + String(baseAngle);
    String shoulderStr = " shoulder : " + String(shoulderAngle);
    String upperarmStr = " upperarm : " + String(upperarmAngle);
    String forearmStr = " forearm  : " + String(forearmAngle);

    display.clearDisplay();
    
    display.setTextSize(1);                              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  // Draw 'inverse' text

    display.setCursor(0, 0);  // Start at top-left corner
    display.println(" Robot Arm Move");
    display.setTextColor(SSD1306_WHITE);  // Draw white text
    display.println(" --------------");
    display.println(" ");
    display.setTextSize(1);  // Normal 1:1 pixel scale
    display.println(baseStr);
    display.println(shoulderStr);
    display.println(upperarmStr);
    display.println(forearmStr);
    display.println("\n");  // OLED 에 마지막 줄이 안 보여서 넣어준 것임

    display.setCursor(0, 0);  // Start at top-left corner
    display.display();

    int done = 0;

    while (done == 0) {
      baseStatus = servoParallelControl(baseAngle, base, 20);
      shoulderStatus = servoParallelControl(shoulderAngle, shoulder, 20);
      upperarmStatus = servoParallelControl(upperarmAngle, upperarm, 20);
      forearmStatus = servoParallelControl(forearmAngle, forearm, 20);

      if (baseStatus == 1 && shoulderStatus == 1 && upperarmStatus == 1 && forearmStatus == 1) {
        done = 1;
      }
    }
    delay(100);
  }
}