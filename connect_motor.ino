#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int servoMin = 102;  // 調整最小脈衝寬度
int servoMax = 512;  // 可以試著再增加一些

void setup() {
  Serial.begin(9600);    // 初始化串口通信
  pwm.begin();
  pwm.setPWMFreq(50);    // 設定 PWM 頻率為 50Hz
}

int servoPulse(int angle) {
  return map(angle, 0, 180, servoMin, servoMax);  // 角度轉換為脈衝長度
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    if (commaIndex > 0) {
      int servo_num = data.substring(0, commaIndex).toInt();
      int angle = data.substring(commaIndex + 1).toInt();
      if (servo_num >= 0 && servo_num < 16 && angle >= 0 && angle <= 180) {
        pwm.setPWM(servo_num, 0, servoPulse(angle));  // 設定舵機角度
      }
    }
  }
}