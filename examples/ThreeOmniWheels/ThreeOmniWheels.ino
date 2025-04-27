#include <Adafruit_BNO055.h>
#include <C6x0.h>
#include <ESP32_TWAI.h>
#include <Ps3Controller.h>
#include <ThreeOmniWheels.h>

C6x0 c6x0;
Adafruit_BNO055 bno;
ThreeOmniWheels threeOmniWheels;

void setup() {
  Serial.begin(115200);

  CAN.setRxQueueLen(50);
  c6x0.setCAN(&CAN);
  threeOmniWheels.setC6x0(&c6x0);

  CAN.begin(CanBitRate::BR_1000k);
  bno.begin();
  Ps3.begin();
}

void loop() {
  float x = Ps3.data.analog.stick.lx;
  if (abs(x) < 10.0f) {
    x = 0.0f;
  }

  float y = Ps3.data.analog.stick.ly;
  if (abs(y) < 10.0f) {
    y = 0.0f;
  }

  float w = 0.0f;
  if (Ps3.data.button.l1) {
    w -= 50.0f;
  }
  if (Ps3.data.button.r1) {
    w += 50.0f;
  }

  threeOmniWheels.output(x, y, w);

  float theta = bno.getVector(Adafruit_BNO055::VECTOR_EULER).x() * DEG_TO_RAD;
  Serial.println(theta);

  delay(10);
}