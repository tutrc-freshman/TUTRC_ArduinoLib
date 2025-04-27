#include <C6x0.h>
#include <ESP32_TWAI.h>

C6x0 c6x0;

void setup() {
  Serial.begin(115200);
  CAN.setRxQueueLen(50);
  c6x0.setCAN(&CAN);
  CAN.begin(CanBitRate::BR_1000k);
}

void loop() {
  c6x0.update();

  float kp = 100;
  float rps = c6x0.getRpm(C610_ID_1) / 60.0f;
  float rps_ref = 100;
  float current_ref = kp * (rps_ref - rps);

  c6x0.setCurrent(C610_ID_1, current_ref);

  c6x0.transmit();

  delay(1);
}