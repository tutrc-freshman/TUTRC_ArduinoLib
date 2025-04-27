#pragma once

#include <cmath>

#include "C6x0.h"

class ThreeOmniWheels {
public:
  ThreeOmniWheels(C6x0Id wheel1 = C610_ID_1, C6x0Id wheel2 = C610_ID_2,
                  C6x0Id wheel3 = C610_ID_3)
      : wheel1_{wheel1}, wheel2_{wheel2}, wheel3_{wheel3} {}

  void output(float x, float y, float w, float kp = 100.0f) {
    c6x0_->update();

    auto [wheel1Rpm, wheel2Rpm, wheel3Rpm] = inverseKinematics(x, y);
    c6x0_->setCurrent(wheel1_,
                      kp * (wheel1Rpm + w - c6x0_->getRpm(wheel1_) / 36.0f));
    c6x0_->setCurrent(wheel2_,
                      kp * (wheel2Rpm + w - c6x0_->getRpm(wheel2_) / 36.0f));
    c6x0_->setCurrent(wheel3_,
                      kp * (wheel3Rpm + w - c6x0_->getRpm(wheel3_) / 36.0f));

    c6x0_->transmit();
  }

  void setC6x0(C6x0 *c6x0) { c6x0_ = c6x0; }

  void setTheta(float theta) { theta_ = theta; }

private:
  C6x0Id wheel1_;
  C6x0Id wheel2_;
  C6x0Id wheel3_;

  C6x0 *c6x0_;

  float theta_ = 0.0f;

  std::tuple<float, float, float> inverseKinematics(float x, float y) {
    return {
        x * std::cos(theta_) + y * std::sin(theta_),
        x * std::cos(PI * 2.0f / 3.0f + theta_) +
            y * std::sin(PI * 2.0f / 3.0f + theta_),
        x * std::cos(PI * 4.0f / 3.0f + theta_) +
            y * std::sin(PI * 4.0f / 3.0f + theta_),
    };
  }
};