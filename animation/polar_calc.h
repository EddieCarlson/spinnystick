#pragma once

#include "../common.h"
#include "../strip.h"
#include "../angle.h"
#include <math.h> 

class Cardioid {
  public:
    double a;
    double b;
    double angle_add;
  Cardioid(double _a, double _b, double _angle_add) {
    a = _a;
    b = _b;
    angle_add = _angle_add;
  }
};

Cardioid c1(1, 3, 0);
Cardioid c2(1, 3, M_PI_2);
Cardioid c3(1, 3, M_PI);
Cardioid c4(1, 3, 3 * M_PI_2);

Cardioid cardioid_list[4] = {c1, c2, c3, c4};

double radius_at_theta(Cardioid c, double theta) {
  return c.a + (c.b * cos(theta + c.angle_add));
}

void displayCardioids() {
  displayCardioids(cardioid_list, 4);
}

void displayCardioids(Cardioid *cardioids, int num) {
  strip.clear();
  double angle = curAngle();
  for(int i = 0; i < num; i++) {
    int r = ceil(radius_at_theta(cardioids[i], angle) + ROPE_PIXELS - 10); // should really be ROPE_PIXELS?
    for(int px = r - 3; r < r + 4; r++) {
      if (px > 0 && px < COL_HEIGHT) {
        double factor = 1 - pow((abs(px - r) / 3.2), 1.5);
        strip.setPixelColor(px, ((uint8_t) (127.0 * factor)), 0, ((uint8_t) (255.0 * factor)));
      }
    }
  }
  strip.show();
}