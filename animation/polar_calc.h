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

void displayAtR(double rf) {
  if (rf > 0) {
    int r = ceil((rf * (COL_HEIGHT) / 4.0)); // should really be ROPE_PIXELS?
    for(int px = r - 3; px < r + 4; px++) {
      if (px > 0 && px < COL_HEIGHT) {
        double factor = 1 - pow((abs(px - r) / 3.2), 1.5);
        uint8_t r = ((uint8_t) (127.0 * factor));
        uint8_t b = ((uint8_t) (255.0 * factor));
        setPixels(px, r, 0, b);
      }
    }
  }
}

void displayCardioids(Cardioid *cardioids, int num) {
  clearPixels();
  double angle = curAngle();
  for(int i = 0; i < num; i++) {
    double rf = radius_at_theta(cardioids[i], angle);
    double rf2 = radius_at_theta(cardioids[i], angle + M_PI_2);
    displayAtR(rf);
    displayAtR(rf2);
  }
  displayPixels();
}

void displayCardioids() {
  displayCardioids(cardioid_list, 4);
}
