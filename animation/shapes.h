#pragma once

#include "../common.h"
#include "../strip.h"
#include "../angle.h"
#include <math.h> 
#include "raydisplay.h"

double xFromPolar(int r, double theta) {
  return r * cos(theta);
}

double yFromPolar(int r, double theta) {
  return r * sin(theta);
}

// TODO: how to handle these things moving? all parameters are functions of t?
class Circle {
  public:
    double d; // distance from origin
    double theta; // angle from origin
    double radius;
    double thickness; // thickness in px of line
  
  void addToImage() {
  }
};

class Square {
  public:
    double centerX;
    double centerY;
    double size;

    // derived
    double halfSize;
    double highX;
    double highY;
    double lowX;
    double lowY;

    Square(double x, double y, double s) {
      centerX = x;
      centerY = y;
      size = s;
      halfSize = size / 2.0;
      highX = centerX + halfSize;
      highY = centerY + halfSize;
      lowX = centerX - halfSize;
      lowY = centerY - halfSize;
    }

    double distFromSquare(double x, double y) {
      double dx = max(max(lowX - x, 0), x - highX);
      double dy = max(max(lowY - y, 0), y - highY);
      return sqrt((dx * dx) + (dy * dy));
    }
    
    void addToImage(CRGB color) {
      // CRGB realColor = CRGB(color.r * brightness_factor, color.g * brightness_factor, color.b * brightness_factor);
      for(int ray = 0; ray < NUM_RAYS; ray++) {
        double rad = rayToRad(ray);
        for(int px = 0; px < COL_HEIGHT; px++) {
          double x = xFromPolar(ROPE_PIXELS + px, rad);
          double y = yFromPolar(ROPE_PIXELS + px, rad);
          double dist = distFromSquare(x, y);
          if (dist < 0.01) {
            Serial.print(x);
            Serial.print(" ");
            Serial.println(y);
            imageRays[ray][px] = color;
          } else if (dist < 1) {
            double factor = pow(1 - dist, 2);
            imageRays[ray][px] = CRGB((uint8_t) (color.r * factor), (uint8_t) (color.g * factor), (uint8_t) (color.b * factor));
          }
        }
      }
    }
};

// from +/- 26-122
void addSquares() {
  clearPixels();

  Square s1 = Square(50, 50, 10);

  s1.addToImage(CRGB::Crimson);
  // Square s2 = Square(70, 30, 5);
  // Square s3 = Square(0, 40, 8);
  // Square s4 = Square(-35, 60, 10);
  // Square s5 = Square(-95, 105, 7);
  // Square s11 = Square(70, -80, 10);
  // Square s6 = Square(50, -100, 6);
  // Square s7 = Square(35, -70, 12);
  // Square s8 = Square(-40, -50, 8);
  // Square s9 = Square(-97, -35, 3);
  // Square s10 = Square(80, 0, 15);
  // Square squares[11] = {s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11};

  // for(int i = 0; i < 11; i++) { 
  //   squares[i].addToImage(CRGB::Crimson);
  // }
}