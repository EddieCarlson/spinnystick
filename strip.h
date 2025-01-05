#pragma once
#include <Adafruit_DotStar.h>

#define NUMPIXELS 200

#define DATAPIN    12
#define CLOCKPIN   13

#define COL_HEIGHT 94
#define ROPE_PIXELS 45
#define NUM_RAYS 360

Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);
