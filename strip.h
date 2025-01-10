#pragma once

#include <Adafruit_DotStar.h>
#include "common.h"

#define DATAPIN    12
#define CLOCKPIN   13

Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);
