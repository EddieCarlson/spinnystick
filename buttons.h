#pragma once

#include <arduino.h>
#include <Bounce2.h>
#include "sd_card.h"

const uint16_t debounceMillis = 25;

const uint8_t nextImagePinNext = 13;
const uint8_t prevImagePin = 14;
const uint8_t brightnessPin = 15;
const uint8_t periodPin = 16;

Bounce2::Button nextImageNextButton = Bounce2::Button();
Bounce2::Button prevImageButton = Bounce2::Button();
Bounce2::Button brightnessButton = Bounce2::Button();
Bounce2::Button periodButton = Bounce2::Button();

class ButtonAction {
  public:
    typedef void (*Action)();

    Bounce2::Button button;
    Action action;
    uint8_t pin;

    ButtonAction(Bounce2::Button b, uint8_t p, Action a) {
      button = b;
      pin = p;
      action = a;
    }

    void init() {
      button.attach(pin, INPUT_PULLUP);
      button.interval(debounceMillis);
      button.setPressedState(LOW);
    }

    void update() {
      button.update();
    }

    bool actIfPressed() {
      bool pressed = button.pressed();
      if (pressed) {
        action();
      }
      return pressed;
    }
};

ButtonAction buttonActions[2] = {
  ButtonAction(nextImageNextButton, nextImagePinNext, importNextImage),
  ButtonAction(prevImageButton, prevImagePin, importPrevImage)
};
  // prevImageButton, brightnessButton, periodButton};

void initButtonsNext() {
  for(int i = 0; i < 2; i++) {
    buttonActions[i].init();
  }
}

void checkButtonsNext() {
  for(int i = 0; i < 2; i++) {
    if (buttonActions[i].actIfPressed()) {
      for(int u = i; u < 2; u++) {
        buttonActions[u].button.setPressedState(false);
      }
      break;
    }
  }
}