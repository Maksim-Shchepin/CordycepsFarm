#pragma once
#include <Arduino.h>

class Timer {

  private:
    unsigned long time;
    unsigned long period;
    boolean flag;
    boolean doneFlag;
    boolean enabled;
    int intValue;
    float floatValue;
    String stringValue;

  public:

    Timer(unsigned long period = -1) {
      if (period != -1) { //period is defined
        this -> period = period;
        flag = false;
        enabled = true;
        return;
      }
      // period is undefined -> timer won't enabled & period set to 1000 by default
      this -> period = 1000;
      flag = false;
      enabled = false;
    }

    boolean triggered() {
      if (!enabled) {
        return false;
      }
      unsigned long currentTime = millis();
      if (flag) {
        flag = !flag;
        time = currentTime;
      }
      if ((currentTime - time >= period) && !flag) {
        flag = !flag;
        return flag;
      }
      if (currentTime < time) {
        time = currentTime;
      }
      return false;
    }

    void set(unsigned long period) {
      this -> period = period;
      time = millis();
      flag = false;
    }

    void enable(unsigned long period = -1) {
      if (period != -1) {
        set(period);
      }
      enabled = true;
    }

    void disable() {
      enabled = false;
      flag = false;
    }

    boolean isEnabled() {
      return enabled;
    }

    String setIntValue(int intValue) {
      this -> intValue = intValue;
      return "intValue is set to " + String(intValue);
    }

    String setFloatValue(float floatValue) {
      this -> floatValue = floatValue;
      return "floatValue is set to " + String(floatValue);
    }

    String setStringValue(String stringValue) {
      this -> stringValue = stringValue;
      return "stringValue is set to " + stringValue;
    }

    int getIntValue() {
      return intValue;
    }

    float getFloatValue() {
      return floatValue;
    }

    String getStringValue() {
      return stringValue;
    }

};