#pragma once
#include <Arduino.h>

enum MotionMode : uint8_t {
  MODE_RANDOM = 0,
  MODE_ORBIT  = 1,
  MODE_SCAN   = 2,
  MODE_SMART  = 3
};

enum PresetId : uint8_t {
  PRESET_CUSTOM = 0,
  PRESET_SOFT   = 1,
  PRESET_NORMAL = 2,
  PRESET_WILD   = 3,
  PRESET_SMART  = 4
};

static inline const char* presetName(uint8_t p){
  switch(p){
    case PRESET_SOFT:   return "soft";
    case PRESET_NORMAL: return "normal";
    case PRESET_WILD:   return "wild";
    case PRESET_SMART:  return "smart";
    default:            return "custom";
  }
}

struct TimeWindow {
  uint16_t startMin = 18 * 60;
  uint16_t stopMin  = 20 * 60;
  bool enabled      = false;
  uint8_t preset    = PRESET_CUSTOM;
};

struct DaySchedule2 {
  TimeWindow a;
  TimeWindow b;
  bool dayEnabled = true;
};

struct Config {
  int panMin = 35, panMax = 145;
  int tiltMin = 45, tiltMax = 135;

  int edgeMarginDeg = 6;

  int parkPan = 90, parkTilt = 90;

  float speedDegPerSec = 70.0f;

  uint16_t dwellMsMin = 150;
  uint16_t dwellMsMax = 900;

  MotionMode mode = MODE_RANDOM;

  bool laserEnabled = true;
  bool runEnabled = false;

  uint16_t sessionMaxMin = 10;  // 0 = unendlich
  uint16_t cooldownMin = 2;

  DaySchedule2 days[7];
};