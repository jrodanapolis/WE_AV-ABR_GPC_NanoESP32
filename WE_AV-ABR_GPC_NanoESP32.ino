#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>
#include <Preferences.h>
#include <ArduinoBLE.h>
#include <lunarGateway.h>
#include <vector>
#include <string>
Preferences Preferences;
int potentiometer = 0;
int pumpDACValue = 0;
int lastPumpDACValue = 0;
float pressureStepSize = 0.0;
float lastPressure;
float vPres = 0;
float readPressure = 0.0;
float presPlot = 0.0;
float lastPresPlot = 220;
float presX = 24;
float flowRate_FM;
float flowRate_FM_Display;
float flowX_FM = 24;
float flowPlot_FM = 0.0;
float lastFlowPlot_FM = 220;
float flowRate_PT;
float flowX_PT = 24;
float flowPlot_PT = 0.0;
float lastFlowPlot_PT = 220;
float readLunar = 0.0;
float LunarPlot = 0.0;
float lastLunarPlot = 220;
float LunarX = 24;
float dripFactor = 2.0;
float plotStep = 0.17;
int autofillTimer = 0;
int autofillSpeed = 0;
int counter;
int sliderStart = 20;
int sliderEnd = 275;
int sliderY = 200;
int lastBrightnessX;
float DoseSize = 18;
float DoseSizeInPreferences;
int LastXTouched;
int LastYTouched;
unsigned long timeDisplay;
unsigned long startTime = 0;
int ProfileInPreferences;
int BrightnessPreferences;
int SelectedBrightness = 255;

// Scale variables
#define SCALE_SAMPLERATE_DELAY_MS 100  //how often to read data from the scale
bool ble_active;
lunarGateway lunar;
bool found_lunar;
bool lunar_active;
bool lunar_session_init;
int lunar_state;
bool lunar_start_timer;
unsigned long scale_read_last_time;
bool initSlider = true;
int ShotCounter = 0;
int ShotCounterInPreferences;
bool weightStopper = false;

// Variables for Gicar flow meter ("FM" throughout code)
// Tuning variables: PulsesPerRev = 2; ZeroTimeout = 1,800,000; Constrain = 1,500,000 and 300,000
const byte PulsesPerRevolution_FM = 2;
const unsigned long ZeroTimeout_FM = 1800000;
const byte numReadings_FM = 2;
volatile unsigned long LastTimeWeMeasured_FM;
volatile unsigned long PeriodBetweenPulses_FM = ZeroTimeout_FM + 1000;
volatile unsigned long PeriodAverage_FM = ZeroTimeout_FM + 1000;
unsigned long FrequencyRaw_FM;
unsigned long FrequencyReal_FM;
unsigned long RPM_FM;
unsigned int PulseCounter_FM = 1;
unsigned long PeriodSum_FM = 1000;
unsigned long LastTimeCycleMeasure_FM = LastTimeWeMeasured_FM;
unsigned long CurrentMicros_FM = micros();
unsigned int AmountOfReadings_FM = 1;
unsigned int ZeroDebouncingExtra_FM;
unsigned long readings_FM[numReadings_FM];
unsigned long readIndex_FM;
unsigned long total_FM;
unsigned long average_FM = 0;

// Variables for pump tachometer ("PT" througout code)
// Tuning variables: PulsesPerRev = 32; zeroTimeout = 100,000; constrain values = 40,000 and 5,000
const byte PulsesPerRevolution_PT = 32;
const unsigned long ZeroTimeout_PT = 100000;
const byte numReadings_PT = 2;
volatile unsigned long LastTimeWeMeasured_PT;
volatile unsigned long PeriodBetweenPulses_PT = ZeroTimeout_PT + 1000;
volatile unsigned long PeriodAverage_PT = ZeroTimeout_PT + 1000;
unsigned long FrequencyRaw_PT;
unsigned long FrequencyReal_PT;
unsigned long RPM_PT;
unsigned int PulseCounter_PT = 1;
unsigned long PeriodSum_PT = 10000;
unsigned long LastTimeCycleMeasure_PT = LastTimeWeMeasured_PT;
unsigned long CurrentMicros_PT = micros();
unsigned int AmountOfReadings_PT = 2;
unsigned int ZeroDebouncingExtra_PT;
unsigned long readings_PT[numReadings_PT];
unsigned long readIndex_PT;
unsigned long total_PT;
unsigned long average_PT = 0;
unsigned long PT_kilo = 0;
unsigned long duty_PT = 0;

// Shot timer variables
int phaseTimer = 0;
unsigned long lastPhaseTimeMillis = 0.0;
unsigned long testingTimer = 0.0;
unsigned long lasttestTimer = 0.0;
uint8_t shotTimer;
uint8_t lastShotTimer = 0;
uint8_t shotStartSeconds;
uint8_t currentTimeSeconds;
unsigned long ellapsedShotTime;
unsigned long shotStart;
unsigned long currentTime;
unsigned long seconds = 0;
bool timerRunning;

// pin definitions
#define FLOWMETERPIN A0
#define SWITCHPIN A1
#define POTENTIOMETERPIN A2
#define PRESSUREPIN A3
#define AUTOFILLSPEEDPIN A6
#define BREWSPEEDPIN A7
#define BREWSOLENOIDPIN 2
#define AUTOFILLSOLENOIDPIN 3
#define PUMPTACHPIN 4
#define BRIGHTNESSPIN 5
#define SDCSPIN 6 // For the screen SD card, not used yet
#define OEMAUTOFILLREADPIN 7
#define OEMBREWREADPIN 8
#define TFT_CS 9
#define TFT_DC 10

// Wired screen setup
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
Adafruit_FT6206 ts = Adafruit_FT6206();
const uint16_t BACKGROUND_COLOR = tft.color565(0, 0, 0);
const uint16_t PRESSURE_COLOR = tft.color565(219, 208, 83);
const uint16_t FLOW_COLOR = tft.color565(99, 105, 209);
const uint16_t PUMP_COLOR = tft.color565(79, 201, 120);
const uint16_t TIME_COLOR = tft.color565(238, 99, 82);
const uint16_t RPM_COLOR = tft.color565(230, 101, 45);
const uint16_t PHASE_COLOR = tft.color565(244, 187, 255);
const uint16_t BUTTON_COLOR = tft.color565(27, 45, 42);
const uint16_t BUTTON_OUTLINE_COLOR = tft.color565(247, 247, 255);
const uint16_t TEXT_COLOR = tft.color565(247, 247, 255);
const uint16_t GRID_COLOR = tft.color565(72, 72, 72);

// Profile selection and profile button variables
#define NUMBER_OF_BUTTONS 9
Adafruit_GFX_Button buttons[NUMBER_OF_BUTTONS];
#define PROFILES_NAV 0
#define SETTINGS_NAV 1
#define HOME_NAV 2
Adafruit_GFX_Button NavButtons[3];
Adafruit_GFX_Button UpDown[2];
#define EMP 0
#define THREE69 1
#define SLAYER6 2
#define SLAYER9 3
#define WEPRO2 4
#define WEPRO3 5
#define LEVAX 6
#define FLAT6 7
#define SELECT 8
int selectedProfile = EMP;
int lastSelectedProfile = EMP;
const int NumberOfProfiles = 8;
int lastPaddleState = -1;
enum ProfileMode {
  Interactive,
  Automatic,
  Manual
};
enum ProfileState {
  Running,
  Stopped
};
struct ProfilePhase {
  String name;
  float pressureTarget;
  float flowTarget;
  float weightTarget;
  int timeTarget;
  int rampInSecs;
  bool pressureExitCriteria;
  bool flowExitCriteria;
  bool timeExitCriteria;
  bool weightExitCriteria;
  bool paddleStateExitCriteria;
};

struct ProfileType {
  char name[24];
  int buttonX;
  int buttonY;
  int buttonW;
  int buttonH;

  ProfileMode mode;
  std::vector<ProfilePhase> profilePhases;
  int currentProfilePhase; // = 0;
  ProfileState profileState; // = Stopped;
};
ProfileType Profiles[NumberOfProfiles] = {
  { "EMP", 40, 30, 76, 56, Manual, { { " Manual", 0.0, 0.0, 0.0, 0, 0, false, false, false, false, false } } },
  { "3-6-9", 120, 30, 76, 56, Interactive, { { "3 Bar", 3.0, 0.0, 0.0, 0, 0, false, false, false, false, true }, { "6 Bar", 6.0, 0.0, 0.0, 0, 0, false, false, false, false, true }, { "9 Bar", 9.0, 0.0, 0.0, 0, 0, false, false, false, false, true } } },
  { "Slayer 6", 40, 90, 76, 56, Interactive, { { "  PI  ", 3.0, 0.0, 0.0, 0, 0, false, false, false, false, true }, { "6 Bar", 6.0, 0.0, 0.0, 0, 0, false, false, false, false, true }, { "6 Bar", 6.0, 0.0, 0.0, 0, 0, false, false, false, false, true } } },
  { "Slayer 9", 120, 90, 76, 56, Interactive, { { "  PI  ", 3.0, 0.0, 0.0, 0, 0, false, false, false, false, true }, { "9 Bar", 9.0, 0.0, 0.0, 0, 0, false, false, false, false, true }, { "9 Bar", 9.0, 0.0, 0.0, 0, 0, false, false, false, false, true } } },
  { "WE Pro 2:1", 40, 150, 76, 56, Automatic, { { "Start", 2.5, 0, 40.0, 2, 0, false, false, true, true, false }, { "PI 1/2", 4.5, 0.0, 40.0, 12, 6, false, false, true, true, false }, { "PI 2/2", 2.5, 0.0, 40.0, 15, 10, false, false, true, true, false }, { "9 Bar", 8, 0.0, 40.0, 15, 3, false, false, true, true, false }, { "Decrease", 6.5, 0.0, 40.0, 15, 15, false, false, true, true, false }, { " Hold ", 6.5, 0.0, 40.0, 30, 30, false, false, true, true, false } } },
  { "WE Pro 3:1", 120, 150, 76, 56, Automatic, { { "Start", 2.5, 0, 50.0, 2, 0, false, false, true, true, false }, { "PI 1/2", 4.5, 0.0, 50.0, 12, 6, false, false, true, true, false }, { "PI 2/2", 2.5, 0.0, 50.0, 15, 10, false, false, true, true, false }, { "9 Bar", 8, 0.0, 50.0, 15, 3, false, false, true, true, false }, { "Decrease", 6.5, 0.0, 50.0, 15, 15, false, false, true, true, false }, { " Hold ", 6.5, 0.0, 50.0, 30, 30, false, false, true, true, false } } },
  { "Leva X", 40, 210, 76, 56, Automatic, { { "PI", 3.0, 0.0, 0.0, 10, 1, false, false, true, false, false }, { "Ramp", 9.0, 0.0, 0.0, 2, 1, true, false, true, false, false }, { "Decrease", 4.0, 0.0, 0.0, 40, 40, false, false, true, false, false }, { "Finishing", 4.0, 0.0, 0.0, 8, 3, false, false, true, false, false } } },
  { "Flat 6", 120, 210, 76, 56, Automatic, { { "PI", 3.0, 0.0, 0.0, 15, 1, false, false, true, false, false }, { "Ramp", 6.0, 0.0, 0.0, 3, 0, false, false, true, false, false }, { "6 Bar", 6.0, 0.0, 0.0, 42, 0, false, false, true, false, false } } },
};

// converting target pressure into a DAC value for the profiles... the "pressures" current set in the profiles above won't go off of read pressure, but rather target pressure
// this would be a great place to work in a pressure PID/feedback loop for true pressure profiling instead of targeted pressure profiling
int pressureToDACConvertion(float pressure) {
  float dac = -1.889 * pow(pressure, 2) + (37.667 * pressure) - 46;
  if (dac < 0) {
    dac = 0;
  }
  if (dac > 150) {

    dac = 150;
  }
  return dac;
}
float dacToPressureConvertion(int dacValue) {
  return 0.0007 * pow(dacValue, 2) - 0.0574 * dacValue + 4.2366;
}
bool isCloseEnough(float targetPressure, float currentPressure) {
  float acceptablePercentage = 0.1;
  float difference = abs(targetPressure - currentPressure);
  float acceptableDifference = (acceptablePercentage / 100.0) * targetPressure;
  return difference <= acceptableDifference;
}
int calculatePumpDACValue(ProfileType profile) {
  if (profile.mode == Manual) {
    return UpdateManual();
  }
  float thisPhasePressure = profile.profilePhases[profile.currentProfilePhase].pressureTarget;
  float lastPhasePressure;
  if (profile.currentProfilePhase > 0) {
    lastPhasePressure = profile.profilePhases[profile.currentProfilePhase - 1].pressureTarget;
  } else {
    lastPhasePressure = 0.0;
  }
  if (profile.profilePhases[profile.currentProfilePhase].rampInSecs > 0 && phaseTimer < profile.profilePhases[profile.currentProfilePhase].rampInSecs) {
    pressureStepSize = calculateStepSize(lastPhasePressure, thisPhasePressure,
                                         profile.profilePhases[profile.currentProfilePhase].rampInSecs);
    //Serial.print("StepSize: ");
    //Serial.println(pressureStepSize);
    // Serial.print(lastPhasePressure);
    // Serial.print(",");
    // Serial.print(pressureStepSize);
    // Serial.print(",");
    // Serial.println(thisPhasePressure);
    // Serial.print(",");
    // Serial.println(lastPressure);
    if (!isCloseEnough(thisPhasePressure, lastPressure)) {
      // Serial.print(lastPhasePressure);
      // Serial.print(",");
      // Serial.print(pressureStepSize);
      // Serial.print(",");
      // Serial.println(thisPhasePressure);
      // Serial.print(",");
      // Serial.println(lastPressure);
      if (thisPhasePressure > lastPhasePressure && lastPressure + pressureStepSize > thisPhasePressure) {
        lastPressure = thisPhasePressure;
        return pressureToDACConvertion(thisPhasePressure);
      } else if (thisPhasePressure < lastPhasePressure && lastPressure + pressureStepSize < thisPhasePressure) {
        lastPressure = thisPhasePressure;
        return pressureToDACConvertion(thisPhasePressure);
      } else {
        lastPressure += pressureStepSize;
        return pressureToDACConvertion(lastPressure);
      }
    }
  }
  lastPressure = thisPhasePressure;
  return pressureToDACConvertion(thisPhasePressure);
}

// Reading the state of the paddle and potentiometer... different zones for Slayer and 3-6-9 modes. EMP calculated elsewhere.
int getPaddleState() {
  if (Profiles[selectedProfile].mode != Interactive) {
    return -1;
  }
  if (analogRead(POTENTIOMETERPIN) >= 2203) {
    return 0;
  }
  if ((analogRead(POTENTIOMETERPIN) < 2203) && (analogRead(POTENTIOMETERPIN) >= 1937)) {
    return 1;
  }
  if ((analogRead(POTENTIOMETERPIN) < 1937) && (analogRead(POTENTIOMETERPIN) >= 1489)) {
    return 2;
  }
  if (analogRead(POTENTIOMETERPIN) <= 1489) {
    return 3;
  }
}

// Phase name printout while profiling a shot
void PrintPhase() {
  ProfileType profile = Profiles[selectedProfile];
  tft.setTextSize(2);
  if (profile.currentProfilePhase > 0 && profile.mode) {
    tft.setCursor(getCenteredX(profile.profilePhases[profile.currentProfilePhase - 1].name), 30);
    tft.setTextColor(BACKGROUND_COLOR, BACKGROUND_COLOR);
    tft.println(profile.profilePhases[profile.currentProfilePhase - 1].name);

    tft.setCursor(getCenteredX(profile.profilePhases[profile.currentProfilePhase].name), 30);
    tft.setTextColor(PHASE_COLOR, BACKGROUND_COLOR);
    tft.println(profile.profilePhases[profile.currentProfilePhase].name);
  } else {
    tft.setCursor(getCenteredX("            "), 30);
    tft.setTextColor(BACKGROUND_COLOR, BACKGROUND_COLOR);
    tft.println("            ");
    tft.setCursor(getCenteredX(profile.profilePhases[profile.currentProfilePhase].name), 30);
    tft.setTextColor(PHASE_COLOR, BACKGROUND_COLOR);
    tft.println(profile.profilePhases[profile.currentProfilePhase].name);
  }
}

// Simple void to update pump speed
void UpdatePump() {
  analogWrite(BREWSPEEDPIN, pumpDACValue);
  lastPumpDACValue = pumpDACValue;
}


void setNextProfilePhase() {
  ProfileType &profile = Profiles[selectedProfile];
  pressureStepSize = 0.0;
  lastPressure = profile.profilePhases[profile.currentProfilePhase].pressureTarget;
  //tft.setTextSize(2);
  //tft.setCursor(110, 35);
  //tft.setTextColor(PHASE_COLOR, BACKGROUND_COLOR);
  //tft.print(" ");

  if (profile.mode != Manual) {
    tft.setTextSize(1);
    tft.drawFastVLine(presX, 120, 8, PHASE_COLOR);
    tft.drawFastVLine(presX - 1, 120, 6, PHASE_COLOR);
    tft.drawFastVLine(presX + 1, 120, 6, PHASE_COLOR);
    tft.drawFastVLine(presX - 2, 120, 4, PHASE_COLOR);
    tft.drawFastVLine(presX + 2, 120, 4, PHASE_COLOR);
  }



  if (profile.mode == Automatic) {
    if ((profile.currentProfilePhase + 1) < static_cast<int>(profile.profilePhases.size())) {
      profile.currentProfilePhase++;
    }
  }
  if (profile.mode == Interactive) {
    profile.currentProfilePhase = getPaddleState();
    lastPaddleState = profile.currentProfilePhase;
  }
}

//Check if the profile phase has reached the point where you want it to move to the next phase
bool CheckExitCriteria() {
  bool exit = false;
  ProfileType &profile = Profiles[selectedProfile];
  ProfilePhase &phase = profile.profilePhases[profile.currentProfilePhase];
  if (phase.timeExitCriteria) {
    phaseTimer = getCurrentPhaseTime();
    if (phaseTimer >= phase.timeTarget) {
      exit = true;
    }
  }
  if (phase.paddleStateExitCriteria) {
    if (lastPaddleState != getPaddleState()) {
      exit = true;
    }
  }
  if (phase.pressureExitCriteria) {
    if (isCloseEnough(phase.pressureTarget, readPressure)) {
      exit = true;
    }
  }
  if (phase.weightExitCriteria) {
    if ((readLunar + dripFactor) >= phase.weightTarget) {
      exit = true;
      weightStopper = true;
      Serial.println("weightStopper TRUE line 370");
      Profiles[selectedProfile].profileState = Stopped;
      pumpDACValue = 0;
      analogWrite(BREWSPEEDPIN, pumpDACValue);
    }
  }
  return exit;
}
float calculateStepSize(float previousPressureValue, float newPressureValue, int transitionTime) {
  float stepsPerSecond = 10.4;
  float stepSize = 15;
  if (transitionTime <= 0) {
    stepSize = newPressureValue - previousPressureValue;
    return stepSize;
  }
  stepSize = ((newPressureValue - previousPressureValue) * 1.0) / (transitionTime * stepsPerSecond);
  return stepSize;
}
void RunPhase() {
  ProfileType &profile = Profiles[selectedProfile];
  if (!CheckExitCriteria()) {
    //PrintPhase();
    pumpDACValue = calculatePumpDACValue(profile);
    UpdatePump();
  } else {
    resetPhaseTimer();
    setNextProfilePhase();
    PrintPhase();
  }
}
int NumberofScreens = 3;
#define HOME 0
#define PROFILES 1
#define SETTINGS 2
int SelectedScreen = HOME;
void DrawProfilesScreen() {
  SelectedScreen = PROFILES;
  ClearScreen();
  DrawProfiles();
}
void DrawHomeScreen() {
  SelectedScreen = HOME;
  ClearScreen();
  DrawPlot();
}
void DrawSettingsScreen() {
  SelectedScreen = SETTINGS;
  ClearScreen();
  DrawNavButtons();
  UpdateDoseSize();
  UpDown[0].drawButton();
  UpDown[1].drawButton();
  InitBrightness();
}

uint16_t cacluateRainbow(int progress) {
  int colorCount = 7;
  float colorInterval = 100 / (colorCount - 1);
  float segment = progress / colorInterval;

  int r, g, b;

  if (segment <= 1.0) {
    r = 255;
    g = segment * 255;
    b = 0;
  } else if (segment <= 2.0) {
    r = 255 - ((segment - 1.0) * 255);
    g = 255;
    b = 0;
  } else if (segment <= 3.0) {
    r = 0;
    g = 255;
    b = (segment - 2.0) * 255;
  } else if (segment <= 4.0) {
    r = 0;
    g = 255 - ((segment - 3.0) * 255);
    b = 255;
  } else if (segment <= 5.0) {
    r = (segment - 4.0) * 255;
    g = 0;
    b = 255;
  } else if (segment <= 6.0) {
    r = 255;
    g = 0;
    b = 255 - ((segment - 5.0) * 255);
  } else {
    r = 255;
    g = 0;
    b = 0;
  }

  return tft.color565(r, g, b);
}

void DrawBootScreen() {
  ClearScreen();
  tft.setTextColor(PUMP_COLOR, BACKGROUND_COLOR);
  tft.setTextSize(3);
  tft.setCursor(getCenteredX("Witt's End Coffee"), 100);
  tft.print("Witt's End Coffee");
  for (int progress = 1; progress < 100; progress++) {
    delayMicroseconds(4000);
    tft.drawRect(110, 180, progress, 10, cacluateRainbow(progress));
  }
  delay(500);
}
void ClearScreen() {
  tft.fillScreen(BACKGROUND_COLOR);
}
void DrawNavButtons() {
  if (SelectedScreen == HOME) {
    NavButtons[PROFILES_NAV].drawButton(false);
    NavButtons[SETTINGS_NAV].drawButton(false);
  }
  if (SelectedScreen == SETTINGS) {
    NavButtons[HOME_NAV].drawButton(false);
  }
}
void PrintSelectedProfile() {
  tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
  tft.setTextSize(2);
  tft.setCursor(getCenteredX(Profiles[selectedProfile].name), 5);
  tft.print(Profiles[selectedProfile].name);
}
void DrawPlot() {
  DrawNavButtons();
  PrintSelectedProfile();
  tft.drawLine(22, 117, 22, 220, TEXT_COLOR);  // plot border... 3x pixel wide, thicker lines
  tft.drawLine(23, 117, 23, 220, TEXT_COLOR);
  tft.drawLine(24, 117, 24, 220, TEXT_COLOR);
  tft.drawLine(22, 219, 298, 219, TEXT_COLOR);
  tft.drawLine(22, 220, 298, 220, TEXT_COLOR);
  tft.drawLine(22, 221, 298, 221, TEXT_COLOR);
  tft.drawLine(296, 117, 296, 220, TEXT_COLOR);
  tft.drawLine(297, 117, 297, 220, TEXT_COLOR);
  tft.drawLine(298, 117, 298, 220, TEXT_COLOR);

  tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);  //X axis scale - time (sec)
  tft.setTextSize(1);
  tft.setCursor(23, 225);
  tft.print("0");
  tft.setCursor(50, 225);
  tft.print("10");
  tft.setCursor(80, 225);
  tft.print("20");
  tft.setCursor(110, 225);
  tft.print("30");
  tft.setCursor(140, 225);
  tft.print("40");
  tft.setCursor(170, 225);
  tft.print("50");
  tft.setCursor(200, 225);
  tft.print("60");
  tft.setCursor(230, 225);
  tft.print("70");
  tft.setCursor(260, 225);
  tft.print("80");
  tft.setCursor(290, 225);
  tft.print("90");

  tft.drawLine(15, 130, 25, 130, TEXT_COLOR);   // Longer dash for 9 bar indicator
  tft.drawLine(25, 120, 295, 120, GRID_COLOR);  // Horizontal lines for pressure/flow grid
  tft.drawLine(25, 130, 295, 130, GRID_COLOR);
  tft.drawLine(25, 140, 295, 140, GRID_COLOR);
  tft.drawLine(25, 150, 295, 150, GRID_COLOR);
  tft.drawLine(25, 160, 295, 160, GRID_COLOR);
  tft.drawLine(25, 170, 295, 170, GRID_COLOR);
  tft.drawLine(25, 180, 295, 180, GRID_COLOR);
  tft.drawLine(25, 190, 295, 190, GRID_COLOR);
  tft.drawLine(25, 200, 295, 200, GRID_COLOR);
  tft.drawLine(25, 210, 295, 210, GRID_COLOR);
  tft.drawLine(55, 118, 55, 219, GRID_COLOR);  // vertical lines for pressure/flow grid
  tft.drawLine(85, 118, 85, 219, GRID_COLOR);
  tft.drawLine(115, 118, 115, 219, GRID_COLOR);
  tft.drawLine(145, 118, 145, 219, GRID_COLOR);
  tft.drawLine(175, 118, 175, 219, GRID_COLOR);
  tft.drawLine(205, 118, 205, 219, GRID_COLOR);
  tft.drawLine(235, 118, 235, 219, GRID_COLOR);
  tft.drawLine(265, 118, 265, 219, GRID_COLOR);

  tft.drawLine(15, 120, 21, 120, TEXT_COLOR);  // Unit indicator dashes for pressure side
  tft.setTextColor(PRESSURE_COLOR, BACKGROUND_COLOR);
  tft.setTextSize(1);
  tft.setCursor(3, 117);
  tft.print("10");
  tft.drawLine(15, 130, 21, 130, TEXT_COLOR);
  tft.setCursor(8, 127);
  tft.print("9");
  tft.drawLine(15, 140, 21, 140, TEXT_COLOR);
  tft.setCursor(8, 137);
  tft.print("8");
  tft.drawLine(15, 150, 21, 150, TEXT_COLOR);
  tft.setCursor(8, 147);
  tft.print("7");
  tft.drawLine(15, 160, 21, 160, TEXT_COLOR);
  tft.setCursor(8, 157);
  tft.print("6");
  tft.drawLine(15, 170, 21, 170, TEXT_COLOR);
  tft.setCursor(8, 167);
  tft.print("5");
  tft.drawLine(15, 180, 21, 180, TEXT_COLOR);
  tft.setCursor(8, 177);
  tft.print("4");
  tft.drawLine(15, 190, 21, 190, TEXT_COLOR);
  tft.setCursor(8, 187);
  tft.print("3");
  tft.drawLine(15, 200, 21, 200, TEXT_COLOR);
  tft.setCursor(8, 197);
  tft.print("2");
  tft.drawLine(15, 210, 21, 210, TEXT_COLOR);
  tft.setCursor(8, 207);
  tft.print("1");
  tft.drawLine(15, 220, 21, 220, TEXT_COLOR);
  tft.setCursor(8, 217);
  tft.print("0");

  tft.drawLine(299, 120, 305, 120, TEXT_COLOR);  // Unit indicator dashes and Y axis scale for weight
  tft.setTextColor(PUMP_COLOR, BACKGROUND_COLOR);
  tft.setTextSize(1);
  tft.setCursor(306, 121);
  tft.print("50");
  tft.drawLine(299, 140, 305, 140, TEXT_COLOR);
  tft.setCursor(306, 141);
  tft.print("40");
  tft.drawLine(299, 160, 305, 160, TEXT_COLOR);
  tft.setCursor(306, 161);
  tft.print("30");
  tft.drawLine(299, 180, 305, 180, TEXT_COLOR);
  tft.setCursor(306, 181);
  tft.print("20");
  tft.drawLine(299, 200, 305, 200, TEXT_COLOR);
  tft.setCursor(306, 201);
  tft.print("10");
  tft.drawLine(299, 220, 305, 220, TEXT_COLOR);

  tft.setTextColor(FLOW_COLOR, BACKGROUND_COLOR);  //Y axis scale for flow
  tft.setCursor(306, 112);
  tft.print("10");
  tft.setCursor(309, 132);
  tft.print("8");
  tft.setCursor(309, 152);
  tft.print("6");
  tft.setCursor(309, 172);
  tft.print("4");
  tft.setCursor(309, 192);
  tft.print("2");
  tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
  tft.setCursor(308, 217);
  tft.print("0");

  // tft.drawRoundRect(8, 55, 64, 50, 4, TIME_COLOR);
  tft.setTextSize(1);
  tft.setTextColor(TIME_COLOR, BACKGROUND_COLOR);
  tft.setCursor(5, 55);
  tft.print("Time: ");
  tft.setCursor(8, 90);
  tft.print("sec");
  tft.setTextColor(PRESSURE_COLOR, BACKGROUND_COLOR);
  tft.setCursor(50, 55);
  tft.print("Pressure: ");
  tft.setCursor(65, 90);
  tft.print("Bar");
  tft.setTextSize(1);
  tft.setCursor(30, 110);
  tft.print("Pres.");
  tft.setTextColor(FLOW_COLOR, BACKGROUND_COLOR);
  tft.setCursor(138, 55);
  tft.print("Flow: ");
  tft.setCursor(138, 90);
  tft.print("mL/s");
  tft.setCursor(235, 110);
  tft.print("Flo /");
  tft.setTextColor(PUMP_COLOR, BACKGROUND_COLOR);
  tft.setCursor(200, 55);
  tft.print("Weight: ");
  tft.setCursor(205, 90);
  tft.print("grams");
  tft.setTextSize(1);
  tft.setCursor(275, 110);
  tft.print("Wt.");
  tft.setTextColor(RPM_COLOR, BACKGROUND_COLOR);
  tft.setCursor(280, 55);
  tft.print("Pump: ");
  tft.setCursor(280, 90);
  tft.print("mL/s");
}
void DrawProfiles() {
  for (uint8_t i = 0; i < NumberOfProfiles + 1; i++) {
    if (i == selectedProfile) {
      buttons[i].drawButton(true);
      lastSelectedProfile = selectedProfile;
    } else {
      buttons[i].drawButton(false);
    }
  }
}
void InitButtons() {
  for (uint8_t i = 0; i < NumberOfProfiles; i++) {
    buttons[i].initButton(&tft, Profiles[i].buttonX, Profiles[i].buttonY, Profiles[i].buttonW,

                          Profiles[i].buttonH, BUTTON_OUTLINE_COLOR,

                          BUTTON_COLOR, TEXT_COLOR, Profiles[i].name, 1);
  }
  buttons[SELECT].initButton(&tft, 240, 210, 150, 50, BUTTON_OUTLINE_COLOR,

                             BUTTON_COLOR, TEXT_COLOR, "Select", 1);

  NavButtons[PROFILES_NAV].initButton(&tft, 35, 25, 60, 40, BUTTON_OUTLINE_COLOR,

                                      BUTTON_COLOR, TEXT_COLOR, "Profiles", 1);

  NavButtons[SETTINGS_NAV].initButton(&tft, 290, 25, 60, 40, BUTTON_OUTLINE_COLOR,

                                      BUTTON_COLOR, TEXT_COLOR, "Settings", 1);

  NavButtons[HOME_NAV].initButton(&tft, 35, 25, 60, 40, BUTTON_OUTLINE_COLOR,

                                  BUTTON_COLOR, TEXT_COLOR, "Home", 1);
  UpDown[0].initButton(&tft, 225, 115, 25, 25, BUTTON_OUTLINE_COLOR, BUTTON_COLOR, TEXT_COLOR, "+", 1);
  UpDown[1].initButton(&tft, 225, 145, 25, 25, BUTTON_OUTLINE_COLOR, BUTTON_COLOR, TEXT_COLOR, "-", 1);
}
int UpdateManual() {
  potentiometer = analogRead(POTENTIOMETERPIN);
  Serial.print(potentiometer);
  if (potentiometer >= 1600) {
    return (-0.25 * potentiometer) + 580;
  } else {
    return 0;
  }
}
unsigned long getTimeSinceInit() {
  return millis() - startTime;
}
unsigned long getTestingTimer() {
  return millis() - testingTimer;
}
int getCurrentPhaseTime() {
  return (millis() - lastPhaseTimeMillis) / 1000;
  ;
}
void resetPhaseTimer() {
  lastPhaseTimeMillis = millis();

  phaseTimer = 0;
  lastPressure = 0.0;
}
void UpdateStartupTimer() {
  tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
  tft.setTextSize(2);
  tft.setCursor(20, 80);
  unsigned long currentMillis = getTimeSinceInit();
  unsigned long seconds = currentMillis / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  unsigned long days = hours / 24;
  currentMillis %= 1000;
  seconds %= 60;
  minutes %= 60;
  hours %= 24;
  tft.print(String("Up Time: ") + String(" ") + String(hours) + String(":") + String(minutes) + String(":") + String(seconds) + String(" "));
}
void UpdateShotCounter() {
  tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
  tft.setTextSize(2);
  tft.setCursor(20, 60);
  tft.print(String("Shot Count: ") + String(ShotCounter));
}

void UpdateDoseSize() {
  tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
  tft.setTextSize(2);
  tft.setCursor(20, 120);
  tft.print(String("Dose Size: ") + String(DoseSize));
}

void InitBrightness() {
  tft.fillCircle(lastBrightnessX, 200, 15, BACKGROUND_COLOR);
  tft.drawFastHLine(sliderStart, sliderY, 255, TEXT_COLOR);
  tft.fillCircle(SelectedBrightness, 200, 15, BUTTON_COLOR);
  tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
  tft.setTextSize(2);
  tft.setCursor(20, 165);
  tft.print("Brightness");
  lastBrightnessX = SelectedBrightness;
  analogWrite(BRIGHTNESSPIN, SelectedBrightness);
}

void UpdateBrightness(int newXValue) {
  if (newXValue != (SelectedBrightness + sliderStart)) {
    if (newXValue < (sliderStart) + 30) {
      newXValue = sliderStart + 30;  // don't want the screen to get too dim
    }
    if (newXValue > sliderEnd) {
      newXValue = sliderEnd;
    }
    tft.fillRect(sliderStart - 15, 200 - 15, sliderEnd - sliderStart + 15, 40, BACKGROUND_COLOR);
    tft.fillCircle(lastBrightnessX, 200, 15, BACKGROUND_COLOR);
    tft.drawFastHLine(sliderStart, sliderY, 255, TEXT_COLOR);
    tft.fillCircle(newXValue, 200, 15, BUTTON_COLOR);
    tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);

    SelectedBrightness = newXValue - sliderStart;
    lastBrightnessX = SelectedBrightness;
    analogWrite(BRIGHTNESSPIN, SelectedBrightness);
  }
}

void Pulse_Event_FM()  // Flow Meter interrupt function
{
  PeriodBetweenPulses_FM = micros() - LastTimeWeMeasured_FM;
  LastTimeWeMeasured_FM = micros();
  if (PulseCounter_FM >= AmountOfReadings_FM) {
    PeriodAverage_FM = PeriodSum_FM / AmountOfReadings_FM;
    PulseCounter_FM = 1;
    PeriodSum_FM = PeriodBetweenPulses_FM;
    int RemapedAmountOfReadings_FM = map(PeriodBetweenPulses_FM, 1500000, 300000, 1,
                                         10);
    RemapedAmountOfReadings_FM = constrain(RemapedAmountOfReadings_FM, 1, 10);
    AmountOfReadings_FM = RemapedAmountOfReadings_FM;
  } else {
    PulseCounter_FM++;
    PeriodSum_FM = PeriodSum_FM + PeriodBetweenPulses_FM;
  }
}
void Pulse_Event_PT()  // Pump Tachometer interrupt function
{
  PeriodBetweenPulses_PT = micros() - LastTimeWeMeasured_PT;
  LastTimeWeMeasured_PT = micros();
  if (PulseCounter_PT >= AmountOfReadings_PT) {
    PeriodAverage_PT = PeriodSum_PT / AmountOfReadings_PT;
    PulseCounter_PT = 1;
    PeriodSum_PT = PeriodBetweenPulses_PT;
    int RemapedAmountOfReadings_PT = map(PeriodBetweenPulses_PT, 100000, 10000, 1,
                                         10);
    RemapedAmountOfReadings_PT = constrain(RemapedAmountOfReadings_PT, 100, 500);
    AmountOfReadings_PT = RemapedAmountOfReadings_PT;
  } else {
    PulseCounter_PT++;
    PeriodSum_PT = PeriodSum_PT + PeriodBetweenPulses_PT;
  }
}
void UpdatePlot() {
  // NOT being connected to the Lunar slowed the Uno down, so I broke this out... doesn't seem to make a difference with Nano ESP32 but leaving it in case.
  // Future upgrade may be to tell the Arduino not to search for the Lunar if it isn't connected and a shot starts?
  // In addition to that, I think the plot should be changed to plot every so often, rather than EVERY time through the loop?
  if (lunar.connected()) {
    plotStep = 0.24;
  } else {
    plotStep = 0.24;
  }

  //Plot Pressure
  tft.drawCircle(presX, presPlot, 1, PRESSURE_COLOR);
  tft.drawLine(presX - plotStep, lastPresPlot, presX, presPlot, PRESSURE_COLOR);
  lastPresPlot = presPlot;
  presX = presX + plotStep;
  //Plot Flow Meter Flow
  tft.drawCircle(flowX_FM, flowPlot_FM, 1, FLOW_COLOR);
  tft.drawLine(flowX_FM - plotStep, lastFlowPlot_FM, flowX_FM, flowPlot_FM, FLOW_COLOR);
  lastFlowPlot_FM = flowPlot_FM;
  flowX_FM = flowX_FM + plotStep;
  //Plot Lunar Weight
  tft.drawCircle(LunarX, LunarPlot, 1, PUMP_COLOR);
  tft.drawLine(LunarX - plotStep, lastLunarPlot, LunarX, LunarPlot, PUMP_COLOR);
  lastLunarPlot = LunarPlot;
  LunarX = LunarX + plotStep;
  //Plot Pump Tachometer - not being plotted right now, can be added back in later
  //tft.drawCircle(flowX_PT, flowPlot_PT, 1, PUMP_COLOR);
  //tft.drawLine(flowX_PT - plotStep, lastFlowPlot_PT, flowX_PT, flowPlot_PT, PUMP_COLOR);
  //lastFlowPlot_PT = flowPlot_PT;
  //flowX_PT = flowX_PT + plotStep;
  // Serial.println(presPlot);
  // Serial.print(",");
  // Serial.print(flowPlot_FM);
  // Serial.print(",");
  // Serial.print(flowPlot_PT);
  // Serial.print(",");
  // Serial.print(LunarPlot);
}
void InitFromPreferences() {

  Preferences.begin("BrewSettings", false);
  ProfileInPreferences = Preferences.getUInt("profile", 255);
  ShotCounterInPreferences = Preferences.getUInt("shotCounter", 255);
  BrightnessPreferences = Preferences.getUInt("brightness", 255);

  if (ProfileInPreferences == 255) {
    ProfileInPreferences = selectedProfile;
    lastSelectedProfile = selectedProfile;
    Preferences.putUInt("profile", ProfileInPreferences);
  } else {
    selectedProfile = ProfileInPreferences;
    lastSelectedProfile = selectedProfile;
  }
  if (ShotCounterInPreferences == 255) {
    ShotCounterInPreferences = ShotCounter;
    Preferences.putUInt("shotCount", ShotCounterInPreferences);
  } else {
    ShotCounter = ShotCounterInPreferences;
  }
  if (BrightnessPreferences < 30) {
    BrightnessPreferences = SelectedBrightness;
    Preferences.putUInt("brightness", BrightnessPreferences);
  } else {
    SelectedBrightness = BrightnessPreferences;
  }
}

void WriteShotCounter() {
  if (ShotCounterInPreferences != ShotCounter) {
    ShotCounterInPreferences = ShotCounter;
    Preferences.putUInt("shortCounter", ShotCounterInPreferences);
    tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
    tft.setTextSize(1);
    tft.setCursor(0, 225);
    tft.print(" ");
    tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
    tft.setTextSize(1);
    tft.setCursor(0, 225);
    tft.print(ShotCounter);
  }
}
void WriteSelectedProfile() {
  if (ProfileInPreferences != selectedProfile) {
    ProfileInPreferences = selectedProfile;
    Preferences.putUInt("profile", ProfileInPreferences);
  }
}
void WriteSelectedBrightness() {
  if (BrightnessPreferences != SelectedBrightness) {
    BrightnessPreferences = SelectedBrightness;
    Preferences.putUInt("brightness", BrightnessPreferences);
  }
}
void HandleTouchOnHome() {
  if (NavButtons[PROFILES_NAV].contains(LastXTouched, LastYTouched)) {
    NavButtons[PROFILES_NAV].drawButton(true);
    DrawProfilesScreen();
  }
  if (NavButtons[SETTINGS_NAV].contains(LastXTouched, LastYTouched)) {
    NavButtons[SETTINGS_NAV].drawButton(true);
    DrawSettingsScreen();
  }
}
void HandleTouchOnSettings() {
  if (LastXTouched > (sliderStart - 15) && LastXTouched < (sliderEnd + 15)) {
    if (LastYTouched < (sliderY + 15) && LastYTouched > (sliderY - 15)) {
      UpdateBrightness(LastXTouched);
    }
  }
  if (NavButtons[HOME_NAV].contains(LastXTouched, LastYTouched)) {
    NavButtons[HOME_NAV].drawButton(true);
    WriteSelectedBrightness();
    initSlider = true;
    DrawHomeScreen();
  }
  if (UpDown[0].contains(LastXTouched, LastYTouched)) {
    DoseSize = DoseSize + 0.5;
    UpdateDoseSize();
  }
  if (UpDown[1].contains(LastXTouched, LastYTouched)) {
    DoseSize = DoseSize - 0.5;
    UpdateDoseSize();
  }
}
void HandleTouchOnProfiles() {
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
    if (buttons[i].contains(LastXTouched, LastYTouched)) {
      if (i == SELECT) {
        buttons[i].drawButton(true);
        WriteSelectedProfile();
        DrawHomeScreen();
      } else {
        buttons[lastSelectedProfile].drawButton(false);
        selectedProfile = i;
        buttons[selectedProfile].drawButton(true);
        lastSelectedProfile = i;
      }
    }
  }
}
bool CheckButtonPress() {
  if (!ts.touched() || timerRunning) {
    return false;
  }
  TS_Point p = ts.getPoint();
  // rotate coordinate system
  // flip it around to match the screen.
  p.x = map(p.x, 0, 240, 0, 240);
  p.y = map(p.y, 0, 320, 0, 320);
  LastXTouched = p.y;
  LastYTouched = tft.height() - p.x;
  // tft.drawPixel(LastXTouched, LastYTouched, tft.color565(50,205,50)); //uncomment if you need to see where you're touching
  return true;
}
void UpdateShotTimer() {
  tft.setTextColor(TIME_COLOR, BACKGROUND_COLOR);
  tft.setTextSize(3);
  tft.setCursor(1, 65);
  tft.print(shotTimer);
  if (shotTimer <= 9) {
  tft.print(" ");
  }
}
void UpdatePressure() {
  vPres = (float)analogRead(PRESSUREPIN);
  readPressure = ((((vPres / 4095) - 0.1) * 200 / 0.8) * 0.0689476) + 0.5; //added 0.5 to match machine pressure guage, but not sure if it's more or less accurate?
  // presPlot = readPressure;
  presPlot = map(readPressure * 100, 0.0 * 100, 10 * 100, 218, 118);
  tft.setTextSize(3);
  tft.setTextColor(PRESSURE_COLOR, BACKGROUND_COLOR);
  tft.setCursor(48, 65);
  tft.print(abs(readPressure), 1);
  if (readPressure <= 9.9) {
  tft.print(" ");
  }
}

void UpdateLunar() {
  readLunar = constrain(lunar.weight, 0, 99.9);
  LunarPlot = map(readLunar, 0.0, 50.0, 218, 118);
  LunarPlot = constrain(LunarPlot, 118, 218);
  tft.setTextSize(3);
  tft.setCursor(200, 65);
  tft.setTextColor(PUMP_COLOR, BACKGROUND_COLOR);
  tft.print(abs(readLunar), 1);
  if (readLunar <= 9.9) {
  tft.print(" ");
  }
}

void UpdateFlowMeter() {
  LastTimeCycleMeasure_FM = LastTimeWeMeasured_FM;
  CurrentMicros_FM = micros();
  if (CurrentMicros_FM < LastTimeCycleMeasure_FM) {
    LastTimeCycleMeasure_FM = CurrentMicros_FM;
  }
  FrequencyRaw_FM = 10000000000 / PeriodAverage_FM;  // Calculate the frequency using the period
  if (PeriodBetweenPulses_FM > ZeroTimeout_FM - ZeroDebouncingExtra_FM || CurrentMicros_FM -

                                                                              LastTimeCycleMeasure_FM
                                                                            > ZeroTimeout_FM - ZeroDebouncingExtra_FM)

  {
    FrequencyRaw_FM = 0;            // Set frequency as 0.
    ZeroDebouncingExtra_FM = 2000;  // Change the threshold a little so it doesn't bounce.
  } else {
    ZeroDebouncingExtra_FM = 0;  // Reset the threshold to the normal value so it doesn't bounce.
  }
  FrequencyReal_FM = FrequencyRaw_FM / 10000;
  RPM_FM = FrequencyRaw_FM / PulsesPerRevolution_FM * 60;

  RPM_FM = RPM_FM / 10000;  // Remove the decimals.
  // Smoothing RPM:
  total_FM = total_FM - readings_FM[readIndex_FM];
  readings_FM[readIndex_FM] = RPM_FM;
  total_FM = total_FM + readings_FM[readIndex_FM];
  readIndex_FM = readIndex_FM + 1;
  if (readIndex_FM >= numReadings_FM)  // If we're at the end of the array:
  {
    readIndex_FM = 0;  // Reset array index.
  }
  average_FM = total_FM / numReadings_FM;
  flowRate_FM = average_FM / 60.0;
  flowRate_FM_Display = constrain(flowRate_FM, 0.0, 10.0);
  flowPlot_FM = map(flowRate_FM * 100, 0.0 * 100, 10.0 * 100, 218, 118);
  flowPlot_FM = constrain(flowPlot_FM, 118, 218);
  tft.setTextSize(3);
  tft.setCursor(123, 65);
  tft.setTextColor(FLOW_COLOR, BACKGROUND_COLOR);
  tft.print(flowRate_FM_Display, 1);
  if (flowRate_FM_Display <= 9.9) {
  tft.print(" ");
  }
}
void UpdatePumpTach() {
  LastTimeCycleMeasure_PT = LastTimeWeMeasured_PT;
  CurrentMicros_PT = micros();
  if (CurrentMicros_PT < LastTimeCycleMeasure_PT) {
    LastTimeCycleMeasure_PT = CurrentMicros_PT;
  }
  FrequencyRaw_PT = 10000000000 / PeriodAverage_PT;  // Calculate the frequency using the period
  if (PeriodBetweenPulses_PT > ZeroTimeout_PT - ZeroDebouncingExtra_PT || CurrentMicros_PT -
                                                                              LastTimeCycleMeasure_PT
                                                                            > ZeroTimeout_PT - ZeroDebouncingExtra_PT)
  {
    FrequencyRaw_PT = 0;            // Set frequency as 0.
    ZeroDebouncingExtra_PT = 2000;  // Change the threshold a little so it doesn't bounce.
  } else {
    ZeroDebouncingExtra_PT = 0;  // Reset the threshold to the normal value so it doesn't bounce.
  }
  FrequencyReal_PT = FrequencyRaw_PT / 10000;
  RPM_PT = FrequencyRaw_PT / PulsesPerRevolution_PT * 60;
  RPM_PT = RPM_PT / 10000;  // Remove the decimals.
  // Smoothing RPM:
  total_PT = total_PT - readings_PT[readIndex_PT];
  readings_PT[readIndex_PT] = RPM_PT;
  total_PT = total_PT + readings_PT[readIndex_PT];
  readIndex_PT = readIndex_PT + 1;
  if (readIndex_PT >= numReadings_PT)  // If we're at the end of the array:
  {
    readIndex_PT = 0;  // Reset array index.
  }
  average_PT = total_PT / numReadings_PT;
  PT_kilo = average_PT / 1000;
  flowRate_PT = average_PT / 60.0 * 0.3;
  duty_PT = average_PT / 50;
  flowPlot_PT = map(flowRate_PT * 100, 0.0 * 100, 20.0 * 100, 218, 118);
  flowPlot_PT = constrain(flowPlot_PT, 118, 218);
  flowRate_PT = RPM_PT / 60.0 * 0.3;
  tft.setTextSize(3);
  tft.setCursor(275, 65);
  tft.setTextColor(RPM_COLOR, BACKGROUND_COLOR);
  tft.print(flowRate_PT,0);
  if (flowRate_PT <= 9) { 
    tft.print(" ");
  }
}

uint16_t getCenteredX(String text) {
  int16_t x1, y1;
  uint16_t w, h;
  //tft.getTextBounds(&text)
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

  return (320 - w) / 2;
}

void (*resetFunc)(void) = 0;

void ble_discover_lunar() {
  BLEDevice peripheral = BLE.available();
  // print the local name, if present
  if (peripheral && peripheral.hasLocalName()) {
    if (peripheral.localName().indexOf("LUNAR-") == 0) {
      if (lunar.connect(&peripheral)) {
        Serial.println("Found LUNAR device");
        found_lunar = true;
        scale_read_last_time = 0;
        BLE.stopScan();
      }
    }
  }
}

void read_scale() {
  if ((millis() - scale_read_last_time) > SCALE_SAMPLERATE_DELAY_MS) {
    lunar.read();
  }
  lunar.sendHeartBeat();
}

void HandleLunar() {
  BLE.poll();
  if (ble_active) {
    //Serial.println("BLE Active");
    if (found_lunar) {
      if (lunar.connected()) {
        Serial.println("Lunar Connected!");
        read_scale();
        if (!lunar_session_init) {
          lunar.sendHeartBeat();
        }
      } else if (!lunar_active) {
        BLE.scan();
        found_lunar = false;
        Serial.println("Lunar not connected");
      }
    } else {
      if (!lunar_active)
        BLE.scan();
      ble_discover_lunar();
      Serial.println("Trying to find Lunar...");
    }
  } else {
    Serial.println("BLE Not Active");
  }
}

void setup() {
  pinMode(A0, INPUT_PULLUP);
  pinMode(SWITCHPIN, INPUT);
  pinMode(POTENTIOMETERPIN, INPUT);
  pinMode(PRESSUREPIN, INPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, INPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLOWMETERPIN), Pulse_Event_FM, RISING);
  attachInterrupt(digitalPinToInterrupt(PUMPTACHPIN), Pulse_Event_PT, RISING);
  delay(1000);
  Serial.begin(115200);

  timerRunning = false;
  
  tft.begin(2400000);
  ts.begin(2400000);
  tft.setRotation(3);
  startTime = millis();
  InitFromPreferences();
  InitButtons();
  analogWrite(BRIGHTNESSPIN, SelectedBrightness);

  DrawBootScreen();
  Serial.println("Welcome to the WE AV-ABR Profiling Mod!");

  // initialize scale
  found_lunar = false;
  ble_active = false;
  if (BLE.begin()) {
    ble_active = true;
    BLE.scan();
  } else {
    Serial.println("Failed to begin BLE");
  }
  DrawHomeScreen();
}
void ClearPlot() {
  tft.fillRect(25, 117, 270, 103, 0x0000);      // clear plot, and then redraw the grid lines below
  tft.drawLine(25, 120, 295, 120, GRID_COLOR);  // Horizontal lines for pressure/flow grid
  tft.drawLine(25, 130, 295, 130, GRID_COLOR);
  tft.drawLine(25, 140, 295, 140, GRID_COLOR);
  tft.drawLine(25, 150, 295, 150, GRID_COLOR);
  tft.drawLine(25, 160, 295, 160, GRID_COLOR);
  tft.drawLine(25, 170, 295, 170, GRID_COLOR);
  tft.drawLine(25, 180, 295, 180, GRID_COLOR);
  tft.drawLine(25, 190, 295, 190, GRID_COLOR);
  tft.drawLine(25, 200, 295, 200, GRID_COLOR);
  tft.drawLine(25, 210, 295, 210, GRID_COLOR);
  tft.drawLine(55, 118, 55, 219, GRID_COLOR);  // vertical lines for pressure/flow grid
  tft.drawLine(85, 118, 85, 219, GRID_COLOR);
  tft.drawLine(115, 118, 115, 219, GRID_COLOR);
  tft.drawLine(145, 118, 145, 219, GRID_COLOR);
  tft.drawLine(175, 118, 175, 219, GRID_COLOR);
  tft.drawLine(205, 118, 205, 219, GRID_COLOR);
  tft.drawLine(235, 118, 235, 219, GRID_COLOR);
  tft.drawLine(265, 118, 265, 219, GRID_COLOR);
  // Clear Profile phase
  tft.setTextSize(2);
  tft.setCursor(110, 35);
  tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
  tft.print(" ");
}
void CheckAutoFill() {
  if (digitalRead(SWITCHPIN) == HIGH) {
    if (digitalRead(OEMAUTOFILLREADPIN) == LOW) {
      autofillTimer = millis();
      if ((autofillTimer + 5000) >= millis()) {
        autofillSpeed = 120;
        analogWrite(AUTOFILLSPEEDPIN, autofillSpeed);
      }
      if (SelectedScreen == HOME) {
        tft.setTextSize(2);
        tft.setCursor(getCenteredX("Autofill"), 30);
        tft.setTextColor(TIME_COLOR, BACKGROUND_COLOR);
        tft.print("Autofill");
      }
    } else {
      autofillSpeed = 0;
      analogWrite(AUTOFILLSPEEDPIN, autofillSpeed);
      tft.setTextSize(2);
      tft.setCursor(getCenteredX("           "), 30);
      tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
      if (SelectedScreen == HOME) {
        tft.print("           ");
      }
    }
  }
}
void UpdateScreen() {

  if (SelectedScreen == HOME) {
    UpdateHome();
  }
  if (SelectedScreen == PROFILES) {
    UpdateProfiles();
  }
  if (SelectedScreen == SETTINGS) {
    UpdateSettings();
  }
}
void UpdateHome() {
  UpdatePressure();
  UpdateFlowMeter();
  UpdatePumpTach();
  UpdateLunar();
  UpdateShotTimer();
  if (CheckButtonPress()) {
    HandleTouchOnHome();
  }
}
void UpdateProfiles() {
  if (CheckButtonPress()) {
    HandleTouchOnProfiles();
  }
}
void UpdateSettings() {
  UpdateStartupTimer();
  UpdateShotCounter();
  if (CheckButtonPress()) {
    HandleTouchOnSettings();
  }
}
void MakeCoffee() {
  if (digitalRead(SWITCHPIN) == HIGH) {
    digitalWrite(BREWSOLENOIDPIN, LOW);
    Serial.println("Switch is off");
    CheckAutoFill();
    pumpDACValue = 0;
    analogWrite(BREWSPEEDPIN, pumpDACValue);
    if (weightStopper = true) {
      weightStopper = false;
      Serial.println("weightStopper FALSE Line 1262");
    }
    if (timerRunning == true) {
      timerRunning = false;
      Profiles[selectedProfile].profileState = Stopped;
      if (shotTimer > 10) {
        ShotCounter++;
        WriteShotCounter();
      }
    }
    average_FM = 0;
    average_PT = 0;
    presX = 25;
    flowX_FM = 25;
    flowX_PT = 25;
    LunarX = 25;
  }
  if (digitalRead(SWITCHPIN) == LOW) {
    Serial.println("Switch On");
    if (weightStopper == false) {
      digitalWrite(BREWSOLENOIDPIN, HIGH);
      Serial.println("Solenoid open!");
    }
    if (weightStopper == true) {
      Serial.println("weightStopper TRUE Line 1354");
      digitalWrite(BREWSOLENOIDPIN, LOW);
      Serial.println("Weight reached! Solenoid closed!");
    }
    if (SelectedScreen != HOME) {
      DrawHomeScreen();
    }
    if (!timerRunning) {
      ClearPlot();
      shotStart = millis();

      shotTimer = 0;
      resetPhaseTimer();
      //ResetFlowCalculations();
      timerRunning = true;
      Profiles[selectedProfile].currentProfilePhase = 0;
      Profiles[selectedProfile].profileState = Running;
      PrintPhase();
      pumpDACValue = 0;
      analogWrite(BREWSPEEDPIN, pumpDACValue);
    }
    if (weightStopper == false) {
      RunPhase();
      UpdateShotTimer(); //is this a duplicate? UpdateHome also calls out UpdateShotTimer, I think
      UpdatePlot();
      currentTime = millis(); 
    }
    shotTimer = (currentTime - shotStart) / 1000;
  }
}
void loop(void) {
  CheckButtonPress();
  UpdateScreen();
  MakeCoffee();
  HandleLunar();
}
