#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_HX8357.h>   //Screen library
#include <Adafruit_FT5336.h>   //Touchpad library
#include <Adafruit_MCP4725.h>  //DAC library
#include <Preferences.h>
#include <ArduinoBLE.h>
#include <lunarGateway.h>
#include <vector>
#include <string>
#include <I2CScanner.h>
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
float presX = 36;
float flowRate_FM;
float flowRate_FM_Display;
float flowX_FM = 36;
float flowPlot_FM = 0.0;
float lastFlowPlot_FM = 220;
float flowRate_PT;
float flowX_PT = 36;
float flowPlot_PT = 0.0;
float lastFlowPlot_PT = 220;
float readLunarBattery = 0;
float readLunarWeight = 0.0;
float LunarPlot = 0.0;
float lastLunarPlot = 220;
float LunarX = 36;
float dripFactor = 2.0;
float plotStep = 0.117;
int autofillTimer = 0;
int counter;
int sliderStart = 20;
int sliderEnd = 230;
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
#define PRESSUREPIN A1
#define SWITCHPIN A2
#define POTENTIOMETERPIN A3
#define PUMPTACHPIN A7
#define AUTOFILLSOLENOIDPIN 2
#define BREWSOLENOIDPIN 3
#define SDCSPIN 5  // SD card (mounted to screen), not used yet
#define BRIGHTNESSPIN 6
#define OEMAUTOFILLREADPIN 7
#define OEMBREWREADPIN 8
#define TFT_CS 9
#define TFT_DC 10
#define TFT_RST -1

//DAC setup
Adafruit_MCP4725 dac;

//I2C Scanner
I2CScanner scanner;

// 3.5" Wired screen and touchpad setup
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC);
Adafruit_FT5336 ctp = Adafruit_FT5336();
#define FT5336_MAXTOUCHES 5
const uint16_t BACKGROUND_COLOR = tft.color565(0, 0, 0);
const uint16_t TIME_COLOR = tft.color565(238, 99, 82);
const uint16_t PRESSURE_COLOR = tft.color565(219, 208, 83);
const uint16_t FLOW_COLOR = tft.color565(99, 105, 209);
const uint16_t RPM_COLOR = tft.color565(230, 101, 45);
const uint16_t PUMP_COLOR = tft.color565(79, 201, 120);
const uint16_t PHASE_COLOR = tft.color565(244, 187, 255);
const uint16_t BUTTON_COLOR = tft.color565(27, 45, 42);
const uint16_t BUTTON_OUTLINE_COLOR = tft.color565(247, 247, 255);
const uint16_t TEXT_COLOR = tft.color565(247, 247, 255);
const uint16_t GRID_COLOR = tft.color565(72, 72, 72);

// Profile selection and profile button variables
#define NUMBER_OF_BUTTONS 13
Adafruit_GFX_Button buttons[NUMBER_OF_BUTTONS];
#define PROFILES_NAV 0
#define SETTINGS_NAV 1
#define HOME_NAV 2
Adafruit_GFX_Button NavButtons[3];
Adafruit_GFX_Button UpDown[2];
#define EMP 0
#define THREE69 1
#define EXDOS 2
#define FLATSIX 3
#define SLAYER6 4
#define WEPRO1 5
#define FLATNINE 6
#define SLAYER9 7
#define WEPRO3 8
#define LEVAX 9
#define TURBO 10
#define SPROOVER 11
#define SELECT 12
int selectedProfile = EMP;
int lastSelectedProfile = EMP;
const int NumberOfProfiles = 12;
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
  int currentProfilePhase;    // = 0;
  ProfileState profileState;  // = Stopped;
};
ProfileType Profiles[NumberOfProfiles] = {
  { "EMP", 50, 40, 96, 76, Manual, { { " Manual", 0.0, 0.0, 0.0, 0, 0, false, false, false, false, false } } },
  { "3-6-9", 150, 40, 96, 76, Interactive, { { "  Line  ", 0, 0.0, 0.0, 0, 0, false, false, false, false, true }, { "3 Bar", 3.0, 0.0, 0.0, 0, 0, false, false, false, false, true }, { "6 Bar", 6.0, 0.0, 0.0, 0, 0, false, false, false, false, true }, { "9 Bar", 9.0, 0.0, 0.0, 0, 0, false, false, false, false, true } } },
  { "ExDos!", 250, 40, 96, 76, Automatic, { { "PI", 3.0, 0.0, 0.0, 15, 1, false, false, true, false, false }, { "Ramp", 6.0, 0.0, 0.0, 3, 0, false, false, true, false, false }, { "6 Bar", 6.0, 0.0, 0.0, 42, 0, false, false, true, false, false } } },
  { "Flat 6", 50, 120, 96, 76, Automatic, { { "PI", 3.0, 0.0, 0.0, 15, 1, false, false, true, false, false }, { "Ramp", 6.0, 0.0, 0.0, 3, 0, false, false, true, false, false }, { "6 Bar", 6.0, 0.0, 0.0, 42, 0, false, false, true, false, false } } },
  { "Slayer 6", 150, 120, 96, 76, Interactive, { { "  Line  ", 0, 0.0, 0.0, 0, 0, false, false, false, false, true }, { "  PI  ", 3.0, 0.0, 0.0, 0, 0, false, false, false, false, true }, { "  PI  ", 3.0, 0.0, 0.0, 0, 0, false, false, false, false, true }, { "6 Bar", 6.0, 0.0, 0.0, 0, 0, false, false, false, false, true } } },
  { "WE Pro 2:1", 250, 120, 96, 76, Automatic, { { "Start", 1.5, 0, 40.0, 2, 0, false, false, true, true, false }, { "PI 1/2", 4, 0.0, 40.0, 14, 12, false, false, true, true, false }, { "PI 2/2", 1.5, 0.0, 40.0, 14, 12, false, false, true, true, false }, { "9 Bar", 8, 0.0, 40.0, 20, 2, false, false, true, true, false }, { "Decrease", 6.5, 0.0, 40.0, 15, 15, false, false, true, true, false }, { " Hold ", 6.5, 0.0, 40.0, 55, 30, false, false, true, true, false } } },
  { "Flat 9", 50, 200, 96, 76, Automatic, { { "PI", 3.0, 0.0, 0.0, 15, 1, false, false, true, false, false }, { "Ramp", 9.0, 0.0, 0.0, 3, 0, false, false, true, false, false }, { "9 Bar", 9.0, 0.0, 0.0, 42, 0, false, false, true, false, false } } },
  { "Slayer 9", 150, 200, 96, 76, Interactive, { { "  Line  ", 0, 0.0, 0.0, 0, 0, false, false, false, false, true }, { "  PI  ", 3.0, 0.0, 0.0, 0, 0, false, false, false, false, true }, { "  PI  ", 3.0, 0.0, 0.0, 0, 0, false, false, false, false, true }, { "9 Bar", 9.0, 0.0, 0.0, 0, 0, false, false, false, false, true } } },
  { "WE Pro 3:1", 250, 200, 96, 76, Automatic, { { "Start", 1.5, 0, 50.0, 2, 0, false, false, true, true, false }, { "PI 1/2", 4, 0.0, 50.0, 14, 12, false, false, true, true, false }, { "PI 2/2", 1.5, 0.0, 50.0, 14, 12, false, false, true, true, false }, { "9 Bar", 8, 0.0, 50.0, 20, 2, false, false, true, true, false }, { "Decrease", 6.5, 0.0, 50.0, 15, 15, false, false, true, true, false }, { " Hold ", 6.5, 0.0, 50.0, 55, 30, false, false, true, true, false } } },
  { "Leva X", 50, 280, 96, 76, Automatic, { { "PI", 3.0, 0.0, 0.0, 10, 1, false, false, true, false, false }, { "Ramp", 9.0, 0.0, 0.0, 2, 1, true, false, true, false, false }, { "Decrease", 4.0, 0.0, 0.0, 40, 40, false, false, true, false, false }, { "Hold", 4.0, 0.0, 0.0, 8, 3, false, false, true, false, false } } },
  { "Turbo", 150, 280, 96, 76, Automatic, { { "PI", 3.0, 0.0, 0.0, 15, 1, false, false, true, false, false }, { "Ramp", 6.0, 0.0, 0.0, 3, 0, false, false, true, false, false }, { "Extract", 6.0, 0.0, 0.0, 42, 0, false, false, true, false, false } } },
  { "Spro Over", 250, 280, 96, 76, Automatic, { { "PI", 3.0, 0.0, 0.0, 15, 1, false, false, true, false, false }, { "Ramp", 6.0, 0.0, 0.0, 3, 0, false, false, true, false, false }, { "Extract", 6.0, 0.0, 0.0, 42, 0, false, false, true, false, false } } },
};

// converting target pressure into a DAC value for the profiles... the "pressures" current set in the profiles above won't go off of read pressure, but rather target pressure
// this would be a great place to work in a pressure PID or feedback loop for true pressure profiling instead of targeted pressure profiling
int pressureToDACConvertion(float pressure) {
  float dac = ((294.55 * pressure) + 338.18);
  if (dac < 0) {
    dac = 0;
  }
  if (dac > 3000) {

    dac = 3000;
  }
  return dac;
}
float dacToPressureConvertion(int dacValue) {
  return ((0.0034 * dacValue) - 1.148);
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
  if (Profiles[selectedProfile].mode == Interactive) {
    potentiometer = analogRead(POTENTIOMETERPIN);
    if (potentiometer >= 1500) {
      return 0;
    }
    if ((potentiometer < 1500) && (potentiometer >= 1000)) {
      return 1;
    }
    if ((potentiometer < 1000) && (potentiometer >= 500)) {
      return 2;
    }
    if (potentiometer <= 500) {
      return 3;
    }
  }
}

// Phase name printout while profiling a shot
void PrintPhase() {
  ProfileType profile = Profiles[selectedProfile];
  tft.setTextSize(3);
  if (profile.currentProfilePhase > 0 && profile.mode) {
    tft.setCursor(getCenteredX(profile.profilePhases[profile.currentProfilePhase - 1].name), 40);
    tft.setTextColor(BACKGROUND_COLOR, BACKGROUND_COLOR);
    tft.println(profile.profilePhases[profile.currentProfilePhase - 1].name);

    tft.setCursor(getCenteredX(profile.profilePhases[profile.currentProfilePhase].name), 40);
    tft.setTextColor(PHASE_COLOR, BACKGROUND_COLOR);
    tft.println(profile.profilePhases[profile.currentProfilePhase].name);
  } else {
    tft.setCursor(getCenteredX("            "), 40);
    tft.setTextColor(BACKGROUND_COLOR, BACKGROUND_COLOR);
    tft.println("            ");
    tft.setCursor(getCenteredX(profile.profilePhases[profile.currentProfilePhase].name), 40);
    tft.setTextColor(PHASE_COLOR, BACKGROUND_COLOR);
    tft.println(profile.profilePhases[profile.currentProfilePhase].name);
  }
}

// Simple void to update pump speed
void UpdatePump() {
  dac.setVoltage(pumpDACValue, false);
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
    tft.drawFastVLine(presX, 150, 8, PHASE_COLOR);
    tft.drawFastVLine(presX - 1, 150, 6, PHASE_COLOR);
    tft.drawFastVLine(presX + 1, 150, 6, PHASE_COLOR);
    tft.drawFastVLine(presX - 2, 150, 4, PHASE_COLOR);
    tft.drawFastVLine(presX + 2, 150, 4, PHASE_COLOR);
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
    if ((readLunarWeight + dripFactor) >= phase.weightTarget) {
      exit = true;
      weightStopper = true;
      Serial.println("weightStopper TRUE line 370");
      Profiles[selectedProfile].profileState = Stopped;
      pumpDACValue = 0;
      dac.setVoltage(pumpDACValue, false);
    }
  }
  return exit;
}
float calculateStepSize(float previousPressureValue, float newPressureValue, int transitionTime) {
  float stepsPerSecond = 40;
  float stepSize = 10;
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
  InitBrewTemp();
  InitSteamPres();
  InitFillLevel();
  InitLunarData();
}

uint16_t cacluateRainbow(int progress) {
  int colorCount = 7;
  float colorInterval = 260 / (colorCount - 1);
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
  tft.setCursor(getCenteredX("Witt's End Coffee Co."), 100);
  tft.print("Witt's End Coffee Co.");
  for (int progress = 1; progress < 260; progress++) {
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
  tft.setTextSize(3);
  tft.setCursor(getCenteredX(Profiles[selectedProfile].name), 5);
  tft.print(Profiles[selectedProfile].name);
}
void DrawPlot() {
  DrawNavButtons();
  PrintSelectedProfile();
  tft.drawLine(27, 150, 27, 300, TEXT_COLOR);  // plot border... 3x pixel wide, thicker lines
  tft.drawLine(28, 150, 28, 300, TEXT_COLOR);
  tft.drawLine(29, 150, 29, 300, TEXT_COLOR);
  tft.drawLine(27, 299, 453, 299, TEXT_COLOR);
  tft.drawLine(27, 300, 453, 300, TEXT_COLOR);
  tft.drawLine(27, 301, 453, 301, TEXT_COLOR);
  tft.drawLine(451, 150, 451, 300, TEXT_COLOR);
  tft.drawLine(452, 150, 452, 300, TEXT_COLOR);
  tft.drawLine(453, 150, 453, 300, TEXT_COLOR);

  tft.drawLine(30, 150, 450, 150, GRID_COLOR);  // Horizontal lines for pressure/flow grid
  tft.drawLine(30, 165, 450, 165, GRID_COLOR);
  tft.drawLine(30, 180, 450, 180, GRID_COLOR);
  tft.drawLine(30, 195, 450, 195, GRID_COLOR);
  tft.drawLine(30, 210, 450, 210, GRID_COLOR);
  tft.drawLine(30, 225, 450, 225, GRID_COLOR);
  tft.drawLine(30, 240, 450, 240, GRID_COLOR);
  tft.drawLine(30, 255, 450, 255, GRID_COLOR);
  tft.drawLine(30, 270, 450, 270, GRID_COLOR);
  tft.drawLine(30, 285, 450, 285, GRID_COLOR);

  tft.drawLine(82, 150, 82, 298, GRID_COLOR);  // vertical lines for pressure/flow grid
  tft.drawLine(128, 150, 128, 298, GRID_COLOR);
  tft.drawLine(174, 150, 174, 298, GRID_COLOR);
  tft.drawLine(220, 150, 220, 298, GRID_COLOR);
  tft.drawLine(266, 150, 266, 298, GRID_COLOR);
  tft.drawLine(312, 150, 312, 298, GRID_COLOR);
  tft.drawLine(358, 150, 358, 298, GRID_COLOR);
  tft.drawLine(404, 150, 404, 298, GRID_COLOR);


  tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);  // X axis scale - time (sec)
  tft.setTextSize(1);
  tft.setCursor(25, 305);
  tft.print("0");
  tft.setCursor(77, 305);
  tft.print("10");
  tft.setCursor(123, 305);
  tft.print("20");
  tft.setCursor(169, 305);
  tft.print("30");
  tft.setCursor(215, 305);
  tft.print("40");
  tft.setCursor(261, 305);
  tft.print("50");
  tft.setCursor(307, 305);
  tft.print("60");
  tft.setCursor(353, 305);
  tft.print("70");
  tft.setCursor(399, 305);
  tft.print("80");
  tft.setCursor(445, 305);
  tft.print("90");

  // Y axis unit indicator dashes for pressure side
  tft.setTextColor(PRESSURE_COLOR, BACKGROUND_COLOR);
  tft.drawLine(20, 150, 30, 150, TEXT_COLOR);
  tft.setTextSize(1);
  tft.setCursor(3, 147);
  tft.print("10");
  tft.drawLine(20, 165, 30, 165, TEXT_COLOR);
  tft.setCursor(8, 162);
  tft.print("9");
  tft.drawLine(20, 180, 30, 180, TEXT_COLOR);
  tft.setCursor(8, 177);
  tft.print("8");
  tft.drawLine(20, 195, 30, 195, TEXT_COLOR);
  tft.setCursor(8, 192);
  tft.print("7");
  tft.drawLine(20, 210, 30, 210, TEXT_COLOR);
  tft.setCursor(8, 207);
  tft.print("6");
  tft.drawLine(20, 225, 30, 225, TEXT_COLOR);
  tft.setCursor(8, 222);
  tft.print("5");
  tft.drawLine(20, 240, 30, 240, TEXT_COLOR);
  tft.setCursor(8, 237);
  tft.print("4");
  tft.drawLine(20, 255, 30, 255, TEXT_COLOR);
  tft.setCursor(8, 252);
  tft.print("3");
  tft.drawLine(20, 270, 30, 270, TEXT_COLOR);
  tft.setCursor(8, 267);
  tft.print("2");
  tft.drawLine(20, 285, 30, 285, TEXT_COLOR);
  tft.setCursor(8, 282);
  tft.print("1");
  tft.drawLine(20, 300, 30, 300, TEXT_COLOR);
  tft.setCursor(8, 297);
  tft.print("0");

  tft.setTextColor(PUMP_COLOR, BACKGROUND_COLOR);  // Y axis unit indicator dashes and scale for weight side
  tft.setTextSize(1);
  tft.drawLine(450, 150, 460, 150, TEXT_COLOR);
  tft.setCursor(465, 155);
  tft.print("50");
  tft.drawLine(450, 180, 460, 180, TEXT_COLOR);
  tft.setCursor(465, 185);
  tft.print("40");
  tft.drawLine(450, 210, 460, 210, TEXT_COLOR);
  tft.setCursor(465, 215);
  tft.print("30");
  tft.drawLine(450, 240, 460, 240, TEXT_COLOR);
  tft.setCursor(465, 245);
  tft.print("20");
  tft.drawLine(450, 270, 460, 270, TEXT_COLOR);
  tft.setCursor(465, 275);
  tft.print("10");
  tft.drawLine(450, 300, 460, 300, TEXT_COLOR);

  tft.setTextColor(FLOW_COLOR, BACKGROUND_COLOR);  //Y axis scale for flow
  tft.setCursor(467, 142);
  tft.print("10");
  tft.setCursor(470, 172);
  tft.print("8");
  tft.setCursor(470, 202);
  tft.print("6");
  tft.setCursor(470, 232);
  tft.print("4");
  tft.setCursor(470, 262);
  tft.print("2");
  tft.setCursor(470, 295);
  tft.print("0");

  // tft.drawRoundRect(8, 55, 64, 50, 4, TIME_COLOR);
  tft.setTextSize(2);
  tft.setTextColor(TIME_COLOR, BACKGROUND_COLOR);
  tft.setCursor(10, 75);
  tft.print("Time: ");
  tft.setCursor(20, 120);
  tft.print("sec");
  tft.setTextColor(PRESSURE_COLOR, BACKGROUND_COLOR);
  tft.setCursor(85, 75);
  tft.print("Pressure: ");
  tft.setCursor(120, 120);
  tft.print("Bar");
  tft.setTextSize(1);
  tft.setCursor(35, 140);
  tft.print("Pressure");
  tft.setTextSize(2);
  tft.setTextColor(FLOW_COLOR, BACKGROUND_COLOR);
  tft.setCursor(220, 75);
  tft.print("Flow: ");
  tft.setCursor(220, 120);
  tft.print("mL/s");
  tft.setTextSize(1);
  tft.setCursor(370, 140);
  tft.print("Flow /");
  tft.setTextSize(2);
  tft.setTextColor(RPM_COLOR, BACKGROUND_COLOR);
  tft.setCursor(310, 75);
  tft.print("Pump: ");
  tft.setCursor(310, 120);
  tft.print("mL/s");
  tft.setTextColor(PUMP_COLOR, BACKGROUND_COLOR);
  tft.setCursor(395, 75);
  tft.print("Weight: ");
  tft.setCursor(405, 120);
  tft.print("grams");
  tft.setTextSize(1);
  tft.setCursor(415, 140);
  tft.print("Weight");
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
  tft.setTextSize(3);
  tft.setCursor(320, 20);
  tft.print("Profiles");
  tft.drawLine(320, 46, 460, 46, TEXT_COLOR);
  tft.drawLine(320, 47, 460, 47, TEXT_COLOR);
}
void InitButtons() {
  for (uint8_t i = 0; i < NumberOfProfiles; i++) {
    buttons[i].initButton(&tft, Profiles[i].buttonX, Profiles[i].buttonY, Profiles[i].buttonW,

                          Profiles[i].buttonH, BUTTON_OUTLINE_COLOR,

                          BUTTON_COLOR, TEXT_COLOR, Profiles[i].name, 1);
  }
  buttons[SELECT].initButton(&tft, 390, 280, 176, 76, BUTTON_OUTLINE_COLOR,

                             BUTTON_COLOR, TEXT_COLOR, "Select", 1);

  NavButtons[PROFILES_NAV].initButton(&tft, 45, 35, 80, 60, BUTTON_OUTLINE_COLOR,

                                      BUTTON_COLOR, TEXT_COLOR, "Profiles", 1);

  NavButtons[SETTINGS_NAV].initButton(&tft, 435, 35, 80, 60, BUTTON_OUTLINE_COLOR,

                                      BUTTON_COLOR, TEXT_COLOR, "Settings", 1);

  NavButtons[HOME_NAV].initButton(&tft, 45, 35, 80, 60, BUTTON_OUTLINE_COLOR,

                                  BUTTON_COLOR, TEXT_COLOR, "Home", 1);
  UpDown[0].initButton(&tft, 225, 135, 25, 25, BUTTON_OUTLINE_COLOR, BUTTON_COLOR, TEXT_COLOR, "+", 1);
  UpDown[1].initButton(&tft, 225, 155, 25, 25, BUTTON_OUTLINE_COLOR, BUTTON_COLOR, TEXT_COLOR, "-", 1);
}
int UpdateManual() {
  potentiometer = analogRead(POTENTIOMETERPIN);
  if (potentiometer <= 1940) {
    return (-1.32 * potentiometer) + 2975;
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
  tft.setCursor(20, 100);
  tft.print(String("Shot Count: ") + String(ShotCounter));
}

void UpdateDoseSize() {
  tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
  tft.setTextSize(2);
  tft.setCursor(20, 140);
  tft.print(String("Dose Size: ") + String(DoseSize));
}

void InitBrightness() {
  tft.fillCircle(lastBrightnessX, 200, 15, BACKGROUND_COLOR);
  tft.drawFastHLine(sliderStart, sliderY, (sliderEnd-sliderStart), TEXT_COLOR);
  tft.fillCircle(SelectedBrightness, 200, 15, BUTTON_COLOR);
  tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
  tft.setTextSize(2);
  tft.setCursor(20, 205);
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
    tft.drawFastHLine(sliderStart, sliderY, (sliderEnd-sliderStart), TEXT_COLOR);
    tft.fillCircle(newXValue, 200, 15, BUTTON_COLOR);
    tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);

    SelectedBrightness = newXValue - sliderStart;
    lastBrightnessX = SelectedBrightness;
    analogWrite(BRIGHTNESSPIN, SelectedBrightness);
  }
}

void InitBrewTemp() {
  tft.drawRoundRect(260, 11, 200, 67, 5, TIME_COLOR);
  tft.setTextSize(2);
  tft.setCursor(295, 21);
  tft.print("Brew Boiler");
  tft.setTextSize(1);
  tft.setCursor(280, 41);
  tft.print("Set Temp:");
  tft.setCursor(360, 41);
  tft.print("Current Temp:");
}

void UpdateBrewTemp() {

}

void InitSteamPres() {
  tft.drawRoundRect(260, 88, 200, 67, 5, TIME_COLOR);
  tft.setTextSize(2);
  tft.setCursor(290, 98);
  tft.print("Steam Boiler");
  tft.setTextSize(1);
  tft.setCursor(280, 118);
  tft.print("Set Pres:");
  tft.setCursor(360, 118);
  tft.print("Current Pres:");
}

void UpdateSteamPres() {

}

void InitFillLevel() {
  tft.drawRoundRect(260, 165, 200, 67, 5, FLOW_COLOR);
  tft.setTextSize(2);
  tft.setCursor(295, 175);
  tft.print("Fill Levels");
  tft.setTextSize(1);
  tft.setCursor(280, 195);
  tft.print("Reservoir:");
  tft.setCursor(360, 195);
  tft.print("Steam Boiler:");
}

void UpdateFillLevel() {

}

void  InitLunarData() {
  tft.drawRoundRect(260, 242, 200, 67, 5, PUMP_COLOR);
  tft.setTextSize(2);
  tft.setCursor(325, 252);
  tft.print("Lunar");
  tft.setTextSize(1);
  tft.setCursor(290, 272);
  tft.print("Weight:");
  tft.setCursor(375, 272);
  tft.print("Battery:");
}

void UpdateLunarData() {
    readLunarWeight = constrain(lunar.weight, 0, 99.9);
    tft.setTextSize(2);
    tft.setCursor(285, 285);
    tft.print(readLunarWeight, 1);
    if (readLunarWeight <= 9.9) {
      tft.print(" ");
    }
    readLunarBattery = constrain(lunar.battery, 0, 100);
    tft.setCursor(380, 285);
    tft.print(readLunarBattery, 0);
    if (10 <= readLunarBattery <= 99) {
      tft.print(" ");
    }
    else if (readLunarBattery <= 9.9) {
      tft.print("  ");
    }
    tft.print("%");
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
    // NOT being connected to the Lunar slowed the Uno R4 down, so I broke this out... but it doesn't seem to make a difference with Nano ESP32 but leaving it in case.
    // Future upgrade may be to tell the Arduino to stop searching for the Lunar if it isn't connected and a shot starts?
    // In addition to that, I think the plot should be changed to plot every so often, rather than EVERY time through the loop?
    if (lunar.connected()) {
      plotStep = 0.117;
    } else {
      plotStep = 0.117;
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
    //Plot Pump Tachometer - not being plotted right now, can be added back in later if we want to track pump tach/flow
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
      //tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
      //tft.setTextSize(1);
      //tft.setCursor(0, 225);
      //tft.print(" ");
      //tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
      //tft.setTextSize(1);
      //tft.setCursor(0, 225);
      //tft.print(ShotCounter);
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
    if (!ctp.touched() || timerRunning) {
      return false;
    }

    // Retrieve the points, up to 5!
    TS_Point ps[FT5336_MAXTOUCHES];
    ctp.getPoints(ps, FT5336_MAXTOUCHES);

    for (int j = 0; j < FT5336_MAXTOUCHES; j++) {
      // Check if z (pressure) is zero, skip if so
      if (ps[j].z == 0) continue;

      //swaps x values to match touchpoints on screen
      ps[j].x = map(ps[j].x, 0, 320, 320, 0);

      // Print out the remapped/rotated coordinates
      Serial.print("(");
      Serial.print(ps[j].x);
      Serial.print(", ");
      Serial.print(ps[j].y);
      Serial.print(")\t");
    }


    LastXTouched = ps[0].y;  // why are these switched? should they be?
    LastYTouched = ps[0].x;  // why are these switched? should they be? This used to say "tft.height() - ps[i].x"
    //tft.drawPixel(LastXTouched, LastYTouched, tft.color565(50, 205, 50));  //uncomment if you need to see what x/y you're touching
    Serial.println(ps[0].x, ps[0].y);
    Serial.println(LastXTouched, LastYTouched);
    return true;
  }

  void UpdateShotTimer() {
    tft.setTextColor(TIME_COLOR, BACKGROUND_COLOR);
    tft.setTextSize(3);
    tft.setCursor(15, 95);
    tft.print(shotTimer);
    if (shotTimer <= 9) {
      tft.print(" ");
    }
  }
  void UpdatePressure() {
    vPres = (float)analogRead(PRESSUREPIN);
    readPressure = ((((vPres / 4095) - 0.1) * 200 / 0.8) * 0.0689476) + 0.5;  //added 0.5 to calibrate/match OEM machine pressure guage, but not sure if it's more or less accurate?
    // presPlot = readPressure;
    presPlot = map(readPressure * 100, 0.0 * 100, 10 * 100, 300, 148);
    tft.setTextSize(3);
    tft.setTextColor(PRESSURE_COLOR, BACKGROUND_COLOR);
    tft.setCursor(105, 95);
    tft.print(abs(readPressure), 1);
    if (readPressure <= 9.9) {
      tft.print(" ");
    }
  }

  void UpdateLunar() {
    readLunarWeight = constrain(lunar.weight, 0, 99.9);
    LunarPlot = map(readLunarWeight, 0.0, 50.0, 298, 152);
    LunarPlot = constrain(LunarPlot, 152, 298);
    tft.setTextSize(3);
    tft.setCursor(405, 95);
    tft.setTextColor(PUMP_COLOR, BACKGROUND_COLOR);
    tft.print(abs(readLunarWeight), 1);
    if (readLunarWeight <= 9.9) {
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
    flowPlot_FM = map(flowRate_FM * 100, 0.0 * 100, 10.0 * 100, 298, 152);
    flowPlot_FM = constrain(flowPlot_FM, 152, 298);
    tft.setTextSize(3);
    tft.setCursor(215, 95);
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
    if (PeriodBetweenPulses_PT > ZeroTimeout_PT - ZeroDebouncingExtra_PT || CurrentMicros_PT - LastTimeCycleMeasure_PT > ZeroTimeout_PT - ZeroDebouncingExtra_PT) {
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
    flowPlot_PT = map(flowRate_PT * 100, 0.0 * 100, 20.0 * 100, 298, 152);
    flowPlot_PT = constrain(flowPlot_PT, 152, 298);
    flowRate_PT = RPM_PT / 60.0 * 0.3;  //From pump datasheet, where 1 rotation = 0.3 mL
    tft.setTextSize(3);
    tft.setCursor(305, 95);
    tft.setTextColor(RPM_COLOR, BACKGROUND_COLOR);
    tft.print(flowRate_PT, 1);
    if (flowRate_PT <= 9.9) {
      tft.print(" ");
    }
  }

  uint16_t getCenteredX(String text) {
    int16_t x1, y1;
    uint16_t w, h;
    //tft.getTextBounds(&text)
    tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

    return (480 - w) / 2;
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
          scale_read_last_time = 0;  //I don't think this is right - doesn't this need to be = millis() when it's read? needs fixed
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
        // Serial.println("Trying to find Lunar...");
      }
    } else {
      Serial.println("BLE Not Active");
    }
  }

  void setup() {
    pinMode(FLOWMETERPIN, INPUT_PULLDOWN);
    pinMode(SWITCHPIN, INPUT);
    pinMode(POTENTIOMETERPIN, INPUT_PULLDOWN);
    pinMode(PRESSUREPIN, INPUT);
    pinMode(PUMPTACHPIN, INPUT);
    pinMode(AUTOFILLSOLENOIDPIN, OUTPUT);
    pinMode(BREWSOLENOIDPIN, OUTPUT);
    pinMode(SDCSPIN, OUTPUT);
    pinMode(BRIGHTNESSPIN, OUTPUT);
    pinMode(OEMAUTOFILLREADPIN, INPUT);
    pinMode(OEMBREWREADPIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(FLOWMETERPIN), Pulse_Event_FM, FALLING);
    attachInterrupt(digitalPinToInterrupt(PUMPTACHPIN), Pulse_Event_PT, RISING);
    delay(1000);
    Serial.begin(115200);

    timerRunning = false;

    dac.begin(0x60);
    Serial.println("DAC started");
    dac.setVoltage(0, true);  // "true" sets EEPROM DAC value to zero so that the pump doesn't run when power cycled

    //initialize I2C scanner
    scanner.Init();

    //initialize touchscreen
    tft.begin();
    if (!ctp.begin(FT53XX_DEFAULT_ADDR, &Wire)) {  // pass in 'sensitivity' coefficient and I2C bus
      Serial.println("Couldn't start FT5336 touchscreen controller");
      while (1) delay(10);
    }
    Serial.println("Capacitive touchscreen started");

    tft.setRotation(1);
    startTime = millis();
    InitFromPreferences();
    InitButtons();
    analogWrite(BRIGHTNESSPIN, SelectedBrightness);

    Serial.println("Starting...");
    DrawBootScreen();

    // initialize scale
    found_lunar = false;
    ble_active = false;
    if (BLE.begin()) {
      ble_active = true;
      BLE.scan();
      Serial.println("Bluetooth started");
    } else {
      Serial.println("Failed to begin BLE");
    }
    DrawHomeScreen();
    Serial.println("Welcome to the WE AV-ABR Profiling Mod!");
  }
  void ClearPlot() {
    tft.fillRect(30, 147, 420, 152, 0x0000);      // clear plot, and then redraw the grid lines below
    tft.drawLine(30, 150, 450, 150, GRID_COLOR);  // Horizontal lines for pressure/flow grid
    tft.drawLine(30, 165, 450, 165, GRID_COLOR);
    tft.drawLine(30, 180, 450, 180, GRID_COLOR);
    tft.drawLine(30, 195, 450, 195, GRID_COLOR);
    tft.drawLine(30, 210, 450, 210, GRID_COLOR);
    tft.drawLine(30, 225, 450, 225, GRID_COLOR);
    tft.drawLine(30, 240, 450, 240, GRID_COLOR);
    tft.drawLine(30, 255, 450, 255, GRID_COLOR);
    tft.drawLine(30, 270, 450, 270, GRID_COLOR);
    tft.drawLine(30, 285, 450, 285, GRID_COLOR);
    tft.drawLine(82, 150, 82, 298, GRID_COLOR);  // vertical lines for pressure/flow grid
    tft.drawLine(128, 150, 128, 298, GRID_COLOR);
    tft.drawLine(174, 150, 174, 298, GRID_COLOR);
    tft.drawLine(220, 150, 220, 298, GRID_COLOR);
    tft.drawLine(266, 150, 266, 298, GRID_COLOR);
    tft.drawLine(312, 150, 312, 298, GRID_COLOR);
    tft.drawLine(358, 150, 358, 298, GRID_COLOR);
    tft.drawLine(404, 150, 404, 298, GRID_COLOR);

    // Clear Profile phase
    tft.setTextSize(2);
    tft.setCursor(110, 35);
    tft.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
    tft.print(" ");
  }
  void CheckAutoFill() {
    if (digitalRead(SWITCHPIN) == HIGH) {
      if (digitalRead(OEMAUTOFILLREADPIN) == LOW) {
        digitalWrite(AUTOFILLSOLENOIDPIN, HIGH);
        Serial.println("Autofill (steam boiler) solenoid open...");
        //autofillTimer = millis();
        //if ((autofillTimer + 5000) >= millis()) {  // This and the line above are supposed to be a 5 second autofill timer to keep it from bouncing on and off but it isn't quite right
        pumpDACValue = 3000;  //DAC value of 3000 runs at about the pump speed for 6 bar
        dac.setVoltage(pumpDACValue, false);
        //}
        if (SelectedScreen == HOME) {
          tft.setTextSize(3);
          tft.setCursor(getCenteredX("Autofill"), 40);
          tft.setTextColor(TIME_COLOR, BACKGROUND_COLOR);
          tft.print("Autofill");
        }
      } else {
        digitalWrite(AUTOFILLSOLENOIDPIN, LOW);
        pumpDACValue = 0;
        dac.setVoltage(pumpDACValue, false);
        tft.setTextSize(3);
        tft.setCursor(getCenteredX("           "), 40);
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
    UpdateBrewTemp();
    UpdateSteamPres();
    UpdateFillLevel();
    UpdateLunarData();
    if (CheckButtonPress()) {
      HandleTouchOnSettings();
    }
  }
  void MakeCoffee() {
    if (digitalRead(SWITCHPIN) == HIGH) {
      digitalWrite(BREWSOLENOIDPIN, LOW);
      Serial.println("Switch is off");
      CheckAutoFill();
      if (digitalRead(OEMAUTOFILLREADPIN) == HIGH) {
        pumpDACValue = 0;
        dac.setVoltage(pumpDACValue, false);
      }
      if (weightStopper = true) {
        weightStopper = false;
        // Serial.println("weightStopper FALSE Line 1262");
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
      presX = 36;
      flowX_FM = 36;
      flowRate_FM_Display = 0;
      flowX_PT = 36;
      flowRate_PT = 0;
      LunarX = 36;
    }
    if (digitalRead(SWITCHPIN) == LOW) {
      Serial.println("Switch On");
      if (weightStopper == false) {
        digitalWrite(BREWSOLENOIDPIN, HIGH);
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
        dac.setVoltage(pumpDACValue, false);
      }
      if (weightStopper == false) {
        RunPhase();
        UpdateShotTimer();  //is this a duplicate? UpdateHome also calls out UpdateShotTimer, I think
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
    Serial.print("Pot: ");
    Serial.println(potentiometer);
    Serial.print("DAC: ");
    Serial.println(pumpDACValue);
    //scanner.Scan(); //I2C address scanner - uncomment if you need to see device I2c addresses
  }
