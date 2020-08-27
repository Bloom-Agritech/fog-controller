#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans9pt7b.h>
//#include <SHTSensor.h>
#include "states.h"
#include <EasyButton.h>
#include <uFire_sht20.h>
#include <Ticker.h>
#include <ArduinoNvs.h>

#define IDLE_TIMEOUT_MS 10000
#define SSR_PIN A9

uFire_SHT20 sht20;

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

// OLED FeatherWing buttons map to different pins depending on board:
#if defined(ESP8266)
  #define BUTTON_A  0
  #define BUTTON_B 16
  #define BUTTON_C  2
#elif defined(ESP32)
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
#elif defined(ARDUINO_STM32_FEATHER)
  #define BUTTON_A PA15
  #define BUTTON_B PC7
  #define BUTTON_C PC5
#elif defined(TEENSYDUINO)
  #define BUTTON_A  4
  #define BUTTON_B  3
  #define BUTTON_C  8
#elif defined(ARDUINO_FEATHER52832)
  #define BUTTON_A 31
  #define BUTTON_B 30
  #define BUTTON_C 27
#else // 32u4, M0, M4, nrf52840 and 328p
  #define BUTTON_A  9
  #define BUTTON_B  6
  #define BUTTON_C  5
#endif

#define OK_BUTTON   BUTTON_C
#define UP_BUTTON   BUTTON_A
#define DOWN_BUTTON BUTTON_B

EasyButton okBtn(OK_BUTTON);
EasyButton upBtn(UP_BUTTON);
EasyButton dnBtn(DOWN_BUTTON);

state_codes currentState = IDLE;

double getVPDUpperBound(float vpdSP, float vpdDB);
int getCycleOffTime(double dutyCycle, double period);
int getCycleOnTime(double dutyCycle, double period); 
int lightAboveThreshold(double light, double threshold);

void onIdleTimeout() {
  currentState = IDLE;
}

void onCycleDone();
void offCycleDone();
int inCycle;

Ticker idleTimer(onIdleTimeout, IDLE_TIMEOUT_MS);
Ticker onCycleTimer(onCycleDone, 1000, 1);
Ticker offCycleTimer(offCycleDone, 1000, 1);

void onCycleDone() {
  offCycleTimer.start();
  inCycle = false; 
  //digitalWrite(LED_BUILTIN, LOW);
}

void offCycleDone() {
  onCycleTimer.start();
  inCycle = true;
  //digitalWrite(LED_BUILTIN, HIGH);
}

// these are initialized with defaults in case NVS storage fails for some reason
double vpdSetpoint = 0.5, vpdDeadband = 0.2, dutyCycle = 0.5, lightThreshold = 0.5, period = 60;
char displayBuffer[20];

double VPD;
double lightLevel;

bool nvsWritesPending = false;

void onOkPress() {
    state_codes nextState = IDLE;
    if (okBtn.read()) {   
      for (int s = IDLE; s < LAST; s++) {
        if (s == currentState) {
          nextState = state_transitions[s].next_state;
        }
      };
      currentState = nextState; 
      Serial.println("In display state: " + String(currentState));
      idleTimer.start();
    }
};

struct setpointConfig {
  enum state_codes current_state;
  double* variable;
  double increment;
  double minValue;
  double maxValue;
};

struct setpointConfig setpointConfigSetup[] = {
  {SET_VPD_SETPOINT, &vpdSetpoint, 0.05, 0.1, 2},
  {SET_VPD_DEADBAND, &vpdDeadband, 0.05, 0.1, 0.5},
  {SET_DUTY_CYCLE, &dutyCycle, 0.05, 0.1, 0.9},
  {SET_PERIOD, &period, 5, 10, 300},
  {SET_LIGHT_THRESHOLD, &lightThreshold, .01, 0.01, 1}
};


void onDnPress() {
   if (dnBtn.read()) {
    for (int s = 0; s < sizeof(setpointConfigSetup) / sizeof(setpointConfigSetup[0]); s++) {
      if (setpointConfigSetup[s].current_state == currentState) {
        double* var = setpointConfigSetup[s].variable;
        double inc = setpointConfigSetup[s].increment;
        double min = setpointConfigSetup[s].minValue;
        double max = setpointConfigSetup[s].maxValue;
        // Check if variable is in bounds and increment the correct amount
        if ((*var) > min) {
          (*var) = round(((*var) - inc) * 100) / 100; 
          nvsWritesPending = true;
        }
      }
    }
    idleTimer.start(); // Reset idle timeout
  } 
};

void onUpPress() {
  if (upBtn.read()) {
    for (int s = 0; s < sizeof(setpointConfigSetup) / sizeof(setpointConfigSetup[0]); s++) {
      if (setpointConfigSetup[s].current_state == currentState) {
        double* var = setpointConfigSetup[s].variable;
        double inc = setpointConfigSetup[s].increment;
        double min = setpointConfigSetup[s].minValue;
        double max = setpointConfigSetup[s].maxValue;
        // Check if variable is in bounds and increment the correct amount
        if ((*var) < max) {
          (*var) = round(((*var) + inc) * 100) / 100; 
          nvsWritesPending = true;
        }
      }
    }
    idleTimer.start(); // Reset idle timeout
  }
}




void drawCentreString(const char *buf, int x, int y)
{
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h); //calc width of new string
    display.setCursor(x - w / 2, y);
    display.print(buf);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  sht20.begin();
  NVS.begin();

  Serial.println("OLED FeatherWing test");
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

  Serial.println("OLED begun");

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();
  
  Serial.println("IO test");

  pinMode(A0, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SSR_PIN, OUTPUT);

  vpdSetpoint = (double)(NVS.getInt("vpdSetpoint")) / 1000;
  vpdDeadband = (double)(NVS.getInt("vpdDeadband")) / 1000;
  dutyCycle = (double)(NVS.getInt("dutyCycle")) / 1000;
  period = (double)(NVS.getInt("period")) / 1000;
  lightThreshold = (double)(NVS.getInt("lightThreshold")) / 1000;
  //NVS.setInt("vpdSetpoint", (int)(vpdSetpoint * 1000));
  onCycleTimer.interval(getCycleOnTime(dutyCycle, period));
  offCycleTimer.interval(getCycleOffTime(dutyCycle, period));

  okBtn.begin();
  upBtn.begin();
  dnBtn.begin();

  okBtn.enableInterrupt(onOkPress);
  upBtn.enableInterrupt(onUpPress);
  dnBtn.enableInterrupt(onDnPress);

  onCycleTimer.start(); 
  
  // Set up Buttons
  
}

int currentDisplayPage = 0;
char temp[13], rh[13], vpd[13];

#define NUM_DISPLAY_ELEMENTS 3
char *displayStrings[NUM_DISPLAY_ELEMENTS] = {temp, rh, vpd};

int cycleDisplay(int numItems, int cycleTimeMs) {
  static int lastChange, currentItem;
  if (millis() >= lastChange + cycleTimeMs) {
    currentItem++;
    if (currentItem == numItems)
      currentItem = 0;
    lastChange = millis();
  }
  return currentItem;
}

double getVPDUpperBound(float vpdSP, float vpdDB) {
  return vpdSP + vpdDB / 2;
}

double getVPDLowerBound(float vpdSP, float vpdDB) {
  return vpdSP - vpdDB / 2;
}

int hysteresisLatchOn() {
  static int latchOn = false;
  if (VPD >= getVPDUpperBound(vpdSetpoint, vpdDeadband)) {
    latchOn = true;
  }
  if (VPD <= getVPDLowerBound(vpdSetpoint, vpdDeadband)) {
    latchOn = false;
  }
  return latchOn;
}

void updateOutput() {
  if (hysteresisLatchOn() && inCycle && lightLevel > lightThreshold) {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(SSR_PIN, HIGH);
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(SSR_PIN, LOW);
  }
}

int getCycleOnTime(double dutyCycle, double period) {
  return (int)(dutyCycle * period) * 1000;
}

int getCycleOffTime(double dutyCycle, double period) {
  return (int)((1 - dutyCycle) * period) * 1000;
}


float get_vpd(float tempC, float RH) {
  float es = 0.6108 * exp(17.27 * tempC / (tempC + 237.3));
  float ae = RH / 100 * es;
  return es - ae;
}

void updateInputs(int pollRateMs) {
  static int lastChange;
  if (millis() >= lastChange + pollRateMs) {
    sht20.measure_all(); 
    float tempC = sht20.tempC;
    float RH = sht20.RH;
    VPD = get_vpd(tempC, RH);
    sprintf(temp, "Temp: %0.1fc", tempC);
    sprintf(rh, "RH: %0.1f%%",RH);
    sprintf(vpd, "VPD: %0.1fkPa", VPD);
    lastChange = millis();
    lightLevel = (double)analogRead(A0) / 4095;
  } 
};

void saveConfig() {
  NVS.setInt("vpdSetpoint", (int)(vpdSetpoint * 1000));
  NVS.setInt("vpdDeadband", (int)(vpdDeadband * 1000));
  NVS.setInt("dutyCycle", (int)(dutyCycle * 1000));
  NVS.setInt("period", (int)(period * 1000));
  NVS.setInt("lightThreshold", (int)(lightThreshold * 1000));
  Serial.println("Saved config");
  onCycleTimer.interval(getCycleOnTime(dutyCycle, period));
  offCycleTimer.interval(getCycleOffTime(dutyCycle, period));
  nvsWritesPending = false; // Clear flag
}


void loop() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  //display.println("RH: " + (String)sht20.RH + "%");
  //display.print("T:" + (String)sht20.tempC + (char)247 + "C");
  //display.println("VPD: " + (String)sht20.vpd() + "kPa");
  currentDisplayPage = cycleDisplay(NUM_DISPLAY_ELEMENTS, 5000);

  // if any of the variables were updated in the ISR, save to NVM
  if (nvsWritesPending) {
    saveConfig();
  }

  // Get new inputs and update display
  updateInputs(2000);

  /* MAIN STATE MACHINE */
  switch (currentState) {
    case IDLE:
      drawCentreString(displayStrings[currentDisplayPage], 64, 20);
      break;
    case SET_VPD_SETPOINT:
      sprintf(displayBuffer, "vpdSP: %0.2f", vpdSetpoint);
      drawCentreString(displayBuffer, 64, 20);
      break;
    case SET_VPD_DEADBAND:
      sprintf(displayBuffer, "vpdDB :%0.2f", vpdDeadband);
      drawCentreString(displayBuffer, 64, 20);
      break;
    case SET_DUTY_CYCLE:
      sprintf(displayBuffer, "duty: %.0f%%", dutyCycle * 100);
      drawCentreString(displayBuffer, 64, 20);
      break;
    case SET_PERIOD:
      sprintf(displayBuffer, "period: %.0fsec", period);
      drawCentreString(displayBuffer, 64, 20);
      break;
    case SET_LIGHT_THRESHOLD:
      sprintf(displayBuffer, "light: %.0f%%", lightThreshold * 100);
      drawCentreString(displayBuffer, 64, 20);
      break;
  }
  /* =================== */

  idleTimer.update(); 
  onCycleTimer.update();
  offCycleTimer.update();

  //yield();
  display.display();
  updateOutput();

  delay(1);
}