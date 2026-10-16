#include <Arduino.h>
#include <Wire.h>
#include <max6675.h>
#include <ESP32Servo.h>
#include <DIYables_LCD_I2C.h>

// Pins
static const int fanPin = 0;           // DRV8871 IN1
static const int thermoSO = 1;
static const int thermoCS = 3;
static const int thermoSCK = 4;
static const int servoPin = 5;         // via level shifter
static const int lcdSDA = 6;           // via level shifter
static const int lcdSCL = 7;           // via level shifter
static const int beeperPin = 10;       // via level shifter, beeps when LOW
static const int switchClosePin = 20;  // LOW when the switch is in "close"
static const int switchOpenPin = 21;   // LOW when the switch is in "open", 1 kOhm in series

// Servo angles for fully closed and fully open
static const int servoClosedAngle = 0;
static const int servoOpenAngle = 90;

// Auto mode: servo fully open below 100 C, fully closed above 400 C
static const float servoOpenTemp = 100;
static const float servoCloseTemp = 400;
// Fan off below 50 C, full speed at 300 C
static const float fanStartTemp = 50;
static const float fanFullTemp = 300;
// Beep while the servo is fully open and the temperature has dropped by at least
// dropThreshold C over the last DROP_WINDOW seconds
static const int DROP_WINDOW = 300;
static const float dropThreshold = 10;

enum Mode { MODE_CLOSE, MODE_AUTO, MODE_OPEN };
static const char *MODE_NAMES[] = {"CLOSE", "AUTO", "OPEN"};

MAX6675 thermocouple(thermoSCK, thermoCS, thermoSO);
Servo servo1;
ESP32PWM fanPwm;
DIYables_LCD_I2C *lcd = nullptr;

float temperature = NAN;
Mode mode = MODE_AUTO;
int servoPercent = 100;  // 0 = closed, 100 = open
int fanPercent = 0;

unsigned long lastTempRead = 0;

// Readings from the last DROP_WINDOW seconds, one per second
float tempHistory[DROP_WINDOW];
int tempHistoryCount = 0;
int tempHistoryIndex = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastScreenChange = 0;
int screen = 0;

Mode pendingMode = MODE_AUTO;
unsigned long pendingModeSince = 0;

// Where t sits between t0 and t1, as 0.0-1.0
float ramp(float t, float t0, float t1) {
  return constrain((t - t0) / (t1 - t0), 0.0f, 1.0f);
}

Mode readSwitch() {
  if (digitalRead(switchClosePin) == LOW) return MODE_CLOSE;
  if (digitalRead(switchOpenPin) == LOW) return MODE_OPEN;
  return MODE_AUTO;
}

// Recalculates servo and fan from the mode and temperature; keeps the last values on a sensor error
void updateOutputs() {
  if (mode == MODE_CLOSE) {
    servoPercent = 0;
  } else if (mode == MODE_OPEN) {
    servoPercent = 100;
  } else if (!isnan(temperature)) {
    servoPercent = lroundf(100 * (1 - ramp(temperature, servoOpenTemp, servoCloseTemp)));
  }
  servo1.write(servoClosedAngle + (servoOpenAngle - servoClosedAngle) * servoPercent / 100);

  if (!isnan(temperature)) {
    fanPercent = lroundf(100 * ramp(temperature, fanStartTemp, fanFullTemp));
  }
  fanPwm.writeScaled(fanPercent / 100.0);
}

// Stores the reading and returns true if it is dropThreshold lower than DROP_WINDOW seconds ago
bool recordTemperature(float t) {
  float oldest = tempHistory[tempHistoryIndex];  // the slot about to be overwritten
  bool full = tempHistoryCount == DROP_WINDOW;
  tempHistory[tempHistoryIndex] = t;
  tempHistoryIndex = (tempHistoryIndex + 1) % DROP_WINDOW;
  if (!full) tempHistoryCount++;
  return full && oldest - t >= dropThreshold;
}

bool temperatureDropping = false;

// Beeps 0.5 s on / 0.5 s off while needed
void updateBeeper() {
  bool sensorError = isnan(temperature);
  bool openAndDropping = servoPercent == 100 && temperatureDropping;
  bool on = (sensorError || openAndDropping) && (millis() / 500) % 2 == 0;
  digitalWrite(beeperPin, on ? LOW : HIGH);
}

void printLine(int row, const String &text) {
  lcd->setCursor(0, row);
  lcd->print(text);
  for (int i = text.length(); i < 16; i++) lcd->print(' ');
}

// Cycles through temperature, servo and fan, 2 s each
void updateDisplay() {
  if (lcd == nullptr) return;
  if (millis() - lastScreenChange >= 2000) {
    lastScreenChange = millis();
    screen = (screen + 1) % 3;
  }
  if (screen == 0) {
    printLine(0, "Temperature");
    printLine(1, isnan(temperature) ? String("Sensor error") : String(temperature, 1) + " \xDF" "C");
  } else if (screen == 1) {
    printLine(0, String("Servo  ") + MODE_NAMES[mode]);
    printLine(1, String(servoPercent) + " % open");
  } else {
    printLine(0, "Fan");
    printLine(1, String(fanPercent) + " %");
  }
}

// The I2C adapter is at 0x27 or 0x3F depending on the manufacturer
void setupDisplay() {
  Wire.setPins(lcdSDA, lcdSCL);
  Wire.begin();
  for (uint8_t addr : {0x27, 0x3F}) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("LCD found at 0x%02X\n", addr);
      lcd = new DIYables_LCD_I2C(addr, 16, 2);
      lcd->init();
      lcd->backlight();
      return;
    }
  }
  Serial.println("LCD not found, running without display");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Fireplace regulator starts");

  // Set HIGH before enabling the output so the beeper doesn't click on boot
  digitalWrite(beeperPin, HIGH);
  pinMode(beeperPin, OUTPUT);

  pinMode(switchClosePin, INPUT_PULLUP);
  pinMode(switchOpenPin, INPUT_PULLUP);
  mode = pendingMode = readSwitch();

  servo1.attach(servoPin);
  fanPwm.attachPin(fanPin, 20000, 10);
  fanPwm.writeScaled(0);

  setupDisplay();

  // Give the MAX6675 time to stabilize before the first read
  delay(500);
}

void loop() {
  // Debounce the switch: accept a new position once it is stable for 50 ms
  Mode current = readSwitch();
  if (current != pendingMode) {
    pendingMode = current;
    pendingModeSince = millis();
  } else if (current != mode && millis() - pendingModeSince >= 50) {
    mode = current;
    Serial.printf("Mode: %s\n", MODE_NAMES[mode]);
    updateOutputs();
  }

  // The MAX6675 needs at least 250 ms between reads
  if (lastTempRead == 0 || millis() - lastTempRead >= 1000) {
    lastTempRead = millis();
    temperature = thermocouple.readCelsius();
    temperatureDropping = recordTemperature(temperature);
    updateOutputs();
    Serial.printf("temperature %.2f C, mode %s, servo %d %%, fan %d %%%s\n",
                  temperature, MODE_NAMES[mode], servoPercent, fanPercent,
                  temperatureDropping ? ", dropping" : "");
  }

  if (millis() - lastDisplayUpdate >= 500) {
    lastDisplayUpdate = millis();
    updateDisplay();
  }

  updateBeeper();
}
