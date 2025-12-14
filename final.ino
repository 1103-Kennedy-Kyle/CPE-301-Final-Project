// Kyle Kennedy - Fixed/Refactored State Machine Version

#include <LiquidCrystal.h>
#include <AccelStepper.h>
#include <RTClib.h>
#include <Wire.h>
#include <math.h>

LiquidCrystal lcd(22, 23, 24, 25, 26, 27);

#define RDA 0x80
#define TBE 0x20

// Hysteresis thresholds (recommended)
#define TEMP_ON  85.0
#define TEMP_OFF 80.0

// ---------------- Register pointers ----------------
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int  *)0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

volatile unsigned char *my_ADMUX  = (unsigned char *)0x7C;
volatile unsigned char *my_ADCSRB = (unsigned char *)0x7B;
volatile unsigned char *my_ADCSRA = (unsigned char *)0x7A;

volatile unsigned char *my_ADCL = (unsigned char *)0x78;
volatile unsigned char *my_ADCH = (unsigned char *)0x79;

volatile unsigned char *ddr_b  = (unsigned char *)0x24;
volatile unsigned char *port_b = (unsigned char *)0x25;

volatile unsigned char *ddr_e  = (unsigned char *)0x2D;
volatile unsigned char *port_e = (unsigned char *)0x2E;
volatile unsigned char *pin_e  = (unsigned char *)0x2C;

volatile unsigned char *ddr_g  = (unsigned char *)0x33;
volatile unsigned char *port_g = (unsigned char *)0x34;
volatile unsigned char *pin_g  = (unsigned char *)0x32;

volatile unsigned char *ddr_h  = (unsigned char *)0x101;
volatile unsigned char *port_h = (unsigned char *)0x102;

// ---------------- Thermistor (Steinhart–Hart) ----------------
const int thermistorChannel = 2;   // A2
const float R1 = 10000.0;
const float c1 = 1.009249522e-03;
const float c2 = 2.378405444e-04;
const float c3 = 2.019202697e-07;

// ---------------- Pins ----------------
const int startButton = 2;    // PE4 (INT on Arduino Mega = D2)
const int stopButton  = 4;    // PG5
const int resetButton = 5;    // PE3
const int fanPin      = 3;    // PE5 (PWM)

const int yellowLED = 8;      // PH5
const int greenLED  = 9;      // PH6
const int redLED    = 10;     // PB4
const int blueLED   = 11;     // PB5

const int waterChannel = 0;   // A0

const int potChannel = 1;  // A1
AccelStepper ventStepper(
  AccelStepper::FULL4WIRE,
  43, 42, 41, 40
);

// ---------------- RTC ----------------
RTC_DS1307 rtc;

// ---------------- Thresholds ----------------
const unsigned int waterThreshold = 102;  // tune for your sensor

// ---------------- State machine ----------------
enum State { DISABLED, IDLE, RUNNING, ERROR_STATE };
volatile State state = DISABLED;

// ISR flag (START)
volatile bool startEvent = false;

// Sensor values
float tempF = 0.0;
unsigned int waterValue = 0;

// Timing
const unsigned long debounceDelay = 50;
unsigned long lastStopTime  = 0;
unsigned long lastResetTime = 0;
unsigned long lastLCDTime   = 0;
unsigned long lastSensorRead = 0;

// -------------- Forward decls --------------
void U0init(int U0baud);
void U0putchar(unsigned char U0pdata);
void printString(const char *str);

void adc_init();
unsigned int adc_read(unsigned char adc_channel_num);

float readThermistorF();
void startISR();

void updateLEDs();
void updateFan();
void updateLCDTimed();
void logStateChange(State oldState, State newState);
const char *stateToString(State s);

bool stopPressedEdge();
bool resetPressedEdge();

// ---------------- Setup ----------------
void setup() {
  U0init(9600);

  lcd.begin(16, 2);     // IMPORTANT: begin before printing
  Wire.begin();
  rtc.begin();
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  adc_init();

  // LED outputs
  *ddr_b |= (1 << 4) | (1 << 5);  // PB4 red, PB5 blue
  *ddr_h |= (1 << 5) | (1 << 6);  // PH5 yellow, PH6 green

  // Fan output (PE5)
  *ddr_e |= (1 << 5);

  // Buttons inputs + pullups
  *ddr_e &= ~(1 << 4);  // START PE4
  *port_e |= (1 << 4);

  *ddr_g &= ~(1 << 5);  // STOP PG5
  *port_g |= (1 << 5);

  *ddr_e &= ~(1 << 3);  // RESET PE3
  *port_e |= (1 << 3);

  // ISR for START only (flags only)
  attachInterrupt(digitalPinToInterrupt(startButton), startISR, FALLING);

  lastStopTime  = millis();
  lastResetTime = millis();
  lastLCDTime   = millis();
  lastSensorRead = millis();

  // Initialize outputs for DISABLED state
  updateLEDs();
  updateFan();
  lcd.clear();
  lcd.print("Disabled");
  // Vent speed
  ventStepper.setMaxSpeed(400);
  ventStepper.setAcceleration(200);

  printString("Setup complete\n");
}

// ---------------- Loop ----------------
void loop() {
  // --------- Sensor reads (rate-limited) ----------
  if (millis() - lastSensorRead >= 500) {
    float newTemp = readThermistorF();
    if (!isnan(newTemp)) tempF = newTemp;

    waterValue = adc_read(waterChannel);

    lastSensorRead = millis();
  }

  bool waterLow = (waterValue < waterThreshold);

  // --------- Button edges (poll + debounce) ----------
  bool stopEdge  = stopPressedEdge();
  bool resetEdge = resetPressedEdge();

  // --------- Next state decision ----------
  State next = state;

  switch (state) {
    case DISABLED:
      if (startEvent) {
        startEvent = false;
        next = IDLE;
      }
      break;

    case IDLE:
      if (stopEdge) {
        next = DISABLED;
      } else if (waterLow) {
        next = ERROR_STATE;
      } else if (tempF >= TEMP_ON) {
        next = RUNNING;
      }
      break;

    case RUNNING:
      if (stopEdge) {
        next = DISABLED;
      } else if (waterLow) {
        next = ERROR_STATE;
      } else if (tempF <= TEMP_OFF) {
        next = IDLE;
      }
      break;

    case ERROR_STATE:
      if (stopEdge) {
        next = DISABLED;
      } else if (resetEdge && !waterLow) {
        next = IDLE;
      }
      break;
  }

  // --------- Apply state change once ----------
  if (next != state) {
    State old = state;
    state = next;

    logStateChange(old, state);

    updateLEDs();
    updateFan();

    // immediate LCD feedback on state change
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(stateToString(state));
  }

  // --------- Timed LCD refresh ----------
  updateLCDTimed();
  updateVentStepper();
  ventStepper.run();
}

// ---------------- ISR ----------------
void startISR() {
  startEvent = true; // NOTHING ELSE
}

// ---------------- Button helpers (edge + debounce) ----------------
bool stopPressedEdge() {
  static bool last = true; // pullup => HIGH idle
  bool now = (*pin_g & (1 << 5)) != 0; // true=HIGH, false=LOW
  bool edge = false;

  if (last && !now) { // falling edge
    if (millis() - lastStopTime > debounceDelay) {
      edge = true;
      lastStopTime = millis();
    }
  }
  last = now;
  return edge;
}

bool resetPressedEdge() {
  static bool last = true; // pullup => HIGH idle
  bool now = (*pin_e & (1 << 3)) != 0; // PE3
  bool edge = false;

  if (last && !now) {
    if (millis() - lastResetTime > debounceDelay) {
      edge = true;
      lastResetTime = millis();
    }
  }
  last = now;
  return edge;
}

// ---------------- Output functions ----------------
void updateLEDs() {
  // OFF all
  *port_h &= ~((1 << 5) | (1 << 6));  // yellow/green
  *port_b &= ~((1 << 4) | (1 << 5));  // red/blue

  switch (state) {
    case DISABLED:     *port_h |= (1 << 5); break; // yellow
    case IDLE:         *port_h |= (1 << 6); break; // green
    case RUNNING:      *port_b |= (1 << 5); break; // blue
    case ERROR_STATE:  *port_b |= (1 << 4); break; // red
  }
}

void updateFan() {
  static bool fanWasOn = false;
  bool fanOn = (state == RUNNING);

  if (fanOn && !fanWasOn) {
    printString("Fan motor ON\n");
  } 
  else if (!fanOn && fanWasOn) {
    printString("Fan motor OFF\n");
  }

  analogWrite(fanPin, fanOn ? 255 : 0);
  fanWasOn = fanOn;
}


void updateLCDTimed() {
  // refresh every 1s while debugging (set to 60000 later if required)
  if (millis() - lastLCDTime < 1000) return;
  lastLCDTime = millis();

  if (state == ERROR_STATE) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error:");
    lcd.setCursor(0, 1);
    lcd.print("Water Too Low");
    return;
  }

  if (state == DISABLED) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Disabled");
    lcd.setCursor(0, 1);
    lcd.print("Press START");
    return;
  }

  // IDLE or RUNNING: show temp & water
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(tempF, 1);
  lcd.print("F ");

  lcd.setCursor(0, 1);
  lcd.print("W:");
  lcd.print(waterValue);
  lcd.print(" ");
  lcd.print(stateToString(state));
}

// ---------------- Logging ----------------
void logStateChange(State oldState, State newState) {
  DateTime now = rtc.now();
  char buf[96];

  sprintf(buf,
          "State %s -> %s @ %02d/%02d/%02d %02d:%02d:%02d\n",
          stateToString(oldState),
          stateToString(newState),
          now.month(), now.day(), now.year() % 100,
          now.hour(), now.minute(), now.second());
  printString(buf);
}

const char *stateToString(State s) {
  switch (s) {
    case DISABLED:     return "DISABLED";
    case IDLE:         return "IDLE";
    case RUNNING:      return "RUNNING";
    case ERROR_STATE:  return "ERROR";
  }
  return "UNKNOWN";
}

// ---------------- UART ----------------
void printString(const char *str) {
  while (*str) U0putchar(*str++);
}

void U0init(int U0baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud = (FCPU / 16 / U0baud - 1);
  *myUCSR0A = 0x00;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0 = tbaud;
}

void U0putchar(unsigned char U0pdata) {
  while ((*myUCSR0A & TBE) == 0);
  *myUDR0 = U0pdata;
}

// ---------------- ADC ----------------
void adc_init() {
  *my_ADMUX = (1 << 6); // AVCC reference
  *my_ADCSRA = (1 << 7) | (1 << 2) | (1 << 1) | (1 << 0); // ADEN + prescaler 128
  *my_ADCSRB = 0x00;

  // dummy conversion
  *my_ADCSRA |= (1 << 6);
  while (*my_ADCSRA & (1 << 6));
  (void)*my_ADCL;
  (void)*my_ADCH;
}

unsigned int adc_read(unsigned char adc_channel_num) {
  *my_ADMUX = (*my_ADMUX & 0xE0) | (adc_channel_num & 0x1F);
  if (adc_channel_num > 7) *my_ADCSRB |= (1 << 3);
  else *my_ADCSRB &= ~(1 << 3);

  *my_ADCSRA |= (1 << 6); // ADSC

  unsigned long start = millis();
  while (*my_ADCSRA & (1 << 6)) {
    if (millis() - start > 5) return 0;
  }

  unsigned char low  = *my_ADCL;
  unsigned char high = *my_ADCH;
  return (high << 8) | low;
}

// ---------------- Thermistor ----------------
float readThermistorF() {
  unsigned int Vo = adc_read(thermistorChannel);
  if (Vo < 5 || Vo > 1018) return NAN;

  float R2 = R1 * (1023.0 / Vo - 1.0);
  if (R2 <= 0) return NAN;

  float logR2 = log(R2);
  float invT = c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2;
  if (invT == 0) return NAN;

  float Tc = (1.0 / invT) - 273.15;
  return (Tc * 9.0 / 5.0) + 32.0;
}
void updateVentStepper() {
  if (state == DISABLED) return;

  static bool initialized = false;

  const long VENT_MIN = 0;
  const long VENT_MAX = 300;   // tune carefully

  unsigned int potVal = adc_read(potChannel);
  long targetPos = map(potVal, 0, 1023, VENT_MIN, VENT_MAX);

  if (!initialized) {
    ventStepper.setCurrentPosition(targetPos);
    initialized = true;
    return;
  }

  ventStepper.moveTo(targetPos);
  ventStepper.run();   // REQUIRED for AccelStepper
  }

