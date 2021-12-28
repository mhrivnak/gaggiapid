#include <LiquidCrystal.h>
#include "Adafruit_MAX31865.h"

#define DHT_SENSOR_TYPE DHT_TYPE_11

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  101.0

const float targetTemp = 95.0;

const byte timer1OutputB = 10;
// period in milliseconds. 900 gives us 108 zero-crossings at 60hz,
// aka a granularity of 108 heat levels to set
const int FREQ = 900;
const unsigned long countTo = ((float) F_CPU / 1024.0) / (1000.0 / FREQ);

const int buttonPin = 2;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7, 8, 9, 11, 12, 13);

Adafruit_MAX31865 thermo = Adafruit_MAX31865(6, 5, 4, 3);

// PID vars
unsigned long lastCycleTime;
unsigned long nextCycleTime;
double lastTemp;
double errSum;
// .04, .02, .06
const double KP = 0.06;
const double KI = 0.003; //0.008;
const double KDfall = 0.5;
const double KDrise = 0.1;
float currentDuty;
unsigned long stateBegan = 0;
float brewTime = 0;

enum state {
  maintain,
  brewing,
};
state currentState = maintain;
char errMessage[16] = "";

void setup() {
  Serial.begin(9600);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // start the temp sensor
  thermo.begin(MAX31865_2WIRE);

  // setup PWM, top at OCR1A
  pinMode (timer1OutputB, OUTPUT);
  TCCR1A = bit (WGM10) | bit (WGM11); // fast PWM
  TCCR1B = bit (WGM12) | bit (WGM13) | bit (CS12) | bit (CS10);   // fast PWM, prescaler of 1024
  OCR1A = countTo - 1;                 // zero relative 
  bitSet (TCCR1A, COM1B1);   // clear OC1B on compare
  OCR1B = 0; // initial setting is off
  
  pinMode(buttonPin, INPUT);
}

void loop() {
  uint16_t rtd = thermo.readRTD();
  float ratio = float(rtd) / 32768;
  float res = ratio * RREF;
  float temp = 0;

  uint8_t fault = thermo.readFault();
  if (fault) {
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      strcpy(errMessage, "RTD High");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      strcpy(errMessage, "RTD Low"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      strcpy(errMessage, "REFIN- >0.85"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      strcpy(errMessage, "REFIN- <0.85"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      strcpy(errMessage, "RTDIN- <0.85"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      strcpy(errMessage, "Under/Over V"); 
    }
    thermo.clearFault();
    handleError();
    return;
  }

  temp = thermo.temperature(RNOMINAL, RREF);

  if (temp < 0) {
    strcpy(errMessage, "Temp < 0");
    handleError();
    return;
  }

  unsigned long now = millis();

  // set current state
  if (now - stateBegan > 200) {
    int buttonState = digitalRead(buttonPin);
    if (buttonState == HIGH) {
      currentState = currentState == maintain ? brewing : maintain;
      stateBegan = now;
    }
  }

  if (currentState == brewing) {
    brewTime = float(now - stateBegan) / 1000.;
  }

  if (now >= nextCycleTime) {
    currentDuty = duty(temp, now);
    float limit = countTo * currentDuty - 1;
    if (limit < 0) limit = 0;
    OCR1B = limit;
    lastCycleTime = now;
    nextCycleTime = nextCycleTime + 2*FREQ;
  }

  lcd.setCursor(0, 0);
  lcd.print("T: ");
  lcd.print(temp);
  lcd.print(" C  ");
  if (brewTime < 10) {
    lcd.print(" ");
  }
  lcd.print(String(brewTime, 1));

  lcd.setCursor(0, 1);
  lcd.print("D: ");
  lcd.print(currentDuty);
  lcd.print(currentState == maintain ? "    maint" : "     brew");
}

void handleError() {
  OCR1B = 0; // heater off
  lcd.setCursor(0, 0);
  lcd.print("Error");
  lcd.setCursor(0, 1);
  lcd.print(errMessage);
  return;
}

float duty(float currentTemp, long now) {
  // if we're not yet within 35 degrees of the target temp, stay full power
  if (currentTemp < targetTemp - 35) {
    lastTemp = currentTemp;
    return 1.0;
  }

  double err = targetTemp - currentTemp;
  double dTemp = lastTemp - currentTemp;

  float pTerm = KP * err;
  float iTerm = KI * errSum;
  float kd = (dTemp > .25 && currentTemp < targetTemp) ? KDfall : KDrise;
  // reduce jitter by ignoring small corrections
  float dTerm = (abs(dTemp) < .15) ? 0 : kd * dTemp;

  // don't include this cycle's error in "I" until next cycle
  errSum += err;
  if (errSum > 30) errSum = 30;
  if (errSum < -5) errSum = -5;

  float duty = pTerm + iTerm + dTerm;
  // when brewing, go full power for 2s
  if (currentState == brewing & now - stateBegan < 2000) duty = 1.0;
  // keep at least a little power in to soften the descent.
  float maxFallingPower = (currentState == brewing) ? .3 : .05;
  if ((currentTemp > targetTemp) && (dTemp > 0)) {
    if (currentState == brewing) {
      duty = max(duty, .5);
    } else {
      duty = max(duty, min(-.015*err, maxFallingPower));
    }
  }
  if (duty < 0) duty = 0;
  if (duty > 1) duty = 1;

  printCSV(lastTemp);
  printCSV(currentTemp);
  printCSV(pTerm);
  printCSV(iTerm);
  printCSV(dTerm);
  Serial.print(duty);
  Serial.print(",");
  Serial.println(currentState);

  lastTemp = currentTemp;

  return duty;
}

void printCSV(float x) {
  Serial.print(x);
  Serial.print(",");
}
