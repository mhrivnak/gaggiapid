// include the library code:
#include <LiquidCrystal.h>
#include <dht_nonblocking.h>
#include "Adafruit_MAX31865.h"

#define DHT_SENSOR_TYPE DHT_TYPE_11

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  101.0

const float targetTemp = 95.0;

const byte timer1OutputB = 10;
// period in milliseconds. 450 gives us 54 zero-crossings at 60hz,
// aka a granularity of 54 heat levels to set
const int FREQ = 450;
const unsigned long countTo = ((float) F_CPU / 1024.0) / (1000.0 / FREQ);

static const int DHT_SENSOR_PIN = 2;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7, 8, 9, 11, 12, 13);

Adafruit_MAX31865 thermo = Adafruit_MAX31865(6, 5, 4, 3);

// PID vars
unsigned long lastCycleTime;
unsigned long nextCycleTime;
double lastTemp;
double errSum;
const double KP = 1.5;
const double KI = .3;
const double KD = 1;
float currentDuty;

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
  
}

void loop() {
  uint16_t rtd = thermo.readRTD();
  float ratio = float(rtd) / 32768;
  float res = ratio * RREF;

  uint8_t fault = thermo.readFault();
  if (fault) {
    lcd.setCursor(0, 0);
    lcd.print("Fault 0x"); lcd.print(fault, HEX);
    lcd.setCursor(0, 1);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      lcd.print("RTD High"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      lcd.print("RTD Low"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      lcd.print("REFIN- >0.85xBias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      lcd.print("REFIN- <0.85xBias"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      lcd.print("RTDIN- <0.85xBias"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      lcd.print("Under/Over V"); 
    }
    thermo.clearFault();
    return;
  }

  float temp = thermo.temperature(RNOMINAL, RREF);

  unsigned long now = millis();
  if (now >= nextCycleTime) {
    currentDuty = duty(temp);
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

  lcd.setCursor(0, 1);
  lcd.print("D: ");
  lcd.print(currentDuty);
}

float duty(float currentTemp) {
  // if we're not yet within 5 degrees of the target temp, stay full power
  if (currentTemp < targetTemp - 5) {
    lastTemp = currentTemp;
    return 1.0;
  }

  double err = targetTemp - currentTemp;
  double dTemp = currentTemp - lastTemp;

  lastTemp = currentTemp;
  float pTerm = KP * err;
  float iTerm = KI * errSum;
  // reduce jitter by ignoring small corrections
  float dTerm = (abs(dTemp) < .25) ? 0 : KD * (err - dTemp);

  // don't include this cycle's error in "I" until next cycle
  errSum += err;

  float duty = pTerm + iTerm + dTerm;
  if (duty < 0) duty = 0;
  if (duty > 1) duty = 1;

  printCSV(lastTemp);
  printCSV(currentTemp);
  printCSV(pTerm);
  printCSV(iTerm);
  printCSV(dTerm);
  Serial.println(duty);

  return duty;
}

void printCSV(float x) {
  Serial.print(x);
  Serial.print(",");
}