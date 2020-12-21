#include <Arduino.h>
#include <SoftPWM.h>
#include <LowPower.h>

#define FIREFLIES_NUM 4
#define ACC_PIN 19 // 19-20-21 for A1-A2-A3. A0 is for random seeding
#define LIGHT_DURATION 5000
#define PAUSE_FRACTION 3 // pause will last LIGHT_DURATION * 1 / PAUSE_FRACTION
#define OFFSET_DIV 25
#define MAX_PWM 200
#define MAX_ACC 1.5
#define MAX_EQUAL_ITERATIONS 6
#define WAKEUP_PIN 3

byte fireflies_pins[FIREFLIES_NUM] = {5, 6, 7, 8};
byte fireflies_value[FIREFLIES_NUM];
byte fireflies_offset[FIREFLIES_NUM];
byte equal_iterations;
bool fireflies_updated[FIREFLIES_NUM];
bool fireflies_first[FIREFLIES_NUM];
bool shaken;
volatile bool awake;

unsigned long started;
const unsigned int loop_interval = LIGHT_DURATION / 255;


void reset() {
  Serial.println("RESET");

  randomSeed(analogRead(A0));

  for (byte i = 0; i < FIREFLIES_NUM; i++) {  
    fireflies_value[i] = 0;
    fireflies_offset[i] = random(255);
    fireflies_updated[i] = true;
    fireflies_first[i] = true;
  }

  equal_iterations = 0;
  started = millis();
}

float acceleration() {
  float ax = (float) analogRead(ACC_PIN) / 1024 * 3;
  float ay = (float) analogRead(ACC_PIN + 1) / 1024 * 3;
  float az = (float) analogRead(ACC_PIN + 2) / 1024 * 3;
  float magnitude = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 3));
  return magnitude;
}

void turn_off() {
  for (byte i = 0; i < FIREFLIES_NUM; i++) {
    SoftPWMSet(fireflies_pins[i], 0);
  }
}

void toggle_sleep() {
  if (awake) {
    awake = false;
    Serial.println("AWAKE");
    reset();
  } else {
    awake = true;
    Serial.println("SLEEPING...");
    turn_off();
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
}

void setup() {
  Serial.begin(9600);
    
  for (byte i = 0; i < FIREFLIES_NUM; i++) {
    pinMode(fireflies_pins[i], OUTPUT);
  }
  pinMode(ACC_PIN, INPUT);
  pinMode(WAKEUP_PIN, INPUT_PULLUP);

  SoftPWMBegin();

  reset();
  shaken = false;
  awake = false;
  attachInterrupt(digitalPinToInterrupt(WAKEUP_PIN), toggle_sleep, CHANGE);
  toggle_sleep();
}

void loop() {
  if (millis() - started > loop_interval) {
    if (shaken) {
       started = millis();

      float total_offset = 0;
      for (byte i = 0; i < FIREFLIES_NUM; i++) {
        total_offset += fireflies_offset[i];
      }
      byte avg_offset = total_offset / FIREFLIES_NUM;

      for (byte i = 0; i < FIREFLIES_NUM; i++) {
        byte level = fireflies_offset[i] + fireflies_value[i];

        if (level > 255 / PAUSE_FRACTION) {
          float value = map(level, 255 / PAUSE_FRACTION, 255, 0, MAX_PWM);

          if (!fireflies_first[i]) SoftPWMSet(fireflies_pins[i], value);

          if (level == 255 && !fireflies_updated[i]) {
            fireflies_updated[i] = true;

            float offset_diff = (float) (fireflies_offset[i] - avg_offset) / OFFSET_DIV;
            if (offset_diff > 0 && offset_diff < 1) {
              offset_diff = 1;
            } else if (offset_diff < 0 && offset_diff > -1) {
              offset_diff = -1;
            }

            fireflies_offset[i] += offset_diff;
          }
        } else {
          if (fireflies_updated[i]) fireflies_updated[i] = false;
          if (fireflies_first[i]) fireflies_first[i] = false;
          SoftPWMSet(fireflies_pins[i], 0);
        }

        fireflies_value[i]++;
      }

      bool all_equal = true || (equal_iterations != 0);
      for (byte i = 1; (i < FIREFLIES_NUM && all_equal) || equal_iterations != 0; i++) {
        all_equal = abs(fireflies_offset[i] -= fireflies_offset[i-1]) < 1.5;
      }
      if (all_equal) {
        equal_iterations++;
        if (equal_iterations > MAX_EQUAL_ITERATIONS) reset();
      }

      if (acceleration() > MAX_ACC) {
        turn_off();
        while(acceleration() > MAX_ACC) {
          delay(5000);
        }
        reset();
      }      
    } else if (acceleration() > MAX_ACC) {
      shaken = true;
    }
  }
}