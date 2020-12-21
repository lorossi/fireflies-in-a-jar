/*#include <Arduino.h>
#include <SoftPWM.h>

#define FIREFLIES_NUM 4
#define MAX_FIREFLIES_SPEED 4000
#define DELAY_DELTA 20
#define LOOP_INTERVAL 5
#define ACC_PIN A3
#define MAX_Z_ACC 250
#define MAX_PWM 225

unsigned long started;
unsigned long offset;
unsigned long fireflies_started[FIREFLIES_NUM];
byte fireflies_pins[FIREFLIES_NUM] = {5, 6, 7, 8};
byte fireflies_speed;

void reset() {
  // initalize random generator
  randomSeed(analogRead(A0));

  // initialize fireflies pins and delay
  for (byte i = 0; i < FIREFLIES_NUM; i++) {
    fireflies_started[i] = 0;
  }

  // set fireflies speed
  fireflies_speed = random(127, 255);

  // set start time
  started = millis();
}

int acceleration() {
  return analogRead(ACC_PIN);
}

void setup() {
  // Serial begin (debuging purposes)
  //Serial.begin(9600);

  // set pin mode
  for (byte i = 0; i < FIREFLIES_NUM; i++) {
    pinMode(fireflies_pins[i], OUTPUT);
  }
  pinMode(ACC_PIN, INPUT);

  // reset everything
  reset();

  // start software PWM
  SoftPWMBegin();
}

void loop() {
  // check if enough time has passed
  if (millis() - started > LOOP_INTERVAL) {
    // ok, now go
    started = millis();
    // pre-calculated firefly fadein-fadeout duration
    unsigned long duration = float(fireflies_speed) * MAX_FIREFLIES_SPEED / 255;

    // now time 
    unsigned long now = millis() - offset;

    // check if fireflies should light up
    for (byte i = 0; i < FIREFLIES_NUM; i++) {
      if (now > fireflies_started[i] + duration * 2) {
        fireflies_started[i] = now;
      } else if (now > fireflies_started[i] + duration) {
        SoftPWMSet(fireflies_pins[i], 0);
      } else {
        unsigned long age = now - fireflies_started[i];
        byte pwm_value = age / duration * MAX_PWM;
        SoftPWMSet(fireflies_pins[i], pwm_value);
        //Serial.println(pwm_value);
        
      }

      //Serial.print(fireflies_delay[i]);
      //Serial.print(" ");
    }
    // now check acceleration
    if (acceleration() > MAX_Z_ACC) {
      // if it's too big, reset all
      while (acceleration() > MAX_Z_ACC) {
        delay(500);
      };
      reset();
    }
  }
}*/