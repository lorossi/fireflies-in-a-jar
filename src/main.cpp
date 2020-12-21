#include <Arduino.h>
#include <SoftPWM.h>
#include <avr/sleep.h>


#define FIREFLIES_NUM 4 // number of fireflies
#define ACC_PIN 15 // A1 - pin connected to the first accelerometer axis - the other 2 are the following pins
#define SLEEP_PIN 2 // pin connected to the power button - must be interrupt
#define STATUS_PIN 3 // status led pin
#define LIGHT_DURATION 5000 // duration of an animation
#define PAUSE_FRACTION 3 // pause will last LIGHT_DURATION * 1 / PAUSE_FRACTION
#define OFFSET_DIV 25 // ratio of movement towards average
#define MAX_PWM 200 // max PWM duty cicle of each firefly
#define MAX_ACC 1.75 // max acceleration before reset
#define MAX_EQUAL_ITERATIONS 6 // number of iterations with same offset before the whole program is reset with random values


byte fireflies_pins[FIREFLIES_NUM] = {5, 6, 7, 8}; // this array should be changed to keep track of where the LEDs are connected
byte fireflies_value[FIREFLIES_NUM]; // this array contains the current PWM value of the led
byte fireflies_offset[FIREFLIES_NUM]; // this array contains the current offset (animation delay) of the led
byte equal_iterations; // number of animations with the same offset for each led

bool fireflies_updated[FIREFLIES_NUM]; // has the firefly offset already been updated in this iteration?
bool fireflies_first[FIREFLIES_NUM]; // is this the first iteration of the firefly?
bool shook; // has the arduino been shook?

unsigned long started; // start time (msec)
const unsigned int loop_interval = LIGHT_DURATION / 255; // delay between loop function executions

void reset();
float acceleration();
void turn_off();
void wake_up();
void go_to_sleep();

// reset all variables
void reset() {
  // random seeding with floating pin
  randomSeed(analogRead(A0));

  // reset each firefly array
  for (byte i = 0; i < FIREFLIES_NUM; i++)
  {
    fireflies_value[i] = 0;
    fireflies_offset[i] = random(255);
    fireflies_updated[i] = true;
    fireflies_first[i] = true;
  }

  // reset the number of iterations with the same offset for each firefly
  equal_iterations = 0;
  // start time (msec)
  started = millis();
}

// calculate acceleration module
float acceleration()
{
  float ax = float(analogRead(ACC_PIN)) / 1024 * 3;
  float ay = float(analogRead(ACC_PIN + 1)) / 1024 * 3;
  float az = float(analogRead(ACC_PIN + 2)) / 1024 * 3;
  float module = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));
  return module;
}

// turn off all leds
void turn_off() {
  for (byte i = 0; i < FIREFLIES_NUM; i++) {
    SoftPWMSet(fireflies_pins[i], 0);
  }
}

// wake up arduino from deep sleep
void wake_up() {
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(SLEEP_PIN));
}

// put arduino into deep sleep
void go_to_sleep() {
  // turn off status led
  SoftPWMSet(STATUS_PIN, 0);
  // turn off all other leds
  turn_off();
  // wait for the to release the button
  while (digitalRead(SLEEP_PIN) == LOW) {
    delay(50);
  }
  // go to sleep
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  // interrupt to wake up
  attachInterrupt(digitalPinToInterrupt(SLEEP_PIN), wake_up, LOW);
  // now sleep
  sleep_cpu();

  // this is where the code executions starts again
  shook = false;
  reset();

  // wait for user to release the button
  while (digitalRead(SLEEP_PIN) == LOW) {
    delay(50);
  }
  // turn on status led
  SoftPWMSet(STATUS_PIN, 5);
}

void setup() {
  // set each pin direction
  // LEDS
  for (byte i = 0; i < FIREFLIES_NUM; i++) {
    pinMode(fireflies_pins[i], OUTPUT);
  }
  // Accelerometers pins
  for (byte i = 0; i < 3; i++) {
    pinMode(ACC_PIN + 1, INPUT);
  }
  // power button
  pinMode(SLEEP_PIN, INPUT_PULLUP);
  // status pin
  pinMode(STATUS_PIN, OUTPUT);

  SoftPWMBegin();

  shook = false;
  reset();
  // now sleep until there's user interaction
  go_to_sleep();
}

void loop() {
  if (millis() - started > loop_interval) {
    // enough time has passed
    // if the user is pressing the power button, go to sleep
    if (digitalRead(SLEEP_PIN) == LOW) {
      go_to_sleep();
    }
    // update started time
    started = millis();
    if (shook) {
      // the arduino has been shook, we can animate
      // average offset calculation
      float total_offset = 0;
      for (byte i = 0; i < FIREFLIES_NUM; i++) {
        total_offset += fireflies_offset[i];
      }
      byte avg_offset = total_offset / FIREFLIES_NUM;

      // now start animating fireflies
      for (byte i = 0; i < FIREFLIES_NUM; i++) {
        // calculate PWM level (0-255)
        byte level = fireflies_offset[i] + fireflies_value[i];

        // check if it's time to actually animate or it's just pause time
        if (level > 255 / PAUSE_FRACTION) {
          float value = map(level, 255 / PAUSE_FRACTION, 255, 0, MAX_PWM);

          // if this isn't the first iteration, light the led up
          // otherwise it woulnd't start from off and weird things would happen
          if (!fireflies_first[i]){
            SoftPWMSet(fireflies_pins[i], value);
          } 

          // update firefly offset (delay) only if the animation has ended
          if (level == 255 && !fireflies_updated[i]) {
            // this prevents multiple updates
            fireflies_updated[i] = true;

            // delta offset calculation and rounding
            float offset_diff = (float) (fireflies_offset[i] - avg_offset) / OFFSET_DIV;
            if (offset_diff > 0.5 && offset_diff < 1) {
              offset_diff = 1;
            } else if (offset_diff < -0.5 && offset_diff > -1) {
              offset_diff = -1;
            }
            fireflies_offset[i] += offset_diff;
          }
        } else {
          // it's pause time, reset all and turn off lights
          if (fireflies_updated[i])  fireflies_updated[i] = false;
          if (fireflies_first[i]) fireflies_first[i] = false;
          SoftPWMSet(fireflies_pins[i], 0);
        }

        // increment firefly light level
        fireflies_value[i]++;
      }

      // check if all fireflies have the same delay (approx)
      bool all_equal = true || (equal_iterations != 0);
      for (byte i = 1; i < FIREFLIES_NUM && all_equal; i++) {
        all_equal = abs(fireflies_offset[i] - fireflies_offset[i - 1]) < 1.5;
      }
      // if so, check how many times it has happened before resetting
      if (all_equal) {
        equal_iterations++;
        if (equal_iterations > MAX_EQUAL_ITERATIONS) reset();
      }
    }

    // check if the arduino is being shook enough
    if (acceleration() > MAX_ACC) {
      if (shook) {
        // if it has already been shook, reset all and go on
        turn_off();
        reset();
      } else {
        // otherwise, start everything up
        shook = true;
      }

      // wait until acceleration goes under threshold to avoid multiple firings
      while (acceleration() > MAX_ACC) {
        delay(50);
      }
    }
  }
}
