# Fireflies in a jar

`I wanted to make something special for my girlfriend. You know, I'm terrible at making gifts.`

*So I thought: why don't I make something by myself?* Lucky me, I had many Arduino Pro Micro, a bunch of leds, a big lipo battery and enough cables to connect Rome to Tokyo.

I bought a glass jar and I filled it with led. By making it a little bit opaque (*pro tip: don't try to use a dremel to grind, the jar exploded in my hands and glass shards flew everywhere*) and hopefully only using it in the dark, it makes a pretty cool effect!

The lights blink for a random period of time and are each delayed by a random amount of time. After each time turn on, the delay is slightly adjusted so it's closer to the others. After a while, they will all blink at the same time!

After turning the Arduino on by pressing the button on the jar lid, shake it gently to wake up the Fireflies!

## Hardware

I used:
- Sparkfun Arduino Pro Micro (it has a lot of available pins)
- 13 leds with one resistor each
- A button wired between the `RST` and `GND` pins of the microcontroller to make life a little easier when I wanted to flash it again
- A LiPo battery (a big one, `7000mAh) with a charging controller to prevent explosions (nobody likes those)
- A cheap accelerometer (I used a `GY521`, but anything cheap and not fancy is good) to detect when the jar is shaken
- A glass jar with a cork lid

I would say that the total price is `15€ if you buy the electronic parts from Aliexpress (as I did).

## Wiring
- Pins `A0` to `A3` are connected to the accelerometer
- Pin `2` is connected to a pushbutton (this pin must have interrupt)
- Pin `3` is connected to the status led
- Pins `0, 1, 4, 5, 6, 7, 8, 9, 10, 13, 14, 15` are connected to the yellow LEDS
- Pin `A3` should be left floating to seed the random function generator

All these variables can be changed as they are declared on top.

## Dependencies
This firmware makes use of the [SoftPWM](https://github.com/bhagman/SoftPWM) library by [bhagman](https://github.com/bhagman).

To handle the deep sleep, it uses the built in `avr/sleep.h` library.

## Credits
This project is distributed under Attribution 4.0 International (CC BY 4.0) license.
