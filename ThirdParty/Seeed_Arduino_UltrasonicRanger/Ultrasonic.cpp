/*
    Ultrasonic.cpp
    A library for ultrasonic ranger

    Copyright (c) 2012 seeed technology inc.
    Website    : www.seeed.cc
    Author     : LG, FrankieChu
    Create Time: Jan 17,2013
    Change Log :

    The MIT License (MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "pico/stdlib.h"

#include "Ultrasonic.h"

unsigned long pulseIn(int pin, int state, unsigned long timeout)
{
    unsigned long startMicros = time_us_64();

    // wait for any previous pulse to end
    while (gpio_get(pin) == state) {
        tight_loop_contents();
        if (time_us_64() - startMicros > timeout)
            return 0;
    }

    // wait for the pulse to start
    while (gpio_get(pin) != state) {
        tight_loop_contents();
        if (time_us_64() - startMicros > timeout)
            return 0;
    }

    unsigned long start = time_us_64();
    // wait for the pulse to stop
    while (gpio_get(pin) == state) {
        tight_loop_contents();
        if (time_us_64() - startMicros > timeout)
            return 0;
    }
    return time_us_64() - start;
}

Ultrasonic::Ultrasonic(int pin) {
    _pin = pin;
    gpio_init(_pin);
}

long Ultrasonic::duration() {
    gpio_set_dir(_pin, GPIO_OUT);
    gpio_put(_pin, 0);
    sleep_us(2);
    gpio_put(_pin, 1);
    sleep_us(5);
    gpio_put(_pin, 0);
    gpio_set_dir(_pin, GPIO_IN);

    long duration;
    duration = pulseIn(_pin, 1, 20000UL);
    return duration;
}
/*The measured distance from the range 0 to 400 Centimeters*/
long Ultrasonic::MeasureInCentimeters(void) {
    long RangeInCentimeters;
    RangeInCentimeters = duration() / 29 / 2;
    return RangeInCentimeters;
}
/*The measured distance from the range 0 to 4000 Millimeters*/
long Ultrasonic::MeasureInMillimeters(void) {
    long RangeInMillimeters;
    RangeInMillimeters = duration() * (10/2) / 29;
    return RangeInMillimeters;
}
/*The measured distance from the range 0 to 157 Inches*/
long Ultrasonic::MeasureInInches(void) {
    long RangeInInches;
    RangeInInches = duration() / 74 / 2;
    return RangeInInches;
}