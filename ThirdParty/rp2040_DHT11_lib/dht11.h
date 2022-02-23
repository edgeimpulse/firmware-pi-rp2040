/* Modified from sample RP2040 DHT11 code by
 * Raspberry Pi (Trading) Ltd.
 * 2022 EdgeImpulse Inc.
 * Dmitry Maslov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef DHT11_H
#define DHT11_H

#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

typedef struct {
    float humidity;
    float temp_celsius;
} dht_reading;

class DHT11 {
public:
    DHT11(uint8_t pin);
    bool begin(void);
    bool read();
    float readTemperature(bool S = false);
    float readHumidity(void);
    float convertCtoF(void);

private:
    long duration();

    uint8_t _pin;
    unsigned long _lastreadtime;
    bool firstreading;
};

#endif