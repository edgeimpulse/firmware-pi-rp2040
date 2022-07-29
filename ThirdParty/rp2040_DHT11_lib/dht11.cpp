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

#include "dht11.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

#include "ei_device_raspberry_rp2040.h"

#include "FreeRTOS.h"
#include "task.h"

const uint MAX_TRIES = 5;
const uint MAX_TIMINGS = 85;
bool data_correct = false;

dht_reading reading;

DHT11::DHT11(uint8_t pin)
{
    _pin = pin;
}

bool DHT11::begin(void)
{
    gpio_init(_pin);

    gpio_set_dir(_pin, GPIO_OUT);
    gpio_put(_pin, 1);
    ei_sleep(10);

    for (uint i = 0; i < MAX_TRIES; i++) {

        data_correct = read();
        ei_sleep(100);
        if (data_correct > 0) { break; }
    }

    return (bool)data_correct;
}

float DHT11::readHumidity(void)
{

    return reading.humidity;
}

float DHT11::readTemperature(bool S)
{

    if (S) {
        return convertCtoF();
    }

    return reading.temp_celsius;
}

float DHT11::convertCtoF()
{
    return reading.temp_celsius * 9 / 5 + 32;
}

uint8_t DHT11::read(void)
{
    uint8_t res = 1;
    int data[5] = {0, 0, 0, 0, 0};
    uint last = 1;
    uint j = 0;
    dht_reading *result = &reading;

    portDISABLE_INTERRUPTS();
    uint32_t ints = save_and_disable_interrupts();

    gpio_set_dir(_pin, GPIO_OUT);
    gpio_put(_pin, 0);
    ei_sleep(20);

    gpio_put(_pin, 1);
    sleep_us(40);
    gpio_set_dir(_pin, GPIO_IN);

    for (uint i = 0; i < MAX_TIMINGS; i++) {
        uint count = 0;
        while (gpio_get(_pin) == last) {
            count++;
            busy_wait_us_32(1);
            if (count == 255)
                break;
        }
        last = gpio_get(_pin);
        if (count == 255)
            break;

        if ((i >= 4) && (i % 2 == 0)) {
            data[j / 8] <<= 1;
            if (count > 50)
                data[j / 8] |= 1;
            j++;
        }
    }

    if ((j >= 40) && (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF))) {
        result->humidity = (float)((data[0] << 8) + data[1]) / 10;
        if (result->humidity > 100) {
            result->humidity = data[0];
        }
        result->temp_celsius = (float)(((data[2] & 0x7F) << 8) + data[3]) / 10;
        if (result->temp_celsius > 125) {
            result->temp_celsius = data[2];
        }
        if (data[2] & 0x80) {
            result->temp_celsius = -result->temp_celsius;
        }
        }
    else {
        DEBUG_PRINT("Bad data %d %d %d %d %d\n", data[0], data[1], data[2], data[3], data[4]);
        res = 2;
    }

    if (data[0] == 0) {
        DEBUG_PRINT("Zero data %d %d %d %d %d\n", data[0], data[1], data[2], data[3], data[4]);
        res = 0;
    }

    portENABLE_INTERRUPTS();
    restore_interrupts(ints);
    DEBUG_PRINT(
        "Humidity = %.1f%%, Temperature = %.1fC (%.1fF)\n",
        reading.humidity,
        reading.temp_celsius,
        convertCtoF());
    return res;
}
