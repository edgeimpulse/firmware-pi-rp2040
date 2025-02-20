/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Modified from sample RP2040 DHT11 code by
 * Raspberry Pi (Trading) Ltd.
 * Dmitry Maslov
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
