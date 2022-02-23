/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
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

/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>

#include "Ultrasonic.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "ei_device_raspberry_rp2040.h"
#include "ei_ultrasonicsensor.h"
#include "sensor_aq.h"

/* Constant defines -------------------------------------------------------- */
static float ultrasonic_data[ULTRASONIC_AXIS_SAMPLED];

Ultrasonic ultrasonic(16);

bool ei_ultrasonic_sensor_init(void)
{
    ei_add_sensor_to_fusion_list(ultrasonic_sensor);
    return true;
}

float *ei_fusion_ultrasonic_sensor_read_data(int n_samples)
{
    ultrasonic_data[0] = ultrasonic.MeasureInCentimeters();

    return ultrasonic_data;
}
