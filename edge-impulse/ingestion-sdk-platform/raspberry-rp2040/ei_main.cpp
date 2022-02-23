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

#include <stdio.h>

#include "ei_device_raspberry_rp2040.h"
#include "ei_main.h"
#include "ei_rp2040_fs_commands.h"

#include "ei_at_handlers.h"
#include "ei_classifier_porting.h"
#include "ei_run_impulse.h"

#include "ei_dht11sensor.h"
#include "ei_inertialsensor.h"
#include "ei_analogsensor.h"
#include "ei_ultrasonicsensor.h"

EiDeviceInfo *EiDevInfo = dynamic_cast<EiDeviceInfo *>(&EiDevice);
static ATServer *at;

void ei_init(void)
{

    ei_printf(
        "Hello from Edge Impulse Device SDK.\r\n"
        "Compiled on %s %s\r\n",
        __DATE__,
        __TIME__);

    /* Setup the inertial sensor */
    if (ei_inertial_sensor_init() == false) {
        ei_printf("Inertial sensor communication error occurred\r\n");
    }

    /* Setup the temp&humidity sensor */
    if (ei_dht11_sensor_init() == false) {
        ei_printf("DHT11 initialization failed\r\n");
    }

    /* Setup the ultrasonic sensor */
    if (ei_ultrasonic_sensor_init() == false) {
        ei_printf("Ultrasonic ranger initialization failed\r\n");
    }

    /* Setup the light sensor */
    if (ei_analog_sensor_init() == false) {
        ei_printf("ADC sensor initialization failed\r\n");
    }

    ei_rp2040_fs_init();

#ifdef DEBUG
    test_flash();
#endif

    at = ei_at_init();
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    at->print_prompt();
}

void ei_main(void)
{
    /* handle command comming from uart */
    char data = ei_get_serial_byte();

    while (data != 0xFF) {
        at->handle(data);
        data = ei_get_serial_byte();
    }
}
