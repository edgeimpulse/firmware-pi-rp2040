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

#include "ei_inertialsensor.h"
#include <LSM6DSOX.h>

#include "ei_device_raspberry_rp2040.h"
#include "sensor_aq.h"

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2 9.80665f

static float imu_data[INERTIAL_AXIS_SAMPLED];

bool ei_inertial_sensor_init(void)
{
    uint8_t acc_type = IMU.begin();
    if (!acc_type) {
        return false;
    }
    else if (acc_type == 1) {
        DEBUG_PRINT("Using LSM6DSOX\n");
    }
    else if (acc_type == 2) {
        DEBUG_PRINT("Using LSM6DS3\n");        
    }

    ei_add_sensor_to_fusion_list(inertial_sensor);
    return true;
}

float *ei_fusion_inertial_sensor_read_data(int n_samples)
{

    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(imu_data[0], imu_data[1], imu_data[2]);

        imu_data[0] *= CONVERT_G_TO_MS2;
        imu_data[1] *= CONVERT_G_TO_MS2;
        imu_data[2] *= CONVERT_G_TO_MS2;
    }

    if (n_samples > 3 && IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(imu_data[3], imu_data[4], imu_data[5]);
    }

#ifdef DEBUG
    for (int i = 0; i < INERTIAL_AXIS_SAMPLED; i++) {
        ei_printf("%f ", imu_data[i]);
    }
    ei_printf("\n");
#endif
    return imu_data;
}
