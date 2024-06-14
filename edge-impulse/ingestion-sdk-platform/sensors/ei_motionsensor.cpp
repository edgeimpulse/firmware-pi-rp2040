/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>

#include "ei_motionsensor.h"
#include <LIS2DH12.h>

#include "ei_device_raspberry_rp2040.h"
#include "sensor_aq.h"

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2 9.80665f

static float motion_data[MOTION_AXIS_SAMPLED];

bool ei_motion_sensor_init(void)
{
    uint8_t acc_type = MOTION.begin();

    if ((acc_type == 0) || (acc_type != MOTION_SENSOR_LIS2DH12)) {
        return false;
    }

    ei_add_sensor_to_fusion_list(motion_sensor);
    return true;
}

float *ei_fusion_motion_sensor_read_data(int n_samples)
{
    if (MOTION.accelerationAvailable())
    {
        MOTION.readAcceleration(motion_data[0], motion_data[1], motion_data[2]);

        motion_data[0] *= CONVERT_G_TO_MS2;
        motion_data[1] *= CONVERT_G_TO_MS2;
        motion_data[2] *= CONVERT_G_TO_MS2;
    }

#ifdef DEBUG
    for (int i = 0; i < MOTION_AXIS_SAMPLED; i++) {
        ei_printf("%f ", motion_data[i]);
    }
    ei_printf("\n");
#endif
    return motion_data;
}
