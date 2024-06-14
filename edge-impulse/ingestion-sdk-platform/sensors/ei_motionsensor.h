#ifndef _EI_MOTIONSENSOR_H
#define _EI_MOTIONSENSOR_H

/* Include ----------------------------------------------------------------- */
#include "ei_fusion.h"
#include "ei_sampler.h"

/** Number of axis used and sample data format */
#define MOTION_AXIS_SAMPLED 3

/* Function prototypes ----------------------------------------------------- */
bool ei_motion_sensor_init(void);
float *ei_fusion_motion_sensor_read_data(int n_samples);

static const ei_device_fusion_sensor_t motion_sensor = {
    // name of sensor module to be displayed in fusion list
    "Motion",
    // number of sensor module axis
    MOTION_AXIS_SAMPLED,
    // sampling frequencies
    { 100.0f, 62.5f, 20.0f },
    // axis name and units payload (must be same order as read in)
    {
        { "accX", "m/s2" },
        { "accY", "m/s2" },
        { "accZ", "m/s2" },
    },
    // reference to read data function
    &ei_fusion_motion_sensor_read_data
};

#endif /* _EI_MOTIONSENSOR_H */
