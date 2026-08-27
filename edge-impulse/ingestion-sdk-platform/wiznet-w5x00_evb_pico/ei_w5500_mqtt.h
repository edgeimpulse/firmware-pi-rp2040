#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "edge-impulse-sdk/classifier/ei_classifier_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool ei_w5500_mqtt_init(void);
bool ei_w5500_mqtt_is_connected(void);
bool ei_w5500_mqtt_connect(void);
bool ei_w5500_mqtt_yield(void);
bool ei_w5500_mqtt_publish_result(const ei_impulse_result_t *result);

#ifdef __cplusplus
}
#endif
