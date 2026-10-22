#ifndef EI_W5500_MQTT_INTERFACE_H_
#define EI_W5500_MQTT_INTERFACE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t socket;
    uint8_t connected;
} Network;

void mqtt_network_init(Network *n, uint8_t socket_no);
int mqtt_network_connect(Network *n, const uint8_t *addr, uint16_t port);
int mqtt_network_read(Network *n, unsigned char *buffer, int len, int timeout_ms);
int mqtt_network_write(Network *n, const unsigned char *buffer, int len, int timeout_ms);
void mqtt_network_disconnect(Network *n);

#ifdef __cplusplus
}
#endif

#endif
