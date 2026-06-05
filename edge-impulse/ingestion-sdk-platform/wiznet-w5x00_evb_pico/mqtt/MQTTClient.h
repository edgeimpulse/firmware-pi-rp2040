#ifndef EI_W5500_MQTT_CLIENT_H_
#define EI_W5500_MQTT_CLIENT_H_

#include <stdint.h>

#include "mqtt_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *cstring;
} MQTTString;

typedef struct {
    unsigned char qos;
    unsigned char retained;
    const void *payload;
    int payloadlen;
} MQTTMessage;

typedef struct {
    unsigned char MQTTVersion;
    unsigned short keepAliveInterval;
    unsigned char cleansession;
    MQTTString clientID;
} MQTTPacket_connectData;

typedef struct {
    Network *ipstack;
    unsigned char *sendbuf;
    int sendbuf_size;
    unsigned char *readbuf;
    int readbuf_size;
    unsigned short keepAliveInterval;
    uint32_t last_tx_ms;
    unsigned char isconnected;
} MQTTClient;

#define MQTTPacket_connectData_initializer { 4, 60, 1, { 0 } }

void MQTTClientInit(MQTTClient *c,
                    Network *network,
                    unsigned char *sendbuf,
                    int sendbuf_size,
                    unsigned char *readbuf,
                    int readbuf_size);
int MQTTConnect(MQTTClient *c, MQTTPacket_connectData *options);
int MQTTPublish(MQTTClient *c, const char *topicName, MQTTMessage *message);
int MQTTYield(MQTTClient *c, int timeout_ms);
int MQTTDisconnect(MQTTClient *c);
int MQTTIsConnected(MQTTClient *c);

#ifdef __cplusplus
}
#endif

#endif
