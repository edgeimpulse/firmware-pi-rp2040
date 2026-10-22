#include "MQTTClient.h"

#include "pico/time.h"

#include <stddef.h>
#include <string.h>

#define MQTT_PKT_CONNECT 0x10
#define MQTT_PKT_CONNACK 0x20
#define MQTT_PKT_PUBLISH 0x30
#define MQTT_PKT_PINGREQ 0xC0
#define MQTT_PKT_PINGRESP 0xD0
#define MQTT_PKT_DISCONNECT 0xE0

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static int mqtt_write_remaining_length(unsigned char *buf, int len)
{
    int i = 0;

    do {
        unsigned char encoded = (unsigned char)(len % 128);
        len /= 128;
        if (len > 0) {
            encoded |= 0x80;
        }
        buf[i++] = encoded;
    } while (len > 0 && i < 4);

    return i;
}

static int mqtt_encode_string(unsigned char *buf, int buf_len, const char *s)
{
    int slen;

    if (s == NULL || buf_len < 2) {
        return -1;
    }

    slen = (int)strlen(s);
    if ((slen + 2) > buf_len) {
        return -1;
    }

    buf[0] = (unsigned char)((slen >> 8) & 0xFF);
    buf[1] = (unsigned char)(slen & 0xFF);
    memcpy(&buf[2], s, (size_t)slen);

    return slen + 2;
}

void MQTTClientInit(MQTTClient *c,
                    Network *network,
                    unsigned char *sendbuf,
                    int sendbuf_size,
                    unsigned char *readbuf,
                    int readbuf_size)
{
    if (c == NULL) {
        return;
    }

    c->ipstack = network;
    c->sendbuf = sendbuf;
    c->sendbuf_size = sendbuf_size;
    c->readbuf = readbuf;
    c->readbuf_size = readbuf_size;
    c->keepAliveInterval = 60;
    c->last_tx_ms = now_ms();
    c->isconnected = 0;
}

int MQTTConnect(MQTTClient *c, MQTTPacket_connectData *options)
{
    int rc;
    int offset;
    int rem_len;
    int fh_len;
    int vh_len;
    int payload_len;
    int client_id_len;
    int total_len;
    unsigned char connect_flags;

    if (c == NULL || c->ipstack == NULL || options == NULL || options->clientID.cstring == NULL) {
        return -1;
    }

    client_id_len = (int)strlen(options->clientID.cstring);
    vh_len = 10;
    payload_len = 2 + client_id_len;
    rem_len = vh_len + payload_len;

    offset = 0;
    c->sendbuf[offset++] = MQTT_PKT_CONNECT;
    fh_len = mqtt_write_remaining_length(&c->sendbuf[offset], rem_len);
    offset += fh_len;

    c->sendbuf[offset++] = 0x00;
    c->sendbuf[offset++] = 0x04;
    c->sendbuf[offset++] = 'M';
    c->sendbuf[offset++] = 'Q';
    c->sendbuf[offset++] = 'T';
    c->sendbuf[offset++] = 'T';
    c->sendbuf[offset++] = options->MQTTVersion;

    connect_flags = 0;
    if (options->cleansession) {
        connect_flags |= 0x02;
    }
    c->sendbuf[offset++] = connect_flags;

    c->sendbuf[offset++] = (unsigned char)((options->keepAliveInterval >> 8) & 0xFF);
    c->sendbuf[offset++] = (unsigned char)(options->keepAliveInterval & 0xFF);

    rc = mqtt_encode_string(&c->sendbuf[offset], c->sendbuf_size - offset, options->clientID.cstring);
    if (rc < 0) {
        return -1;
    }
    offset += rc;

    total_len = offset;
    rc = mqtt_network_write(c->ipstack, c->sendbuf, total_len, 1000);
    if (rc != total_len) {
        return -1;
    }

    rc = mqtt_network_read(c->ipstack, c->readbuf, c->readbuf_size, 1500);
    if (rc < 4) {
        return -1;
    }

    if (c->readbuf[0] != MQTT_PKT_CONNACK || c->readbuf[1] != 0x02 || c->readbuf[3] != 0x00) {
        return -1;
    }

    c->keepAliveInterval = options->keepAliveInterval;
    c->last_tx_ms = now_ms();
    c->isconnected = 1;

    return 0;
}

int MQTTPublish(MQTTClient *c, const char *topicName, MQTTMessage *message)
{
    int topic_len;
    int rem_len;
    int offset;
    int fh_len;
    int total_len;
    int rc;

    if (c == NULL || c->ipstack == NULL || topicName == NULL || message == NULL || message->payload == NULL) {
        return -1;
    }

    if (!c->isconnected) {
        return -1;
    }

    topic_len = (int)strlen(topicName);
    rem_len = 2 + topic_len + message->payloadlen;

    offset = 0;
    c->sendbuf[offset++] = (unsigned char)(MQTT_PKT_PUBLISH | ((message->qos & 0x03) << 1) | (message->retained ? 0x01 : 0x00));
    fh_len = mqtt_write_remaining_length(&c->sendbuf[offset], rem_len);
    offset += fh_len;

    rc = mqtt_encode_string(&c->sendbuf[offset], c->sendbuf_size - offset, topicName);
    if (rc < 0) {
        return -1;
    }
    offset += rc;

    if ((offset + message->payloadlen) > c->sendbuf_size) {
        return -1;
    }

    memcpy(&c->sendbuf[offset], message->payload, (size_t)message->payloadlen);
    offset += message->payloadlen;
    total_len = offset;

    rc = mqtt_network_write(c->ipstack, c->sendbuf, total_len, 1000);
    if (rc != total_len) {
        c->isconnected = 0;
        return -1;
    }

    c->last_tx_ms = now_ms();
    return 0;
}

int MQTTYield(MQTTClient *c, int timeout_ms)
{
    uint32_t now;
    int rc;

    if (c == NULL || c->ipstack == NULL || !c->isconnected) {
        return -1;
    }

    now = now_ms();
    if (c->keepAliveInterval > 0 && (now - c->last_tx_ms) >= ((uint32_t)c->keepAliveInterval * 1000U)) {
        c->sendbuf[0] = MQTT_PKT_PINGREQ;
        c->sendbuf[1] = 0x00;

        rc = mqtt_network_write(c->ipstack, c->sendbuf, 2, 500);
        if (rc != 2) {
            c->isconnected = 0;
            return -1;
        }

        c->last_tx_ms = now_ms();
    }

    rc = mqtt_network_read(c->ipstack, c->readbuf, c->readbuf_size, timeout_ms);
    if (rc < 0) {
        c->isconnected = 0;
        return -1;
    }

    if (rc >= 2 && c->readbuf[0] == MQTT_PKT_PINGRESP && c->readbuf[1] == 0x00) {
        return 0;
    }

    return 0;
}

int MQTTDisconnect(MQTTClient *c)
{
    if (c == NULL || c->ipstack == NULL) {
        return -1;
    }

    if (c->isconnected) {
        c->sendbuf[0] = MQTT_PKT_DISCONNECT;
        c->sendbuf[1] = 0x00;
        (void)mqtt_network_write(c->ipstack, c->sendbuf, 2, 200);
    }

    mqtt_network_disconnect(c->ipstack);
    c->isconnected = 0;

    return 0;
}

int MQTTIsConnected(MQTTClient *c)
{
    if (c == NULL) {
        return 0;
    }

    return c->isconnected ? 1 : 0;
}
