#include "ei_w5500_mqtt.h"

#if defined(EI_W5500_ETHERNET) && defined(EI_MQTT_PUBLISH)

#include "ei_classifier_porting.h"
#include "ei_w5500_ethernet.h"
#include "mqtt/MQTTClient.h"
#include "mqtt/mqtt_interface.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef EI_MQTT_BROKER_HOST
#define EI_MQTT_BROKER_HOST "192.168.11.100"
#endif

#ifndef EI_MQTT_BROKER_PORT
#define EI_MQTT_BROKER_PORT 1883
#endif

#ifndef EI_MQTT_CLIENT_ID
#define EI_MQTT_CLIENT_ID "edge-impulse-w5500"
#endif

#ifndef EI_MQTT_TOPIC
#define EI_MQTT_TOPIC "edgeimpulse/result"
#endif

#ifndef EI_MQTT_SOCKET
#define EI_MQTT_SOCKET 1
#endif

#ifndef EI_MQTT_KEEPALIVE_SEC
#define EI_MQTT_KEEPALIVE_SEC 30
#endif

static bool g_mqtt_initialized = false;
static uint8_t g_broker_ip[4] = {0};

static Network g_mqtt_network;
static MQTTClient g_mqtt_client;
static unsigned char g_mqtt_send_buf[256];
static unsigned char g_mqtt_read_buf[256];

static bool parse_ipv4(const char *host, uint8_t out_ip[4])
{
    unsigned long parts[4] = {0, 0, 0, 0};
    const char *cursor = host;
    char *end_ptr = nullptr;

    if (host == nullptr || out_ip == nullptr) {
        return false;
    }

    for (int i = 0; i < 4; i++) {
        parts[i] = strtoul(cursor, &end_ptr, 10);
        if (end_ptr == cursor || parts[i] > 255UL) {
            return false;
        }

        if (i < 3) {
            if (*end_ptr != '.') {
                return false;
            }
            cursor = end_ptr + 1;
        }
        else {
            if (*end_ptr != '\0') {
                return false;
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        out_ip[i] = (uint8_t)parts[i];
    }

    return true;
}

static bool is_network_ready(void)
{
    ei_w5500_ethernet_netinfo_t netinfo;

    if (!ei_w5500_ethernet_get_netinfo(&netinfo)) {
        return false;
    }

    if (!netinfo.initialized || !netinfo.dhcp_leased) {
        return false;
    }

    return (netinfo.phy_link == EI_W5500_PHY_LINK_ON);
}

bool ei_w5500_mqtt_init(void)
{
    if (g_mqtt_initialized) {
        return true;
    }

    if (!parse_ipv4(EI_MQTT_BROKER_HOST, g_broker_ip)) {
        ei_printf("[MQTT] Invalid broker IPv4: %s\r\n", EI_MQTT_BROKER_HOST);
        return false;
    }

    mqtt_network_init(&g_mqtt_network, EI_MQTT_SOCKET);
    MQTTClientInit(&g_mqtt_client,
                   &g_mqtt_network,
                   g_mqtt_send_buf,
                   sizeof(g_mqtt_send_buf),
                   g_mqtt_read_buf,
                   sizeof(g_mqtt_read_buf));

    g_mqtt_initialized = true;
    return true;
}

bool ei_w5500_mqtt_is_connected(void)
{
    if (!g_mqtt_initialized) {
        return false;
    }

    return MQTTIsConnected(&g_mqtt_client) != 0;
}

bool ei_w5500_mqtt_connect(void)
{
    MQTTPacket_connectData connect_data = MQTTPacket_connectData_initializer;

    if (!ei_w5500_mqtt_init()) {
        return false;
    }

    if (!is_network_ready()) {
        return false;
    }

    if (ei_w5500_mqtt_is_connected()) {
        return true;
    }

    if (mqtt_network_connect(&g_mqtt_network, g_broker_ip, EI_MQTT_BROKER_PORT) != 0) {
        return false;
    }

    connect_data.clientID.cstring = EI_MQTT_CLIENT_ID;
    connect_data.keepAliveInterval = EI_MQTT_KEEPALIVE_SEC;
    connect_data.cleansession = 1;

    if (MQTTConnect(&g_mqtt_client, &connect_data) != 0) {
        mqtt_network_disconnect(&g_mqtt_network);
        return false;
    }

    return true;
}

bool ei_w5500_mqtt_yield(void)
{
    if (!g_mqtt_initialized || !ei_w5500_mqtt_is_connected()) {
        return false;
    }

    if (MQTTYield(&g_mqtt_client, 1) != 0) {
        (void)MQTTDisconnect(&g_mqtt_client);
        return false;
    }

    return true;
}

bool ei_w5500_mqtt_publish_result(const ei_impulse_result_t *result)
{
    const char *best_label = nullptr;
    float best_score = 0.0f;
    char payload[128];
    MQTTMessage message;

    if (result == nullptr) {
        return false;
    }

    if (!ei_w5500_mqtt_connect()) {
        return false;
    }

    // TODO: Add object detection MQTT payload publishing when OD transport format is defined.
    if (result->bounding_boxes_count > 0) {
        return false;
    }

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        const float score = result->classification[ix].value;
        if (best_label == nullptr || score > best_score) {
            best_score = score;
            best_label = result->classification[ix].label;
        }
    }

    if (best_label == nullptr) {
        return false;
    }

    (void)snprintf(payload,
                   sizeof(payload),
                   "{\"label\":\"%s\",\"score\":%.5f}",
                   best_label,
                   (double)best_score);

    memset(&message, 0, sizeof(message));
    message.qos = 0;
    message.retained = 0;
    message.payload = payload;
    message.payloadlen = (int)strlen(payload);

    if (MQTTPublish(&g_mqtt_client, EI_MQTT_TOPIC, &message) != 0) {
        (void)MQTTDisconnect(&g_mqtt_client);
        return false;
    }

    return true;
}

#else

bool ei_w5500_mqtt_init(void)
{
    return false;
}

bool ei_w5500_mqtt_is_connected(void)
{
    return false;
}

bool ei_w5500_mqtt_connect(void)
{
    return false;
}

bool ei_w5500_mqtt_yield(void)
{
    return false;
}

bool ei_w5500_mqtt_publish_result(const ei_impulse_result_t *result)
{
    (void)result;
    return false;
}

#endif
