#include "mqtt_interface.h"

#include "pico/stdlib.h"

#include "socket.h"
#include "wizchip_conf.h"

static int is_socket_established(uint8_t sn)
{
    return getSn_SR(sn) == SOCK_ESTABLISHED;
}

void mqtt_network_init(Network *n, uint8_t socket_no)
{
    if (n == 0) {
        return;
    }

    n->socket = socket_no;
    n->connected = 0;
}

int mqtt_network_connect(Network *n, const uint8_t *addr, uint16_t port)
{
    int32_t rc;
    uint32_t start_ms;

    if (n == 0 || addr == 0) {
        return -1;
    }

    close(n->socket);

    rc = socket(n->socket, Sn_MR_TCP, 0, 0);
    if (rc < 0) {
        return -1;
    }

    rc = connect(n->socket, (uint8_t *)addr, port);
    if (rc < 0) {
        close(n->socket);
        return -1;
    }

    start_ms = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start_ms) < 3000U) {
        if (is_socket_established(n->socket)) {
            n->connected = 1;
            return 0;
        }
        sleep_ms(2);
    }

    close(n->socket);
    n->connected = 0;
    return -1;
}

int mqtt_network_read(Network *n, unsigned char *buffer, int len, int timeout_ms)
{
    uint32_t start_ms;
    uint16_t available;
    int32_t rc;

    if (n == 0 || buffer == 0 || len <= 0 || n->connected == 0) {
        return -1;
    }

    start_ms = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start_ms) < (uint32_t)timeout_ms) {
        if (!is_socket_established(n->socket)) {
            n->connected = 0;
            return -1;
        }

        available = getSn_RX_RSR(n->socket);
        if (available > 0) {
            int32_t to_read = (available > (uint16_t)len) ? (int32_t)len : (int32_t)available;
            rc = recv(n->socket, buffer, to_read);
            return (rc < 0) ? -1 : (int)rc;
        }

        sleep_ms(2);
    }

    return 0;
}

int mqtt_network_write(Network *n, const unsigned char *buffer, int len, int timeout_ms)
{
    uint32_t start_ms;
    int32_t rc;

    (void)timeout_ms;

    if (n == 0 || buffer == 0 || len <= 0 || n->connected == 0) {
        return -1;
    }

    start_ms = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start_ms) < 3000U) {
        if (!is_socket_established(n->socket)) {
            n->connected = 0;
            return -1;
        }

        rc = send(n->socket, (uint8_t *)buffer, (uint16_t)len);
        if (rc > 0) {
            return (int)rc;
        }

        if (rc < 0) {
            n->connected = 0;
            return -1;
        }

        sleep_ms(2);
    }

    return -1;
}

void mqtt_network_disconnect(Network *n)
{
    if (n == 0) {
        return;
    }

    disconnect(n->socket);
    close(n->socket);
    n->connected = 0;
}
