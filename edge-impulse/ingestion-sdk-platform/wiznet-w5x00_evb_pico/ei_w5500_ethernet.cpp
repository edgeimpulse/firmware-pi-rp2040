#include "ei_w5500_ethernet.h"

#if defined(EI_W5500_ETHERNET)

#include "ei_classifier_porting.h"

extern "C" {
#include "dhcp.h"
#include "socket.h"
#include "wizchip_conf.h"
}

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/unique_id.h"

#include <string.h>

#ifndef W5500_SPI_PORT
#define W5500_SPI_PORT spi0
#endif

#ifndef W5500_SPI_BAUDRATE_HZ
#define W5500_SPI_BAUDRATE_HZ 10000000
#endif

#ifndef W5500_PIN_MISO
#define W5500_PIN_MISO PICO_DEFAULT_SPI_RX_PIN
#endif

#ifndef W5500_PIN_CS
#define W5500_PIN_CS PICO_DEFAULT_SPI_CSN_PIN
#endif

#ifndef W5500_PIN_SCK
#define W5500_PIN_SCK PICO_DEFAULT_SPI_SCK_PIN
#endif

#ifndef W5500_PIN_MOSI
#define W5500_PIN_MOSI PICO_DEFAULT_SPI_TX_PIN
#endif

#ifndef W5500_PIN_RST
#define W5500_PIN_RST 20
#endif

#ifndef W5500_PIN_INT
#define W5500_PIN_INT 21
#endif

#define DHCP_SOCKET_IDX 0
#define DHCP_POLL_INTERVAL_MS 250
#define DHCP_LOG_RETRY_INTERVAL_MS 10000
#define PHY_LOG_INTERVAL_MS 5000

#ifndef EI_W5500_DEBUG
#define EI_W5500_DEBUG 0
#endif

#if EI_W5500_DEBUG
#define W5500_DBG(fmt, ...) ei_printf("[W5500][DBG] " fmt "\r\n", ##__VA_ARGS__)
#else
#define W5500_DBG(fmt, ...)
#endif

static bool g_initialized = false;
static bool g_has_lease = false;
static uint8_t g_phy_link = PHY_LINK_OFF;
static uint32_t g_last_dhcp_poll_ms = 0;
static uint32_t g_last_dhcp_tick_ms = 0;
static uint32_t g_last_dhcp_fail_log_ms = 0;
static uint32_t g_last_phy_log_ms = 0;
static uint32_t g_irq_state = 0;
static uint8_t g_last_dhcp_state = 0xFF;

static uint8_t g_dhcp_buffer[1024] = {0};

static wiz_NetInfo g_net_info = {
    .mac = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01},
    .ip = {0, 0, 0, 0},
    .sn = {0, 0, 0, 0},
    .gw = {0, 0, 0, 0},
    .dns = {0, 0, 0, 0},
    .dhcp = NETINFO_DHCP,
};

static inline uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static const char *dhcp_state_to_str(uint8_t state)
{
    switch (state) {
        case DHCP_FAILED:
            return "FAILED";
        case DHCP_RUNNING:
            return "RUNNING";
        case DHCP_IP_ASSIGN:
            return "IP_ASSIGN";
        case DHCP_IP_CHANGED:
            return "IP_CHANGED";
        case DHCP_IP_LEASED:
            return "IP_LEASED";
        case DHCP_STOPPED:
            return "STOPPED";
        default:
            return "UNKNOWN";
    }
}

static void log_net_info(void)
{
    W5500_DBG("[W5500] MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n",
              g_net_info.mac[0], g_net_info.mac[1], g_net_info.mac[2],
              g_net_info.mac[3], g_net_info.mac[4], g_net_info.mac[5]);
    W5500_DBG("[W5500] IP %u.%u.%u.%u\r\n",
              g_net_info.ip[0], g_net_info.ip[1], g_net_info.ip[2], g_net_info.ip[3]);
    W5500_DBG("[W5500] SN %u.%u.%u.%u\r\n",
              g_net_info.sn[0], g_net_info.sn[1], g_net_info.sn[2], g_net_info.sn[3]);
    W5500_DBG("[W5500] GW %u.%u.%u.%u\r\n",
              g_net_info.gw[0], g_net_info.gw[1], g_net_info.gw[2], g_net_info.gw[3]);
    W5500_DBG("[W5500] DNS %u.%u.%u.%u\r\n",
              g_net_info.dns[0], g_net_info.dns[1], g_net_info.dns[2], g_net_info.dns[3]);
}

static void update_phy_link_cache(void)
{
    uint8_t phy_link = PHY_LINK_OFF;

    if (g_initialized && ctlwizchip(CW_GET_PHYLINK, (void *)&phy_link) == 0) {
        g_phy_link = phy_link;
    }
    else {
        g_phy_link = PHY_LINK_OFF;
    }
}

static void dbg_log_assigned_ip(const char *reason)
{
    W5500_DBG("%s assigned IP=%u.%u.%u.%u GW=%u.%u.%u.%u SN=%u.%u.%u.%u DNS=%u.%u.%u.%u",
              reason,
              g_net_info.ip[0], g_net_info.ip[1], g_net_info.ip[2], g_net_info.ip[3],
              g_net_info.gw[0], g_net_info.gw[1], g_net_info.gw[2], g_net_info.gw[3],
              g_net_info.sn[0], g_net_info.sn[1], g_net_info.sn[2], g_net_info.sn[3],
              g_net_info.dns[0], g_net_info.dns[1], g_net_info.dns[2], g_net_info.dns[3]);
}

static void apply_dhcp_lease(void)
{
    getIPfromDHCP(g_net_info.ip);
    getGWfromDHCP(g_net_info.gw);
    getSNfromDHCP(g_net_info.sn);
    getDNSfromDHCP(g_net_info.dns);
    g_net_info.dhcp = NETINFO_DHCP;

    ctlnetwork(CN_SET_NETINFO, (void *)&g_net_info);

    g_has_lease = true;
    W5500_DBG("[W5500] DHCP lease acquired\r\n");
    dbg_log_assigned_ip("DHCP");
    log_net_info();
}

static void on_dhcp_assign(void)
{
    apply_dhcp_lease();
}

static void on_dhcp_update(void)
{
    W5500_DBG("[W5500] DHCP lease updated\r\n");
    apply_dhcp_lease();
}

static void on_dhcp_conflict(void)
{
    g_has_lease = false;
    W5500_DBG("[W5500] DHCP conflict detected\r\n");
}

static inline void wizchip_select(void)
{
    gpio_put(W5500_PIN_CS, 0);
}

static inline void wizchip_deselect(void)
{
    gpio_put(W5500_PIN_CS, 1);
}

static uint8_t wizchip_read(void)
{
    uint8_t tx = 0xFF;
    uint8_t rx = 0x00;
    spi_write_read_blocking(W5500_SPI_PORT, &tx, &rx, 1);
    return rx;
}

static void wizchip_write(uint8_t tx_data)
{
    spi_write_blocking(W5500_SPI_PORT, &tx_data, 1);
}

static void wizchip_read_burst(uint8_t *buffer, uint16_t len)
{
    uint8_t tx = 0xFF;
    spi_read_blocking(W5500_SPI_PORT, tx, buffer, len);
}

static void wizchip_write_burst(uint8_t *buffer, uint16_t len)
{
    spi_write_blocking(W5500_SPI_PORT, buffer, len);
}

static void wizchip_critical_enter(void)
{
    g_irq_state = save_and_disable_interrupts();
}

static void wizchip_critical_exit(void)
{
    restore_interrupts(g_irq_state);
}

static void w5500_spi_init(void)
{
    spi_init(W5500_SPI_PORT, W5500_SPI_BAUDRATE_HZ);

    gpio_set_function(W5500_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(W5500_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(W5500_PIN_MISO, GPIO_FUNC_SPI);

    gpio_init(W5500_PIN_CS);
    gpio_set_dir(W5500_PIN_CS, GPIO_OUT);
    gpio_put(W5500_PIN_CS, 1);

    gpio_init(W5500_PIN_INT);
    gpio_set_dir(W5500_PIN_INT, GPIO_IN);
    gpio_pull_up(W5500_PIN_INT);

    W5500_DBG("SPI initialized port=spi0 baud=%u SCK=%u MOSI=%u MISO=%u CS=%u INT=%u",
              (unsigned)W5500_SPI_BAUDRATE_HZ,
              (unsigned)W5500_PIN_SCK,
              (unsigned)W5500_PIN_MOSI,
              (unsigned)W5500_PIN_MISO,
              (unsigned)W5500_PIN_CS,
              (unsigned)W5500_PIN_INT);
}

static void w5500_reset(void)
{
    gpio_init(W5500_PIN_RST);
    gpio_set_dir(W5500_PIN_RST, GPIO_OUT);

    gpio_put(W5500_PIN_RST, 0);
    sleep_ms(100);
    gpio_put(W5500_PIN_RST, 1);
    sleep_ms(100);

    W5500_DBG("Reset toggled on RST=%u", (unsigned)W5500_PIN_RST);
}

static bool w5500_check_version(void)
{
    uint8_t version = getVERSIONR();
    if (version != 0x04) {
        W5500_DBG("[W5500] version mismatch: 0x%02X\r\n", version);
        return false;
    }

    W5500_DBG("[W5500] version OK: 0x%02X\r\n", version);
    return true;
}

static bool w5500_hw_init(void)
{
    uint8_t memsize[2][8] = {
        {2, 2, 2, 2, 2, 2, 2, 2},
        {2, 2, 2, 2, 2, 2, 2, 2},
    };

    w5500_spi_init();

    reg_wizchip_cris_cbfunc(wizchip_critical_enter, wizchip_critical_exit);
    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
    reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
    reg_wizchip_spiburst_cbfunc(wizchip_read_burst, wizchip_write_burst);

    w5500_reset();

    if (ctlwizchip(CW_INIT_WIZCHIP, (void *)memsize) == -1) {
        W5500_DBG("[W5500] CW_INIT_WIZCHIP failed\r\n");
        return false;
    }

    W5500_DBG("WIZCHIP memory initialized (8 sockets, TX/RX 2KB each)");

    if (!w5500_check_version()) {
        return false;
    }

    return true;
}

static void init_mac_address(void)
{
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);

    g_net_info.mac[0] = 0x00;   
    g_net_info.mac[1] = 0x08;   
    g_net_info.mac[2] = 0xDC;  
    g_net_info.mac[3] = id.id[4];
    g_net_info.mac[4] = id.id[5];
    g_net_info.mac[5] = id.id[6];
}

bool ei_w5500_ethernet_init(void)
{
    if (g_initialized) {
        return false;
    }

    init_mac_address();
    W5500_DBG("Generated MAC %02X:%02X:%02X:%02X:%02X:%02X",
              g_net_info.mac[0], g_net_info.mac[1], g_net_info.mac[2],
              g_net_info.mac[3], g_net_info.mac[4], g_net_info.mac[5]);

    if (!w5500_hw_init()) {
        return false;
    }

    ctlnetwork(CN_SET_NETINFO, (void *)&g_net_info);

    W5500_DBG("Post-init netinfo IP=%u.%u.%u.%u (expected 0.0.0.0 before DHCP lease)",
              g_net_info.ip[0], g_net_info.ip[1], g_net_info.ip[2], g_net_info.ip[3]);

    DHCP_init(DHCP_SOCKET_IDX, g_dhcp_buffer);
    reg_dhcp_cbfunc(on_dhcp_assign, on_dhcp_update, on_dhcp_conflict);

    g_initialized = true;
    g_last_dhcp_tick_ms = now_ms();
    g_last_dhcp_poll_ms = g_last_dhcp_tick_ms;
    g_last_dhcp_fail_log_ms = g_last_dhcp_tick_ms;
    g_last_phy_log_ms = g_last_dhcp_tick_ms;
    update_phy_link_cache();

    W5500_DBG("[W5500] Ethernet init complete, starting DHCP\r\n");

    return true;
}

bool ei_w5500_ethernet_get_netinfo(ei_w5500_ethernet_netinfo_t *netinfo)
{
    if (netinfo == nullptr) {
        return false;
    }

    uint32_t irq_state = save_and_disable_interrupts();

    memset(netinfo, 0, sizeof(*netinfo));
    netinfo->initialized = g_initialized;
    netinfo->dhcp_leased = g_has_lease;
    netinfo->phy_link = g_phy_link;

    if (g_initialized) {
        memcpy(netinfo->mac, g_net_info.mac, sizeof(netinfo->mac));
        memcpy(netinfo->ip, g_net_info.ip, sizeof(netinfo->ip));
        memcpy(netinfo->sn, g_net_info.sn, sizeof(netinfo->sn));
        memcpy(netinfo->gw, g_net_info.gw, sizeof(netinfo->gw));
        memcpy(netinfo->dns, g_net_info.dns, sizeof(netinfo->dns));
    }

    restore_interrupts(irq_state);

    return true;
}

void ei_w5500_ethernet_poll(void)
{
    if (!g_initialized) {
        return;
    }

    const uint32_t now = now_ms();

    while ((now - g_last_dhcp_tick_ms) >= 1000) {
        DHCP_time_handler();
        g_last_dhcp_tick_ms += 1000;
    }

    if ((now - g_last_dhcp_poll_ms) < DHCP_POLL_INTERVAL_MS) {
        return;
    }
    g_last_dhcp_poll_ms = now;

    uint8_t dhcp_state = DHCP_run();

    if (dhcp_state != g_last_dhcp_state) {
        W5500_DBG("DHCP state changed: %s (%u)", dhcp_state_to_str(dhcp_state), (unsigned)dhcp_state);
        g_last_dhcp_state = dhcp_state;
    }

    if ((now - g_last_phy_log_ms) >= PHY_LOG_INTERVAL_MS) {
        update_phy_link_cache();
        W5500_DBG("PHY link: %s", (g_phy_link == PHY_LINK_ON) ? "UP" : "DOWN");
        g_last_phy_log_ms = now;
    }

    if (dhcp_state == DHCP_IP_LEASED) {
        if (!g_has_lease) {
            apply_dhcp_lease();
        }
    }
    else if (dhcp_state == DHCP_FAILED) {
        g_has_lease = false;
        if ((now - g_last_dhcp_fail_log_ms) >= DHCP_LOG_RETRY_INTERVAL_MS) {
            W5500_DBG("[W5500] DHCP not acquired yet, retrying in background\r\n");
            g_last_dhcp_fail_log_ms = now;
        }
    }
}

#else

void ei_w5500_ethernet_init(void)
{
}

void ei_w5500_ethernet_poll(void)
{
}

bool ei_w5500_ethernet_get_netinfo(ei_w5500_ethernet_netinfo_t *netinfo)
{
    (void)netinfo;
    return false;
}

#endif
