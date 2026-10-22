#ifndef EI_W5500_ETHERNET_H_
#define EI_W5500_ETHERNET_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EI_W5500_PHY_LINK_OFF             0     ///< Link Off
#define EI_W5500_PHY_LINK_ON              1     ///< Link On

typedef struct {
	bool initialized;
	bool dhcp_leased;
	uint8_t phy_link;
	uint8_t mac[6];
	uint8_t ip[4];
	uint8_t sn[4];
	uint8_t gw[4];
	uint8_t dns[4];
} ei_w5500_ethernet_netinfo_t;

bool ei_w5500_ethernet_init(void);
void ei_w5500_ethernet_poll(void);
bool ei_w5500_ethernet_get_netinfo(ei_w5500_ethernet_netinfo_t *netinfo);

#ifdef __cplusplus
}
#endif

#endif // EI_W5500_ETHERNET_H_
