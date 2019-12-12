/*
 * phy.h: High-level PHY handling
 *
 * (c) 2019 Tarlogic Security S.L. - All rights reserved
 *
 * Company confidential. Any unauthorized use, disclosure, reproduction or
 * distribution of this file is strictly prohibited.
 */


#ifndef PHY_PHY_H
#define PHY_PHY_H

#include <defs.h>

typedef void (*phy_rx_handler_t) (void *, const uint8_t *, size_t size);

void phy_init(void);
void phy_set_rx_handler(phy_rx_handler_t, void *userdata);

#endif /* PHY_H_ */