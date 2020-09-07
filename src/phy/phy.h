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
#include <atpl360_comm.h>
typedef void (*phy_rx_handler_t) (void *, const uint8_t *, size_t size);

void phy_init(void);
BOOL phy_tx_take_exception(void);
void phy_reset_params(void);
BOOL phy_send_data(tx_msg_t *template, const void *data, size_t size);
void phy_set_rx_handler(phy_rx_handler_t, void *userdata);

#endif /* PHY_H_ */