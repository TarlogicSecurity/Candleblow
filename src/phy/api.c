/*
 * api.c: High-level PHY layer
 *
 * (c) 2019 Tarlogic Security S.L. - All rights reserved
 *
 * Company confidential. Any unauthorized use, disclosure, reproduction or
 * distribution of this file is strictly prohibited.
 */

#include "phy.h"

phy_rx_handler_t rx_handler = NULL;
void *rx_handler_userdata = NULL;

void 
phy_set_rx_handler(phy_rx_handler_t handler, void *userdata)
{
  rx_handler = handler;
  rx_handler_userdata = userdata;
}
