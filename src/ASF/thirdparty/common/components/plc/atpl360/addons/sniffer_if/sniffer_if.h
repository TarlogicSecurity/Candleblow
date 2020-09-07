/**
 * \file
 *
 * \brief ATPL360_Host sniffer Interface for Physical layer
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef SNIFFER_IF_H_INCLUDED
#define SNIFFER_IF_H_INCLUDED

/* System includes */
#include <string.h>

#include "addon_api.h"
#include "atpl360_comm.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/**
 * \ingroup phy_plc_group
 * \defgroup sniffer_plc_group PLC PHY sniffer Interface
 *
 * This module provides configuration and utils to snifferize the PLC PHY layer.
 *
 */

/** \brief Sniffer interface commands identifiers */
/* @{ */
#define SNIFFER_IF_PHY_COMMAND_SET_CHANNEL               2     /* SET PLC channel (1 = CENELEC- A) */
#define SNIFFER_IF_PHY_COMMAND_ENABLE_PRIME_PLUS_ROBUST  3     /* Enable robust modes of PRIME */
#define SNIFFER_IF_PHY_COMMAND_MESSAGE                   4     /* Inject message in PLC */
#define SNIFFER_IF_PHY_ICOMMAND_TX_MSG                   10    /* Report TX message to sniffer tool */
#define SNIFFER_IF_PHY_MESSAGE_TYPE_A                    0x20  /* TYPE A pdu received */
#define SNIFFER_IF_PHY_MESSAGE_TYPE_B                    0x21  /* TYPE B pdu received */
#define SNIFFER_IF_PHY_MESSAGE_TYPE_BC                   0x22  /* TYPE BC pdu received */
  
/** \brief G3-PHY Sniffer interface commands identifiers */
/* @{ */  
#define SNIFFER_IF_PHY_G3_SET_TONE_MASK                   1     /* SET PLC channel (1 = CENELEC- A) */
/* @} */
  
#define TONE_MASK_SIZE                                    9     /* Byte size of the bitfield containing the tone mask */

void sniffer_if_init(atpl360_addon_descriptor_t *sx_desc);

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */

#endif   /* SNIFFER_IF_H_INCLUDED */
