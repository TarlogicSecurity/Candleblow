/**
 * \file
 *
 * \brief PHY_TX_TEST_CONSOLE : ATMEL PLC PHY TX Test Console Application
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

/**
 *  \mainpage ATMEL PLC PHY TX Test Console Application
 *
 *  \section Purpose
 *
 *  The PHY TX Test Console Application demonstrates how to configure some
 * parameters from the PHY layer on PLC boards.
 *
 *  \section Requirements
 *
 *  This package should be used with any PLC board on which there is PLC
 * hardware dedicated.
 *
 *  \section Description
 *
 *  This application can configure the PHY with a serial interface and test
 * PLC transmission/reception processes.
 *
 *  \section Usage
 *
 *  The tool is ready to configure, transmit and receive.
 */

#include <string.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Atmel boards includes. */
#include "board.h"

/* Atmel library includes. */
#include "asf.h"
#include "hal_private.h"
#include "math.h"

/* PHY includes */
#include "atpl360.h"
#include "conf_atpl360.h"
#if (BOARD == PL360BN)
#include "conf_eth.h"
#endif
#include "phy_embedded_app.h"

/* Example Includes */
#include "conf_app_example.h"

/* Tarlogic includes */
#include <console.h>
#include <app.h>

/****************************************************************************/
/* Function declarations */
static void prvSetupHardware(void);
#if (BOARD == PL360BN)
void prvSetupSdram(void);
void prvEnableEth(void);
void prvDisableEth(void);
#endif

/* Define HAL API interface */
extern const hal_api_t hal_api;

/* Task Monitor parameters */
#define mainMONITOR_TIMER_RATE    (500 / portTICK_RATE_MS)
#define mainMONITOR_BLOCK_TIME    (1000 / portTICK_RATE_MS)
static void prvProcessMonitorTasks(xTimerHandle pxTimer);

/* WDT configuration */
#define WATCHDOG_1s                     1000000
#define WATCHDOG_5s                     5000000

/* FreeRTOS utils */
void vApplicationIdleHook( void );
void vApplicationMallocFailedHook( void );
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName );
void vApplicationTickHook( void );

#define STRING_EOL    "\r"
#define STRING_HEADER "-- ATMEL PLC Getting Started Application --\r\n"	\
	"-- "BOARD_NAME " --\r\n" \
	"-- Compiled: "__DATE__ " "__TIME__ " --\r\n"

#if (BOARD == PL360BN)
#define MENU_HEADER "\n\r-- Menu Configuration --------------\n\r" \
	"0: Select buffer to transmit\n\r" \
	"1: Select attenuation level\n\r" \
	"2: Select scheme to transmit\n\r" \
	"3: Disable Rx in transmission\n\r" \
	"4: Select mode to transmit\n\r" \
	"5: Select time period between messages to transmit(ms.)\n\r" \
	"6: Enter data to transmit\n\r"	\
	"7: Enter channel to transmit\n\r" \
	"8: Config Auto-Detect impedance\n\r" \
	"v: View TX configuration values\n\r" \
        "m: Enable/Disable Memory SDRAM TEST\n\r" \
        "n: Enable/Disable Network ETH Test\n\r" \
	"e: Execute transmission application\n\r" \
	"otherwise: Display this main menu\n\n\r"
#else
#define MENU_HEADER "\n\r-- Menu Configuration --------------\n\r" \
	"0: Select buffer to transmit\n\r" \
	"1: Select attenuation level\n\r" \
	"2: Select scheme to transmit\n\r" \
	"3: Disable Rx in transmission\n\r" \
	"4: Select mode to transmit\n\r" \
	"5: Select time period between messages to transmit(ms.)\n\r" \
	"6: Enter data to transmit\n\r"	\
	"7: Enter channel to transmit\n\r" \
	"8: Config Auto-Detect impedance\n\r" \
	"v: View TX configuration values\n\r" \
	"e: Execute transmission application\n\r" \
	"otherwise: Display this main menu\n\n\r"

/*	"7: Select channel to transmit\n\r" \
 *      "8: Select coupling board in use\n\r" \
 *      "9: Config Auto-Detect impedance \n\r" \ */
#endif

#define MENU_SCHEME "\n\r-- Modulation Scheme --------------\r\n" \
	"0: PROTOCOL_DBPSK\n\r"	\
	"1: PROTOCOL_DQPSK\n\r"	\
	"2: PROTOCOL_D8PSK\n\r"	\
	"3: PROTOCOL_DBPSK_C\n\r" \
	"4: PROTOCOL_DQPSK_C\n\r" \
	"5: PROTOCOL_D8PSK_C\n\r" \
	"6: PROTOCOL_R_DBPSK\n\r" \
	"7: PROTOCOL_R_DQPSK\n\r"

#define MENU_COUPLING "\n\r-- Coupling Board --------------\r\n" \
	"0: ATPLCOUP000_v1\n\r"	\
	"1: ATPLCOUP000_v2\n\r"	\
	"2: ATPLCOUP001_v1\n\r"	\
	"3: ATPLCOUP002_v1\n\r"	\
	"4: ATPLCOUP002_v2\n\r"	\
	"5: ATPLCOUP003_v1\n\r"	\
	"6: ATPLCOUP004_v1\n\r"	\
	"7: ATPLCOUP005_v1\n\r"	\
	"8: ATPLCOUP006_v1\n\r"

#define MENU_MODE "\n\r-- PRIME Mode --------------\r\n" \
	"0: MODE_TYPE_A\n\r" \
	"2: MODE_TYPE_B\n\r" \
	"3: MODE_TYPE_BC\n\r"

#define MENU_CHANNEL "\n\r-- PRIME Channel --------------\r\n" \
	"1: CENELEC A\n\r" \
	"3-8: FCC channel\n\r"

#if (SAM4C) || (SAM4CM)
#define MENU_DATA_MODE "\n\r-- Select Data Mode --------------\r\n" \
	"0: Random Data\n\r" \
	"1: Fixed Data\n\r" \
	"2: Manual Data\n\r"
#else
#define MENU_DATA_MODE "\n\r-- Select Data Mode --------------\r\n" \
	"1: Fixed Data\n\r" \
	"2: Manual Data\n\r"
#endif

#define MENU_IMPEDANCE "\n\r-- Autodetect Impedance Mode --------------\r\n" \
	"0: Autodetect mode\n\r" \
	"1: Fixed Mode. High Impedance\n\r" \
	"2: Fixed Mode. Low Impedance\n\r" \
	"3: Fixed Mode. Vlow Impedance\n\r"

#define MENU_CONSOLE "\n\rPHY-Console>"

/* Phy data configuration */
static txPhyEmbeddedConfig_t xAppPhyCfgTx;

/* Tx data buffer */
uint8_t ucv_tx_data_buffer[512];

/**
 * Read from Serial
 * \retval 0 on success.
 * \retval 1 if no data is available or errors.
 */
uint32_t serial_read_char(uint8_t *c);

uint32_t serial_read_char(uint8_t *c)
{
#if SAMG55
	uint32_t ul_char;
	uint32_t ul_res;

	ul_res = usart_read((Usart *)CONF_UART, (uint32_t *)&ul_char);
	*c = (uint8_t)ul_char;

	return ul_res;

#else
	return uart_read((Uart *)CONF_UART, c);
#endif
}


/**
 * Set configuration parameters in GPBR
 */
static void save_config(uint8_t cmd_start_mode)
{
	uint32_t ul_gpbr_value;

	ul_gpbr_value = cmd_start_mode;
	ul_gpbr_value += xAppPhyCfgTx.uc_coupling << 4;
	ul_gpbr_value += xAppPhyCfgTx.uc_channel << 12;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.uc_buffer_id << 16;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.uc_att_level << 20;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.uc_scheme << 25;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.uc_disable_rx << 28;
	ul_gpbr_value += xAppPhyCfgTx.xPhyMsg.uc_mod_type << 29;
	gpbr_write(GPBR0, ul_gpbr_value);
	gpbr_write(GPBR1, xAppPhyCfgTx.ul_tx_period);
	ul_gpbr_value = xAppPhyCfgTx.xPhyMsg.us_data_len;
	ul_gpbr_value += xAppPhyCfgTx.uc_autodetect << 16;
	ul_gpbr_value += xAppPhyCfgTx.uc_impedance << 20;
#if (BOARD == PL360BN)
        ul_gpbr_value += (xAppPhyCfgTx.uc_enable_ram << 30);
        ul_gpbr_value += (xAppPhyCfgTx.uc_enable_eth << 31);
#endif
	gpbr_write(GPBR2, ul_gpbr_value);
}

/**
 * Get configuration parameters from GPBR
 */
static uint8_t load_config(void)
{
	uint32_t uc_gpbr_value;
	uint8_t uc_start_mode;

	uc_gpbr_value = gpbr_read(GPBR0);
	uc_start_mode = uc_gpbr_value & 0x0F;
	if ((uc_start_mode == PHY_APP_CMD_MENU_START_MODE) || (uc_start_mode == PHY_APP_CMD_TX_START_MODE)) {
		xAppPhyCfgTx.uc_coupling = (uc_gpbr_value >> 4) & 0xFF;
		xAppPhyCfgTx.uc_channel = (uc_gpbr_value >> 12) & 0x0F;
		xAppPhyCfgTx.xPhyMsg.uc_buffer_id = (enum buffer_id)((uc_gpbr_value >> 16) & 0x0F);
		xAppPhyCfgTx.xPhyMsg.uc_att_level = (uc_gpbr_value >> 20) & 0x1F;
		xAppPhyCfgTx.xPhyMsg.uc_scheme = (enum mod_schemes)((uc_gpbr_value >> 25) & 0x07);
		xAppPhyCfgTx.xPhyMsg.uc_disable_rx = (uc_gpbr_value >> 28) & 0x01;
		xAppPhyCfgTx.xPhyMsg.uc_mod_type = (enum mode_types)((uc_gpbr_value >> 29) & 0x07);

		xAppPhyCfgTx.ul_tx_period = gpbr_read(GPBR1);
		uc_gpbr_value = gpbr_read(GPBR2);
		xAppPhyCfgTx.xPhyMsg.us_data_len = uc_gpbr_value & 0xFFFF;
		xAppPhyCfgTx.uc_autodetect = (uc_gpbr_value >> 16) & 0x0F;
		xAppPhyCfgTx.uc_impedance = (uc_gpbr_value >> 20) & 0x0F;
#if (BOARD == PL360BN)
                xAppPhyCfgTx.uc_enable_ram = (uc_gpbr_value >> 30) & 0x01;
                xAppPhyCfgTx.uc_enable_eth = (uc_gpbr_value >> 31) & 0x01;
#endif

		/* upload the content of data message from flash memroy */
		if (xAppPhyCfgTx.xPhyMsg.us_data_len > sizeof(ucv_tx_data_buffer)) {
			xAppPhyCfgTx.xPhyMsg.us_data_len = sizeof(ucv_tx_data_buffer);
		}

		memcpy(ucv_tx_data_buffer, (uint8_t *)ADDR_APP_PHY_MESSAGE_DATA, xAppPhyCfgTx.xPhyMsg.us_data_len );
	} else {
		uc_start_mode = PHY_APP_CMD_DEFAULT_MODE;
	}

	/* Always use relative mode */
	xAppPhyCfgTx.xPhyMsg.uc_tx_mode = TX_MODE_RELATIVE;

	return uc_start_mode;
}

/**
 * Display current information
 */
static void display_config(void)
{
	printf("\n\r-- Configuration Info --------------\r\n");
	printf("-I- Buffer: %u\n\r", (uint32_t)xAppPhyCfgTx.xPhyMsg.uc_buffer_id);
	printf("-I- Attenuation Level: %u\n\r", (uint32_t)xAppPhyCfgTx.xPhyMsg.uc_att_level);
	switch (xAppPhyCfgTx.xPhyMsg.uc_scheme) {
	case MOD_SCHEME_DBPSK:
		printf("-I- Modulation Scheme: PROTOCOL_DBPSK\n\r");
		break;

	case MOD_SCHEME_DQPSK:
		printf("-I- Modulation Scheme: PROTOCOL_DQPSK\n\r");
		break;

	case MOD_SCHEME_D8PSK:
		printf("-I- Modulation Scheme: PROTOCOL_D8PSK\n\r");
		break;

	case MOD_SCHEME_DBPSK_C:
		printf("-I- Modulation Scheme: PROTOCOL_DBPSK_C\n\r");
		break;

	case MOD_SCHEME_DQPSK_C:
		printf("-I- Modulation Scheme: PROTOCOL_DQPSK_C\n\r");
		break;

	case MOD_SCHEME_D8PSK_C:
		printf("-I- Modulation Scheme: PROTOCOL_D8PSK_C\n\r");
		break;

	case MOD_SCHEME_R_DBPSK:
		printf("-I- Modulation Scheme: PROTOCOL_R_DBPSK\n\r");
		break;

	case MOD_SCHEME_R_DQPSK:
		printf("-I- Modulation Scheme: PROTOCOL_R_DQPSK\n\r");
		break;

	default:
		printf("-I- Modulation Scheme: ERROR CFG\n\r");
	}
	printf("-I- Disable Rx: %u\n\r", (uint32_t)xAppPhyCfgTx.xPhyMsg.uc_disable_rx);
	switch (xAppPhyCfgTx.xPhyMsg.uc_mod_type) {
	case MODE_TYPE_A:
		printf("-I- PRIME mode: MODE_TYPE_A\n\r");
		break;

	case MODE_TYPE_B:
		printf("-I- PRIME mode: MODE_TYPE_B\n\r");
		break;

	case MODE_TYPE_BC:
		printf("-I- PRIME mode: MODE_TYPE_BC\n\r");
		break;

	default:
		printf("-I- PRIME mode: ERROR CFG\n\r");
	}
	printf("-I- Time Period: %u\n\r", xAppPhyCfgTx.ul_tx_period);
	printf("-I- Data Len: %u\n\r", (uint32_t)xAppPhyCfgTx.xPhyMsg.us_data_len);

	switch (xAppPhyCfgTx.uc_channel) {
	case 1:
		printf("-I- PRIME channel: 1\n\r");
		break;

	case 3:
		printf("-I- PRIME channel: 3\n\r");
		break;

	case 4:
		printf("-I- PRIME channel: 4\n\r");
		break;

	case 5:
		printf("-I- PRIME channel: 5\n\r");
		break;

	case 6:
		printf("-I- PRIME channel: 6\n\r");
		break;

	case 7:
		printf("-I- PRIME channel: 7\n\r");
		break;

	case 8:
		printf("-I- PRIME channel: 8\n\r");
		break;

	default:
		printf("-I- PRIME channel: ERROR CFG\n\r");
	}

	if (xAppPhyCfgTx.uc_autodetect == 0) {
		printf("-I- Auto Impedance disable\n\r");
		switch (xAppPhyCfgTx.uc_impedance) {
		case HI_STATE:
			printf("-I- Impedance HIGH\n\r");
			break;

		case LOW_STATE:
			printf("-I- Impedance LOW\n\r");
			break;

		case VLO_STATE:
			printf("-I- Impedance Vlow\n\r");
			break;

		default:
			printf("-I- PRIME mode: ERROR CFG\n\r");
		}
	} else {
		printf("-I- Auto Impedance enable\n\r");
	}

	printf(MENU_CONSOLE);
	fflush(stdout);
}

/**
 * Get ID of transmission buffer.
 */
static void get_transmission_buffer_id(void)
{
	uint8_t uc_char;

	printf("Enter the buffer to use in tx [0,1] : ");
	fflush(stdout);
	while (1) {
		while (serial_read_char((uint8_t *)&uc_char)) {
			/* Restart watchdog */
			wdt_restart(WDT);
		}
		switch (uc_char) {
		case '0':
		case '1':
			printf("%c\r\n", uc_char);
			printf("->Buffer ID %c\r\n", uc_char);
			xAppPhyCfgTx.xPhyMsg.uc_buffer_id = (enum buffer_id)(uc_char - 0x30);
			printf(MENU_CONSOLE);
			fflush(stdout);
			return;

		default:
			continue;
		}
	}
}

/**
 * Get ID of attenuation level.
 */
static void get_transmission_att_level(void)
{
	uint8_t uc_char;
	uint16_t us_att;

	printf("Enter attenuation level using 2 digits [00..21] : ");
	fflush(stdout);
	while (1) {
		while (serial_read_char((uint8_t *)&uc_char)) {
			/* Restart watchdog */
			wdt_restart(WDT);
		}
		printf("%c", uc_char);
		us_att = (uc_char - 0x30) * 10;
		while (serial_read_char((uint8_t *)&uc_char)) {
			/* Restart watchdog */
			wdt_restart(WDT);
		}
		printf("%c\r\n", uc_char);
		us_att += (uc_char - 0x30);
		if (us_att < 22) {
			printf("->Attenuation level %u ok\r\n", (uint32_t)us_att);
			xAppPhyCfgTx.xPhyMsg.uc_att_level = us_att;
			printf(MENU_CONSOLE);
			fflush(stdout);
			return;
		} else {
			printf("ERROR: Attenuation level not permitted [0..21]. Try again.\n\r");
		}
	}
}

/**
 * Get scheme of modulation.
 */
static void get_transmission_scheme(void)
{
	uint8_t uc_char;
	uint8_t uc_scheme;

	puts(MENU_SCHEME);
	fflush(stdout);
	while (1) {
		while (serial_read_char((uint8_t *)&uc_char)) {
			/* Restart watchdog */
			wdt_restart(WDT);
		}
		switch (uc_char) {
		case '0':
		case '1':
		case '2':
			uc_scheme = uc_char - 0x30;
			break;

		case '3':
		case '4':
		case '5':
			uc_scheme = uc_char - 0x30 + 1;
			break;

		case '6':
		case '7':
			uc_scheme = uc_char - 0x30 + 6;
			break;

		default:
			continue;
		}
		printf("->Scheme %c ok\r\n", uc_char);
		xAppPhyCfgTx.xPhyMsg.uc_scheme = (enum mod_schemes)uc_scheme;
		printf(MENU_CONSOLE);
		fflush(stdout);
		break;
	}
}

/**
 * Get disable RX configuration.
 */
static void get_transmission_disable_rx(void)
{
	uint8_t uc_char;
	uint8_t uc_disable;

	printf("Force disable Rx in tx [Y/N] : ");
	fflush(stdout);
	while (1) {
		while (serial_read_char((uint8_t *)&uc_char)) {
			/* Restart watchdog */
			wdt_restart(WDT);
		}
		switch (uc_char) {
		case 'y':
		case 'Y':
			uc_disable = true;
			printf("Disable RX in tx\r\n");
			break;

		case 'n':
		case 'N':
			uc_disable = false;
			printf("Enable RX in tx\r\n");
			break;

		default:
			continue;
		}

		xAppPhyCfgTx.xPhyMsg.uc_disable_rx = uc_disable;
		printf(MENU_CONSOLE);
		fflush(stdout);
		break;
	}
}

/**
 * Get Transmission Mode
 */
static void get_transmission_mode(void)
{
	uint8_t uc_char;
	uint8_t uc_mode;

	puts(MENU_MODE);
	fflush(stdout);
	while (1) {
		while (serial_read_char((uint8_t *)&uc_char)) {
			/* Restart watchdog */
			wdt_restart(WDT);
		}
		switch (uc_char) {
		case '0':
			uc_mode = MODE_TYPE_A;
			printf("MODE_TYPE_A\r\n");
			break;

		case '2':
			uc_mode = MODE_TYPE_B;
			printf("MODE_TYPE_B\r\n");
			break;

		case '3':
			uc_mode = MODE_TYPE_BC;
			printf("MODE_TYPE_BC\r\n");
			break;

		default:
			continue;
		}

		xAppPhyCfgTx.xPhyMsg.uc_mod_type = (enum mode_types)uc_mode;
		printf(MENU_CONSOLE);
		fflush(stdout);
		break;
	}
}

/**
 * Get Period of transmission.
 */
static void get_transmission_period(void)
{
	uint8_t uc_char;
	uint8_t ucv_period[10];
	uint8_t i, c, uc_unit;
	uint16_t us_mul;

	printf("Enter transmission period in ms. (max. 10 digits): ");
	fflush(stdout);
	while (1) {
		for (i = 0; i < 10; i++) {
			while (serial_read_char((uint8_t *)&uc_char)) {
				/* Restart watchdog */
				wdt_restart(WDT);
			}

			if (uc_char == 0x0D) {
				xAppPhyCfgTx.ul_tx_period = 0;
				for (c = i; c > 0; c--) {
					us_mul = (uint16_t)pow(10, (i - c));
					uc_unit = ucv_period[c - 1];
					xAppPhyCfgTx.ul_tx_period += uc_unit * us_mul;
				}
				printf("\r\n->Transmission period %u ms\r\n", (uint32_t)xAppPhyCfgTx.ul_tx_period);
				printf(MENU_CONSOLE);
				return;
			} else if ((uc_char >= '0') && (uc_char <= '9')) {
				printf("%c", uc_char);
				ucv_period[i] = (uc_char - 0x30);
			} else {
				printf("Error. Try again\r\n");
				break;
			}

			fflush(stdout);
		}
	}
}

/**
 * Get Length of data to transmit.
 */
static void get_data_len(void)
{
	uint8_t uc_char;
	uint8_t ucv_len[4];
	uint8_t i, c, uc_unit;
	uint16_t us_mul;

	printf("Enter length of data to transmit in bytes. (max. 500 bytes): ");
	fflush(stdout);

	xAppPhyCfgTx.xPhyMsg.us_data_len = 0;
	while (1) {
		for (i = 0; i < 4; i++) {
			while (serial_read_char((uint8_t *)&uc_char)) {
				/* Restart watchdog */
				wdt_restart(WDT);
			}

			if (uc_char == 0x0D) {
				xAppPhyCfgTx.xPhyMsg.us_data_len = 0;
				for (c = i; c > 0; c--) {
					us_mul = (uint16_t)pow(10, (i - c));
					uc_unit = ucv_len[c - 1];
					xAppPhyCfgTx.xPhyMsg.us_data_len += uc_unit * us_mul;
				}
				printf("\r\n->Message Data length %u bytes\r\n", (uint32_t)xAppPhyCfgTx.xPhyMsg.us_data_len);
				fflush(stdout);
				return;
			} else if ((uc_char >= '0') && (uc_char <= '9')) {
				printf("%c", uc_char);
				ucv_len[i] = (uc_char - 0x30);
			} else {
				printf("Error. Try again\r\n");
				break;
			}

			fflush(stdout);
		}
	}
}

/**
 * Get Transmission Channel
 */
static void get_transmission_channel(void)
{
	uint8_t uc_char;
	uint8_t uc_chn;

	puts(MENU_CHANNEL);
	fflush(stdout);
	while (1) {
		while (serial_read_char((uint8_t *)&uc_char)) {
			/* Restart watchdog */
			wdt_restart(WDT);
		}
		switch (uc_char) {
		case '1':
			uc_chn = 1;
			printf("CENELEC A\r\n");
			break;

		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
			uc_chn = uc_char - 0x30;
			printf("FCC Channel %c\r\n", uc_char);
			break;

		default:
			printf("Invalid Channel\r\n");
			continue;
		}

		xAppPhyCfgTx.uc_channel = uc_chn;
		printf(MENU_CONSOLE);
		fflush(stdout);
		break;
	}
}

/**
 * Fix impedance mode
 */
static void fix_impedance_mode(uint8_t uc_impedance)
{
	xAppPhyCfgTx.uc_autodetect = 0;
	xAppPhyCfgTx.uc_impedance = uc_impedance;
}

/**
 * Get Autodetect Impedance mode.
 */
static void get_impedance_mode(void)
{
	uint8_t uc_char;

	puts(MENU_IMPEDANCE);

	while (1) {
		while (serial_read_char((uint8_t *)&uc_char)) {
			/* Restart watchdog */
			wdt_restart(WDT);
		}
		switch (uc_char) {
		case '0':
			xAppPhyCfgTx.uc_autodetect = 1;
			printf("Autodetec mode selected\r\n");
			break;

		case '1':
			fix_impedance_mode(HI_STATE);
			printf("Fix high impedance mode selected\r\n");
			break;

		case '2':
			fix_impedance_mode(LOW_STATE);
			printf("Fix low impedance mode selected\r\n");
			break;

		case '3':
			fix_impedance_mode(VLO_STATE);
			printf("Fix very low impedance mode selected\r\n");
			break;

		default:
			continue;
		}

		printf(MENU_CONSOLE);
		fflush(stdout);
		break;
	}
}

#if (SAM4C) || (SAM4CM)

/**
 * Fill data message in random mode.
 */
static void fill_msg_random(void)
{
	uint8_t *p_data_buf;
	uint16_t us_len;
	uint32_t ul_random_num;

	/* Asgin pointer to tx data buffer */
	p_data_buf = ucv_tx_data_buffer;

	/* init vars */
	us_len = xAppPhyCfgTx.xPhyMsg.us_data_len;

	/* Configure PMC */
	pmc_enable_periph_clk(ID_TRNG);

	/* Enable TRNG */
	trng_enable(TRNG);

	/* fill message */
	while (us_len) {
		while ((trng_get_interrupt_status(TRNG) & TRNG_ISR_DATRDY) != TRNG_ISR_DATRDY) {
		}
		ul_random_num = trng_read_output_data(TRNG);
		*p_data_buf++ = (uint8_t)ul_random_num;
		if (!us_len--) {
			break;
		}

		*p_data_buf++ = (uint8_t)(ul_random_num >> 8);
		if (!us_len--) {
			break;
		}

		*p_data_buf++ = (uint8_t)(ul_random_num >> 16);
		if (!us_len--) {
			break;
		}

		*p_data_buf++ = (uint8_t)(ul_random_num >> 24);
		if (!us_len--) {
			break;
		}
	}

	/* set header type to generic message */
	xAppPhyCfgTx.xPhyMsg.puc_data_buf[0] = 0;

	/* store the content of message in flash memory */
	flash_unlock((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, (uint32_t)ADDR_APP_PHY_MESSAGE_DATA + xAppPhyCfgTx.xPhyMsg.us_data_len, 0, 0);
	flash_erase_page((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, 2);
	flash_write((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, ucv_tx_data_buffer, xAppPhyCfgTx.xPhyMsg.us_data_len, 0);

	printf("->Random message ready\r\n");
	printf(MENU_CONSOLE);
	fflush(stdout);
}

#else

/**
 * Fill data message in random mode.
 */
static void fill_msg_random(void)
{
}

#endif

/**
 * Fill data message in fixed mode.
 */
static void fill_msg_fixed(void)
{
	uint8_t uc_i;
	uint8_t *p_data_buf;
	uint16_t us_len;

	/* Asgin pointer to tx data buffer */
	p_data_buf = ucv_tx_data_buffer;

	/* init vars */
	us_len = xAppPhyCfgTx.xPhyMsg.us_data_len;
	uc_i = 0;

	/* fill message */
	while (us_len--) {
		*p_data_buf++ = 0x30 + uc_i++;
		if (uc_i == 10) {
			uc_i = 0;
		}
	}

	/* set header type to generic message */
	xAppPhyCfgTx.xPhyMsg.puc_data_buf[0] = 0;

	/* store the content of message in flash memory */
	flash_unlock((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, (uint32_t)ADDR_APP_PHY_MESSAGE_DATA + xAppPhyCfgTx.xPhyMsg.us_data_len, 0, 0);
	flash_erase_page((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, 2);
	flash_write((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, ucv_tx_data_buffer, xAppPhyCfgTx.xPhyMsg.us_data_len, 0);

	printf("->Fixed message ready\r\n");
	printf(MENU_CONSOLE);
	fflush(stdout);
}

/**
 * Fill data message in manual mode.
 */
static void fill_msg_manual(void)
{
	uint16_t uc_i;
	uint8_t uc_char;
	uint8_t *p_data_buf;

	printf("Enter data message to transmit (max. 500 bytes): ");
	fflush(stdout);

	/* Asgin pointer to tx data buffer */
	p_data_buf = ucv_tx_data_buffer;

	xAppPhyCfgTx.xPhyMsg.us_data_len = 0;
	while (1) {
		for (uc_i = 0; uc_i < 500; uc_i++) {
			while (serial_read_char((uint8_t *)&uc_char)) {
				/* Restart watchdog */
				wdt_restart(WDT);
			}

			if (uc_char == 0x0D) {
				/* set header type to generic message */
				xAppPhyCfgTx.xPhyMsg.puc_data_buf[0] = 0;

				/* store the content of message in flash memory */
				flash_unlock((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, (uint32_t)ADDR_APP_PHY_MESSAGE_DATA + xAppPhyCfgTx.xPhyMsg.us_data_len, 0, 0);
				flash_erase_page((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, 2);
				flash_write((uint32_t)ADDR_APP_PHY_MESSAGE_DATA, ucv_tx_data_buffer, xAppPhyCfgTx.xPhyMsg.us_data_len, 0);

				printf("\r\n->Manual message ready.\r\n");
				printf(MENU_CONSOLE);
				fflush(stdout);
				return;
			} else {
				printf("%c", uc_char);
				*p_data_buf++ = uc_char;
				xAppPhyCfgTx.xPhyMsg.us_data_len++;
			}
		}
		printf("\r\n->End: Maximum Length is 500 bytes\r\n");
		fflush(stdout);
	}
}

/**
 * Get Transmission Data.
 */
static void get_transmission_data(void)
{
	uint8_t uc_char;

	puts(MENU_DATA_MODE);
	fflush(stdout);

	while (1) {
		while (serial_read_char((uint8_t *)&uc_char)) {
			/* Restart watchdog */
			wdt_restart(WDT);
		}
		switch (uc_char) {
		case '0':
			printf("%c\r\n", uc_char);
			get_data_len();
			fill_msg_random();
			break;

		case '1':
			printf("%c\r\n", uc_char);
			get_data_len();
			fill_msg_fixed();
			break;

		case '2':
			printf("%c\r\n", uc_char);
			fill_msg_manual();
			break;

		default:
			continue;
		}
		break;
	}
}

/**
 * Execute TX test
 */
static void execute_tx_test(void)
{
	printf("Press 'x' to finish transmission...\r\n");
	fflush(stdout);
	/* Init Phy Embedded App */
	vPhyEmbeddedAppTask(&xAppPhyCfgTx);
	/* Start the tasks and timer running. */
	vTaskStartScheduler();
}

/**
 * \brief Main code entry point.0
 */
int noMain( void )
{
	xTimerHandle xMonitorTimer;
	uint8_t uc_choice;
	uint8_t uc_start_mode;
	uint32_t timeout_value;
	uint32_t wdt_mode;
#if (BOARD == PL360BN)
        uint32_t ul_value;
        uint8_t uc_rc;
#endif

	/* Prepare the hardware */
	prvSetupHardware();

#if (BOARD == PL360BN)

        /* Enable Sleep manager */
        sleepmgr_init();

	/* Ensure that the Ethernet PHY device is in power down mode and slow-clock
           This is needed to reduce RF emissions to a minimum.
        */

        /* Enable GMAC */
        pmc_enable_periph_clk(ID_GMAC);

        // Init MAC PHY driver
        if (ethernet_phy_init(GMAC, BOARD_GMAC_PHY_ADDR, sysclk_get_cpu_hz())
                                    != GMAC_OK) {
            printf("PHY Initialize ERROR!\r");
        }


        gmac_enable_management(GMAC, true);
        // read PHY config
        uc_rc = gmac_phy_read(GMAC, BOARD_GMAC_PHY_ADDR, GMII_BMCR, &ul_value);


        // set PHY in power-down mode to stop the PLL/Clock output
        ul_value |= (uint32_t)GMII_POWER_DOWN; // power up phy
        gmac_phy_write(GMAC, BOARD_GMAC_PHY_ADDR, GMII_BMCR, ul_value);

        // set PHY in Slow oscilator mode
        uc_rc = gmac_phy_read(GMAC, BOARD_GMAC_PHY_ADDR, GMII_AFECR1, &ul_value);
        ul_value |= (1 <<5);
        gmac_phy_write(GMAC, BOARD_GMAC_PHY_ADDR, GMII_AFECR1, ul_value);

        gmac_enable_management(GMAC, false);
        // Disable GMAC clock
        pmc_disable_periph_clk(ID_GMAC);

        // Disable SDRAMC clock
        pmc_disable_periph_clk(ID_SDRAMC);

#endif
        /* get value to init wdog from time in us. */
	timeout_value = wdt_get_timeout_value(WATCHDOG_5s, BOARD_FREQ_SLCK_XTAL);
	/* Configure WDT to trigger a reset. */
#if (BOARD == PL360BN) || (BOARD == SAME70_XPLAINED)
	wdt_mode = WDT_MR_WDRSTEN | WDT_MR_WDDBGHLT | WDT_MR_WDIDLEHLT;
#else
	wdt_mode = WDT_MR_WDRSTEN | WDT_MR_WDRPROC | WDT_MR_WDDBGHLT | WDT_MR_WDIDLEHLT;
#endif
	/* Initialize WDT with the given parameters. */
	wdt_init(WDT, wdt_mode, timeout_value, timeout_value);

	/* Create timer to monitor tasks execution */
	xMonitorTimer = xTimerCreate((const signed char *const)"Monitor timer",
			mainMONITOR_TIMER_RATE,
			pdTRUE,
			NULL,
			prvProcessMonitorTasks
			);
	configASSERT(xMonitorTimer);
	xTimerStart(xMonitorTimer, mainMONITOR_BLOCK_TIME);

	puts(STRING_HEADER);
	fflush(stdout);

	/* Configuration management */
	xAppPhyCfgTx.xPhyMsg.puc_data_buf = ucv_tx_data_buffer;

	uc_start_mode = load_config();

	if (uc_start_mode == PHY_APP_CMD_DEFAULT_MODE) {
		xAppPhyCfgTx.ul_tx_period = 1000;
		xAppPhyCfgTx.xPhyMsg.uc_att_level = 0;
		xAppPhyCfgTx.xPhyMsg.uc_disable_rx = true;
		xAppPhyCfgTx.xPhyMsg.uc_mod_type = MODE_TYPE_A;
		xAppPhyCfgTx.xPhyMsg.uc_scheme = MOD_SCHEME_DBPSK_C;
		xAppPhyCfgTx.xPhyMsg.ul_tx_time = 0;
		xAppPhyCfgTx.xPhyMsg.uc_buffer_id = TX_BUFFER_0;
		xAppPhyCfgTx.xPhyMsg.us_data_len = 64;
		xAppPhyCfgTx.uc_channel = 1;
		/* Fill Data of message: Fixed by default */
		fill_msg_fixed();
		/* save configuration parameters */
		save_config(PHY_APP_CMD_MENU_START_MODE);
	} else if (uc_start_mode == PHY_APP_CMD_TX_START_MODE) {
#if (BOARD == PL360BN)
		/* execute test */
                if(xAppPhyCfgTx.uc_enable_ram) {
			/* Enable SDRAM */
			prvSetupSdram();
                }

                if(xAppPhyCfgTx.uc_enable_eth) {
			/* Enable SDRAM if needed*/
			prvEnableEth();
                }
#endif
		/* execute test */
		execute_tx_test();
	}

	/* Console Application menu */
	puts(MENU_HEADER);
	printf(MENU_CONSOLE);
	fflush(stdout);
        
        
	while (1) {
		while (serial_read_char(&uc_choice)) {
			delay_ms(10);
			/* Reset watchdog */
			wdt_restart(WDT);
		}
		printf("%c\r\n", uc_choice);
		fflush(stdout);

		switch (uc_choice) {
		case '0':
			get_transmission_buffer_id();
			break;

		case '1':
			get_transmission_att_level();
			break;

		case '2':
			get_transmission_scheme();
			break;

		case '3':
			get_transmission_disable_rx();
			break;

		case '4':
			get_transmission_mode();
			break;

		case '5':
			get_transmission_period();
			break;

		case '6':
			get_transmission_data();
			break;

		case '7':
			get_transmission_channel();
			break;

		case '8':
			get_impedance_mode();
			break;

		case 'v':
		case 'V':
			display_config();
			break;

		case 'e':
		case 'E':
			/* save configuration parameters */
			save_config(PHY_APP_CMD_TX_START_MODE);
			/* execute test */
			execute_tx_test();
			break;

#if (BOARD == PL360BN)
                case 'm':
                case 'M':
                        if (xAppPhyCfgTx.uc_enable_ram == 0) {
				xAppPhyCfgTx.uc_enable_ram =1;
				printf("SDRAM TEST Enabled\n");
                        } else {
				pmc_disable_periph_clk(ID_SDRAMC);
				xAppPhyCfgTx.uc_enable_ram =0;
				printf("SDRAM TEST Disabled\n");
                        }

                        break;
#endif
#if (BOARD == PL360BN)
                case 'n':
                case 'N':
                        if (xAppPhyCfgTx.uc_enable_eth == 0) {
				printf("ETH TX Enabled\n");
				xAppPhyCfgTx.uc_enable_eth =1;
				/* Enable Eth */

                        } else {
				xAppPhyCfgTx.uc_enable_eth =0;
				printf("ETH TX Disabled\n");

				/* Disable Eth to stop any clock signals */
				prvDisableEth();
                        }

                        break;
#endif
		default:
			puts(MENU_HEADER);
			printf(MENU_CONSOLE);
			fflush(stdout);
			break;
		}
	}
}

#if (BOARD == PL360BN)

void prvSetupSdram()
{
  	volatile uint32_t i;
	volatile uint16_t *pSdram = (uint16_t *)(BOARD_SDRAM_ADDR);
	uint32_t ul_clk = sysclk_get_peripheral_hz();

	/* Enable peripheral clocks*/
  	sysclk_enable_peripheral_clock(ID_SDRAMC);
	sysclk_enable_peripheral_clock(ID_PIOA);
	sysclk_enable_peripheral_clock(ID_PIOB);
	sysclk_enable_peripheral_clock(ID_PIOC);
	sysclk_enable_peripheral_clock(ID_PIOD);
	sysclk_enable_peripheral_clock(ID_PIOE);
	MATRIX->CCFG_SMCNFCS = CCFG_SMCNFCS_SDRAMEN;

	/* Prevent setting the SDRAMC entering to sleep mode */
	sleepmgr_lock_mode(SLEEPMGR_ACTIVE);

	/* SDRAM device configuration */
	/* Step 1. */
	/* Set the features of SDRAM device into the Configuration Register */
	SDRAMC->SDRAMC_CR =
		  SDRAMC_CR_NC_COL8      // 8 column bits
		| SDRAMC_CR_NR_ROW12     // 12 row bits (4K)
		| SDRAMC_CR_CAS_LATENCY3 // CAS Latency 3
		| SDRAMC_CR_NB_BANK4     // 4 banks
		| SDRAMC_CR_DBW          // 16 bit
		| SDRAMC_CR_TWR(4)       // 4 SDCK cycles minimum.
		| SDRAMC_CR_TRC_TRFC(10) // Command period (Ref to Ref / ACT to ACT) 63ns minimum. If SDCK=143MHz minimum TRFC=10
		| SDRAMC_CR_TRP(3)       // Command period (PRE to ACT) 15 ns min. If SDCK=143MHz minimum TRP=3
		| SDRAMC_CR_TRCD(3)      // Active Command to read/Write Command delay 15ns. If SDCK=143MHz minimum TRCD=3
		| SDRAMC_CR_TRAS(7)      // Command period (ACT to PRE)  42ns min. If SDCK=143MHz minimum TRCD=7
		| SDRAMC_CR_TXSR(11);   // Exit self-refresh to active time  70ns Min. If SDCK=143MHz minimum TRCD=11

	/* Step 2. */

	/* For low-power SDRAM, Temperature-Compensated Self Refresh (TCSR),
	   Drive Strength (DS) and Partial Array Self Refresh (PASR) must be set
	   in the Low-power Register. */
	SDRAMC->SDRAMC_LPR = 0;

	/* Step 3. */
	/* Program the memory device type into the Memory Device Register */
	SDRAMC->SDRAMC_MDR = SDRAMC_MDR_MD_SDRAM;

	/* Step 4. */

	/* A minimum pause of 200 µs is provided to precede any signal toggle.
	   (6 core cycles per iteration) */
	for (i = 0; i < (( ul_clk / 1000000) * 200 / 6); i++) {
		;
	}

	/* Step 5. */

	/* A NOP command is issued to the SDR-SDRAM. Program NOP command into
	   Mode Register, and the application must set Mode to 1 in the Mode
	   Register. Perform a write access to any SDR-SDRAM address to
	   acknowledge this command. Now the clock which drives SDR-SDRAM
	   device is enabled. */
	SDRAMC->SDRAMC_MR = SDRAMC_MR_MODE_NOP;
	*pSdram = 0x0;
	for (i = 0; i < 100000; i++){
		;
	}

	/* Step 6. */

	/* An all banks precharge command is issued to the SDR-SDRAM. Program
	   all banks precharge command into Mode Register, and the application
	   must set Mode to 2 in the Mode Register. Perform a write access to
	   any SDRSDRAM address to acknowledge this command. */
	SDRAMC->SDRAMC_MR = SDRAMC_MR_MODE_ALLBANKS_PRECHARGE;
	*pSdram = 0x0;

	/* Add some delays after precharge */
	for (i = 0; i < ((ul_clk / 1000000) * 200 / 6); i++) {
		;
	}

	/* Step 7. */
	/* Eight auto-refresh (CBR) cycles are provided. Program the auto
	   refresh command (CBR) into Mode Register, and the application
	   must set Mode to 4 in the Mode Register. Once in the idle state,
	   eight AUTO REFRESH cycles must be performed. */
	for (i = 0 ; i< 8; i++) {
		SDRAMC->SDRAMC_MR = SDRAMC_MR_MODE_AUTO_REFRESH;
		*pSdram = 0;
	}
	for (i = 0; i < 100000; i++);

	/* Step 8. */
	/* A Mode Register Set (MRS) cycle is issued to program the parameters
	   of the SDRAM devices, in particular CAS latency and burst length. */
	SDRAMC->SDRAMC_MR = SDRAMC_MR_MODE_LOAD_MODEREG;
	*pSdram = 0;
	for (i = 0; i < 100000; i++);

	/* Step 9. */

	/* For low-power SDR-SDRAM initialization, an Extended Mode Register Set
	   (EMRS) cycle is issued to program the SDR-SDRAM parameters (TCSR,
	   PASR, DS). The write address must be chosen so that BA[1] is set to
	   1 and BA[0] is set to 0. */
	/* No Need*/

	/* Step 10. */
	/* The application must go into Normal Mode, setting Mode to 0 in the
	   Mode Register and perform a write access at any location in the\
	   SDRAM to acknowledge this command. */
	SDRAMC->SDRAMC_MR = SDRAMC_MR_MODE_NORMAL;
	*pSdram = 0x0;
	for (i = 0; i < 100000; i++);

	/* Step 11. */

	/* Write the refresh rate into the count field in the SDRAMC Refresh
	   Timer register. Set Refresh timer to 15.625 us. */

	/* For IS42S16400J, 4096 refresh cycle every 64ms, every 15.625 . SDRAM_TR= ((64 x 10(^-3))/4096) xSDCK(MHz) */
        SDRAMC->SDRAMC_TR = 2343;
	SDRAMC->SDRAMC_CFR1 |= SDRAMC_CFR1_UNAL;        //Enable of unaligned addressing mode
	*pSdram = 0;
	for (i = 0; i < 100000; i++);

	/* End of Initialization */
	/* Erase SDRAM. SDRAM is not available at early stages of the
	initialiation, we do it now. */
	uint32_t  *p_sdram = (uint32_t *)BOARD_SDRAM_ADDR;

	p_sdram = (uint32_t *)BOARD_SDRAM_ADDR;
	while ((uint32_t)p_sdram < (uint32_t)(BOARD_SDRAM_ADDR + BOARD_SDRAM_SIZE)) {
		*p_sdram =0;
		p_sdram++;
                if ( (uint32_t)p_sdram % 0x3FF == 0) {
                    /* Each kb, surrender time */
                    wdt_restart(WDT);
                }
	}
}

void  prvEnableEth()
{
    uint32_t ul_value=0;
    /* Enable GMAC */
    pmc_enable_periph_clk(ID_GMAC);

    /* Force PHY reset */
    ethernet_phy_reset(GMAC,BOARD_GMAC_PHY_ADDR);

    /* Enable PHY access */
    gmac_enable_management(GMAC, true);

    /* Disable slow oscilator */
    gmac_phy_read(GMAC, BOARD_GMAC_PHY_ADDR, GMII_AFECR1, &ul_value);
    ul_value = 0;
    gmac_phy_write(GMAC, BOARD_GMAC_PHY_ADDR, GMII_AFECR1, ul_value);
    ul_value = 0xFFFFF; while(ul_value--);
    /* Disable power down mode */
    gmac_phy_read(GMAC, BOARD_GMAC_PHY_ADDR, GMII_BMCR, &ul_value);
    ul_value &= ~(uint32_t)GMII_POWER_DOWN; /* power up phy  */
    gmac_phy_write(GMAC, BOARD_GMAC_PHY_ADDR, GMII_BMCR, ul_value);
    ul_value = 0xFFFFF; while(ul_value--);

    /* Initiate reset */
    gmac_phy_read(GMAC, BOARD_GMAC_PHY_ADDR, GMII_BMCR, &ul_value);
    ul_value |= (uint32_t)GMII_RESET;
    gmac_phy_write(GMAC, BOARD_GMAC_PHY_ADDR, GMII_BMCR, ul_value);
    ul_value = 0xFFFFF; while(ul_value--);

    /* Disable PHY access */
    gmac_enable_management(GMAC, false);
}

void  prvDisableEth()
{
    uint32_t ul_value=0;
    /* Enable GMAC */
    pmc_enable_periph_clk(ID_GMAC);

    /* Enable PHY access */
    gmac_enable_management(GMAC, true);

    /* set PHY in power-down mode */
    gmac_phy_read(GMAC, BOARD_GMAC_PHY_ADDR, GMII_BMCR, &ul_value);
    ul_value |= (uint32_t)GMII_POWER_DOWN; /* power up phy  */
    gmac_phy_write(GMAC, BOARD_GMAC_PHY_ADDR, GMII_BMCR, ul_value);

    /*Enable Slow-oscilator mode stop the PLL/Clock output */
    gmac_phy_read(GMAC, BOARD_GMAC_PHY_ADDR, GMII_AFECR1, &ul_value);
    ul_value |= (1<<5);
    gmac_phy_write(GMAC, BOARD_GMAC_PHY_ADDR, GMII_AFECR1, ul_value);

    /* Disable PHY access */
    gmac_enable_management(GMAC, false);

    /* Disable GMAC */
    pmc_disable_periph_clk(ID_GMAC);
}

#endif

/**
 * \brief Configure the hardware.
 */
static void prvSetupHardware(void)
{
	uint32_t ul_wait_counter;
	uint8_t uc_num_blinks;
#ifdef CONF_BOARD_LCD_EN
	status_code_t status;
#endif

	/* ASF function to setup clocking. */
	sysclk_init();

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS);

	/* Atmel library function to setup for the evaluation kit being used. */
	board_init();

#if (BOARD == PL360BN) || (BOARD == SAME70_XPLAINED)
	/* Initialize flash: 6 wait states for flash writing. */
	flash_init(FLASH_ACCESS_MODE_128, (6U));
#else
	/* Initialize flash wait states for writing. */
	flash_init(FLASH_ACCESS_MODE_128, CHIP_FLASH_WRITE_WAIT_STATE);
#endif

	console_init();

	/* LED signalling */
	for (uc_num_blinks = 0; uc_num_blinks < 30; uc_num_blinks++) {
		ul_wait_counter = 0xFFFFF;
		while (ul_wait_counter--) {
		}
		LED_Toggle(LED0);
#ifndef PL360G55CF_EK
		LED_Toggle(LED1);
#endif
	}

#ifdef CONF_BOARD_LCD_EN
#if BOARD == ATPL360ASB
	/* Initialize the vim878 LCD glass component. */
	status = vim878_init();
	if (status != STATUS_OK) {
		puts("-- LCD Initialization fails! --\r\n");
		while (1) {
		}
	}

	vim878_set_contrast(1);
	vim878_clear_all();
	vim878_show_text((const uint8_t *)"phycon");
#elif BOARD == ATPL360AMB
	status = (status_code_t)c0216CiZ_init();
	if (status != STATUS_OK) {
		puts("-- LCD Initialization fails! --\r\n");
		while (1) {
		}
	}

	c0216CiZ_set_cursor(C0216CiZ_LINE_UP, 0);
	c0216CiZ_show((const char *)"ATPL360AMB PRIME");
	c0216CiZ_set_cursor(C0216CiZ_LINE_DOWN, 0);
	c0216CiZ_show((const char *)"Phy TX console");
#elif BOARD == ATPL360MB
	status = (status_code_t)c0216CiZ_init();
	if (status != STATUS_OK) {
		puts("-- LCD Initialization fails! --\r\n");
		while (1) {
		}
	}

	c0216CiZ_set_cursor(C0216CiZ_LINE_UP, 0);
	c0216CiZ_show((const char *)"ATPL360MB PRIME");
	c0216CiZ_set_cursor(C0216CiZ_LINE_DOWN, 0);
	c0216CiZ_show((const char *)"Candelblow");
#else
#error ERROR in board definition
#endif
#endif
}

/**
 * \brief Display scheduler activity led.
 */
static void prvProcessMonitorTasks( xTimerHandle pxTimer )
{
	UNUSED(pxTimer);
#if (BOARD != SAM4CMP_DB && BOARD != SAM4CMS_DB)
	LED_Toggle(LED0);
#endif
}

/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	 * configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	 * function that will get called if a call to pvPortMalloc() fails.
	 * pvPortMalloc() is called internally by the kernel whenever a task, queue,
	 * timer or semaphore is created.  It is also called by various parts of the
	 * demo application.  If heap_1.c or heap_2.c are used, then the size of the
	 * heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	 * FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	 * to query the size of free heap space that remains (although it does not
	 * provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for (;;) {
		while (1) {
		}
	}
}

/*-----------------------------------------------------------*/
void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	 * to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	 * task.  It is essential that code added to this hook function never attempts
	 * to block in any way (for example, call xQueueReceive() with a block time
	 * specified, or call vTaskDelay()).  If the application makes use of the
	 * vTaskDelete() API function (as this demo application does) then it is also
	 * important that vApplicationIdleHook() is permitted to return to its calling
	 * function, because it is the responsibility of the idle task to clean up
	 * memory allocated by the kernel to any task that has since been deleted. */
}

/*-----------------------------------------------------------*/
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	(void)pcTaskName;
	(void)pxTask;

	/* Run time stack overflow checking is performed if
	 * configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	 * function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for (;;) {
		while (1) {
		}
	}
}

/*-----------------------------------------------------------*/
void vApplicationTickHook( void )
{
	/* This function will be called by each tick interrupt if
	* configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	* added here, but the tick hook is called from an interrupt context, so
	* code must not attempt to block, and only the interrupt safe FreeRTOS API
	* functions can be used (those that end in FromISR()). */
}
