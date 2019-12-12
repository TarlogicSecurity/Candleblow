/**
 * \file
 *
 * \brief Connection of the ATPL360 to SPI interface driver.
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

/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef ATPL360_HAL_SPI_H_INCLUDED
#define ATPL360_HAL_SPI_H_INCLUDED

/* System includes */
#include <string.h>
#include "atpl360_exception.h"
#include "general_defs.h"

#ifdef __cplusplus
extern "C" {
#endif
/** Delay between boot satus checks */
#define ATPL360_DELAY_BOOT_CMD              1

/** External Interrupt Status line */
#define ATPL360_EXT_INT_ACTIVE              0
#define ATPL360_EXT_INT_INACTIVE            1

/** READ/Write command header size */
#define ATPL360_CMD_READ_HDR_KEY            0x0200
#define ATPL360_CMD_WRITE_HDR_KEY           0x0400
#define ATPL360_HIGH_ADDR                   0x2000

#define ATPL360_CMD_READ                    0
#define ATPL360_CMD_WRITE                   1
#define ATPL360_WR_RD_POS                   15
#define ATPL360_LEN_MASK                    0x7FFF

/** Bootloader command: Disable SPI control to bootloader */
#define ATPL360_BOOT_CMD_DIS_SPI_CTRL       0xA55A
/** Bootloader command: Disable SPI control to bootloader and disable Boot_CLK */
#define ATPL360_BOOT_CMD_DIS_SPI_CLK_CTRL   0xA66A
/** Bootloader Address of CPUWAIT */
#define ATPL360_BOOT_CMD_ENABLE_WRITE       0xDE05
/** Bootloader Address for writing program */
#define ATPL360_BOOT_PROGRAM_ADDR           0x00000000
/** Bootloader Passwords for enable writing  */
#define ATPL360_BOOT_WRITE_KEY              0x5345ACBA

/** Bootloader command: Write Word (32 bits) */
#define ATPL360_BOOT_CMD_WRITE_WORD         0x0000
/** Bootloader command: Write buffer */
#define ATPL360_BOOT_CMD_WRITE_BUF          0x0001
/** Bootloader command: Read buffer */
#define ATPL360_BOOT_CMD_READ_BUF           0x0002
/** Bootloader command: Read Word (32 bits) */
#define ATPL360_BOOT_CMD_READ_WORD          0x0003
/** Bootloader command: Set number of decryption packets */
#define ATPL360_BOOT_SET_DEC_NUM_PKTS       0x0004
/** Bootloader command: Set decryption initial vector */
#define ATPL360_BOOT_SET_DEC_INIT_VECT      0x0005
/** Bootloader command: Set decryption signature */
#define ATPL360_BOOT_SET_DEC_SIGN           0x0006
/** Bootloader command: Set 128 bits fuses value */
#define ATPL360_BOOT_SET_128_FUSES_VALUE    0x0007
/** Bootloader command: Write buffer register to tamper value at KEY_ENC_FUSES */
#define ATPL360_BOOT_SET_ENC_FUSES          0x0008
/** Bootloader command: Write buffer register to tamper value at KEY_TAG_FUSES */
#define ATPL360_BOOT_SET_TAG_FUSES          0x0009
/** Bootloader command: Write buffer register to tamper value at CONTROL_FUSES */
#define ATPL360_BOOT_SET_CTRL_FUSES         0x000B
/** Bootloader command: Blown desired fuses */
#define ATPL360_BOOT_BLOWN_FUSES            0x000C
/** Bootloader command: Read KEY_ENC_FUSES to tamper register */
#define ATPL360_BOOT_GET_ENC_FUSES          0x000D
/** Bootloader command: Read KEY_TAG_FUSES to tamper register */
#define ATPL360_BOOT_GET_TAG_FUSES          0x000E
/** Bootloader command: Read CONTROL_FUSES to tamper register */
#define ATPL360_BOOT_GET_CTRL_FUSES         0x0010
/** Bootloader command: Read tamper register */
#define ATPL360_BOOT_READ_TAMPER_REG        0x0011
/** Bootloader command: Read bootloader status */
#define ATPL360_BOOT_READ_BOOT_STATUS       0x0012
/** Bootloader command: Start Decryption */
#define ATPL360_BOOT_START_DECRYPT          0x0013
/** Bootloader command: Stop Window */
#define ATPL360_BOOT_STOP_WINDOW            0x0014

/** SPI clock setting (Hz). */
#define ATPL360_SPI_CLK_1MHz                1000000
#define ATPL360_SPI_CLK_2MHz                2000000
#define ATPL360_SPI_CLK_3MHz                3000000
#define ATPL360_SPI_CLK_3M5                 3500000
#define ATPL360_SPI_CLK_3M75                3750000
#define ATPL360_SPI_CLK_4MHz                4000000
#define ATPL360_SPI_CLK_4M4                 4444444
#define ATPL360_SPI_CLK_20MHz              20000000

/** SPI configuration requeriments */
/** Clock polarity */
#define ATPL360_SPI_CLK_POLARITY            0
/** Clock phase */
#define ATPL360_SPI_CLK_PHASE               1
/** Delay before SPCK */
#define ATPL360_SPI_DLYBS                   1
/** Delay between consecutive transfers */
#define ATPL360_SPI_DLYBCT                  0

/** SPI Header field when bootload is in the other side of spi*/
#define ATPL360_SPI_HEADER_BOOT                      0x5634
/** SPI Header MASK for bootloader heade*/
#define ATPL360_SPI_HEADER_BOOT_MASK                 0xFFFE
/** SPI Header field when atpl360 is in the other side of spi*/
#define ATPL360_SPI_HEADER_CORTEX                    0x1122

/** User rest flag in bootloader header*/
#define ATPL360_SPI_HEADER_RESET_FLAG_USER           0x00010000
/** Cortex(debugger) rest flag in bootloader header*/
#define ATPL360_SPI_HEADER_RESET_FLAG_CORTEX         0x00008000
/** Watch Dog flag in bootloader header*/
#define ATPL360_SPI_HEADER_RESET_FLAG_WATCHDOG       0x00004000
/** Power-ON reset is indicated when the three flags are 0, mask will be used to detect it*/
#define ATPL360_SPI_HEADER_RESET_POWER_ON_MASK       0x0001C000

/** MACROS */
#define ATPL360_GET_ID_HEADER(b0, b1)               ((((uint16_t)b1 << 8) + b0) & ATPL360_SPI_HEADER_BOOT_MASK)
#define ATPL360_GET_FLAGS_FROM_BOOT(b0, b2, b3)     ((((uint32_t)b3) << 8) + ((uint32_t)b2) + ((uint32_t)(b0 & 0x01) << 16))
#define ATPL360_GET_FLAGS_FROM_CORTEX(b2, b3)       ((((uint32_t)b3) << 8) + ((uint32_t)b2))

#define ATPL360_CHECK_ID_BOOT_HEADER(val)           ((val & ATPL360_SPI_HEADER_BOOT_MASK) == ATPL360_SPI_HEADER_BOOT)
#define ATPL360_CHECK_ID_CORTEX_HEADER(val)         (val == ATPL360_SPI_HEADER_CORTEX)

#define ATPL360_CHECK_USER_RESET(val)               (val & ATPL360_SPI_HEADER_RESET_FLAG_USER)
#define ATPL360_CHECK_CORTEX_RESET(val)             (val & ATPL360_SPI_HEADER_RESET_FLAG_CORTEX)
#define ATPL360_CHECK_WATCHDOG_RESET(val)           (val & ATPL360_SPI_HEADER_RESET_FLAG_WATCHDOG)
#define ATPL360_CHECK_POWER_ON_RESET(val)           (val & ATPL360_SPI_HEADER_RESET_POWER_ON_MASK)

/* -------- Register Definition -------- */
#define ATPL360_MISCR                               (0x400E1800U) /**< \brief (MISC) Miscelaneous Register */
#define ATPL360_RSTR                                (0x400E1804U) /**< \brief (RSTR) Reset Register */
#define ATPL360_SR                                  (0x400E1808U) /**< \brief (SR) Status Register */

/* -------- ATPL360_MISCR : Miscelaneous Register -------- */
#define ATPL360_MISCR_CPUWAIT                       (0x1u << 0) /**< \brief (ATPL360_MISCR) Cortex M7 Hold */
#define ATPL360_MISCR_PPM_CALIB_ON                  (0x1u << 8) /**< \brief (ATPL360_MISCR) PPM Calibration On */
#define ATPL360_MISCR_PPM_CALIB_OFF                 (0x0u << 8) /**< \brief (ATPL360_MISCR) PPM Calibration Off */
#define ATPL360_MISCR_MEM_128_64_CFG                (0x0u << 16) /**< \brief (ATPL360_MISCR) Memory configuration: 128kB ITCM - 64kB DTCM */
#define ATPL360_MISCR_MEM_96_96_CFG                 (0x1u << 16) /**< \brief (ATPL360_MISCR) Memory configuration: 96kB ITCM - 96kB DTCM */
#define ATPL360_MISCR_EN_ACCESS_ERROR               (0x1u << 24) /**< \brief (ATPL360_MISCR) Access Errors from CM7 enable */
#define ATPL360_MISCR_SET_GPIO_12_ZC                (0x0u << 25) /**< \brief (ATPL360_MISCR) Change GPIO ZeroCross: ZC by GPIO_12 */
#define ATPL360_MISCR_SET_GPIO_2_ZC                 (0x1u << 25) /**< \brief (ATPL360_MISCR) Change GPIO ZeroCross: ZC by GPIO_2 */
#define ATPL360_MISCR_SIGN_FAIL                     (0x1u << 26) /**< \brief (ATPL360_MISCR) Check fail in Signature check */

/* -------- ATPL360_RSTR : Reset Register -------- */
#define ATPL360_RSTR_EN_PROC_RESET                  (0x1u << 0) /**< \brief (ATPL360_RSTR) Enable Processor Reset */
#define ATPL360_RSTR_EN_PER_RESET                   (0x1u << 1) /**< \brief (ATPL360_RSTR) Enable Peripheral Reset */
#define ATPL360_RSTR_EN_PERSLCK_RESET               (0x1u << 2) /**< \brief (ATPL360_RSTR) Enable Peripheral SLCK Reset */
#define ATPL360_RSTR_EN_LOCK_PLL                    (0x1u << 3) /**< \brief (ATPL360_RSTR) Enable Lock PLL */
#define ATPL360_RSTR_VALUE_CONST_RST                (0x24u << 16) /**< \brief (ATPL360_RSTR) Number of cycles on active reset */

/* -------- ATPL360_SR : Status Register -------- */
#define ATPL360_SR_WDT_RESET                        (0x1u << 0) /**< \brief (ATPL360_RSTR) Watchdog Reset */
#define ATPL360_SR_CM7_RESET                        (0x1u << 1) /**< \brief (ATPL360_RSTR) Cortex-M7 Reset */
#define ATPL360_SR_USR_RESET                        (0x1u << 2) /**< \brief (ATPL360_RSTR) User Reset */

/* -------- ATPL360_Fuses : Fuses Control---- --- */
#define ATPL360_FUSES_ENCRNOTPLAIN                  (0x1u << 0) /**< \brief (ATPL360_Fuses) Set to enable secure mode */
#define ATPL360_FUSES_READ_AES_KEY                  (0x1u << 1) /**< \brief (ATPL360_Fuses) Set to disable AES key fuses reading.*/
#define ATPL360_FUSES_WRITE_AES_KEY                 (0x1u << 2) /**< \brief (ATPL360_Fuses) Set to disable AES key fuses writing.*/
#define ATPL360_FUSES_READ_CONTROL                  (0x1u << 5) /**< \brief (ATPL360_Fuses) Set to disable fuses control reading. */
#define ATPL360_FUSES_WRITE_CONTROL                 (0x1u << 6) /**< \brief (ATPL360_Fuses) Set to disable fuses control writing. */
#define ATPL360_FUSES_READ_RAM                      (0x1u << 7) /**< \brief (ATPL360_Fuses) Set to disable ram reading. */
#define ATPL360_FUSES_KEY_MCHP_RDY                  (0x1u << 9) /**< \brief (ATPL360_Fuses) Set to disable bootloader in master mode */
#define ATPL360_FUSES_SIG_IV_NB                     (0x1u << 10) /**< \brief (ATPL360_Fuses) Set to force IV + NB in Signature */
#define ATPL360_FUSES_DISABLE_DBG                   (0x1u << 16) /**< \brief (ATPL360_Fuses) Set to disable Debug access */
#define ATPL360_FUSES_DBG_MSSC                      (0x1u << 17) /**< \brief (ATPL360_Fuses) Set to Debug access depending on MSSC register */

/* -------- ATPL360_Boot_status : Boot status ----------- */
#define ATPL360_FUSES_BOOT_ST_FUSES_ACT             (0x1u << 0) /**< \brief (ATPL360_Boot_status) Fuses Blown active */
#define ATPL360_FUSES_BOOT_ST_AES_ACT               (0x1u << 1) /**< \brief (ATPL360_Boot_status) AES active */
#define ATPL360_FUSES_BOOT_ST_SIGN_OK               (0x1u << 2) /**< \brief (ATPL360_Boot_status) Signature OK */

typedef struct atpl360_spi_data {
	uint16_t us_len;
	uint16_t us_mem_id;
	uint8_t *puc_data_buf;
} atpl360_spi_data_t;

typedef struct atpl360_spi_status_info {
	uint32_t ul_flags;
	uint16_t us_header_id;
} atpl360_spi_status_info_t;

/* ATPL360 SPI Status */
typedef enum {
	ATPL360_SPI_STATUS_UNKNOWN = 0,
	ATPL360_SPI_STATUS_BOOT,
	ATPL360_SPI_STATUS_CORTEX,
	ATPL360_SPI_STATUS_FW_VALIDATING
} atpl360_spi_status_t;

void atpl360_spi_initialize(void);
void atpl360_spi_set_handler(void (*p_handler)(void));

void atpl360_spi_update_fw(uint32_t ul_fw_addr, uint32_t ul_fw_size);
void atpl360_spi_update_fw_sec(uint32_t ul_fw_addr, uint32_t ul_fw_size);

void atpl360_spi_write_buf(atpl360_spi_data_t *px_data);
bool atpl360_spi_read_buf(atpl360_spi_data_t *px_data);
atpl360_spi_status_t atpl360_spi_get_status(void);
void atpl360_spi_get_status_info(atpl360_spi_status_info_t *px_status_info);

void atpl360_spi_boot_write_cmd_enable(void);
void atpl360_spi_boot_disable_cpuwait(void);
void atpl360_spi_boot_disable(void);
void atpl360_spi_clk_boot_disable(void);
void atpl360_spi_boot_config_sys(void);
void atpl360_spi_boot_reset_cmd(void);

void atpl360_spi_boot_sec_set_num_pkts(uint16_t us_num_pkts);
void atpl360_spi_boot_sec_set_init_vector(uint8_t *puc_iv);
void atpl360_spi_boot_sec_set_signature(uint8_t *puc_signature);

#ifdef __cplusplus
}
#endif

#endif /* ATPL360_HAL_SPI_H_INCLUDED */
