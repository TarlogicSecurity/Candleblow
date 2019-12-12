/**
 * \file
 *
 * \brief API driver for C0216CiZ COG (Chip-on-Glass) Liquid Crystal Display component.
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

#ifndef C0216CiZ_H_INCLUDED
#define C0216CiZ_H_INCLUDED

#include "compiler.h"
#include "board.h"
#include "conf_c0216CiZ.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/* CONFIGURATION */
#define C0216CiZ_LINES                       2
#define C0216CiZ_LINE_UP                     0
#define C0216CiZ_LINE_DOWN                   1
#define C0216CiZ_LINE_CHARS                  28

/* SLAVE ADDRESS */
#define C0216CiZ_SLAVE_ADDRESS               (0x7C >> 1)

/* CONTROL BYTE */
#define C0216CiZ_CONTROL_WR_DAT              (0x40)
#define C0216CiZ_CONTROL_WR_CMD              (0x00)

/* Level 1 Commands (from the display Datasheet) */
#define C0216CiZ_CMD_CLEAR_DISPLAY           (1 << 0)
#define C0216CiZ_CMD_RETURN_HOME             (1 << 1)
#define C0216CiZ_CMD_ENTRY_MODE_SET          (1 << 2)
#define C0216CiZ_CMD_DISPLAY_ON              (1 << 3)
#define C0216CiZ_CMD_FUNCTION_SET            (1 << 5)
#define C0216CiZ_CMD_SET_DDRAM_ADDR          (1 << 7)

/* Level 2 Commands (IS=0) (from the display Datasheet) */
#define C0216CiZ_CMD_CD_SHIFT                (1 << 4)
#define C0216CiZ_CMD_SET_CGRAM_ADDR          (1 << 6)

/* Level 2 Commands (IS=1) (from the display Datasheet) */
#define C0216CiZ_CMD_SET_INTERNAL_OSC        (1 << 4)
#define C0216CiZ_CMD_SET_ICON_ADDR           (4 << 4)
#define C0216CiZ_CMD_SET_PIC                 (5 << 4)
#define C0216CiZ_CMD_SET_FOLLOWER_C          (6 << 4)
#define C0216CiZ_CMD_SET_CONTRAST            (7 << 4)

/* Entry Mode Option Flags */
#define C0216CiZ_SHIFT_INC_DDRAM_ADDR        (0x02)
#define C0216CiZ_SHIFT_DEC_DDRAM_ADDR        (0x01)

/* Dispaly On Option Flags */
#define C0216CiZ_DISPLAY_D_MSK               (1 << 2)
#define   C0216CiZ_DISPLAY_ON                (1 << 2)
#define   C0216CiZ_DISPLAY_OFF               (0 << 2)
#define C0216CiZ_DISPLAY_C_MSK               (1 << 1)
#define   C0216CiZ_CURSOR_ON                 (1 << 1)
#define   C0216CiZ_CURSOR_OFF                (0 << 1)
#define C0216CiZ_DISPLAY_B_MSK               (1 << 0)
#define   C0216CiZ_CURSOR_BLINK_EN           (1 << 0)
#define   C0216CiZ_CURSOR_BLINK_DIS          (0 << 0)

/* Curor/Display Shift Option Flags */
#define C0216CiZ_CD_SHIFT_SC_MSK             (1 << 3)
#define   C0216CiZ_CD_SHIFT_SCREEN_SEL       (1 << 3)
#define   C0216CiZ_CD_SHIFT_CURSOR_SEL       (0 << 3)
#define C0216CiZ_CD_SHIFT_RL_MSK             (1 << 2)
#define   C0216CiZ_CD_SHIFT_RIGHT            (1 << 2)
#define   C0216CiZ_CD_SHIFT_LEFT             (0 << 2)

/* Function Set Option Flags */
#define C0216CiZ_FSET_DL_MSK                 (1 << 4)
#define   C0216CiZ_FSET_8_BITS               (1 << 4)
#define C0216CiZ_FSET_N_MSK                  (1 << 3)
#define   C0216CiZ_FSET_N_2LN                (1 << 3)
#define   C0216CiZ_FSET_N_1LN                (0 << 3)
#define C0216CiZ_FSET_DH_MSK                 (1 << 2)
#define   C0216CiZ_FSET_DH_16                (1 << 2)
#define   C0216CiZ_FSET_DH_8                 (0 << 2)
#define C0216CiZ_FSET_IS_MSK                 (1 << 0)
#define   C0216CiZ_FSET_IS_EXT               (1 << 0)
#define   C0216CiZ_FSET_IS_NOR               (0 << 0)

/* CGRAM Address Option Flags */
#define C0216CiZ_CGRAM_ADDRESS_MSK           (0x3F)

/* DDRAM Address Option Flags */
#define C0216CiZ_DDRAM_ADDRESS_MSK           (0x7F)

/* Bias selection/Internal OSC frequency adjust Option Flags */
#define C0216CiZ_INT_OSC_BS_MSK              (1 << 3)
#define   C0216CiZ_INT_OSC_BS_1_4            (1 << 3)
#define   C0216CiZ_INT_OSC_BS_1_5            (0 << 3)
#define C0216CiZ_INT_OSC_F_ADJ_MSK           (7 << 0)
#define   C0216CiZ_INT_OSC_F_ADJ_122         (0 << 0)
#define   C0216CiZ_INT_OSC_F_ADJ_131         (1 << 0)
#define   C0216CiZ_INT_OSC_F_ADJ_144         (2 << 0)
#define   C0216CiZ_INT_OSC_F_ADJ_161         (3 << 0)
#define   C0216CiZ_INT_OSC_F_ADJ_183         (4 << 0)
#define   C0216CiZ_INT_OSC_F_ADJ_221         (5 << 0)
#define   C0216CiZ_INT_OSC_F_ADJ_274         (6 << 0)
#define   C0216CiZ_INT_OSC_F_ADJ_347         (7 << 0)
#define C0216CiZ_ICON_RAM_ADDRESS_MSK        (0x0F)

/* Power/ICON control/Contrast set Option Flags */
#define C0216CiZ_PIC_ION_MSK                 (1 << 3)
#define   C0216CiZ_ICON_DISPLAY_ON           (1 << 3)
#define   C0216CiZ_ICON_DISPLAY_OFF          (0 << 3)
#define C0216CiZ_PIC_BON_MSK                 (1 << 2)
#define   C0216CiZ_ICON_BOOSTER_ON           (1 << 2)
#define   C0216CiZ_ICON_BOOSTER_OFF          (0 << 2)
#define C0216CiZ_PIC_CONTRAST_C5C4_MSK       (0x03)

/* Follower Control Option Flags */
#define C0216CiZ_FOLLOWER_FON_MSK            (1 << 3)
#define   C0216CiZ_FOLLOWER_ON               (1 << 3)
#define   C0216CiZ_FOLLOWER_OFF              (0 << 3)
#define C0216CiZ_V0_GEN_MSK                  (0x07)

/* Contrast Set Option Flags */
#define C0216CiZ_CONTRAST_C3C2C1C0_MSK       (0x0F)

uint8_t c0216CiZ_init(void);

/* low level functions */
void c0216CiZ_clear_display(void);
void c0216CiZ_return_home(void);
void c0216CiZ_set_entry_mode(uint8_t uc_flags);
void c0216CiZ_set_display(uint8_t uc_flags);
void c0216CiZ_set_function(uint8_t uc_flags);
void c0216CiZ_set_ddram_addr(uint8_t uc_addr);
void c0216CiZ_set_cd_shift(uint8_t uc_flags);
void c0216CiZ_set_cgram(uint8_t uc_addr);
void c0216CiZ_set_internal_osc(uint8_t uc_flags);
void c0216CiZ_set_icon_addr(uint8_t uc_flags);
void c0216CiZ_set_pic(uint8_t uc_flags);
void c0216CiZ_set_follower(uint8_t uc_flags);
void c0216CiZ_set_contrast(uint8_t uc_value);

/* high level functions */
void c0216CiZ_show(const char *puc_char);
void c0216CiZ_set_cursor(uint8_t uc_line, uint8_t uc_pos);
void c0216CiZ_shift_cursor_right(void);
void c0216CiZ_shift_cursor_left(void);
void c0216CiZ_shift_display_right(void);
void c0216CiZ_shift_display_left(void);

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */

#endif /* C0216CiZ_H_INCLUDED */
