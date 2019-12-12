/**
 * \file
 *
 * \brief HAL_AES
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
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

#include "string.h"
#include "asf.h"
#include "hal_private.h"
#include "hal.h"

#if PRIME_HAL_VERSION == HAL_PRIME_1_4

#if !(SAM4E || SAM4C || SAM4CP || SAM4CM || SAMV70 || SAMV71 || SAME70 || SAMS70)
#include "mbedtls/aes.h"
#endif

/** Priority of the AES interrupt */
#define AES_PRIO           1

/** AES configuration */
#if SAM4E || SAM4C || SAM4CP || SAM4CM || SAMV70 || SAMV71 || SAME70 || SAMS70
struct aes_config g_aes_cfg;
#else
mbedtls_aes_context aes_ctx;
#endif

/** Key in byte format */
static uint8_t spuc_key[256];

#if !(SAM4E || SAM4C || SAM4CP || SAM4CM || SAMV70 || SAMV71 || SAME70 || SAMS70)
/** Key size in bits */
static uint16_t sus_key_size;
#endif

#if SAM4E || SAM4C || SAM4CP || SAM4CM || SAMV70 || SAMV71 || SAME70 || SAMS70
/** Output data */
static uint32_t *spul_output_data;

/* State indicate */
volatile bool b_aes_state;
#endif

/** Callback function pointer for AES encryption*/
static void (*_aes_cb_function)(void);

#if SAM4E || SAM4C || SAM4CP || SAM4CM || SAMV70 || SAMV71 || SAME70 || SAMS70
/**
 * \brief The AES interrupt call back function.
 */
static void aes_int_callback(void)
{
	/* Read the output. */
	aes_read_output_data(AES, spul_output_data);
	b_aes_state = true;
}
#endif

/**
 * \brief Provide key for AES encryption/decryption
 *
 * \param puc_key        Pointer to the key
 * \param uc_key_len     Length of the key in bytes
 *
 */
void hal_aes_key(uint8_t *puc_key, uint8_t uc_key_len)
{
	/* Store the key */
	memcpy(spuc_key, puc_key, uc_key_len);

#if SAM4E || SAM4C || SAM4CP || SAM4CM || SAMV70 || SAMV71 || SAME70 || SAMS70
	/* Set the key size */
	switch(uc_key_len) {
	case 16:
		g_aes_cfg.key_size = AES_KEY_SIZE_128;
		break;
	case 24:
		g_aes_cfg.key_size = AES_KEY_SIZE_192;
		break;
	case 32:
		g_aes_cfg.key_size = AES_KEY_SIZE_256;
		break;
	}
#else
	/* Store the key size */
	sus_key_size = uc_key_len * 8;
#endif
}

/**
 * \brief Request AES encryption/decryption
 *
 * \param b_crypt_mode   Encryption mode (0 = decryption, 1 = encryption)
 * \param puc_in_text    Pointer to the input text
 * \param puc_out_text   Pointer to the output text
 *
 */
void hal_aes_crypt(bool b_crypt_mode, uint8_t *puc_in_text, uint8_t *puc_out_text)
{
#if SAM4E || SAM4C || SAM4CP || SAM4CM || SAMV70 || SAMV71 || SAME70 || SAMS70
	b_aes_state = false;

	/* Configure the AES. */
	g_aes_cfg.encrypt_mode = b_crypt_mode ? AES_ENCRYPTION : AES_DECRYPTION;
	/* g_aes_cfg.key_size set when key received */
	g_aes_cfg.start_mode = AES_AUTO_START;
	g_aes_cfg.opmode = AES_ECB_MODE;
	g_aes_cfg.cfb_size = AES_CFB_SIZE_128;
	g_aes_cfg.lod = false;
	aes_set_config(AES, &g_aes_cfg);

	/* Set the cryptographic key. */
	aes_write_key(AES, (uint32_t const *)spuc_key);

	/* The initialization vector is not used by the ECB cipher mode. */
		  
	/* Set the pointer to the output data */
	spul_output_data = (uint32_t *)puc_out_text;

	/* Write the data to be ciphered to the input data registers. */
	aes_write_input_data(AES, (uint32_t const *)puc_in_text);

	/* Wait for the end of the encryption process. */
	while (false == b_aes_state) {
		;
	}

	/* Return result - the output data is already in the buffer */
	_aes_cb_function();
#else
	/* Initialize the AES */
	mbedtls_aes_init(&aes_ctx);

	 /* Trigger the AES */
	if (b_crypt_mode) {
		mbedtls_aes_setkey_enc(&aes_ctx, spuc_key, sus_key_size);
		mbedtls_aes_encrypt(&aes_ctx, puc_in_text, puc_out_text);
	} else {
		mbedtls_aes_setkey_dec(&aes_ctx, spuc_key, sus_key_size);
		mbedtls_aes_decrypt(&aes_ctx, puc_in_text, puc_out_text);
	}

	/* Return result - the output data is already in the buffer */
	_aes_cb_function();

	/* Free the AES */
	mbedtls_aes_free(&aes_ctx);
#endif
}

/**
 * \brief Set handler to AES encryption callback
 *
 * \param p_handler   AES encryption callback function pointer
 *
 */
void hal_aes_set_callback(void (*p_handler)(void))
{
	_aes_cb_function = p_handler;
}

/**
 * \brief Initialize module
 *
 */
void hal_aes_init(void)
{
	_aes_cb_function = NULL;

#if SAM4E || SAM4C || SAM4CP || SAM4CM || SAMV70 || SAMV71 || SAME70 || SAMS70
	/* Enable the AES module. */
	aes_get_config_defaults(&g_aes_cfg);
	aes_init(AES, &g_aes_cfg);
	aes_enable();

	/* Enable AES interrupt. */
	aes_set_callback(AES, AES_INTERRUPT_DATA_READY, aes_int_callback, AES_PRIO);
#endif
}

#endif /* PRIME_HAL_VERSION == HAL_PRIME_1_4 */
