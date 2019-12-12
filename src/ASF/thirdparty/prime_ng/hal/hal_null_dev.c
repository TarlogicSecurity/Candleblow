/**
 * \file
 *
 * \brief HAL_NULL_DEV
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


#include "hal.h"
#include "hal_private.h"

static hal_null_dev_read_callback_t  p_read_callback;
static hal_null_dev_write_callback_t p_write_callback;



/**
 * \brief This function closes and disables communication in the specified 
 * channel. 
 *
 * \param chn  Communication channel [0, 1]
 *
 * \retval true on success.
 * \retval false on failure.
 */
uint8_t hal_null_dev_close(uint8_t chn) 
{
	(void) chn;

	/* Clean pointers */
	p_read_callback  = NULL;
	p_write_callback = NULL;
	return true;        
}

/**
 * \brief This function opens the NULL DEV Interface. 
 *
 * \note This interface gives access to USI serialized protocols to the 
 * application level. An application willing to get USI data must
 * set the propper callbacks and configure USI system accordingly.
 * 
 * \param chn			Communication channel [0..4]
 * \param bauds			Not used, 
 *
 * \retval true on success.
 * \retval false on failure.
 */
uint8_t hal_null_dev_open(uint8_t chn, uint32_t bauds)
{  	
	(void) chn;
	(void) bauds;

	p_read_callback  = NULL;
	p_write_callback = NULL;
	return true;  
}


/**
 * \brief This function receives a message.
 *
 * \note This function receives a full USI message from the specified
 *       channel. This function will not block waiting for a message.
 *       USI will poll for incoming messages.
 * 
 * \param  chn     Communication channel [0, 1]
 * \param  buffer  Pointer to buffer for information
 * \param  len     Number of characters to receive
 *
 * \retval Number of received characters
 */
uint16_t hal_null_dev_read(uint8_t chn, void *buffer, uint16_t len)
{
  
	if (p_read_callback != NULL)
	  	return p_read_callback(chn,buffer,len);
	
	return 0;
}

/**
 * \brief This function transmits a message.
 *
 * \note This function transmits characters via the specified channel.
 *
 *
 * \param  chn     Communication channel [0, 1]
 * \param  buffer  Pointer to information to transmit
 * \param  len     Number of characters to transmit
 *
 * \retval Number of characters sent
 */
uint16_t hal_null_dev_write(uint8_t chn, const void *buffer, uint16_t len)
{
  
  	if (p_write_callback != NULL)
	  	return p_write_callback(chn,buffer,len);	
	return 0;
}


/**
 * \brief This function configures the read callback function.
 *
 * \note USI will receive messages from an application polling this callback
 *
 *
 * \param  ptr_func 	Read callback function pointer
 *
 * \retval True on success
 */
bool hal_null_dev_set_read_callback(hal_null_dev_read_callback_t ptr_func)
{
  
  	if (p_read_callback == NULL)
	  	p_read_callback = ptr_func;
	else
		return false;
	
	return true;
}


/**
 * \brief This function configures the write callback function.
 *
 * \note USI will call this function whenever has a message to send.
 *
 *
 * \param  ptr_func 	Write callback function pointer
 *
 * \retval True on success
 */
bool hal_null_dev_set_write_callback(hal_null_dev_write_callback_t ptr_func)
{
  
  	if (p_write_callback == NULL)
	  	p_write_callback = ptr_func;
	else
		return false;
	
	return true;
}

