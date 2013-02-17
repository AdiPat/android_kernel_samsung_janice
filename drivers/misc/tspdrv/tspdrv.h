/*
** =========================================================================
** File:
**     tspdrv.h
**
** Description: 
**     Constants and type definitions for the TouchSense Kernel Module.
**
** Portions Copyright (c) 2008-2010 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/

#ifndef _TSPDRV_H
#define _TSPDRV_H

#include <linux/input.h>

/* Constants */
#define MODULE_NAME				"tspdrv"
#define TSPDRV					"/dev/"MODULE_NAME
#define TSPDRV_MAGIC_NUMBER			0x494D4D52
#define TSPDRV_STOP_KERNEL_TIMER		_IO(TSPDRV_MAGIC_NUMBER & 0xFF, 1)
#define TSPDRV_ENABLE_AMP                       _IO(TSPDRV_MAGIC_NUMBER & 0xFF, 3)
#define TSPDRV_DISABLE_AMP                      _IO(TSPDRV_MAGIC_NUMBER & 0xFF, 4)
#define TSPDRV_GET_NUM_ACTUATORS                _IO(TSPDRV_MAGIC_NUMBER & 0xFF, 5)
#define VIBE_MAX_DEVICE_NAME_LENGTH	        64
#define SPI_HEADER_SIZE                 	3   /* DO NOT CHANGE - SPI buffer header size */
#define VIBE_OUTPUT_SAMPLE_SIZE                 50  /* DO NOT CHANGE - maximum number of samples */
#define ISA1200_POLLING_DELAY			5
#define IMMVIBE_RETRY_COUNT			10

typedef struct
{
    u_int8_t nActuatorIndex;  /* 1st byte is actuator index */
    u_int8_t nBitDepth;       /* 2nd byte is bit depth */
    u_int8_t nBufferSize;     /* 3rd byte is data size */
    u_int8_t dataBuffer[VIBE_OUTPUT_SAMPLE_SIZE];
} samples_buffer;

typedef struct
{
    int8_t nIndexPlayingBuffer;
    u_int8_t nIndexOutputValue;
    samples_buffer actuatorSamples[2]; /* Use 2 buffers to receive samples from user mode */
} actuator_samples_buffer;

/* Error and Return value codes */
#define VIBE_S_SUCCESS                      0	/* Success */
#define VIBE_E_FAIL		           -4	/* Generic error */


/* isa1200 specifc */
struct isa1200_data
{
	unsigned char	addr;
	struct clk *mot_clk;
	struct i2c_client *client;
	struct input_dev	*input;
	struct isa1200_platform_data *pdata;
};
#endif
