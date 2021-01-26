/* Edge Impulse ingestion SDK
 * Copyright (c) 2020 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef PORTENTA_FS_COMMANDS_H
#define PORTENTA_FS_COMMANDS_H

/* Include ----------------------------------------------------------------- */
#include <stdint.h>

#define PORTENTA_FS_BLOCK_ERASE_TIME_MS		90

/** Nano fs return values */
typedef enum
{
	PORTENTA_FS_CMD_OK = 0,						/**!< All is well				 */
	PORTENTA_FS_CMD_NOT_INIT,					/**!< FS is not initialised		 */
	PORTENTA_FS_CMD_READ_ERROR,					/**!< Error occured during read  */
	PORTENTA_FS_CMD_WRITE_ERROR,				/**!< Error occured during write */
	PORTENTA_FS_CMD_ERASE_ERROR,				/**!< Erase error occured		 */
	PORTENTA_FS_CMD_NULL_POINTER,				/**!< Null pointer parsed		 */

}ei_portenta_ret_t;

/* Function prototypes ----------------------------------------------------- */
bool ei_portenta_fs_init(void);
int ei_portenta_fs_load_config(uint32_t *config, uint32_t config_size);
int ei_portenta_fs_save_config(const uint32_t *config, uint32_t config_size);

int ei_portenta_fs_prepare_sampling(void);
int ei_portenta_fs_erase_sampledata(uint32_t start_block, uint32_t end_address);
int ei_portenta_fs_write_sample_block(const void *sample_buffer, uint32_t address_offset);
int ei_portenta_fs_write_samples(const void *sample_buffer, uint32_t address_offset, uint32_t num_write_bytes);
int ei_portenta_fs_read_sample_data(void *sample_buffer, uint32_t address_offset, uint32_t num_read_bytes);

uint32_t ei_portenta_fs_get_block_size(void);
uint32_t ei_portenta_fs_get_n_available_sample_blocks(void);

#endif