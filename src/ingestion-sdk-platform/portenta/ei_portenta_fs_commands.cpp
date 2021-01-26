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

/* Include ----------------------------------------------------------------- */
#include "ei_portenta_fs_commands.h"

#include "mbed.h"
#include "FlashIAP.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

/* Private types & constants ---------------------------------------------- */

#define SERIAL_FLASH	0
#define MICRO_SD		1
#define RAM				2

#define SAMPLE_MEMORY	RAM
#define RAM_BLOCK_SIZE	4096
#define RAM_N_BLOCKS	20
#define SIZE_RAM_BUFFER	(RAM_BLOCK_SIZE * RAM_N_BLOCKS)

/**
 * File system config struct.
 * @details Holds all the info needed for config file and sample data.<br>
 * - The config file is stored in the last available sector<br>
 * - The sample data starts at the end of the program data and ends before the
 * config file
 */
typedef struct
{
	uint32_t sector_size;					/*!< Erase sector size 			 */
	uint32_t page_size;						/*!< Minimum page write size 	 */
	uint32_t config_file_address;			/*!< Start address of config file*/
	uint32_t sample_start_address;			/*!< Start of sample storage mem */
	bool     fs_init;						/*!< FS is successfully init  	 */

}ei_portenta_fs_t;

/** 32-bit align write buffer size */
#define WORD_ALIGN(a)	((a & 0x3) ? (a & ~0x3) + 0x4 : a)
/** Align addres to given sector size */
#define SECTOR_ALIGN(a, sec_size)	((a & (sec_size-1)) ? (a & ~(sec_size-1)) + sec_size : a)

/* Private variables ------------------------------------------------------- */
static mbed::FlashIAP iap;
static ei_portenta_fs_t portenta_fs = {0};

#if(SAMPLE_MEMORY == RAM)
static uint8_t ram_memory[SIZE_RAM_BUFFER];
#endif

/* Public functions -------------------------------------------------------- */

/**
 * @brief      Init Flash pheripheral for reading & writing and set all
 * 			   parameters for the file system.
 * @return     true if succesful else false
 */
bool ei_portenta_fs_init(void)
{
	iap.init();

	/* Setup addresses for fs */
	portenta_fs.sector_size = iap.get_sector_size(iap.get_flash_start() + iap.get_flash_size() - 1UL);
    portenta_fs.page_size = iap.get_page_size();
    portenta_fs.config_file_address = (iap.get_flash_start() + iap.get_flash_size()) - (portenta_fs.sector_size);
    portenta_fs.sample_start_address = SECTOR_ALIGN(FLASHIAP_APP_ROM_END_ADDR, portenta_fs.sector_size);

    // ei_printf("Start config: %X start sample: %X size sample: %d\r\n", portenta_fs.config_file_address,
    // 	portenta_fs.sample_start_address, portenta_fs.config_file_address - portenta_fs.sample_start_address);

	/* Check correct init of all parameters */
	if((portenta_fs.sector_size == 0) || (portenta_fs.page_size == 0)
		|| (portenta_fs.config_file_address == 0) || (portenta_fs.sample_start_address == 0)) {
		portenta_fs.fs_init = false;
	}
	else {
		portenta_fs.fs_init = true;
	}

	return portenta_fs.fs_init;
}

/**
 * @brief      Copy configuration data to config pointer
 *
 * @param      config       Destination pointer for config
 * @param[in]  config_size  Size of configuration in bytes
 *
 * @return     ei_portenta_ret_t enum
 */
int ei_portenta_fs_load_config(uint32_t *config, uint32_t config_size)
{
	ei_portenta_ret_t ret;

	if(config == NULL) {
		ret = PORTENTA_FS_CMD_NULL_POINTER;
	}

	else if(portenta_fs.fs_init == true) {

		ret = (iap.read((void *)config, portenta_fs.config_file_address, config_size) == 0)
			? PORTENTA_FS_CMD_OK
			: PORTENTA_FS_CMD_READ_ERROR;
	}
	else {
		ret = PORTENTA_FS_CMD_NOT_INIT;
	}

	return (int)ret;
}

/**
 * @brief      Write config to Flash
 *
 * @param[in]  config       Pointer to configuration data
 * @param[in]  config_size  Size of configuration in bytes
 *
 * @return     ei_portenta_ret_t enum
 */
int ei_portenta_fs_save_config(const uint32_t *config, uint32_t config_size)
{
	ei_portenta_ret_t ret;

	if(config == NULL) {
		ret = PORTENTA_FS_CMD_NULL_POINTER;
	}

	else if(portenta_fs.fs_init == true) {

		ret = (iap.erase(portenta_fs.config_file_address, portenta_fs.sector_size) == 0)
			? PORTENTA_FS_CMD_OK
			: PORTENTA_FS_CMD_ERASE_ERROR;

		if(ret == PORTENTA_FS_CMD_OK) {

			ret = (iap.program((const void *)config, portenta_fs.config_file_address, WORD_ALIGN(config_size)) == 0)
				? PORTENTA_FS_CMD_OK
				: PORTENTA_FS_CMD_WRITE_ERROR;
		}
		else {
			ret = PORTENTA_FS_CMD_ERASE_ERROR;
		}
	}
	else {
		ret = PORTENTA_FS_CMD_NOT_INIT;
	}

	return (int)ret;
}

int ei_portenta_fs_prepare_sampling(void)
{
	ei_portenta_ret_t ret;

	if(portenta_fs.fs_init == true) {
		ret = (iap.erase(portenta_fs.sample_start_address, portenta_fs.config_file_address - portenta_fs.sample_start_address) == 0)
			? PORTENTA_FS_CMD_OK
			: PORTENTA_FS_CMD_ERASE_ERROR;
	}
	else {
		ret = PORTENTA_FS_CMD_NOT_INIT;
	}

	return ret;
}

int ei_portenta_fs_erase_sampledata(uint32_t start_block, uint32_t end_address)
{
	ei_portenta_ret_t ret;

	if(portenta_fs.fs_init == true) {
		ret = (iap.erase(portenta_fs.sample_start_address, SECTOR_ALIGN(end_address, portenta_fs.sector_size)) == 0)
			? PORTENTA_FS_CMD_OK
			: PORTENTA_FS_CMD_ERASE_ERROR;
	}
	else {
		ret = PORTENTA_FS_CMD_NOT_INIT;
	}

	return ret;
}

int ei_portenta_fs_write_sample_block(const void *sample_buffer, uint32_t address_offset)
{
	ei_portenta_ret_t ret;

	if(sample_buffer == NULL) {
		ret = PORTENTA_FS_CMD_NULL_POINTER;
	}
	else if (portenta_fs.fs_init == true) {

		ret = (iap.program(sample_buffer, portenta_fs.sample_start_address + address_offset, portenta_fs.sector_size) == 0)
			? PORTENTA_FS_CMD_OK
			: PORTENTA_FS_CMD_WRITE_ERROR;
	}
	else {
		ret = PORTENTA_FS_CMD_NOT_INIT;
	}

	return ret;
}

int ei_portenta_fs_write_samples(const void *sample_buffer, uint32_t address_offset, uint32_t num_write_bytes)
{
	ei_portenta_ret_t ret = PORTENTA_FS_CMD_OK;

	if(sample_buffer == NULL) {
		return PORTENTA_FS_CMD_NULL_POINTER;
	}

#if(SAMPLE_MEMORY == RAM)

    uint32_t aligned_num_write_bytes = WORD_ALIGN(num_write_bytes);

    if((address_offset + aligned_num_write_bytes) > SIZE_RAM_BUFFER) {
        ret = PORTENTA_FS_CMD_WRITE_ERROR;
    }
    else {
	    for(int i = 0;  i < aligned_num_write_bytes; i++) {
	        ram_memory[address_offset + i] = *((char *)sample_buffer + i);
	    }
	}

#elif(SAMPLE_MEMORY == SERIAL_FLASH)

	if (portenta_fs.fs_init == true) {

		ret = (iap.program(sample_buffer, WORD_ALIGN(portenta_fs.sample_start_address + address_offset), WORD_ALIGN(num_write_bytes)) == 0)
			? PORTENTA_FS_CMD_OK
			: PORTENTA_FS_CMD_WRITE_ERROR;
	}
	else {
		ret = PORTENTA_FS_CMD_NOT_INIT;
	}
#endif

	return ret;
}

int ei_portenta_fs_read_sample_data(void *sample_buffer, uint32_t address_offset, uint32_t num_read_bytes)
{
	ei_portenta_ret_t ret = PORTENTA_FS_CMD_OK;

	if(sample_buffer == NULL) {
		return PORTENTA_FS_CMD_NULL_POINTER;
	}

#if(SAMPLE_MEMORY == RAM)

	uint32_t aligned_num_read_bytes = WORD_ALIGN(num_read_bytes);

    if((address_offset + num_read_bytes) > SIZE_RAM_BUFFER) {
        ret =  PORTENTA_FS_CMD_READ_ERROR;
    }
    else {
	    for(int i = 0;  i < aligned_num_read_bytes; i++) {
	        *((char *)sample_buffer + i) = ram_memory[address_offset + i];
	    }
	}

#elif(SAMPLE_MEMORY == SERIAL_FLASH)

	if(portenta_fs.fs_init == true) {

		ret = (iap.read((void *)sample_buffer, portenta_fs.sample_start_address + address_offset, num_read_bytes) == 0)
			? PORTENTA_FS_CMD_OK
			: PORTENTA_FS_CMD_READ_ERROR;
	}
	else {
		ret = PORTENTA_FS_CMD_NOT_INIT;
	}

#endif

	return (int)ret;
}

uint32_t ei_portenta_fs_get_block_size(void)
{
	uint32_t block_size = 0;

#if(SAMPLE_MEMORY == RAM)
    return block_size = RAM_BLOCK_SIZE;
#elif(SAMPLE_MEMORY == SERIAL_FLASH)

	if(portenta_fs.fs_init == true) {
		block_size = portenta_fs.sector_size;
	}
#endif
	return block_size;
}

uint32_t ei_portenta_fs_get_n_available_sample_blocks(void)
{
	uint32_t n_sample_blocks = 0;

#if(SAMPLE_MEMORY == RAM)
    n_sample_blocks = RAM_N_BLOCKS;
#elif(SAMPLE_MEMORY == SERIAL_FLASH)

	if(portenta_fs.fs_init == true) {
		n_sample_blocks = (portenta_fs.config_file_address - portenta_fs.sample_start_address) / portenta_fs.sector_size;
	}
#endif
	return n_sample_blocks;
}
