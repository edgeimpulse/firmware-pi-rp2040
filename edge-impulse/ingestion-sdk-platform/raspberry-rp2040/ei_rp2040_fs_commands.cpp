/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
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
#include "ei_rp2040_fs_commands.h"
#include "ei_device_raspberry_rp2040.h"

#include "ei_classifier_porting.h"

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include <cmath>
#include <stdio.h>
#include <string.h>

/* Private types & constants ---------------------------------------------- */

/**
 * File system config struct.
 * @details Holds all the info needed for config file and sample data.<br>
 * - The config file is stored in the last available sector<br>
 * - The sample data starts at the end of the program data and ends before the
 * config file
 */
typedef struct {
    uint32_t sector_size; /*!< Erase sector size 			 */
    uint32_t page_size; /*!< Minimum page write size 	 */
    uint32_t config_file_address; /*!< Start address of config file*/
    uint32_t sample_start_address; /*!< Start of sample storage mem */
    bool fs_init; /*!< FS is successfully init  	 */

} ei_rp2040_fs_t;

/** 32-bit align write buffer size */
#define WORD_ALIGN(a) ((a & 0x3) ? (a & ~0x3) + 0x4 : a)
/** Align addres to given sector size */
#define SECTOR_ALIGN(a, sec_size) ((a & (sec_size - 1)) ? (a & ~(sec_size - 1)) + sec_size : a)

/* Private variables ------------------------------------------------------- */

static ei_rp2040_fs_t rp2040_fs = { 0 };
extern char __flash_binary_end;

uint8_t internal_buffer[256];
uint32_t buf_pos = 0;
uint32_t _address_offset = 0;

/* Public functions -------------------------------------------------------- */

/**
 * @brief      Init Flash peripheral for reading & writing and set all
 * 			   parameters for the file system.
 * @return     true if successful else false
 */
bool ei_rp2040_fs_init(void)
{

    /* Setup addresses for fs */
    rp2040_fs.sector_size = FLASH_SECTOR_SIZE;
    rp2040_fs.page_size = FLASH_PAGE_SIZE;
    rp2040_fs.config_file_address = (XIP_BASE + 2048 * 1024 - (rp2040_fs.sector_size));

    uintptr_t flash_end = (uintptr_t)&__flash_binary_end;

    rp2040_fs.sample_start_address =
        SECTOR_ALIGN((uintptr_t)&__flash_binary_end, rp2040_fs.sector_size);

    DEBUG_PRINT("Flash binary ends at %08x\n", flash_end);
    DEBUG_PRINT(
        "Start config: %08x \nstart sample: %08x \nsize sample: %d\r\n",
        rp2040_fs.config_file_address,
        rp2040_fs.sample_start_address,
        rp2040_fs.config_file_address - rp2040_fs.sample_start_address);

    /* Check correct init of all parameters */
    if ((rp2040_fs.sector_size == 0) || (rp2040_fs.page_size == 0) ||
        (rp2040_fs.config_file_address == 0) || (rp2040_fs.sample_start_address == 0)) {
        rp2040_fs.fs_init = false;
    }
    else {
        rp2040_fs.fs_init = true;
    }

    return rp2040_fs.fs_init;
}

/**
 * @brief      Copy configuration data to config pointer
 *
 * @param      config       Destination pointer for config
 * @param[in]  config_size  Size of configuration in bytes
 *
 * @return     ei_rp2040_ret_t enum
 */
int ei_rp2040_fs_load_config(uint32_t *config, uint32_t config_size)
{
    ei_rp2040_ret_t ret;

    if (config == NULL) {
        ret = RP2040_FS_CMD_NULL_POINTER;
    }

    else if (rp2040_fs.fs_init == true) {

        ei_rp2040_copy_buf((uint8_t *)config, (uint8_t *)rp2040_fs.config_file_address, config_size);
        ret = RP2040_FS_CMD_OK;
    }

    else {
        ret = RP2040_FS_CMD_NOT_INIT;
    }

    return (int)ret;
}

/**
 * @brief      Write config to Flash
 *
 * @param[in]  config       Pointer to configuration data
 * @param[in]  config_size  Size of configuration in bytes
 *
 * @return     ei_rp2040_ret_t enum
 */
int ei_rp2040_fs_save_config(const uint32_t *config, uint32_t config_size)
{
    ei_rp2040_ret_t ret;

    if (config == NULL) {
        ret = RP2040_FS_CMD_NULL_POINTER;
    }

    else if (rp2040_fs.fs_init == true) {
        uint32_t ints = save_and_disable_interrupts();
        DEBUG_PRINT("erase\n");
        DEBUG_PRINT("%d \n", rp2040_fs.config_file_address - XIP_BASE);
        flash_range_erase(rp2040_fs.config_file_address - XIP_BASE, rp2040_fs.sector_size);
        DEBUG_PRINT("erase suc\n");
        flash_range_program(
            rp2040_fs.config_file_address - XIP_BASE,
            (const uint8_t *)config,
            WORD_ALIGN(config_size));
        DEBUG_PRINT("program suc\n");
        restore_interrupts(ints);
        ret = RP2040_FS_CMD_OK;
    }
    else {
        ret = RP2040_FS_CMD_NOT_INIT;
    }

    return (int)ret;
}

/**
 * @brief      Erases the whole flash sample region
 *
 *
 * @return     ei_rp2040_ret_t enum
 */

int ei_rp2040_fs_prepare_sampling(void)
{
    ei_rp2040_ret_t ret;

    if (rp2040_fs.fs_init == true) {
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(
            rp2040_fs.sample_start_address - XIP_BASE,
            rp2040_fs.config_file_address - rp2040_fs.sample_start_address);
        restore_interrupts(ints);
        ret = RP2040_FS_CMD_OK;
    }
    else {
        ret = RP2040_FS_CMD_NOT_INIT;
    }

    return ret;
}

/**
 * @brief      Erases data from flash
 *
 * @param[in]  start_block  Number of block to start erasing
 * @param[in]  count  Number of bytes to be erased
 *
 * @return     ei_rp2040_ret_t enum
 */

int ei_rp2040_fs_erase_sampledata(uint32_t start_block, uint32_t count)
{
    ei_rp2040_ret_t ret;
    DEBUG_PRINT("erasing %d bytes\n", SECTOR_ALIGN(count, rp2040_fs.sector_size));
    if (rp2040_fs.fs_init == true) {
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(
            rp2040_fs.sample_start_address - XIP_BASE,
            SECTOR_ALIGN(count, rp2040_fs.sector_size));
        restore_interrupts(ints);
        ret = RP2040_FS_CMD_OK;
    }
    else {
        ret = RP2040_FS_CMD_NOT_INIT;
    }
    return ret;
}

/**
 * @brief      Writes a whole block(page) of data to flash
 *
 * @param[in]  sample_buffer   Pointer to sample data block
 * @param[in]  address_offset  Offset in memory from the beginning of sample space in flash
 *
 * @return     ei_rp2040_ret_t enum
 */

int ei_rp2040_fs_write_sample_block(const void *sample_buffer, uint32_t address_offset)
{
    ei_rp2040_ret_t ret;

    DEBUG_PRINT("data to write\n");
    print_buf((uint8_t *)sample_buffer, 256);
    DEBUG_PRINT("\n");

    DEBUG_PRINT("checking if space is empty\n");
    print_buf((uint8_t *)(rp2040_fs.sample_start_address + address_offset), 256);
    DEBUG_PRINT("\n");

    DEBUG_PRINT(
        "target addr: %08x %d\n",
        rp2040_fs.sample_start_address + address_offset,
        rp2040_fs.sample_start_address + address_offset);

    if (sample_buffer == NULL) {
        ret = RP2040_FS_CMD_NULL_POINTER;
    }
    else if (rp2040_fs.fs_init == true) {
        uint32_t ints = save_and_disable_interrupts();
        flash_range_program(
            rp2040_fs.sample_start_address + address_offset - XIP_BASE,
            (const uint8_t *)sample_buffer,
            rp2040_fs.page_size);
        restore_interrupts(ints);
        ret = RP2040_FS_CMD_OK;
    }
    else {
        ret = RP2040_FS_CMD_NOT_INIT;
    }

    DEBUG_PRINT("reading back\n");
    print_buf((uint8_t *)(rp2040_fs.sample_start_address + address_offset), 256);
    DEBUG_PRINT("\n");

    return ret;
}

/**
 * @brief      Writes samples to internal buffer, when buffer is filled, writes data to flash
 *
 * @param[in]  sample_buffer  Pointer to sample data 
 * @param[in]  address_offset  NOT USED
 * @param[in]  n_samples  Size of data, in bytes
 * 
 * @return     ei_rp2040_ret_t enum
 */

int ei_rp2040_fs_write_samples(
    const void *sample_buffer,
    uint32_t address_offset,
    uint32_t n_samples)
{
    ei_rp2040_ret_t ret;

    DEBUG_PRINT("inter data to write\n");
    print_buf((uint8_t *)sample_buffer, n_samples);
    DEBUG_PRINT("\n");

    uint32_t num_blocks = ceil(
        float(n_samples) / float(rp2040_fs.page_size)); //< 1 ? 1 : n_samples / rp2040_fs.page_size;

    DEBUG_PRINT("number of blocks %d\n", num_blocks);

    for (uint32_t i = 0; i < num_blocks; i++) {
        DEBUG_PRINT("block_num %d\n", i);

        if (n_samples < 256) {

            for (uint32_t i = 0; i < n_samples; i++) {

                if (buf_pos < 256) {
                    internal_buffer[buf_pos++] = ((const uint8_t *)sample_buffer)[i];
                }
                else {
                    ei_rp2040_fs_write_sample_block(internal_buffer, _address_offset);
                    _address_offset += buf_pos;
                    DEBUG_PRINT("%d\n", _address_offset);
                    buf_pos = 0;
                    internal_buffer[buf_pos++] = ((const uint8_t *)sample_buffer)[i];
                }
            }
        }

        else {

            for (uint32_t j = i * rp2040_fs.page_size;
                 j < i * rp2040_fs.page_size + rp2040_fs.page_size;
                 j++) {

                if (j > n_samples - 1)
                    break;

                if (buf_pos < 256) {
                    internal_buffer[buf_pos++] = ((const uint8_t *)sample_buffer)[j];
                }
                else {
                    ei_rp2040_fs_write_sample_block(internal_buffer, _address_offset);
                    _address_offset += buf_pos;
                    DEBUG_PRINT("%d\n", _address_offset);
                    buf_pos = 0;
                    internal_buffer[buf_pos++] = ((const uint8_t *)sample_buffer)[j];
                }
            }
        }
    }
    ret = RP2040_FS_CMD_OK;

    return ret;
}

/**
 * @brief      Copy sample data from flash to sample buffer
 *
 * @param[in]  sample_buffer Pointer to buffer, where to place the data after reading
 * @param[in]  address_offset  Flash address where to start reading from
 * @param[in]  n_read_bytes  Number of bytes to read
 * 
 * @return     ei_rp2040_ret_t enum
 */

int ei_rp2040_fs_read_sample_data(
    void *sample_buffer,
    uint32_t address_offset,
    uint32_t n_read_bytes)
{
    ei_rp2040_ret_t ret;

    if (sample_buffer == NULL) {
        ret = RP2040_FS_CMD_NULL_POINTER;
    }

    else if (rp2040_fs.fs_init == true) {

        ei_rp2040_copy_buf(
            (uint8_t *)sample_buffer,
            (uint8_t *)(rp2040_fs.sample_start_address + address_offset),
            n_read_bytes);
        print_buf((uint8_t *)(rp2040_fs.sample_start_address + address_offset), n_read_bytes);
        ret = RP2040_FS_CMD_OK;
    }
    else {
        ret = RP2040_FS_CMD_NOT_INIT;
    }

    return (int)ret;
}

/**
 * @brief      Get block (sector) size
 *
 *
 * @return     block (sector) size in bytes
 */

uint32_t ei_rp2040_fs_get_block_size(void)
{
    uint32_t block_size = 0;

    if (rp2040_fs.fs_init == true) {
        block_size = rp2040_fs.sector_size;
    }

    return block_size;
}

/**
 * @brief      Get number of available blocks (sectors)
 *
 *
 * @return     number of available blocks (sectors)
 */

uint32_t ei_rp2040_fs_get_n_available_sample_blocks(void)
{
    uint32_t n_sample_blocks = 0;
    if (rp2040_fs.fs_init == true) {
        n_sample_blocks = (rp2040_fs.config_file_address - rp2040_fs.sample_start_address) /
            rp2040_fs.sector_size;
    }
    return n_sample_blocks;
}

/**
 * @brief      Copy data from flash to buffer, possibly need to optimize
 *
 * @param[in]  buf      Pointer to destination buffer
 * @param[in]  src_add  Source address
 * @param[in]  size  number of bytes to copy
 * 
 * @return     ei_rp2040_ret_t enum
 */

void ei_rp2040_copy_buf(uint8_t *buf, uint8_t *src_add, uint32_t size)
{
    //memcpy(&buf, &src_add, size);
    for (uint32_t i = 0; i < size; ++i) {
        buf[i] = src_add[i];
    }
}

/**
 * @brief      Flush data in internal buffer to make sure all the data is written
 *
 * @param[in]  address_offset NOT USED
 *
 */

void ei_rp2040_flush_buf(uint32_t address_offset)
{

    if (buf_pos < 255) {
        DEBUG_PRINT("flushing %d bytes\n", 255 - buf_pos);

        for (uint8_t i = buf_pos; i < 255; i++) {
            internal_buffer[buf_pos++] = 0x00;
        }
    }

    ei_rp2040_fs_write_sample_block(internal_buffer, _address_offset);
    _address_offset += buf_pos + 1;
    DEBUG_PRINT("global offset now %d\n", _address_offset);
    buf_pos = 0;
    _address_offset = 0;
}

/* Debug functions, delete later ----------------------------------------------------- */

void print_buf(const uint8_t *buf, size_t len)
{
#ifdef DEBUG
    ei_printf("size of data: %d \n", len);
    for (size_t i = 0; i < len; ++i) {
        ei_printf("%02x", buf[i]);
        if (i % 16 == 15)
            ei_printf("\n");
        else
            ei_printf(" ");
    }
#endif
}

void test_flash()
{

    uint8_t flash_target_contents[FLASH_PAGE_SIZE];
    uint8_t random_data[FLASH_PAGE_SIZE];
    
    for (int i = 0; i < FLASH_PAGE_SIZE; ++i)
        random_data[i] = rand() >> 16;

    ei_printf("Generated random data:\n");
    print_buf(random_data, FLASH_PAGE_SIZE);

    // Note that a whole number of sectors must be erased at a time.
    ei_printf("\nErasing target region...\n");
    uint64_t start_time = ei_read_timer_ms();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(rp2040_fs.sample_start_address - XIP_BASE, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
    ei_printf("\nDone. Flash erase time: %lld ms\n", ei_read_timer_ms() -start_time);

    ei_printf("\nRead back target region:\n");
    start_time = ei_read_timer_ms();    
    print_buf((const uint8_t *)rp2040_fs.sample_start_address, FLASH_PAGE_SIZE);
    ei_printf("\nFlash read time: %lld ms\n", ei_read_timer_ms() - start_time);

    ei_printf("\nProgramming target region...\n");
    start_time = ei_read_timer_ms();
    ints = save_and_disable_interrupts();
    flash_range_program(rp2040_fs.sample_start_address - XIP_BASE, random_data, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
    ei_printf("Done. Flash write time: %lld ms\n", ei_read_timer_ms() - start_time);

    ei_printf("\nDone. Checking content\n");
    ei_rp2040_copy_buf(
        (uint8_t *)flash_target_contents,
        (uint8_t *)rp2040_fs.sample_start_address,
        FLASH_PAGE_SIZE);

    bool mismatch = false;
    for (int i = 0; i < FLASH_PAGE_SIZE; ++i) {
        if (random_data[i] != flash_target_contents[i])
            mismatch = true;
    }
    if (mismatch)
        ei_printf("Programming failed!\n");
    else
        ei_printf("Programming successful!\n");

}