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
#include "ei_flash_memory.h"

#include "ei_device_raspberry_rp2040.h"

#include "pico/mutex.h"

/** 32-bit align write buffer size */
#define WORD_ALIGN(a) ((a & 0x3) ? (a & ~0x3) + 0x4 : a)
/** Align addres to given sector size */
#define SECTOR_ALIGN(a, sec_size) ((a & (sec_size - 1)) ? (a & ~(sec_size - 1)) + sec_size : a)

extern char __flash_binary_end;

//auto_init_mutex(xSDKMutex);

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

void test_flash(uint32_t test_start_address)
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
    flash_range_erase(test_start_address - XIP_BASE, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
    ei_printf("\nDone. Flash erase time: %lld ms\n", ei_read_timer_ms() - start_time);

    ei_printf("\nRead back target region:\n");
    start_time = ei_read_timer_ms();    
    print_buf((const uint8_t *)test_start_address, FLASH_PAGE_SIZE);
    ei_printf("\nFlash read time: %lld ms\n", ei_read_timer_ms() - start_time);

    ei_printf("\nProgramming target region...\n");
    start_time = ei_read_timer_ms();
    ints = save_and_disable_interrupts();
    flash_range_program(test_start_address - XIP_BASE, random_data, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
    ei_printf("Done. Flash write time: %lld ms\n", ei_read_timer_ms() - start_time);

    ei_printf("\nDone. Checking content\n");
    memcpy(flash_target_contents, (void *)(test_start_address), FLASH_PAGE_SIZE);

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

/**
 * @brief      Writes a whole block(page) of data to flash
 *
 * @param[in]  sample_buffer   Pointer to sample data block
 * @param[in]  address_offset  Offset in memory from the beginning of sample space in flash
 *
 * @return     ei_rp2040_ret_t enum
 */

int write_sample_block(const void *sample_buffer, uint32_t address)
{

    DEBUG_PRINT("WRITE BLOCK\n");
    print_buf((uint8_t *)sample_buffer, 256);
    DEBUG_PRINT("\n");

    DEBUG_PRINT("READ EMPTY\n");
    print_buf((uint8_t *)(address + XIP_BASE), 256);
    DEBUG_PRINT("\n");

    DEBUG_PRINT(
        "target addr: %08x %d\n",
        address,
        address);

    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(
        address,
        (const uint8_t *)sample_buffer,
        FLASH_PAGE_SIZE);
    restore_interrupts(ints);

    DEBUG_PRINT("READ BACK\n");
    print_buf((uint8_t *)(address + XIP_BASE), 256);
    DEBUG_PRINT("\n");

    return 0;
}

uint32_t EiFlashMemory::read_data(uint8_t *data, uint32_t address, uint32_t num_bytes)
{

    DEBUG_PRINT("READ\n");

    DEBUG_PRINT(
        "address: %08x %d\n",
        address + XIP_BASE,
        address + XIP_BASE);

    DEBUG_PRINT(
        "offset: %08x %d\n",
        address,
        address);

    memcpy(data, (void *)(address + XIP_BASE), num_bytes);
    print_buf((uint8_t *)(address + XIP_BASE), num_bytes);

    return num_bytes;
}

uint32_t EiFlashMemory::write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes)
{

    uint32_t num_blocks = ceil(
        float(num_bytes) / float(FLASH_PAGE_SIZE)); //< 1 ? 1 : n_samples / rp2040_fs.page_size;

    for (uint32_t i = 0; i < num_blocks; i++) {

        if (num_bytes < 256) {

            for (uint32_t i = 0; i < num_bytes; i++) {

                if (buf_pos < 256) {
                    internal_buffer[buf_pos++] = (data)[i];
                }
                else {
                    write_sample_block(internal_buffer, address + address_offset);
                    address_offset += buf_pos;
                    DEBUG_PRINT("%d\n", address_offset);
                    buf_pos = 0;
                    internal_buffer[buf_pos++] = (data)[i];
                }
            }
        }

        else {

            for (uint32_t j = i * FLASH_PAGE_SIZE;
                 j < i * FLASH_PAGE_SIZE + FLASH_PAGE_SIZE;
                 j++) {

                if (j > num_bytes - 1)
                    break;

                if (buf_pos < 256) {
                    internal_buffer[buf_pos++] = (data)[j];
                }
                else {
                    write_sample_block(internal_buffer, address + address_offset);
                    address_offset += buf_pos;
                    DEBUG_PRINT("address_offset %d\n", address_offset - XIP_BASE);
                    buf_pos = 0;
                    internal_buffer[buf_pos++] = (data)[j];
                }
            }
        }
    }

    return num_bytes;
}

uint32_t EiFlashMemory::erase_data(uint32_t address, uint32_t num_bytes)
{
    /**
     * Address can point to the middle of sector, but num_bytes may be reaching
     *  part of the last sector
     * +-------+-------+-------+-------+
     * |       |       |       |       |
     * +-------+-------+-------+-------+
     *     ^
     *     address
     *     <-----num_bytes--------->
     */
    DEBUG_PRINT("ERASE\n");

    DEBUG_PRINT(
        "address: %08x %d\n",
        address - XIP_BASE,
        address - XIP_BASE);

    DEBUG_PRINT(
        "offset: %08x %d\n",
        address,
        address);

    //mutex_enter_blocking(&xSDKMutex);
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(address, SECTOR_ALIGN(num_bytes, FLASH_SECTOR_SIZE));
    restore_interrupts(ints);
    //mutex_exit(&xSDKMutex);
    DEBUG_PRINT("erase success\n");
    return num_bytes;
}

EiFlashMemory::EiFlashMemory(uint32_t config_size):
    EiDeviceMemory(config_size, RP2040_FS_BLOCK_ERASE_TIME_MS, 2048 * 1024, FLASH_SECTOR_SIZE)
{
    // For RP2040 we need to find out where user program binary ends in flash
    // we write data (config and samples) to the next sector

    uintptr_t flash_end = (uintptr_t)&__flash_binary_end;
    
    config_address = SECTOR_ALIGN(flash_end, FLASH_SECTOR_SIZE);
    used_blocks = ((config_address - XIP_BASE) / FLASH_SECTOR_SIZE) + 
    ((config_size < block_size) ? 1 : ceil(float(config_size) / block_size));

    address_offset = 0;
}

bool EiFlashMemory::save_config(const uint8_t *config, uint32_t config_size)
{

    uint32_t used_bytes = used_blocks * block_size;
    if (erase_data(config_address - XIP_BASE, config_size) != config_size) {
        return false;
    }

    if (write_data(config, config_address - XIP_BASE, config_size) != config_size) {
        return false;
    }
    offset = (used_blocks - 1) * block_size;
    flush_data();
    return true;
}

bool EiFlashMemory::load_config(uint8_t *config, uint32_t config_size)
{
    DEBUG_PRINT("config_address %d\n", config_address);
    DEBUG_PRINT("used_blocks %d\n", used_blocks);
    if (read_data(config, config_address - XIP_BASE, config_size) != config_size) {
        return false;
    }

    return true;
}

uint32_t EiFlashMemory::read_sample_data(uint8_t *sample_data, uint32_t address, uint32_t sample_data_size)
{
    uint32_t offset = used_blocks * block_size;
    return read_data(sample_data, address + offset, sample_data_size);
}

uint32_t EiFlashMemory::write_sample_data(const uint8_t *sample_data, uint32_t address, uint32_t sample_data_size)
{
    // this is class varaible
    offset = used_blocks * block_size;
    return write_data(sample_data, offset, sample_data_size);
}

uint32_t EiFlashMemory::erase_sample_data(uint32_t address, uint32_t num_bytes)
{
    uint32_t offset = used_blocks * block_size;
    return erase_data(offset, num_bytes);
}

uint32_t EiFlashMemory::flush_data(void) {

    DEBUG_PRINT("offset in device memory: %d\n", offset);

    if (buf_pos < 255) {
        DEBUG_PRINT("flushing %d bytes\n", 255 - buf_pos);

        for (uint8_t i = buf_pos; i < 255; i++) {
            internal_buffer[buf_pos++] = 0x00;
        }
    }

    write_sample_block(internal_buffer, offset + address_offset);
    address_offset += buf_pos + 1;
    DEBUG_PRINT("global offset now %d\n", offset + address_offset);
    buf_pos = 0;
    address_offset = 0;
    return 0;
}