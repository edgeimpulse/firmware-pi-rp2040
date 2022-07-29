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

#ifndef EI_FLASH_MEMORY_H
#define EI_FLASH_MEMORY_H

/* Include ----------------------------------------------------------------- */
#include "firmware-sdk/ei_device_memory.h"

#include <cmath>
#include <cstdint>
#include <cstring>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define RP2040_FS_BLOCK_ERASE_TIME_MS 40

class EiFlashMemory : public EiDeviceMemory {
protected:
    uint32_t used_blocks;
    uint32_t read_data(uint8_t *data, uint32_t address, uint32_t num_bytes);
    uint32_t write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes);
    uint32_t write_config_data(const uint8_t *config, uint32_t config_size);
    uint32_t erase_data(uint32_t address, uint32_t num_bytes);

private:
    uint8_t internal_buffer[256];
    uint32_t buf_pos;
    uint32_t address_offset;
    uint32_t offset;
    uint32_t config_address;

public:
    EiFlashMemory(uint32_t config_size);
    bool save_config(const uint8_t *config, uint32_t config_size);
    bool load_config(uint8_t *config, uint32_t config_size);
    uint32_t read_sample_data(uint8_t *sample_data, uint32_t address, uint32_t sample_data_size);
    uint32_t write_sample_data(const uint8_t *sample_data, uint32_t address, uint32_t sample_data_size);
    uint32_t erase_sample_data(uint32_t address, uint32_t num_bytes);
    uint32_t flush_data(void);
};

#endif /* EI_FLASH_MEMORY_H */