/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>

#include "ei_sampler.h"
#include "ei_config_types.h"
#include "ei_device_raspberry_rp2040.h"

#include "sensor_aq_mbedtls_hs256.h"


/** @todo Should be called by function pointer */
extern bool ei_inertial_sample_start(sampler_callback callback, float sample_interval_ms);
extern int ei_inertial_read_data(void);

extern void ei_printf(const char *format, ...);
extern void ei_printf_float(float f);
extern ei_config_t *ei_config_get_config();

/* Forward declarations ---------------------------------------------------- */
static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM *);
static int ei_seek(EI_SENSOR_AQ_STREAM *, long int offset, int origin);
static time_t ei_time(time_t *t);
static void finish_and_upload(char *filename, uint32_t sample_length_ms);
static bool sample_data_callback(const void *sample_buf, uint32_t byteLenght);
static bool create_header(sensor_aq_payload_info *payload);

/* Private variables ------------------------------------------------------- */
static uint32_t samples_required;
static uint32_t samples_required_increase;
static uint32_t current_sample;
static uint32_t sample_buffer_size;
static uint32_t headerOffset = 0;
static uint8_t write_word_buf[4];
static int write_addr = 0;
EI_SENSOR_AQ_STREAM stream;

static unsigned char ei_mic_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_mic_hs_ctx;
static sensor_aq_ctx ei_mic_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &ei_write,
    &ei_seek,
    &ei_time,
};

/**
 * @brief      Write sample data to FLASH
 * @details    Write size is always 4 bytes to keep alignment
 *
 * @param[in]  buffer     The buffer
 * @param[in]  size       The size
 * @param[in]  count      The count
 * @param      EI_SENSOR_AQ_STREAM file pointer (not used)
 *
 * @return     number of bytes handled
 */
static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM *)
{
    EiDeviceInfo* dev = EiDeviceInfo::get_device();
    EiDeviceMemory* mem = dev->get_memory();
    for (size_t i = 0; i < count; i++) {
        write_word_buf[write_addr & 0x3] = *((char *)buffer + i);

        if ((++write_addr & 0x03) == 0x00) {

            mem->write_sample_data(write_word_buf, (write_addr - 4) + headerOffset, 4);
        }
    }
    return count;
}

/**
 * @brief      File handle seed function. Not used
 */
static int ei_seek(EI_SENSOR_AQ_STREAM *, long int offset, int origin)
{
    return 0;
}

/**
 * @brief      File handle time function. Not used
 */
static time_t ei_time(time_t *t)
{
    time_t cur_time = 4564867;
    if (t) *(t) = cur_time;
    return cur_time;
}

/**
 * @brief      Write out remaining data in word buffer to FLASH.
 *             And append CBOR end character.
 */
static void ei_write_last_data(void)
{
    EiDeviceInfo* dev = EiDeviceInfo::get_device();
    EiDeviceMemory* mem = dev->get_memory();  

    uint8_t fill = ((uint8_t)write_addr & 0x03);
    uint8_t insert_end_address = 0;

    if (fill != 0x00) {
        for (uint8_t i = fill; i < 4; i++) {
            write_word_buf[i] = 0xFF;
        }
        mem->write_sample_data(write_word_buf, (write_addr & ~0x03) + headerOffset, 4);
        insert_end_address = 4;
    }

    /* Write appending word for end character */
    for (uint8_t i = 0; i < 4; i++) {
        write_word_buf[i] = 0xFF;
    }
    mem->write_sample_data(write_word_buf, (write_addr & ~0x03) + headerOffset + insert_end_address, 4);
}

/**
 * @brief      Setup and start sampling, write CBOR header to flash
 *
 * @param      v_ptr_payload  sensor_aq_payload_info pointer hidden as void
 * @param[in]  sample_size    Number of bytes for 1 sample (include all axis)
 *
 * @return     true if successful
 */
bool ei_sampler_start_sampling(void *v_ptr_payload, starter_callback ei_sample_start, uint32_t sample_size)
{
    EiDeviceInfo* dev = EiDeviceInfo::get_device();
    EiDeviceMemory* mem = dev->get_memory();  

    sensor_aq_payload_info *payload = (sensor_aq_payload_info *)v_ptr_payload;

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: %.5f ms.\n", dev->get_sample_interval_ms());
    ei_printf("\tLength: %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("\tName: %s\n", dev->get_sample_label().c_str());
    ei_printf("\tHMAC Key: %s\n", dev->get_sample_hmac_key().c_str());
    ei_printf("\tFile name: %s\n", dev->get_sample_label().c_str());

    samples_required = (uint32_t)((float)dev->get_sample_length_ms());
    samples_required_increase = (uint32_t)dev->get_sample_interval_ms();

    sample_buffer_size = ((uint32_t)((dev->get_sample_length_ms()) / dev->get_sample_interval_ms()) * sample_size) * 2;
    current_sample = 0;

    ei_printf("Samples req: %d\n", samples_required);

    // Minimum delay of 2000 ms for daemon
    if (((sample_buffer_size / mem->block_size) + 1) * mem->block_erase_time < 2000) {
        ei_printf("Starting in %d ms... (or until all flash was erased)\n", 2000);
        ei_sleep(2000);
        DEBUG_PRINT("Done waiting\n");
    } 
    else {
        ei_printf("Starting in %d ms... (or until all flash was erased)\n",
                    ((sample_buffer_size / mem->block_size) + 1) * mem->block_erase_time);
    }

    dev->set_state(eiStateErasingFlash);
    if(mem->erase_sample_data(0, sample_buffer_size) != (sample_buffer_size)) {
        return false;
    }
    DEBUG_PRINT("Done erasing\n");

    if (create_header(payload) == false) {
        return false;
    }
    DEBUG_PRINT("Done header\n");

    if(ei_sample_start(&sample_data_callback, dev->get_sample_interval_ms()) == false) {
        return false;
    }
    
	ei_printf("Sampling...\n");    

    while(current_sample < samples_required) {
        dev->set_state(eiStateSampling);
    };

    ei_write_last_data();
    write_addr++;   

    uint8_t final_byte[] = {0xff};
    int ctx_err = ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, final_byte, 1);
    if (ctx_err != 0) {
        return ctx_err;
    }

    // finish the signing
    DEBUG_PRINT("Finish the signing \n");
    ctx_err = ei_mic_ctx.signature_ctx->finish(ei_mic_ctx.signature_ctx, ei_mic_ctx.hash_buffer.buffer);

    // load the first page in flash...
    uint8_t *page_buffer = (uint8_t *)ei_malloc(mem->block_size);
    if (!page_buffer) {
        ei_printf("Failed to allocate a page buffer to write the hash\n");
        return false;
    }

    mem->flush_data();
    ei_sleep(100);

    //DEBUG_PRINT("Reading first page \n");
    uint32_t j = mem->read_sample_data(page_buffer, 0, mem->block_size);
    if (j != mem->block_size) {
        ei_printf("Failed to read first page (%d)\n", j);
        ei_free(page_buffer);
        return false;
    }

    // update the hash
    uint8_t *hash = ei_mic_ctx.hash_buffer.buffer;
    // we have allocated twice as much for this data (because we also want to be able to represent
    // in hex) thus only loop over the first half of the bytes as the signature_ctx has written to
    // those
    for (size_t hash_ix = 0; hash_ix < (ei_mic_ctx.hash_buffer.size / 2); hash_ix++) {
        // this might seem convoluted, but snprintf() with %02x is not always supported e.g. by
        // newlib-nano we encode as hex... first ASCII char encodes top 4 bytes
        uint8_t first = (hash[hash_ix] >> 4) & 0xf;
        // second encodes lower 4 bytes
        uint8_t second = hash[hash_ix] & 0xf;

        // if 0..9 -> '0' (48) + value, if >10, then use 'a' (97) - 10 + value
        char first_c = first >= 10 ? 87 + first : 48 + first;
        char second_c = second >= 10 ? 87 + second : 48 + second;

        page_buffer[ei_mic_ctx.signature_index + (hash_ix * 2) + 0] = first_c;
        page_buffer[ei_mic_ctx.signature_index + (hash_ix * 2) + 1] = second_c;
    }

    j = mem->erase_sample_data(0, mem->block_size);
    if (j != mem->block_size) {
        ei_printf("Failed to erase first page (%d)\n", j);
        ei_free(page_buffer);
        return false;
    }

    j = mem->write_sample_data(page_buffer, 0, mem->block_size);

    ei_free(page_buffer);

    if (j != mem->block_size) {
        ei_printf("Failed to write first page with updated hash (%d)\n", j);
        return false;
    }

    mem->flush_data();
    ei_sleep(100);

    finish_and_upload((char*)dev->get_sample_label().c_str(), dev->get_sample_length_ms());

    return true;
}

/**
 * @brief      Create and write the CBOR header to FLASH
 *
 * @param      payload  The payload
 *
 * @return     True on success
 */
static bool create_header(sensor_aq_payload_info *payload)
{
    EiDeviceInfo* dev = EiDeviceInfo::get_device();
    EiDeviceMemory* mem = dev->get_memory();   
    sensor_aq_init_mbedtls_hs256_context(&ei_mic_signing_ctx, &ei_mic_hs_ctx, dev->get_sample_hmac_key().c_str());

    int tr = sensor_aq_init(&ei_mic_ctx, payload, NULL, true);

    if (tr != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", tr);
        return false;
    }
    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_mic_ctx.cbor_buffer.len - 1; ix >= 0; ix--) {
        if (((uint8_t *)ei_mic_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    // Write to blockdevice
    tr = mem->write_sample_data((uint8_t*)ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);
    DEBUG_PRINT("Try to write %d bytes\r\n", end_of_header_ix);

    if (tr != end_of_header_ix) {
        ei_printf("Failed to write to header blockdevice (%d)\n", tr);
        return false;
    }

    ei_mic_ctx.stream = &stream;

    headerOffset = end_of_header_ix;
    write_addr = 0;

    return true;
}

/**
 * @brief      Sampling is finished, signal no uploading file
 *
 * @param      filename          The filename
 * @param[in]  sample_length_ms  The sample length milliseconds
 */
static void finish_and_upload(char *filename, uint32_t sample_length_ms)
{
    ei_printf("Done sampling, total bytes collected: %u\n", samples_required);
    ei_printf("[1/1] Uploading file to Edge Impulse...\n");

    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=%lu, to=%lu.\n", 0, write_addr + headerOffset);

    ei_printf("[1/1] Uploading file to Edge Impulse OK (took %d ms.)\n", 200);

    ei_printf("OK\n");
}

/**
 * @brief      Write samples to FLASH in CBOR format
 *
 * @param[in]  sample_buf  The sample buffer
 * @param[in]  byteLenght  The byte lenght
 *
 * @return     true if all required samples are received. Caller should stop sampling,
 */
static bool sample_data_callback(const void *sample_buf, uint32_t byteLenght)
{
    sensor_aq_add_data(&ei_mic_ctx, (float *)sample_buf, byteLenght / sizeof(float));
    current_sample += samples_required_increase;    

    if(current_sample > samples_required) {
        return true;
    } 
    else {
        return false;
    }
}