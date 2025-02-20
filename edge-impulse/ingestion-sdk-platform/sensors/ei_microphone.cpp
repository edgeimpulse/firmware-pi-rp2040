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
#include "ei_microphone.h"

#include "ei_device_raspberry_rp2040.h"

#include "FreeRTOS.h"
#include "task.h"

#include "ei_config_types.h"
#include "sensor_aq_mbedtls_hs256.h"
#include "firmware-sdk/sensor-aq/sensor_aq_none.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"

#include "pico/stdlib.h"

typedef struct {
    int16_t *buffers[2];
    uint8_t buf_select;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

/* Dummy functions for sensor_aq_ctx type */
static size_t ei_write(const void*, size_t size, size_t count, EI_SENSOR_AQ_STREAM*)
{
    return count;
}

static int ei_seek(EI_SENSOR_AQ_STREAM*, long int offset, int origin)
{
    return 0;
}

/* Private variables ------------------------------------------------------- */
static signed short *sampleBuffer;
static bool is_uploaded = false;
static int record_status = 0;
static uint32_t headerOffset;
static uint32_t samples_required;
static uint32_t current_sample;
static uint32_t audio_sampling_frequency = 16000;

volatile int samples_read = 0;
static inference_t inference;

static unsigned char ei_mic_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_mic_hs_ctx;
static sensor_aq_ctx ei_mic_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &ei_write,
    &ei_seek,
    NULL,
};

/* Private functions ------------------------------------------------------- */

static void audio_write_callback(uint32_t n_bytes)
{
    EiDeviceRP2040* dev = static_cast<EiDeviceRP2040*>(EiDeviceInfo::get_device());
    EiDeviceMemory* mem = dev->get_memory();

    mem->write_sample_data((uint8_t*)sampleBuffer, headerOffset + current_sample, n_bytes);
    ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, (uint8_t*)sampleBuffer, n_bytes);

    current_sample += n_bytes;
    if(current_sample >= (samples_required << 1)) {
        PDM.end();
        record_status = false;
        free(sampleBuffer);
    }
}

static void audio_inference_callback(uint32_t n_bytes)
{

    for(int i = 0; i < n_bytes>>1; i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];

        if(inference.buf_count >= inference.n_samples) {
            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

void on_pdm_samples_ready()
{
    // callback from library when all the samples in the library
    // internal sample buffer are ready for reading
    samples_read = PDM.available();

    PDM.read(sampleBuffer, samples_read);

    // see if are recording samples for ingestion
    // or inference and send them their way
    if (record_status == 1) {
        audio_write_callback(samples_read);
    }
    else if (record_status == 2) {
        audio_inference_callback(samples_read);
    }
    else {
        DEBUG_PRINT("not init\n");
    }
}

// AT+SAMPLESTART=Built-in microphone
static void finish_and_upload(const char *filename, uint32_t sample_length_ms) {

    ei_printf("Done sampling, total bytes collected: %u\n", current_sample);

    ei_printf("[1/1] Uploading file to Edge Impulse...\n");
    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=%lu, to=%lu.\n", 0, current_sample + headerOffset);
    ei_printf("[1/1] Uploading file to Edge Impulse OK (took %d ms.)\n", 0);

    is_uploaded = true;

    ei_printf("OK\n");
}

static int insert_ref(char *buffer, int hdrLength)
{
    #define EXTRA_BYTES(a)  ((a & 0x3) ? 4 - (a & 0x3) : (a & 0x03))
    const char *ref = "Ref-BINARY-i16";
    int addLength = 0;
    int padding = EXTRA_BYTES(hdrLength);

    buffer[addLength++] = 0x60 + 14 + padding;
    for(int i = 0; i < strlen(ref); i++) {
        buffer[addLength++] = *(ref + i);
    }
    for(int i = 0; i < padding; i++) {
        buffer[addLength++] = ' ';
    }

    buffer[addLength++] = 0xFF;

    return addLength;
}

static bool create_header(void)
{
    EiDeviceRP2040* dev = static_cast<EiDeviceRP2040*>(EiDeviceInfo::get_device());
    EiDeviceMemory* mem = dev->get_memory();
    sensor_aq_init_mbedtls_hs256_context(&ei_mic_signing_ctx, &ei_mic_hs_ctx, dev->get_sample_hmac_key().c_str());

    sensor_aq_payload_info payload = {
        dev->get_device_id().c_str(),
        dev->get_device_type().c_str(),
        dev->get_sample_interval_ms(),
        { { "audio", "wav" } }
    };

    int tr = sensor_aq_init(&ei_mic_ctx, &payload, NULL, true);

    if (tr != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", tr);
        return false;
    }

    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_mic_ctx.cbor_buffer.len - 1; ix >= 0; ix--) {
        if (((uint8_t*)ei_mic_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    int ref_size = insert_ref(((char*)ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), end_of_header_ix);

    // and update the signature
    tr = ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, ((uint8_t*)ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), ref_size);
    if (tr != 0) {
        ei_printf("Failed to update signature from header (%d)\n", tr);
        return false;
    }

    end_of_header_ix += ref_size;

    // Write to blockdevice
    tr = mem->write_sample_data((uint8_t*)ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);

    if ((size_t)tr != end_of_header_ix) {
        ei_printf("Failed to write to header blockdevice (%d)\n", tr);
        return false;
    }

    headerOffset = end_of_header_ix;

    return true;
}


/* Public functions -------------------------------------------------------- */

bool ei_microphone_record(uint32_t sample_length_ms, uint32_t start_delay_ms, bool print_start_messages)
{

    EiDeviceRP2040* dev = static_cast<EiDeviceRP2040*>(EiDeviceInfo::get_device());
    EiDeviceMemory* mem = dev->get_memory();

    start_delay_ms = start_delay_ms < 2000 ? 2000 : start_delay_ms;

    if (print_start_messages) {
        ei_printf("Starting in %lu ms... (or until all flash was erased)\n", start_delay_ms);
    }

    dev->set_state(eiStateErasingFlash);
    if(mem->erase_sample_data(0, (samples_required << 1) + 4096) < 0) {
        return false;
    }

    ei_sleep(start_delay_ms);

    create_header();

    record_status = 1;

    if (print_start_messages) {
        ei_printf("Sampling...\n");
    }

    return true;
}

bool ei_microphone_inference_start(uint32_t n_samples, float interval_ms)
{

    inference.buffers[0] = (int16_t *)ei_malloc(n_samples * sizeof(int16_t));

    if(inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (int16_t *)ei_malloc(n_samples * sizeof(int16_t));

    if(inference.buffers[1] == NULL) {
        ei_free(inference.buffers[0]);
        return false;
    }

    // theoretically we needed to do this
    // however it causes issues
    // TO-DO: investigate what sound is actually like during the inference
    //sampleBuffer = (int16_t *)ei_malloc((n_samples / 100) * sizeof(int16_t));

    sampleBuffer = (int16_t *)ei_malloc(512 * sizeof(int16_t));

    if(sampleBuffer == NULL) {
        ei_free(inference.buffers[0]);
        ei_free(inference.buffers[1]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    // Calculate sample rate from sample interval
    audio_sampling_frequency = (uint32_t)(1000.f / interval_ms);

    // set callback that is called when all the samples in the library
    // internal sample buffer are ready for reading
    PDM.onReceive(on_pdm_samples_ready);

     // start capturing data from the PDM microphone
    if (!PDM.begin(1, audio_sampling_frequency)) {
        printf("PDM microphone start failed!\n");
        return false;
    }

    ei_sleep(100);

    record_status = 2;

    return true;

}

/**
 * @brief      Wait for a full buffer
 *
 * @return     In case of an buffer overrun return false
 */
bool ei_microphone_inference_record(void)
{
    bool ret = true;

    if (inference.buf_ready == 1) {
        ei_printf(
            "Error sample buffer overrun. Decrease the number of slices per model window "
            "(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)\n");
        ret = false;
    }

    while (inference.buf_ready == 0) {
        ei_sleep(1);
    }

    inference.buf_ready = 0;

    return ret;
}

bool ei_microphone_inference_is_recording(void)
{
    return inference.buf_ready == 0;
}

/**
 * @brief      Reset buffer counters for non-continuous inferencing
 */
void ei_microphone_inference_reset_buffers(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
}

/**
 * Get raw audio signal data
 */
int ei_microphone_inference_get_data(size_t offset, size_t length, float *out_ptr)
{
    return ei::numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
}


bool ei_microphone_inference_end(void)
{
    record_status = 0;
    PDM.end();
    ei_sleep(100);
    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);
    ei_free(sampleBuffer);
    return 0;
}

/**
 * Sample raw data
 */
bool ei_microphone_sample_start(void)
{

    EiDeviceRP2040* dev = static_cast<EiDeviceRP2040*>(EiDeviceInfo::get_device());
    EiDeviceMemory* mem = dev->get_memory();

    int sample_length_blocks;
    int ret;

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: %.5f ms.\n", dev->get_sample_interval_ms());
    ei_printf("\tLength: %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("\tName: %s\n", dev->get_sample_label().c_str());
    ei_printf("\tHMAC Key: %s\n", dev->get_sample_hmac_key().c_str());
    ei_printf("\tFile name: %s\n", dev->get_sample_label().c_str());

    samples_required = (uint32_t)((dev->get_sample_length_ms()) / dev->get_sample_interval_ms());

    // Round to even number of samples for word align flash write
    if(samples_required & 1) {
        samples_required++;
    }

    current_sample = 0;

    is_uploaded = false;

    sampleBuffer = (int16_t *)malloc(SAMPLE_BUFFER_SIZE * 2);

    if (sampleBuffer == NULL) {
        return false;
    }

    // Calculate sample rate from sample interval
    audio_sampling_frequency = (uint32_t)(1000.0f / dev->get_sample_interval_ms());

    // set callback that is called when all the samples in the library
    // internal sample buffer are ready for reading
    PDM.onReceive(on_pdm_samples_ready);

     // start capturing data from the PDM microphone
    if (!PDM.begin(1, audio_sampling_frequency)) {
        printf("PDM microphone start failed!\n");
        return false;
    }

    bool r = ei_microphone_record(dev->get_sample_length_ms(), (((samples_required <<1)/ mem->block_size) * mem->block_erase_time), true);
    if (!r) {
        return r;
    }

    while(record_status) {
        dev->set_state(eiStateSampling);
    };

    int ctx_err = ei_mic_ctx.signature_ctx->finish(ei_mic_ctx.signature_ctx, ei_mic_ctx.hash_buffer.buffer);
    if (ctx_err != 0) {
        ei_printf("Failed to finish signature (%d)\n", ctx_err);
        return false;
    }

    DEBUG_PRINT("Creating page buffer \n");

    // load the first page in flash...
    uint8_t *page_buffer = (uint8_t*)ei_malloc(mem->block_size);
    if (!page_buffer) {
        ei_printf("Failed to allocate a page buffer to write the hash\n");
        return false;
    }

    DEBUG_PRINT("Flushing \n");
    mem->flush_data();
    ei_sleep(100);

    DEBUG_PRINT("Reading first page \n");

    ret = mem->read_sample_data(page_buffer, 0, mem->block_size);
    if ((uint32_t)ret != mem->block_size) {
        ei_printf("Failed to read first page\n");
        ei_free(page_buffer);
        return false;
    }

    // update the hash
    uint8_t *hash = ei_mic_ctx.hash_buffer.buffer;
    // we have allocated twice as much for this data (because we also want to be able to represent in hex)
    // thus only loop over the first half of the bytes as the signature_ctx has written to those
    for (size_t hash_ix = 0; hash_ix < ei_mic_ctx.hash_buffer.size / 2; hash_ix++) {
        // this might seem convoluted, but snprintf() with %02x is not always supported e.g. by newlib-nano
        // we encode as hex... first ASCII char encodes top 4 bytes
        uint8_t first = (hash[hash_ix] >> 4) & 0xf;
        // second encodes lower 4 bytes
        uint8_t second = hash[hash_ix] & 0xf;

        // if 0..9 -> '0' (48) + value, if >10, then use 'a' (97) - 10 + value
        char first_c = first >= 10 ? 87 + first : 48 + first;
        char second_c = second >= 10 ? 87 + second : 48 + second;

        page_buffer[ei_mic_ctx.signature_index + (hash_ix * 2) + 0] = first_c;
        page_buffer[ei_mic_ctx.signature_index + (hash_ix * 2) + 1] = second_c;
    }

    ret = mem->erase_sample_data(0, mem->block_size);
    if ((uint32_t)ret != mem->block_size) {
        ei_printf("Failed to erase first page (read %d, requested %lu)\n", ret, mem->block_size);
        ei_free(page_buffer);
        return false;
    }

    ret = mem->write_sample_data(page_buffer, 0, mem->block_size);
    ei_free(page_buffer);

    if ((uint32_t)ret != mem->block_size) {
        ei_printf("Failed to write first page with updated hash (read %d, requested %lu)\n", ret, mem->block_size);
        return false;
    }

    mem->flush_data();
    ei_sleep(100);

    finish_and_upload(dev->get_sample_label().c_str(), dev->get_sample_length_ms());

    return true;
}

//AT+READRAW=0,3200
//AT+SAMPLESETTINGS=test,0.0625,100
//AT+SAMPLESTART=Built-in microphone