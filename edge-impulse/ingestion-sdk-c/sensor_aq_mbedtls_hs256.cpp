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

// #ifndef _EDGE_IMPULSE_SIGNING_MBEDTLS_HMAC_SHA256_H_
// #define _EDGE_IMPULSE_SIGNING_MBEDTLS_HMAC_SHA256_H_

/**
 * HMAC SHA256 implementation using Mbed TLS
 */

#include <string.h>
#include "firmware-sdk/sensor-aq/sensor_aq.h"
#include "mbedtls/md.h"
#include "mbedtls/sha256.h"
//#include "mbed_trace.h"
#include "ei_mbedtls_md.h"

//#ifdef MBEDTLS_MD_C

extern void ei_printf(const char *format, ...);

typedef struct {
    mbedtls_md_context_t md_ctx;
    char hmac_key[33];
} sensor_aq_mbedtls_hs256_ctx_t;

static int sensor_aq_mbedtls_hs256_init(sensor_aq_signing_ctx_t *aq_ctx) {
    sensor_aq_mbedtls_hs256_ctx_t *hs_ctx = (sensor_aq_mbedtls_hs256_ctx_t*)aq_ctx->ctx;

    int err;

    //mbedtls_md_init(&hs_ctx->md_ctx);
    memset(&hs_ctx->md_ctx, 0, sizeof( mbedtls_md_context_t ) );

    err = ei_mbedtls_md_setup(&hs_ctx->md_ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256) , 1); //use hmac
    if (err != 0) {
        return err;
    }

    return ei_mbedtls_md_hmac_starts(&hs_ctx->md_ctx, (const unsigned char *)hs_ctx->hmac_key, strlen(hs_ctx->hmac_key));
}

static int sensor_aq_mbedtls_hs256_update(sensor_aq_signing_ctx_t *aq_ctx, const uint8_t *buffer, size_t buffer_size) {
    sensor_aq_mbedtls_hs256_ctx_t *hs_ctx = (sensor_aq_mbedtls_hs256_ctx_t*)aq_ctx->ctx;

    return ei_mbedtls_md_hmac_update(&hs_ctx->md_ctx, buffer, buffer_size);
}

static int sensor_aq_mbedtls_hs256_finish(sensor_aq_signing_ctx_t *aq_ctx, uint8_t *buffer) {
    sensor_aq_mbedtls_hs256_ctx_t *hs_ctx = (sensor_aq_mbedtls_hs256_ctx_t*)aq_ctx->ctx;

    int ret = ei_mbedtls_md_hmac_finish(&hs_ctx->md_ctx, buffer);
    mbedtls_md_free(&hs_ctx->md_ctx);
    return ret;
}

/**
 * Construct a new signing context for HMAC SHA256 using Mbed TLS
 *
 * @param aq_ctx An empty signing context (can declare it without arguments)
 * @param hs_ctx An empty sensor_aq_mbedtls_hs256_ctx_t context (can declare it on the stack without arguments)
 * @param hmac_key The secret key - **NOTE: this is limited to 32 characters, the rest will be truncated**
 */
void sensor_aq_init_mbedtls_hs256_context(sensor_aq_signing_ctx_t *aq_ctx, sensor_aq_mbedtls_hs256_ctx_t *hs_ctx, const char *hmac_key) {
    memcpy(hs_ctx->hmac_key, hmac_key, 32);
    hs_ctx->hmac_key[32] = 0;

    if (strlen(hmac_key) > 32) {
        ei_printf("!!! sensor_aq_init_mbedtls_hs256_context, HMAC key is longer than 32 characters - will be truncated !!!\n");
    }

    aq_ctx->alg = "HS256"; // JWS algorithm
    aq_ctx->signature_length = 32;
    aq_ctx->ctx = (void*)hs_ctx;
    aq_ctx->init = &sensor_aq_mbedtls_hs256_init;
    aq_ctx->set_protected = NULL;
    aq_ctx->update = &sensor_aq_mbedtls_hs256_update;
    aq_ctx->finish = &sensor_aq_mbedtls_hs256_finish;
}

// #else

// #error "sensor_aq_mbedtls_hs256 loaded but Mbed TLS was not found, or MBEDTLS_MD_C was disabled"

// #endif // MBEDTLS_MD_C

// #endif // _EDGE_IMPULSE_SIGNING_MBEDTLS_HMAC_SHA256_H_
