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

#ifndef EI_DEVICE_RP2040
#define EI_DEVICE_RP2040

/* Include ----------------------------------------------------------------- */
#include "ei_classifier_porting.h"
#include "ei_device_info_lib.h"
#include "pico/stdlib.h"

/** Baud rates */
#define DEFAULT_BAUD 115200
#define MAX_BAUD     115200

/** Number of sensors used */
#define EI_DEVICE_N_SENSORS            1
#define EI_MAX_FREQUENCIES             5

#ifdef DEBUG
#define DEBUG_PRINT ei_printf
#else
#define DEBUG_PRINT null_printf
#endif

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

/** C Callback types */
typedef int (*c_callback)(uint8_t out_buffer[32], size_t *out_size);
typedef bool (*c_callback_status)(void);
typedef bool (*c_callback_read_sample_buffer)(
    size_t begin,
    size_t length,
    void (*data_fn)(uint8_t *, size_t));

/**
 * @brief      Class description and implementation of device specific 
 * 			   characteristics
 */
class EiDeviceRP2040 : public EiDeviceInfo {
private:

    static const int standalone_sensor_num = 1;
    ei_device_sensor_t standalone_sensor_list[standalone_sensor_num];

    bool camera_present;

    bool network_present;
    bool network_connected;   
    //EiNetworkDevice *net;

    EiDeviceRP2040() = delete;
    std::string mac_address = "00:11:22:33:44:55:66";
    EiState state;
    
#if MULTI_FREQ_ENABLED == 1
    bool is_sampling;
#endif

public:

    EiDeviceRP2040(EiDeviceMemory* mem);
    ~EiDeviceRP2040();

    void delay_ms(uint32_t milliseconds);

    bool scan_networks(void);    
    bool get_wifi_connection_status(void);
    bool get_wifi_present_status();

    std::string get_mac_address(void);

    void (*sample_read_callback)(void);
    void init_device_id(void);
    void clear_config(void);
    bool is_camera_present(void);
    bool start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms) override;
    bool stop_sample_thread(void) override;
    void set_state(EiState state) override;
    EiState get_state(void);
    bool get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size) override;

    void set_default_data_output_baudrate(void) override;
    void set_max_data_output_baudrate(void) override;
    uint32_t get_data_output_baudrate(void) override;

#if MULTI_FREQ_ENABLED == 1
	bool start_multi_sample_thread(void (*sample_multi_read_cb)(uint8_t), float* multi_sample_interval_ms, uint8_t num_fusioned) override;
    bool get_is_sampling(void)
    {
        return is_sampling;
    }
#endif

};

/* Function prototypes ----------------------------------------------------- */
char ei_get_serial_byte(void);
bool ei_user_invoke_stop(void);
static bool read_sample_buffer(size_t begin, size_t length, void (*data_fn)(uint8_t *, size_t));
void null_printf(const char *format, ...);

#endif
