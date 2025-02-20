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

#include <string>
#include "ei_at_handlers.h"
#include "ei_device_raspberry_rp2040.h"
#include "ei_device_lib.h"
#include "ei_device_interface.h"
#include "at_base64_lib.h"

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include "ei_at_command_set.h"
#include "ei_at_server.h"
#include "ei_fusion.h"
#include "ei_image_lib.h"

#include "ei_run_impulse.h"

#include "model-parameters/model_metadata.h"

#include "pico/bootrom.h"

using namespace std;

EiDeviceRP2040* dev = static_cast<EiDeviceRP2040*>(EiDeviceRP2040::get_device());
EiDeviceMemory* mem = dev->get_memory();

#define TRANSFER_BUF_LEN 128

// Helper functions

static void at_error_not_implemented()
{
    ei_printf("Command not implemented\r\n");
}

static inline bool check_args_num(const int &required, const int &received)
{
    if (received < required) {
        ei_printf("Too few arguments! Required: %d\n", required);
        return false;
    }

    return true;
}

// AT Server functions

static bool at_bootmode(void)
{
    ei_printf("ENTERING BOOTLOADER\n");
    ei_sleep(100);

    reset_usb_boot(1u, 0);
    return true;
}

bool at_get_device_id(void)
{
    ei_printf("%s\n", dev->get_device_id().c_str());

    return true;
}

bool at_set_device_id(const char **argv, const int argc)
{
    check_args_num(1, argc);

    dev->set_device_id(argv[0]);

    ei_printf("OK\n");

    return true;
}

bool at_get_upload_host(void)
{
    ei_printf("%s\n", dev->get_upload_host().c_str());

    return true;
}

bool at_set_upload_host(const char **argv, const int argc)
{
    if (argc < 1) {
        ei_printf("Missing argument!\n");
        return true;
    }

    dev->set_upload_host(argv[0]);

    ei_printf("OK\n");

    return true;
}

bool at_get_upload_settings(void)
{
    ei_printf("Api Key:   %s\n", dev->get_upload_api_key().c_str());
    ei_printf("Host:      %s\n", dev->get_upload_host().c_str());
    ei_printf("Path:      %s\n", dev->get_upload_path().c_str());

    return true;
}

bool at_set_upload_settings(const char **argv, const int argc)
{
    if (argc < 2) {
        ei_printf("Missing argument! Required: " AT_UPLOADSETTINGS_ARGS "\n");
        return true;
    }

    //TODO: can we set these values to ""?
    dev->set_upload_api_key(argv[0]);
    dev->set_upload_path(argv[1]);

    ei_printf("OK\n");

    return true;
}

bool at_get_mgmt_url(void)
{
    ei_printf("%s\n", dev->get_management_url().c_str());

    return true;
}

bool at_set_mgmt_url(const char **argv, const int argc)
{
    if (argc < 1) {
        ei_printf("Missing argument!\n");
        return true;
    }

    dev->set_management_url(argv[0]);

    ei_printf("OK\n");

    return true;
}

bool at_get_sample_settings(void)
{
    ei_printf("Label:     %s\n", dev->get_sample_label().c_str());
    ei_printf("Interval:  %.2f ms.\n", dev->get_sample_interval_ms());
    ei_printf("Length:    %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("HMAC key:  %s\n", dev->get_sample_hmac_key().c_str());

    return true;
}

bool at_set_sample_settings(const char **argv, const int argc)
{
    if (argc < 3) {
        ei_printf("Missing argument! Required: " AT_SAMPLESETTINGS_ARGS "\n");
        return true;
    }

    dev->set_sample_label(argv[0]);

    //TODO: sanity check and/or exception handling
    string interval_ms_str(argv[1]);
    dev->set_sample_interval_ms(stof(interval_ms_str));

    //TODO: sanity check and/or exception handling
    string sample_length_str(argv[2]);
    dev->set_sample_length_ms(stoi(sample_length_str));

    if (argc >= 4) {
        dev->set_sample_hmac_key(argv[3]);
    }

    ei_printf("OK\n");

    return true;
}

bool at_clear_config(void)
{
    dev->clear_config();
    dev->init_device_id();

    return true;
}

bool at_unlink_file(const char **argv, const int argc)
{
    ei_printf("\n");

    return true;
}

bool at_read_buffer(const char **argv, const int argc)
{
    if (argc < 2) {
        ei_printf("Missing argument! Required: " AT_READBUFFER_ARGS "\n");
        return true;
    }
    bool success = true;

    size_t start = (size_t)atoi(argv[0]);
    size_t length = (size_t)atoi(argv[1]);

    bool use_max_baudrate = false;
    if (argc >= 3 && argv[2][0] == 'y') {
        use_max_baudrate = true;
    }

    if (use_max_baudrate) {
        ei_printf("OK\r\n");
        ei_sleep(100);
        dev->set_max_data_output_baudrate();
        ei_sleep(100);
    }

    success = read_encode_send_sample_buffer(start, length);

    if (use_max_baudrate) {
        ei_printf("\r\nOK\r\n");
        ei_sleep(100);
        dev->set_default_data_output_baudrate();
        ei_sleep(100);
    }

    if (!success) {
        ei_printf("Failed to read from buffer\n");
    }
    else {
        ei_printf("\n");
    }

    return true;
}

bool at_read_raw(const char **argv, const int argc)
{

    if (check_args_num(2, argc) == false) {
        return true;
    }

    size_t start = (size_t)atoi(argv[0]);
    size_t length = (size_t)atoi(argv[1]);

    uint8_t buffer[32];

    int count = 0;
    int n_display_bytes = 16;

    for (; start < length; start += n_display_bytes) {
        mem->read_sample_data(buffer, start, n_display_bytes);

        for (int i = 0; i < n_display_bytes; i++) {
            ei_printf("%02x", buffer[i]);
            if (i % 16 == 15)
                ei_printf("\n");
            else
                ei_printf(" ");
        }
    }
    ei_printf("\n");

    return true;
}

bool at_sample_start(const char **argv, const int argc)
{
    if (argc < 1) {
        ei_printf("Missing sensor name!\n");
        return true;
    }

    const ei_device_sensor_t *sensor_list;
    size_t sensor_list_size;

    dev->get_sensor_list((const ei_device_sensor_t **)&sensor_list, &sensor_list_size);

    for (size_t ix = 0; ix < sensor_list_size; ix++) {
        if (strcmp(sensor_list[ix].name, argv[0]) == 0) {
            if (!sensor_list[ix].start_sampling_cb()) {
                ei_printf("ERR: Failed to start sampling\n");
            }
            return true;
        }
    }

    if (ei_connect_fusion_list(argv[0], SENSOR_FORMAT)) {
        if (!ei_fusion_setup_data_sampling()) {
            ei_printf("ERR: Failed to start sensor fusion sampling\n");
        }
    }
    else {
        ei_printf("ERR: Failed to find sensor '%s' in the sensor list\n", argv[0]);
    }

    return true;
}

bool at_run_impulse(void)
{
    ei_start_impulse(false, false);

    return true;
}

bool at_run_impulse_static_data(const char **argv, const int argc)
{

    if (check_args_num(2, argc) == false) {
        return false;
    }

    bool debug = (argv[0][0] == 'y');
    size_t length = (size_t)atoi(argv[1]);

    bool res = run_impulse_static_data(debug, length, TRANSFER_BUF_LEN);

    return res;
}

bool at_run_impulse_debug(const char **argv, const int argc)
{
    bool use_max_uart_speed = false;
    if (argc > 0 && argv[0][0] == 'y') {
        use_max_uart_speed = true;
    }

    ei_start_impulse(false, true, use_max_uart_speed);

    return true;
}

bool at_run_impulse_cont(void)
{
    ei_start_impulse(true, false);

    return true;
}

bool at_stop_impulse(void)
{
    ei_stop_impulse();

    return true;
}

bool at_get_config(void)
{
    dev->load_config();

    const ei_device_sensor_t *sensor_list;
    size_t sensor_list_size;

    dev->get_sensor_list((const ei_device_sensor_t **)&sensor_list, &sensor_list_size);

    ei_printf("===== Device info =====\n");
    ei_printf("ID:         %s\n", dev->get_device_id().c_str());
    ei_printf("Type:       %s\n", dev->get_device_type().c_str());
    ei_printf("AT Version: " AT_COMMAND_VERSION "\n");
    ei_printf("Data Transfer Baudrate: %lu\n", dev->get_data_output_baudrate());
    ei_printf("\n");
    ei_printf("===== Sensors ======\n");
    //TODO: move it to Sensor Manager
    for (size_t ix = 0; ix < sensor_list_size; ix++) {
        ei_printf(
            "Name: %s, Max sample length: %hus, Frequencies: [",
            sensor_list[ix].name,
            sensor_list[ix].max_sample_length_s);
        for (size_t fx = 0; fx < EI_MAX_FREQUENCIES; fx++) {
            if (sensor_list[ix].frequencies[fx] != 0.0f) {
                if (fx != 0) {
                    ei_printf(", ");
                }
                ei_printf("%.2fHz", sensor_list[ix].frequencies[fx]);
            }
        }
        ei_printf("]\n");
    }
    ei_built_sensor_fusion_list();
    ei_printf("\n");
    ei_printf("===== Snapshot ======\n");
    ei_printf("\n");
    ei_printf("===== Inference ======\n");
    ei_printf("Sensor:           %d\r\n", EI_CLASSIFIER_SENSOR);
#if EI_CLASSIFIER_OBJECT_DETECTION
    #if EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_FOMO
        const char *model_type = "constrained_object_detection";
    #else
        const char *model_type = "object_detection";
    #endif
#else
    const char *model_type = "classification";
#endif
    ei_printf("Model type:       %s\r\n", model_type);
    ei_printf("\n");
    ei_printf("===== WIFI =====\n");
    ei_printf("SSID:      \n");
    ei_printf("Password:  \n");
    ei_printf("Security:  0\n");
    ei_printf("MAC:       %s\n", dev->get_device_id().c_str());
    ei_printf("Connected: 0\n");
    ei_printf("Present:   0\n");
    ei_printf("\n");
    ei_printf("===== Sampling parameters =====\n");
    ei_printf("Label:     %s\n", dev->get_sample_label().c_str());
    ei_printf("Interval:  %.2f ms.\n", dev->get_sample_interval_ms());
    ei_printf("Length:    %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("HMAC key:  %s\n", dev->get_sample_hmac_key().c_str());
    ei_printf("\n");
    ei_printf("===== Upload settings =====\n");
    ei_printf("Api Key:   %s\n", dev->get_upload_api_key().c_str());
    ei_printf("Host:      %s\n", dev->get_upload_host().c_str());
    ei_printf("Path:      %s\n", dev->get_upload_path().c_str());
    ei_printf("\n");
    ei_printf("===== Remote management =====\n");
    ei_printf("URL:        %s\n", dev->get_management_url().c_str());
    ei_printf("Connected:  0\n");
    ei_printf("Last error: \n");
    ei_printf("\n");

    return true;
}

ATServer *ei_at_init(EiDeviceRP2040 *device)
{
    ATServer *at;
    dev = device;

    at = ATServer::get_instance();

    at->register_command(AT_CONFIG, AT_CONFIG_HELP_TEXT, nullptr, at_get_config, nullptr, nullptr);
    at->register_command(
        AT_SAMPLESTART,
        AT_SAMPLESTART_HELP_TEXT,
        nullptr,
        nullptr,
        at_sample_start,
        AT_SAMPLESTART_ARGS);
    at->register_command(
        AT_READBUFFER,
        AT_READBUFFER_HELP_TEXT,
        nullptr,
        nullptr,
        at_read_buffer,
        AT_READBUFFER_ARGS);
    at->register_command(
        AT_MGMTSETTINGS,
        AT_MGMTSETTINGS_HELP_TEXT,
        nullptr,
        at_get_mgmt_url,
        at_set_mgmt_url,
        AT_MGMTSETTINGS_ARGS);
    at->register_command(
        AT_CLEARCONFIG,
        AT_CLEARCONFIG_HELP_TEXT,
        at_clear_config,
        nullptr,
        nullptr,
        nullptr);
    at->register_command(
        AT_DEVICEID,
        AT_DEVICEID_HELP_TEXT,
        nullptr,
        at_get_device_id,
        at_set_device_id,
        AT_DEVICEID_ARGS);
    at->register_command(
        AT_SAMPLESETTINGS,
        AT_SAMPLESETTINGS_HELP_TEXT,
        nullptr,
        at_get_sample_settings,
        at_set_sample_settings,
        AT_SAMPLESETTINGS_ARGS);
    at->register_command(
        AT_UPLOADSETTINGS,
        AT_UPLOADSETTINGS_HELP_TEXT,
        nullptr,
        at_get_upload_settings,
        at_set_upload_settings,
        AT_UPLOADSETTINGS_ARGS);
    at->register_command(
        AT_UPLOADHOST,
        AT_UPLOADHOST_HELP_TEXT,
        nullptr,
        at_get_upload_host,
        at_set_upload_host,
        AT_UPLOADHOST_ARGS);
    at->register_command(
        AT_UNLINKFILE,
        AT_UNLINKFILE_HELP_TEXT,
        nullptr,
        nullptr,
        at_unlink_file,
        AT_UNLINKFILE_ARGS);
    at->register_command(
        AT_RUNIMPULSE,
        AT_RUNIMPULSE_HELP_TEXT,
        at_run_impulse,
        nullptr,
        nullptr,
        nullptr);
    at->register_command(
        AT_RUNIMPULSEDEBUG,
        AT_RUNIMPULSEDEBUG_HELP_TEXT,
        nullptr,
        nullptr,
        at_run_impulse_debug,
        AT_RUNIMPULSEDEBUG_ARGS);
    at->register_command(
        AT_RUNIMPULSECONT,
        AT_RUNIMPULSECONT_HELP_TEXT,
        at_run_impulse_cont,
        nullptr,
        nullptr,
        nullptr);
    at->register_command(
        AT_RUNIMPULSESTATIC,
        AT_RUNIMPULSESTATIC_HELP_TEXT,
        nullptr,
        nullptr,
        at_run_impulse_static_data,
        AT_RUNIMPULSESTATIC_ARGS);
    at->register_command(
        AT_READRAW,
        AT_READRAW_HELP_TEXT,
        nullptr,
        nullptr,
        at_read_raw,
        AT_READRAW_ARS);
    at->register_command(
        AT_BOOTMODE,
        AT_BOOTMODE_HELP_TEXT,
        at_bootmode,
        nullptr,
        nullptr,
        nullptr);
    return at;
}