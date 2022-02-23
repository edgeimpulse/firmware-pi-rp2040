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

#include "ei_at_handlers.h"
#include "ei_device_raspberry_rp2040.h"
#include "ei_rp2040_fs_commands.h"

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/porting/lib/at_base64_lib.h"

#include "ei_at_command_set.h"
#include "ei_at_server.h"
#include "ei_config.h"
#include "ei_fusion.h"
#include "ei_run_impulse.h"

static inline bool check_args_num(const int &required, const int &received)
{
    if (received < required) {
        ei_printf("Too few arguments! Required: %d\n", required);
        return false;
    }

    return true;
}

static bool at_clear_config(void)
{
    ei_printf("Clearing config and restarting system...\n");
    ei_config_clear();

    return true;
}

static bool at_device_info(void)
{
    uint8_t id_buffer[32] = { 0 };
    size_t id_size;
    int r = ei_config_get_device_id(id_buffer, &id_size);
    if (r == EI_CONFIG_OK) {
        id_buffer[id_size] = 0;
        ei_printf("ID:         %s\n", id_buffer);
    }
    if (ei_config_get_context()->get_device_type == NULL) {
        return true;
    }
    r = ei_config_get_context()->get_device_type(id_buffer, &id_size);
    if (r == EI_CONFIG_OK) {
        id_buffer[id_size] = 0;
        ei_printf("Type:       %s\n", id_buffer);
    }
    ei_printf("AT Version: %s\n", AT_COMMAND_VERSION);

    ei_device_data_output_baudrate_t baudrate;
    r = EiDevice.get_data_output_baudrate(&baudrate);
    if (r == 0) {
        ei_printf("Data Transfer Baudrate: %s\r\n", baudrate.str);
    }
    else {
        ei_printf("Data Transfer Baudrate: UNKNOWN\r\n");
    }
    return true;
}

static bool at_get_sample_settings(void)
{
    char *label;
    float interval;
    uint32_t length;
    char *hmac_key;

    EI_CONFIG_ERROR r = ei_config_get_sample_settings(&label, &interval, &length, &hmac_key);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to retrieve sample settings (%d)\n", r);
        return true;
    }
    ei_printf("Label:     %s\n", label);
    ei_printf("Interval:  ");
    ei_printf_float((float)interval);
    ei_printf("ms.\n");
    ei_printf("Length:    %lu ms.\n", length);
    ei_printf("HMAC key:  %s\n", hmac_key);
    return true;
}

static bool at_set_sample_settings(const char **argv, const int argc)
{
    if (check_args_num(3, argc) == false) {
        return true;
    }

    float interval = atof(argv[1]);
    uint32_t length = (uint32_t)atoi(argv[2]);
    EI_CONFIG_ERROR r;

    if (argc == 3) {
        r = ei_config_set_sample_settings(argv[0], interval, length);
    }
    else if (argc >= 4) {
        //TODO: check if last rgv is not modified and fix ei_config_set_sample_settings declaration
        r = ei_config_set_sample_settings(argv[0], interval, length, (char *)argv[3]);
    }
    else {
        ei_printf("Incorrect number of arguments (at least 3)\n");
        return true;
    }

    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to persist sampling settings (%d)\n", r);
    }
    else {
        ei_printf("OK\n");
    }

    return true;
}

static bool at_get_upload_settings(void)
{
    char *api_key;
    char *host;
    char *path;

    EI_CONFIG_ERROR r = ei_config_get_upload_settings(&api_key, &host, &path);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to retrieve upload settings (%d)\n", r);
        return true;
    }
    ei_printf("Api Key:   %s\n", api_key);
    ei_printf("Host:      %s\n", host);
    ei_printf("Path:      %s\n", path);

    return true;
}

static bool at_set_upload_settings(const char **argv, const int argc)
{
    if (check_args_num(2, argc) == false) {
        return true;
    }

    EI_CONFIG_ERROR r = ei_config_set_upload_path_settings(argv[0], argv[1]);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to persist upload settings (%d)\n", r);
    }
    else {
        ei_printf("OK\n");
    }
    return true;
}

static bool at_set_upload_host(const char **argv, const int argc)
{
    if (check_args_num(1, argc) == false) {
        return true;
    }

    EI_CONFIG_ERROR r = ei_config_set_upload_host_settings(argv[0]);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to persist upload settings (%d)\n", r);
    }
    else {
        ei_printf("OK\n");
    }

    return true;
}

static bool at_get_mgmt_settings(void)
{
    char *mgmt_url;
    bool is_connected;
    char last_error[128] = { '\0' };

    EI_CONFIG_ERROR r = ei_config_get_mgmt_settings(&mgmt_url, &is_connected, last_error, 128);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to retrieve management settings (%d)\n", r);
        return true;
    }
    ei_printf("URL:        %s\n", mgmt_url);
    ei_printf("Connected:  %d\n", is_connected);
    ei_printf("Last error: %s\n", last_error);

    return true;
}

static bool at_set_mgmt_settings(const char **argv, const int argc)
{
    if (check_args_num(1, argc) == false) {
        return true;
    }

    EI_CONFIG_ERROR r = ei_config_set_mgmt_settings(argv[0]);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to persist management settings (%d)\n", r);
    }
    else {
        ei_printf("OK\n");
    }

    return true;
}

static bool at_list_sensors(void)
{
    const ei_device_sensor_t *list;
    size_t list_size;

    int r = EiDevice.get_sensor_list((const ei_device_sensor_t **)&list, &list_size);
    if (r != 0) {
        ei_printf("Failed to get sensor list (%d)\n", r);
        return true;
    }

    for (size_t ix = 0; ix < list_size; ix++) {
        ei_printf(
            "Name: %s, Max sample length: %hus, Frequencies: [",
            list[ix].name,
            list[ix].max_sample_length_s);
        for (size_t fx = 0; fx < EI_MAX_FREQUENCIES; fx++) {
            if (list[ix].frequencies[fx] != 0.0f) {
                if (fx != 0) {
                    ei_printf(", ");
                }
                ei_printf_float(list[ix].frequencies[fx]);
                ei_printf("Hz");
            }
        }
        ei_printf("]\n");
    }

    return true;
}

static void at_list_fusion_sensors()
{
    ei_built_sensor_fusion_list();
}

static void at_read_file_data(uint8_t *buffer, size_t size)
{
    base64_encode((const char *)buffer, size, ei_putchar);
}

static bool at_read_file(const char **argv, const int argc)
{
    if (check_args_num(1, argc) == false) {
        return true;
    }

    // second argument is optional, so check if we have it
    bool use_max_baudrate = false;
    if (argc >= 2 && argv[1][0] == 'y') {
        use_max_baudrate = true;
    }

    // setup data output baudrate
    if (use_max_baudrate) {
        // sleep a little to let the daemon attach on the new baud rate...
        ei_printf("OK\r\n");

        EiDevice.set_max_data_output_baudrate();
        EiDevice.delay_ms(100);
    }

    bool exists = ei_config_get_context()->read_file(argv[0], at_read_file_data);

    if (use_max_baudrate) {
        // lower baud rate
        ei_printf("\r\nOK\r\n");

        EiDevice.set_default_data_output_baudrate();

        // give some time to re-attach
        EiDevice.delay_ms(100);
    }

    if (!exists) {
        ei_printf("File '%s' does not exist\n", argv[0]);
    }
    else {
        ei_printf("\n");
    }

    return true;
}

static bool at_read_buffer(const char **argv, const int argc)
{
    if (check_args_num(2, argc) == false) {
        return true;
    }

    size_t start = (size_t)atoi(argv[0]);
    size_t length = (size_t)atoi(argv[1]);

    bool use_max_baudrate = false;
    if (argc >= 3 && argv[2][0] == 'y') {
        use_max_baudrate = true;
    }

    // setup data output baudrate
    if (use_max_baudrate) {

        // sleep a little to let the daemon attach on the new baud rate...
        ei_printf("OK\r\n");
        EiDevice.delay_ms(100);
        EiDevice.set_max_data_output_baudrate();
        EiDevice.delay_ms(100);
    }

    bool success = ei_config_get_context()->read_buffer(start, length, at_read_file_data);

    if (use_max_baudrate) {
        // lower baud rate
        ei_printf("\r\nOK\r\n");

        EiDevice.delay_ms(100);
        EiDevice.set_default_data_output_baudrate();

        // give some time to re-attach
        EiDevice.delay_ms(100);
    }

    if (!success) {
        ei_printf("Failed to read from buffer\n");
    }
    else {
        ei_printf("\n");
    }

    return true;
}

static bool at_sample_start(const char **argv, const int argc)
{
    if (check_args_num(1, argc) == false) {
        return true;
    }

    bool ret;
    const ei_device_sensor_t *list;
    size_t list_size;

    int r = EiDevice.get_sensor_list((const ei_device_sensor_t **)&list, &list_size);
    if (r != 0) {
        ei_printf("Failed to get sensor list (%d)\n", r);
        return true;
    }

    for (size_t ix = 0; ix < list_size; ix++) {
        if (strcmp(list[ix].name, argv[0]) == 0) {
            bool r = list[ix].start_sampling_cb();
            if (!r) {
                ei_printf("Failed to start sampling\n");
            }
            return true;
        }
    }

    if (ei_connect_fusion_list(argv[0], SENSOR_FORMAT)) {
        ret = ei_fusion_setup_data_sampling();
        if (!ret) {
            ei_printf("ERR: Failed to start sensor fusion sampling\n");
        }
        return true;
    }

    ei_printf("Failed to find sensor '%s' in the sensor list\n", argv[0]);

    return true;
}

static bool at_list_config(void)
{
    ei_printf("===== Device info =====\n");
    at_device_info();
    ei_printf("\n");
    ei_printf("===== Sensors ======\n");
    at_list_sensors();
    at_list_fusion_sensors();
    ei_printf("\n");
    ei_printf("===== Snapshot ======\n");
    ei_printf("Has snapshot:    0\n");
    ei_printf("\n");
    ei_printf("===== WIFI =====\n");
    ei_printf("SSID:      \n");
    ei_printf("Password:  \n");
    ei_printf("Security:  0\n");
    ei_printf("MAC:       00:00:00:00:00:00\n");
    ei_printf("Connected: 0\n");
    ei_printf("Present:   0\n");
    ei_printf("\n");
    ei_printf("===== Sampling parameters =====\n");
    at_get_sample_settings();
    ei_printf("\n");
    ei_printf("===== Upload settings =====\n");
    at_get_upload_settings();
    ei_printf("\n");
    ei_printf("===== Remote management =====\n");
    at_get_mgmt_settings();
    ei_printf("\n");

    return true;
}

static bool at_unlink_file(const char **argv, const int argc)
{
    ei_printf("\n");

    return true;
}

/* help function for list files which is not currently implemented */
void list_files_help_fn(void (*data_fn)(char *))
{
    char no_files[] = "No files";
    data_fn(no_files);
}

static bool at_read_raw(const char **argv, const int argc)
{

    if (check_args_num(2, argc) == false) {
        return true;
    }

    size_t start = (size_t)atoi(argv[0]);
    size_t length = (size_t)atoi(argv[1]);

    char buffer[32];

    int count = 0;
    int n_display_bytes = 16;

    for (start; start < length; start += n_display_bytes) {
        ei_rp2040_fs_read_sample_data(buffer, start, n_display_bytes);

        for (int i = 0; i < n_display_bytes; i++) {
            ei_printf("%02x", buffer[i]);
            if (i % 16 == 15)
                printf("\n");
            else
                printf(" ");
        }
    }

    return true;
}

static bool at_run_nn_normal(void)
{
    run_nn_normal();
    return true;
}

ATServer *ei_at_init(void)
{
    ATServer *at;

    /* Initialize configuration */
    static ei_config_ctx_t config_ctx = { 0 };
    config_ctx.get_device_id = EiDevice.get_id_function();
    config_ctx.get_device_type = EiDevice.get_type_function();
    config_ctx.wifi_connection_status = EiDevice.get_wifi_connection_status_function();
    config_ctx.wifi_present = EiDevice.get_wifi_present_status_function();
    config_ctx.load_config = &ei_rp2040_fs_load_config;
    config_ctx.save_config = &ei_rp2040_fs_save_config;
    config_ctx.list_files = list_files_help_fn;
    config_ctx.read_buffer = EiDevice.get_read_sample_buffer_function();

    EI_CONFIG_ERROR cr = ei_config_init(&config_ctx);

    if (cr != EI_CONFIG_OK) {
        ei_printf("Failed to initialize configuration (%d)\n", cr);
    }
    else {
        ei_printf("Loaded configuration\n");
    }

    at = ATServer::get_instance();

    at->register_command(AT_CONFIG, AT_CONFIG_HELP_TEXT, nullptr, at_list_config, nullptr, nullptr);
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
        AT_READFILE,
        AT_READFILE_HELP_TEXT,
        nullptr,
        nullptr,
        at_read_file,
        AT_READFILE_ARGS);
    at->register_command(
        AT_MGMTSETTINGS,
        AT_MGMTSETTINGS_HELP_TEXT,
        nullptr,
        at_get_mgmt_settings,
        at_set_mgmt_settings,
        AT_MGMTSETTINGS_ARGS);
    at->register_command(
        AT_CLEARCONFIG,
        AT_CLEARCONFIG_HELP_TEXT,
        at_clear_config,
        nullptr,
        nullptr,
        nullptr);
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
        nullptr,
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
        AT_READRAW,
        AT_READRAW_HELP_TEXT,
        nullptr,
        nullptr,
        at_read_raw,
        AT_READRAW_ARS);

    at->register_command(
        "RUNIMPULSE",
        "Run the impulse",
        at_run_nn_normal,
        nullptr,
        nullptr,
        nullptr);
    //at->register_command(RUNIMPULSECONT, "Run the impulse continuously", nullptr, nullptr, at_run_nn_continuous_normal, AT_RUNIMPULSECONT_ARGS);

    return at;
}