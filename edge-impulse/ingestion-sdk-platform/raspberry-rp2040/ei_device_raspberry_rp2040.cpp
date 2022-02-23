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
#include "ei_device_raspberry_rp2040.h"
#include "ei_rp2040_fs_commands.h"

#include <pico/stdlib.h>
#include <pico/unique_id.h>
#include <stdarg.h>
#include <stdio.h>

#include "ei_config_types.h"

/* Constants --------------------------------------------------------------- */

/** Max size for device id array */
#define DEVICE_ID_MAX_SIZE 32

extern ei_config_t *ei_config_get_config();

/** Sensors */
typedef enum
{
    LIGHT = 0

} used_sensors_t;

/** Data Output Baudrate */
const ei_device_data_output_baudrate_t ei_dev_max_data_output_baudrate = {
    "921600",
    MAX_BAUD,
};

const ei_device_data_output_baudrate_t ei_dev_default_data_output_baudrate = {
    "115200",
    DEFAULT_BAUD,
};

#define EDGE_STRINGIZE_(x) #x
#define EDGE_STRINGIZE(x)  EDGE_STRINGIZE_(x)

/** Global flag */
bool stop_sampling;

/** Device type */
static const char *ei_device_type = "RASPBERRY_PI_RP2040"; //EDGE_STRINGIZE(TARGET_NAME);

/** Device id array */
static char ei_device_id[DEVICE_ID_MAX_SIZE];

/** Device object, for this class only 1 object should exist */
EiDeviceRP2040 EiDevice;

/* Private function declarations ------------------------------------------- */
static int get_id_c(uint8_t out_buffer[32], size_t *out_size);
static int get_type_c(uint8_t out_buffer[32], size_t *out_size);
static bool get_wifi_connection_status_c(void);
static bool get_wifi_present_status_c(void);

/* Public functions -------------------------------------------------------- */

EiDeviceRP2040::EiDeviceRP2040(void)
{
    /* Clear frequency arrays */
    for (int i = 0; i < EI_DEVICE_N_SENSORS; i++) {
        for (int y = 0; y < EI_MAX_FREQUENCIES; y++) {
            sensors[i].frequencies[y] = 0.f;
        }
    }
}

/**
 * @brief      For the device ID, the FLASH chip ID is used.
 *             The chip ID string is copied to the out_buffer.
 *
 * @param      out_buffer  Destination array for id string
 * @param      out_size    Length of id string
 *
 * @return     0
 */
int EiDeviceRP2040::get_id(uint8_t out_buffer[32], size_t *out_size)
{
    return get_id_c(out_buffer, out_size);
}

/**
 * @brief      Gets the identifier pointer.
 *
 * @return     The identifier pointer.
 */
const char *EiDeviceRP2040::get_id_pointer(void)
{
    return (const char *)ei_device_id;
}

/**
 * @brief      Copy device type in out_buffer & update out_size
 *
 * @param      out_buffer  Destination array for device type string
 * @param      out_size    Length of string
 *
 * @return     -1 if device type string exceeds out_buffer
 */
int EiDeviceRP2040::get_type(uint8_t out_buffer[32], size_t *out_size)
{
    return get_type_c(out_buffer, out_size);
}

/**
 * @brief      Gets the type pointer.
 *
 * @return     The type pointer.
 */
const char *EiDeviceRP2040::get_type_pointer(void)
{
    return (const char *)ei_device_type;
}

/**
 * @brief      Device specific delay ms implementation
 *
 * @param[in]  milliseconds  The milliseconds
 */
void EiDeviceRP2040::delay_ms(uint32_t milliseconds)
{
    sleep_ms(milliseconds);
}

/**
 * @brief      No Wifi available for device.
 *
 * @return     Always return false
 */
bool EiDeviceRP2040::get_wifi_connection_status(void)
{
    return false;
}

/**
 * @brief      No Wifi available for device.
 *
 * @return     Always return false
 */
bool EiDeviceRP2040::get_wifi_present_status(void)
{
    return false;
}

/**
 * @brief      Create sensor list with sensor specs
 *             The studio and daemon require this list
 * @param      sensor_list       Place pointer to sensor list
 * @param      sensor_list_size  Write number of sensors here
 *
 * @return     False if all went ok
 */
bool EiDeviceRP2040::get_sensor_list(
    const ei_device_sensor_t **sensor_list,
    size_t *sensor_list_size)
{
    /* all sensors are handled by sensors fusion subsystem */
    *sensor_list = NULL;
    *sensor_list_size = 0;

    return false;
}

/**
 * @brief Get byte size of memory block
 *
 * @return uint32_t size in bytes
 */
uint32_t EiDeviceRP2040::filesys_get_block_size(void)
{
    return ei_rp2040_fs_get_block_size();
}

/**
 * @brief Get number of available blocks
 *
 * @return uint32_t
 */
uint32_t EiDeviceRP2040::filesys_get_n_available_sample_blocks(void)
{
    return ei_rp2040_fs_get_n_available_sample_blocks();
}

/**
 * @brief Setup timer or thread with given interval and call cb function each period
 * @param sample_read_cb
 * @param sample_interval_ms
 * @return true
 */
bool EiDeviceRP2040::start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms)
{
    stop_sampling = false;

    while (!stop_sampling) {

        sample_read_cb();
        ei_sleep(sample_interval_ms);
    }

    return true;
}

/**
 * @brief Stop timer of thread
 * @return true
 */
bool EiDeviceRP2040::stop_sample_thread(void)
{
    stop_sampling = true;
    return true;
}

int EiDeviceRP2040::get_data_output_baudrate(ei_device_data_output_baudrate_t *baudrate)
{
    memcpy(baudrate, &ei_dev_default_data_output_baudrate, sizeof(ei_device_data_output_baudrate_t));
    return 0;
}

/**
 * @brief      Get a C callback for the get_id method
 *
 * @return     Pointer to c get function
 */
c_callback EiDeviceRP2040::get_id_function(void)
{
    return &get_id_c;
}

/**
 * @brief      Get a C callback for the get_type method
 *
 * @return     Pointer to c get function
 */
c_callback EiDeviceRP2040::get_type_function(void)
{
    return &get_type_c;
}

/**
 * @brief      Get a C callback for the get_wifi_connection_status method
 *
 * @return     Pointer to c get function
 */
c_callback_status EiDeviceRP2040::get_wifi_connection_status_function(void)
{
    return &get_wifi_connection_status_c;
}

/**
 * @brief      Get a C callback for the wifi present method
 *
 * @return     The wifi present status function.
 */
c_callback_status EiDeviceRP2040::get_wifi_present_status_function(void)
{
    return &get_wifi_present_status_c;
}

/**
 * @brief      Get a C callback to the read sample buffer function
 *
 * @return     The read sample buffer function.
 */
c_callback_read_sample_buffer EiDeviceRP2040::get_read_sample_buffer_function(void)
{
    return &read_sample_buffer;
}

/**
 * @brief      null_printf, all DEBUG_PRINT set to it when not 
 *             specifying DEBUG build option
 * 
 *
 * @param[in]  format     Variable argument list
 */

void null_printf(const char *format, ...)
{
}

/**
 * @brief      Printf function uses vsnprintf and output using Arduino Serial
 *
 * @param[in]  format     Variable argument list
 */
void ei_printf(const char *format, ...)
{
    char print_buf[1024] = { 0 };
    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);
    if (r > 0) {
        printf(print_buf);
    }
}

/**
 * @brief      Write serial data with length to Serial output
 *
 * @param      data    The data
 * @param[in]  length  The length
 */
void ei_write_string(char *data, int length)
{
    printf(data);
}

/**
 * @brief      NOT IMPLEMENTED
 *
 * @return     NOT IMPLEMENTED
 */
int ei_get_serial()
{
    return 0;
}

/**
 * @brief      NOT IMPLEMENTED
 *
 * @return     NOT IMPLEMENTED
 */
int ei_get_serial_available(void)
{
    return 0;
}

/**
 * @brief      Checks for presense of b character to stop the inference
 *
 * @return     Returns true if b character is found, false otherwise
 */

bool ei_user_invoke_stop(void)
{
    bool stop_found = false;
    char data = getchar_timeout_us(1000 * 10);

    if (data == 'b') {
        stop_found = true;
    }

    return stop_found;
}

/**
 * @brief      Get next available byte
 *
 * @return     byte
 */
char ei_get_serial_byte(void)
{
    return getchar();
}

/**
 * @brief      Write character to serial
 *
 * @param      cChar     Char addr to write
 */
void ei_putc(char cChar)
{
    printf(&cChar);
}

/* Private functions ------------------------------------------------------- */

static int get_id_c(uint8_t out_buffer[32], size_t *out_size)
{
    uint id_len = 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1;
    char id_out[id_len];

    pico_get_unique_board_id_string(id_out, id_len);

    /* Setup device ID */
    snprintf(
        &ei_device_id[0],
        DEVICE_ID_MAX_SIZE,
        "%02X:%02X:%02X:%02X:%02X:%02X",
        id_out[0],
        id_out[1],
        id_out[3],
        id_out[4],
        id_out[5],
        id_out[6]);

    size_t length = strlen(ei_device_id);

    if (length < 32) {
        memcpy(out_buffer, ei_device_id, length);

        *out_size = length;
        return 0;
    }

    else {
        *out_size = 0;
        return -1;
    }
}

static int get_type_c(uint8_t out_buffer[32], size_t *out_size)
{
    size_t length = strlen(ei_device_type);

    if (length < 32) {
        memcpy(out_buffer, ei_device_type, length);

        *out_size = length;
        return 0;
    }

    else {
        *out_size = 0;
        return -1;
    }
}

static bool get_wifi_connection_status_c(void)
{
    return false;
}

static bool get_wifi_present_status_c(void)
{
    return false;
}

/**
 * @brief      Read samples from sample memory and send to data_fn function
 *
 * @param[in]  begin    Start address
 * @param[in]  length   Length of samples in bytes
 * @param[in]  data_fn  Callback function for sample data
 *
 * @return     false on flash read function
 */
static bool read_sample_buffer(size_t begin, size_t length, void (*data_fn)(uint8_t *, size_t))
{

    size_t pos = begin;
    size_t bytes_left = length;

    // we're encoding as base64 in AT+READFILE, so this needs to be divisable by 3
    uint8_t buffer[513];

    while (1) {
        size_t bytes_to_read = sizeof(buffer);
        if (bytes_to_read > bytes_left) {
            bytes_to_read = bytes_left;
        }
        if (bytes_to_read == 0) {
            return true;
        }
        int r = ei_rp2040_fs_read_sample_data(buffer, pos, bytes_to_read);
        if (r != 0) {
            return false;
        }
        data_fn(buffer, bytes_to_read);

        pos += bytes_to_read;
        bytes_left -= bytes_to_read;
    }

    return true;
}