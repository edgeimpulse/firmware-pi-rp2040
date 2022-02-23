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

#ifndef EI_DEVICE_RP2040
#define EI_DEVICE_RP2040

/* Include ----------------------------------------------------------------- */
#include "ei_classifier_porting.h"
#include "ei_device_info_lib.h"

/** Baud rates */
#define DEFAULT_BAUD 115200
#define MAX_BAUD     921600

/** Number of sensors used */
#define EI_DEVICE_N_SENSORS            1
#define EI_MAX_FREQUENCIES             5
#define EI_DEVICE_N_RESOLUTIONS        2
#define EI_DEVICE_N_RESIZE_RESOLUTIONS 2

#ifdef DEBUG
#define DEBUG_PRINT ei_printf
#else
#define DEBUG_PRINT null_printf
#endif

/** C Callback types */
typedef int (*c_callback)(uint8_t out_buffer[32], size_t *out_size);
typedef bool (*c_callback_status)(void);
typedef bool (*c_callback_read_sample_buffer)(
    size_t begin,
    size_t length,
    void (*data_fn)(uint8_t *, size_t));

typedef struct {
    size_t width;
    size_t height;
} ei_device_resize_resolutions_t;

/**
 * @brief      Class description and implementation of device specific 
 * 			   characteristics
 */
class EiDeviceRP2040 : public EiDeviceInfo {
private:
    ei_device_sensor_t sensors[EI_DEVICE_N_SENSORS];

public:
    EiDeviceRP2040(void);

    int get_id(uint8_t out_buffer[32], size_t *out_size);
    const char *get_id_pointer(void);
    int get_type(uint8_t out_buffer[32], size_t *out_size);
    void delay_ms(uint32_t milliseconds);
    const char *get_type_pointer(void);
    bool get_wifi_connection_status(void);
    bool get_wifi_present_status();
    bool get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size);
    int get_data_output_baudrate(ei_device_data_output_baudrate_t *baudrate);
    uint32_t filesys_get_block_size(void);
    uint32_t filesys_get_n_available_sample_blocks(void);
    bool start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms);
    bool stop_sample_thread(void);

    c_callback get_id_function(void);
    c_callback get_type_function(void);
    c_callback_status get_wifi_connection_status_function(void);
    c_callback_status get_wifi_present_status_function(void);
    c_callback_read_sample_buffer get_read_sample_buffer_function(void);
};

/* Function prototypes ----------------------------------------------------- */
char ei_get_serial_byte(void);
bool ei_user_invoke_stop(void);
static bool read_sample_buffer(size_t begin, size_t length, void (*data_fn)(uint8_t *, size_t));
void null_printf(const char *format, ...);

/* Reference to object for external usage ---------------------------------- */
extern EiDeviceRP2040 EiDevice;

#endif
