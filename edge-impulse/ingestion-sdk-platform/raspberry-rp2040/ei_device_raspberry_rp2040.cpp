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
#include "ei_config_types.h"
#include "ei_device_raspberry_rp2040.h"
#include "ei_microphone.h"
#include "ei_flash_memory.h"

#include <pico/stdlib.h>
#include <pico/unique_id.h>
#include <stdarg.h>
#include <stdio.h>
#include <malloc.h>

#include <FreeRTOS.h>
#include <timers.h>

/* Constants --------------------------------------------------------------- */

#define EI_LED_OFF    gpio_put(LED_PIN, 0);
#define EI_LED_ON     gpio_put(LED_PIN, 1);

/** Global objects */
TimerHandle_t fusion_timer;
void (*sample_cb_ptr)(void);

/* Private function declarations ------------------------------------------- */
void vTimerCallback(TimerHandle_t xTimer);

/* Public functions -------------------------------------------------------- */

EiDeviceRP2040::EiDeviceRP2040(EiDeviceMemory* mem)
{
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    EiDeviceInfo::memory = mem;

    init_device_id();

    //load_config();

    device_type = "RASPBERRY_PI_RP2040";

    camera_present = false;

    // TODO
    //net = static_cast<EiDeviceRP2040*>(EiDeviceRP2040::get_network_device());
    // the absence of error on device init is success
    //network_present = !(net->init());

    // microphone is not handled by fusion system
    standalone_sensor_list[0].name = "Built-in microphone";
    standalone_sensor_list[0].start_sampling_cb = &ei_microphone_sample_start;
    standalone_sensor_list[0].max_sample_length_s = mem->get_available_sample_bytes() / (16000 * 2);
    standalone_sensor_list[0].frequencies[0] = 16000.0f;
    standalone_sensor_list[0].frequencies[1] = 8000.0f;
}


EiDeviceRP2040::~EiDeviceRP2040()
{

}

void EiDeviceRP2040::init_device_id(void)
{
    // Setup device ID
    char temp[18];

    uint id_len = 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1;
    char id_out[id_len];

    pico_get_unique_board_id_string(id_out, id_len);

    /* Setup device ID */
    sprintf(
        temp,
        "%02X:%02X:%02X:%02X:%02X:%02X",
        id_out[0],
        id_out[1],
        id_out[3],
        id_out[4],
        id_out[5],
        id_out[6]);

    device_id = std::string(temp);
}

EiDeviceInfo* EiDeviceInfo::get_device(void)
{
    //static EiDeviceRAM<256, 132> memory(sizeof(EiConfig));
    static EiFlashMemory memory(sizeof(EiConfig));
    static EiDeviceRP2040 dev(&memory);

    return &dev;
}

void EiDeviceRP2040::clear_config(void)
{
    EiDeviceInfo::clear_config();

    init_device_id();
    save_config();
}

/**
 * @brief      Device specific delay ms implementation
 *
 * @param[in]  milliseconds  The milliseconds
 */
void EiDeviceRP2040::delay_ms(uint32_t milliseconds)
{
    ei_sleep(milliseconds);
}

void EiDeviceRP2040::set_state(EiState state)
{
    switch(state) {
    case eiStateErasingFlash:
        EI_LED_ON;
        break;
    case eiStateSampling:
        EI_LED_ON;
        ei_sleep(50);
        EI_LED_OFF;
        ei_sleep(50);          
        break;
    case eiStateUploading:
        EI_LED_ON;    
        break;
    case eiStateFinished:
        for (int i = 0; i < 2; i++) {    
            EI_LED_ON;
            ei_sleep(100);
            EI_LED_OFF;
            ei_sleep(100);            
        }                                
        break;                
    default:
        EI_LED_OFF;
    }
}

/**
 * @brief      No Wifi available for device.
 *
 * @return     Always return false
 */
bool EiDeviceRP2040::scan_networks(void)
{
    return false;
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
    return network_present;
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
    *sensor_list = this->standalone_sensor_list;
    *sensor_list_size = this->standalone_sensor_num;
    return false;
}

uint32_t EiDeviceRP2040::get_data_output_baudrate(void)
{
    return MAX_BAUD;
}

void EiDeviceRP2040::set_default_data_output_baudrate(void)
{
    //not used
}

void EiDeviceRP2040::set_max_data_output_baudrate(void)
{
    //not used
}

/**
 * @brief Setup timer or thread with given interval and call cb function each period
 * @param sample_read_cb
 * @param sample_interval_ms
 * @return true
 */
bool EiDeviceRP2040::start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms)
{
    sample_cb_ptr = sample_read_cb;   
    fusion_timer = xTimerCreate(
                        "Fusion sampler",
                        (uint32_t)sample_interval_ms / portTICK_PERIOD_MS,
                        pdTRUE,
                        (void *)0,
                        vTimerCallback
                    );
    xTimerStart(fusion_timer, 0);

    return true;
}

/**
 * @brief Stop timer of thread
 * @return true
 */
bool EiDeviceRP2040::stop_sample_thread(void)
{
    xTimerStop(fusion_timer, 0);
    //if (xTimerStop(fusion_timer, 0) != pdPASS)
    //{
    //    ei_printf("Timer has not been stopped \n");
    //}
    return true;
}

/**
 * @brief      Checks for presense of b character to stop the inference
 *
 * @return     Returns true if b character is found, false otherwise
 */

bool ei_user_invoke_stop(void)
{
    bool stop_found = false;
	char ch = getchar_timeout_us(1000 * 10);

    if (ch == 'b') {
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
	char ch = getchar();
    // for some reason ESP32 only gets 10 (\n)and AT server has 13 (\r) as terminator character...
    if (ch == '\n') {
        ch = '\r';
    }
    
    return ch;
}

/**
 * @brief      Write character to serial
 *
 * @param      cChar     Char addr to write
 */
void ei_putc(char cChar)
{
    putchar(cChar);
}


char ei_getchar()
{
	char ch = getchar();
    // for some reason ESP32 only gets 10 (\n)and AT server has 13 (\r) as terminator character...

    if (ch == 255) {
        ch = 0;
    }

    if (ch == '\n') {
        ch = '\r';
    }
    
    return ch;

}

/* Private functions ------------------------------------------------------- */

void vTimerCallback(TimerHandle_t xTimer)
{
    sample_cb_ptr();
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

void printMemoryInfo(void)
{
    struct mallinfo mi;
    memset(&mi, 0, sizeof(struct mallinfo));
    mi = mallinfo();
    printf("Total non-mmapped bytes (arena):       %zu\n", mi.arena);
    printf("# of free chunks (ordblks):            %zu\n", mi.ordblks);
    printf("# of free fastbin blocks (smblks):     %zu\n", mi.smblks);
    printf("# of mapped regions (hblks):           %zu\n", mi.hblks);
    printf("Bytes in mapped regions (hblkhd):      %zu\n", mi.hblkhd);
    printf("Max. total allocated space (usmblks):  %zu\n", mi.usmblks);
    printf("Free bytes held in fastbins (fsmblks): %zu\n", mi.fsmblks);
    printf("Total allocated space (uordblks):      %zu\n", mi.uordblks);
    printf("Total free space (fordblks):           %zu\n", mi.fordblks);
    printf("Topmost releasable block (keepcost):   %zu\n", mi.keepcost);
}
