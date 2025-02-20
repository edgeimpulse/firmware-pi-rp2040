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
#if MULTI_FREQ_ENABLED == 1
void vTimerMultiFreqCallback(TimerHandle_t xTimer);
#endif

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
#if MULTI_FREQ_ENABLED == 1
    this->is_sampling = true;
    this->fusioning = 1;
#endif

    sample_cb_ptr = sample_read_cb;   
    fusion_timer = xTimerCreate(
                        "Fusion sampler",
                        pdMS_TO_TICKS(sample_interval_ms),
                        pdTRUE,
                        (void *)0,
                        vTimerCallback
                    );

    if (xTimerStart(fusion_timer, 0) == pdFAIL){
        ei_printf("Unable to create timer\n");
        return false;
    }

    return true;
}

#if MULTI_FREQ_ENABLED == 1
/**
 * @brief 
 * 
 * @param sample_multi_read_cb 
 * @param multi_sample_interval_ms 
 * @param num_fusioned 
 * @return true 
 * @return false 
 */
bool EiDeviceRP2040::start_multi_sample_thread(void (*sample_multi_read_cb)(uint8_t), float* multi_sample_interval_ms, uint8_t num_fusioned)
{
    uint8_t i;
    uint8_t flag = 0;

    this->is_sampling = true;
    this->sample_multi_read_callback = sample_multi_read_cb;
    this->fusioning = num_fusioned;
    this->multi_sample_interval.clear();

    for (i = 0; i < num_fusioned; i++){
        this->multi_sample_interval.push_back(1.f/multi_sample_interval_ms[i]*1000.f);
    }

    /* to improve, we consider just a 2 sensors case for now */
    this->sample_interval = ei_fusion_calc_multi_gcd(this->multi_sample_interval.data(), this->fusioning);

    /* force first reading */
    for (i = 0; i < this->fusioning; i++){
            flag |= (1<<i);
    }
    this->sample_multi_read_callback(flag);

    this->actual_timer = 0;

    fusion_timer = xTimerCreate(
                    "Fusion sampler",
                    pdMS_TO_TICKS(this->sample_interval),
                    pdTRUE,
                    (void *)0,
                    vTimerMultiFreqCallback
                );
    
    if (xTimerStart(fusion_timer, 0) == pdFAIL){
        ei_printf("Unable to create timer\n");
        return false;
    }

    return true;
}
#endif

/**
 * @brief Stop timer of thread
 * @return true
 */
bool EiDeviceRP2040::stop_sample_thread(void)
{
    ei_printf("STOP\n");
#if MULTI_FREQ_ENABLED == 1
    this->is_sampling = false;
#endif
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
#if MULTI_FREQ_ENABLED == 1
void vTimerMultiFreqCallback(TimerHandle_t xTimer)
{
    EiDeviceRP2040* dev = static_cast<EiDeviceRP2040*>(EiDeviceRP2040::get_device());
    uint8_t flag = 0;
    uint8_t i = 0;
    
    if (dev->get_is_sampling() == true){
        dev->actual_timer += dev->get_sample_interval();  /* update actual time */

        for (i = 0; i < dev->get_fusioning(); i++){
            if (((uint32_t)(dev->actual_timer % (uint32_t)dev->multi_sample_interval.at(i))) == 0) {   /* check if period of sensor is a multiple of actual time*/
                flag |= (1<<i);                                                                     /* if so, time to sample it! */
            }
        }

        if (dev->sample_multi_read_callback != nullptr){
            dev->sample_multi_read_callback(flag);        
        }    
    }
}
#endif

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
