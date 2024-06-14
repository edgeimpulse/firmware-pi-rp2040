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
#include "tusb.h"
#include <stdio.h>
#include "pico/stdlib.h"

#include <stdio.h>

#include "ei_device_raspberry_rp2040.h"

#include "ei_at_handlers.h"
#include "ei_classifier_porting.h"
#include "ei_run_impulse.h"

#include "ei_dht11sensor.h"
#include "ei_inertialsensor.h"
#include "ei_motionsensor.h"
#include "ei_analogsensor.h"
#include "ei_ultrasonicsensor.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

EiDeviceInfo *EiDevInfo = dynamic_cast<EiDeviceInfo *>(EiDeviceRP2040::get_device());
static ATServer *at;

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file. */
extern "C" {
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
}
/* Public functions -------------------------------------------------------- */

void ei_init(void)
{
    EiDeviceRP2040* dev = static_cast<EiDeviceRP2040*>(EiDeviceRP2040::get_device());

    ei_printf(
        "Hello from Edge Impulse Device SDK.\r\n"
        "Compiled on %s %s\r\n",
        __DATE__,
        __TIME__);

    /* Setup the inertial sensor */
    if (ei_inertial_sensor_init() == false) {
        ei_printf("Inertial sensor communication error occurred\r\n");
    }

    /* Setup the motion sensor */
    if (ei_motion_sensor_init() == false) {
        ei_printf("Motion sensor communication error occurred\r\n");
    }

    /* Setup the temp&humidity sensor */
    if (ei_dht11_sensor_init() == false) {
        ei_printf("DHT11 initialization failed\r\n");
    }

    /* Setup the ultrasonic sensor */
    if (ei_ultrasonic_sensor_init() == false) {
        ei_printf("Ultrasonic ranger initialization failed\r\n");
    }

    /* Setup the light sensor */
    if (ei_analog_sensor_init() == false) {
        ei_printf("ADC sensor initialization failed\r\n");
    }

#ifdef DEBUG
    //test_flash();
#endif

    // cannot init device id before main() started on RP2040
    dev->init_device_id();
    dev->load_config();
    dev->set_state(eiStateFinished);

    at = ei_at_init(dev);
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    at->print_prompt();
}

void ei_main(void *pvParameters)
{
    /* Initialize Edge Impulse sensors and commands */
    ei_init();

    while(true) {
        /* handle command coming from uart */
        char data = ei_get_serial_byte();

        while (data != 0xFF) {
           at->handle(data);
           data = ei_get_serial_byte();
        }
    }
}

int main(void)
{
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    stdio_init_all();
    while (!tud_cdc_connected()) {
        tight_loop_contents();
    }

    gpio_put(LED_PIN, 0);

    /* Start the two tasks as described in the comments at the top of this file. */
    xTaskCreate(ei_main,		/* The function that implements the task. */
                "ei_main", 		/* The text name assigned to the task - for debug only as it is not used by the kernel. */
                1024, 			/* The size of the stack to allocate to the task. */
                NULL, 			/* The parameter passed to the task - not used in this case. */
                (tskIDLE_PRIORITY + 1), 	/* The priority assigned to the task. */
                NULL);			/* The task handle is not required, so NULL is passed. */

    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    while(1){ }
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
    /* Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

    /* Force an assert. */
    ei_printf("Malloc Failed\n");
    configASSERT((volatile void*)NULL);
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */

    /* Force an assert. */
    ei_printf("Stack Overflow\n");
    configASSERT((volatile void*)NULL);
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    volatile size_t xFreeHeapSpace;

    /* This is just a trivial example of an idle hook.  It is called on each
    cycle of the idle task.  It must *NOT* attempt to block.  In this case the
    idle task just queries the amount of FreeRTOS heap that remains.  See the
    memory management section on the http://www.FreeRTOS.org web site for memory
    management options.  If there is a lot of heap memory free then the
    configTOTAL_HEAP_SIZE value in FreeRTOSConfig.h can be reduced to free up
    RAM. */
    xFreeHeapSpace = xPortGetFreeHeapSize();

    /* Remove compiler warning about xFreeHeapSpace being set but never used. */
    (void)xFreeHeapSpace;
}