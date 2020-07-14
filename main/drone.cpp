/* espDrone in the making
 * from Github: Simaho99
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "esp_netif.h"
#include "esp_event.h"
//#include "protocol_examples_common.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


#include "esp8266/gpio_register.h"
#include "esp8266/pin_mux_register.h"

#include "driver/pwm.h"



#define PWM_0_OUT_IO_NUM   12
#define PWM_1_OUT_IO_NUM   13
#define PWM_2_OUT_IO_NUM   14
#define PWM_3_OUT_IO_NUM   15

// PWM period 1000us(1Khz), same as depth
// Do nor use period below 20us
#define PWM_PERIOD    (1000)
const uint32_t pin_num[4] = {
    PWM_0_OUT_IO_NUM,
    PWM_1_OUT_IO_NUM,
    PWM_2_OUT_IO_NUM,
    PWM_3_OUT_IO_NUM
};

// duties table, real_duty = duties[x]/PERIOD
uint32_t duties[4] = {
    500, 500, 500, 500,
};

// phase table, delay = (phase[x]/360)*PERIOD
// the total phase should be 180 while all are positive.
int16_t phase[4] = {
    0, 0, 90, -90,
};
extern "C" {
    void app_main(void);
}

extern void task_initI2C(void*);
extern void task_display(void*);

/*This task defines how the PWM signal will be moving
 * The aim is to make one motor to move at any given
 * time in order to have more power on each motor
 * The phases will be determined by the PID values
 * The phases will be within 180 Degrees(TODO: Research)*/
void pwm_task(void*){
    pwm_init(PWM_PERIOD, duties, 4, pin_num);
        pwm_set_phases(phase);
        pwm_start();

        while (1) {
                pwm_start();
                // Delay is a must else, it will keep restarting
                // TODO: Read Doc here
                vTaskDelay(100 / portTICK_RATE_MS);
            }

}

void app_main()
{
//    xTaskCreate(i2c_mpu_init,"MPU_init", 2048,NULL, 10, NULL);
//    vTaskDelay(1000 / portTICK_RATE_MS);
////    xTaskCreate(pwm_task,"PWM Signals", 2048,NULL, 10, NULL);
//    xTaskCreate(mpu_task, "disp_task", 8192, NULL, 5, NULL);

//    ESP_ERROR_CHECK(nvs_flash_init());
//    ESP_ERROR_CHECK(esp_netif_init());
//    ESP_ERROR_CHECK(esp_event_loop_create_default());
    xTaskCreate(&task_initI2C, "mpu_task", 2048, NULL, 5, NULL);
    vTaskDelay(500/portTICK_PERIOD_MS);
    xTaskCreate(&task_display, "disp_task", 8192, NULL, 5, NULL);
//
//    ESP_ERROR_CHECK(example_connect());

//    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
//    printf("Restarting now.\n");
//    fflush(stdout);
//    esp_restart();
}
