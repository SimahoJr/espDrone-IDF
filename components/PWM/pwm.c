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

extern float ypr[3];  //Called from a CPP function


static const char *TAG = "Drone's PWM";

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


/*This task defines how the PWM signal will be moving
 * The aim is to make one motor to move at any given
 * time in order to have more power on each motor
 * The phases will be determined by the PID values
 * The phases will be within 180 Degrees(TODO: Research)*/
void pwm_task(){
    pwm_init(PWM_PERIOD, duties, 4, pin_num);
        pwm_set_phases(phase);
        pwm_start();

        while (1) {
                pwm_start();
                // Delay is a must else, it will keep restarting
                // TODO: Read Doc here
                ESP_LOGI(TAG, "Running");
                vTaskDelay(100 / portTICK_RATE_MS);
                ESP_LOGI(TAG, "YAW: %3.1f, PITCH: %3.1f,ROLL: %3.1f", ypr[0], ypr[1], ypr[2]);
//                printf("YAW: %3.1f, ", ypr[0]);
//                printf("PITCH: %3.1f, ", ypr[1]);
//                printf("ROLL: %3.1f \n", ypr[2]);
                }
                 vTaskDelete(NULL);
    
}

