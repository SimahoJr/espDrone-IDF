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

#include "pid.h"


extern "C" {
    void app_main(void);
}


//  MPU6050 Variables
extern void MPU_initI2C(void*);
extern void MPU_run(void*);
extern "C" float ypr[3];


//  WiFi Variables
extern "C" void wifi_init_softap();


//  PWM Variables
extern "C" void pwm_task(void*);


//  Variables Tests
void print_stuff(void*)
{
    while(1){
    vTaskDelay(100/portTICK_PERIOD_MS);
    printf("YAW: %3.1f, ", ypr[0]);
    printf("PITCH: %3.1f, ", ypr[2]);
    printf("ROLL: %3.1f \n", ypr[2]);
    }
     vTaskDelete(NULL);
}


void app_main()
{
    wifi_init_softap();
    xTaskCreate(pwm_task,"PWM Signals", 2048,NULL, 10, NULL);
    xTaskCreate(&MPU_initI2C, "MPU_task", 2048, NULL, 5, NULL);
    vTaskDelay(500/portTICK_PERIOD_MS);
    xTaskCreate(&MPU_run, "MPU Run", 8192, NULL, 5, NULL);
    xTaskCreate(&print_stuff, "disp_tk", 1000, NULL, 5, NULL);

}
