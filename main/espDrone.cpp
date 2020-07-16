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



extern "C" {
    void app_main(void);
}

extern void task_initI2C(void*);
extern void task_display(void*);
extern void tcp_server_task(void *pvParameters);
extern "C" void wifi_init_softap();
extern "C" void pwm_task(void*);


void app_main()
{
    wifi_init_softap();
    xTaskCreate(pwm_task,"PWM Signals", 2048,NULL, 10, NULL);
    xTaskCreate(&task_initI2C, "mpu_task", 2048, NULL, 5, NULL);
    vTaskDelay(500/portTICK_PERIOD_MS);
    xTaskCreate(&task_display, "disp_task", 8192, NULL, 5, NULL);
    
}
