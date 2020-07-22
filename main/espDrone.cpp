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
    void app_main(); // Must be here for other applications to work
    //  PWM Variables
    void pwm_task(void*);
    //  WiFi Variables
    void wifi_init_softap();
    //void tcpip_adapter_dhcps_cb();
    // TCP Server here
//    void tcp_server_task();
    // PID Variables
    void PID_Init();

}


//  MPU6050 Variables
extern void MPU_run(void*);



void app_main()
{
    wifi_init_softap();
//    xTaskCreate(&MPU_initI2C, "MPU_task", 2048, NULL, 5, NULL);
    vTaskDelay(500/portTICK_PERIOD_MS);
    xTaskCreate(&MPU_run, "MPU Run", 8192, NULL, 5, NULL);
    xTaskCreate(&pwm_task, "pwm_tk", 1000, NULL, 5, NULL);

}
