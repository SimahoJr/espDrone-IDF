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
#include "modbus.h"


extern "C" {
    void app_main(void);
}


//  MPU6050 Variables
extern void MPU_initI2C(void*);
extern void MPU_run(void*);
extern "C" float ypr[3];


//  WiFi Variables
extern "C" void wifi_init_softap();
//extern "C" void tcpip_adapter_dhcps_cb();


//  PWM Variables
extern "C" void pwm_task(void*);


// PID Variables
extern "C" void PID_Init();

// TCP Server here
extern "C" void tcp_server_task();

// Modbus_task here
void modbus_task(void*)
{
//    tcpip_adapter_dhcps_cb();
    // create a modbus object
    
    modbus mb = modbus("192.168.4.1", 502);
    
    printf("Modbus is Running");
     vTaskDelay(100);


    // set slave id
    mb.modbus_set_slave_id(1);

    // connect with the server
    mb.modbus_connect();
    while(1){
        
        // read holding registers           function 0x03
        uint16_t read_holding_regs[1];
        int a = mb.modbus_read_holding_registers(100, 1, read_holding_regs);

        // read input registers             function 0x04
        uint16_t read_input_regs[1];
        mb.modbus_read_input_registers(0, 1, read_input_regs);

        // write single reg                 function 0x06
        mb.modbus_write_register(0, 123);

        // write multiple regs              function 0x10
        uint16_t write_regs[4] = {123, 123, 123};
        mb.modbus_write_registers(0, 4, write_regs);
        printf("The REG is: %d \n",a);
//        printf("IP: %d \n", client_ip[0]);

        vTaskDelay(100);
        
    }
    // close connection and free the memory
    mb.modbus_close();

    vTaskDelete(NULL);
}


//  Variables Tests
void print_stuff(void*)
{
    while(1){
    vTaskDelay(100/portTICK_PERIOD_MS);
    printf("YAW: %3.1f, ", ypr[0]);
    printf("PITCH: %3.1f, ", ypr[1]);
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
    xTaskCreate(&modbus_task, "modbus_tk", 1000, NULL, 5, NULL);

}
