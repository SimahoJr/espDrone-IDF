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
#include "protocol_examples_common.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


#include "esp8266/gpio_register.h"
#include "esp8266/pin_mux_register.h"

#include "driver/pwm.h"



// MPU Libraries
#include <driver/i2c.h>
#include "sdkconfig.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#define I2C_MPU_SCL_IO           5                /*!< gpio number for I2C master clock */
#define I2C_MPU_SDA_IO           4               /*!< gpio number for I2C master data  */
#define I2C_MPU_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_MPU_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_MPU_RX_BUF_DISABLE   0                /*!< I2C master do not need */


extern "C" {
	void app_main(void);
}

#define PWM_0_OUT_IO_NUM   12
#define PWM_1_OUT_IO_NUM   13
#define PWM_2_OUT_IO_NUM   14
#define PWM_3_OUT_IO_NUM   15

// PWM period 1000us(1Khz), same as depth
// Do nor use period below 20us
#define PWM_PERIOD    (1000)



Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

////  VariaFLes
double yawSetpoint, yawInput, yawOutput;
double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput;
double throttle;

static const char *TAG = "espDrone";


#define PORT CONFIG_EXAMPLE_PORT


static void tcp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 destAddr;
        bzero(&destAddr.sin6_addr.un, sizeof(destAddr.sin6_addr.un));
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(listen_sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket binded");

        err = listen(listen_sock, 1);
        if (err != 0) {
            ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket listening");

#ifdef CONFIG_EXAMPLE_IPV6
        struct sockaddr_in6 sourceAddr; // Large enough for both IPv4 or IPv6
#else
        struct sockaddr_in sourceAddr;
#endif
        uint addrLen = sizeof(sourceAddr);
        int sock = accept(listen_sock, (struct sockaddr *)&sourceAddr, &addrLen);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket accepted");

        while (1) {
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occured during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Connection closed
            else if (len == 0) {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }
            // Data received
            else {
#ifdef CONFIG_EXAMPLE_IPV6
                // Get the sender's ip address as string
                if (sourceAddr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&sourceAddr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } else if (sourceAddr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(sourceAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }
#else
                inet_ntoa_r(((struct sockaddr_in *)&sourceAddr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
#endif

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);

                int err = send(sock, rx_buffer, len, 0);
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

/**
 * @PWM parameters and initialization
 */
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




/**
 * @brief i2c master initialization
 */
void i2c_mpu_init(void*)
{
    int i2c_mpu_port = I2C_MPU_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MPU_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MPU_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.clk_stretch_tick = 400000; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MPU_NUM, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(I2C_MPU_NUM, &conf));
    vTaskDelete(NULL);

}




/**
 * @brief MPU initialization
 */
extern void mpu_task(void*){
		MPU6050 mpu = MPU6050();
		mpu.initialize();
		mpu.dmpInitialize();

		// This need to be setup individually
		mpu.setXGyroOffset(220);
		mpu.setYGyroOffset(76);
		mpu.setZGyroOffset(-85);
		mpu.setZAccelOffset(1788);

		mpu.setDMPEnabled(true);

		while(1){
		    mpuIntStatus = mpu.getIntStatus();
			// get current FIFO count
			fifoCount = mpu.getFIFOCount();

		    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		        // reset so we can continue cleanly
		        mpu.resetFIFO();

		    // otherwise, check for DMP data ready interrupt frequently)
		    } else if (mpuIntStatus & 0x02) {
		        // wait for correct available data length, should be a VERY short wait
		        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		        // read a packet from FIFO

		        mpu.getFIFOBytes(fifoBuffer, packetSize);
		 		mpu.dmpGetQuaternion(&q, fifoBuffer);
				mpu.dmpGetGravity(&gravity, &q);
				mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
				printf("YAW: %3.1f, ", ypr[0] * 180/M_PI);
				printf("PITCH: %3.1f, ", ypr[1] * 180/M_PI);
				printf("ROLL: %3.1f \n", ypr[2] * 180/M_PI);
		    }

		    //Best result is to match with DMP refresh rate
		    // Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
		    // Now its 0x13, which means DMP is refreshed with 10Hz rate
			vTaskDelay(100/portTICK_PERIOD_MS);
		}

		vTaskDelete(NULL);
	}




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
	xTaskCreate(&i2c_mpu_init,"MPU_init", 2048,NULL, 10, NULL);
	vTaskDelay(100 / portTICK_RATE_MS);
	xTaskCreate(&pwm_task,"PWM Signals", 2048,NULL, 10, NULL);
	xTaskCreate(&mpu_task, "disp_task", 2048, NULL, 10, NULL);

//	ESP_ERROR_CHECK(nvs_flash_init());
//	ESP_ERROR_CHECK(esp_netif_init());
//	ESP_ERROR_CHECK(esp_event_loop_create_default());
//
//	ESP_ERROR_CHECK(example_connect());

//	xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
//	printf("Restarting now.\n");
	fflush(stdout);
	esp_restart();
}

