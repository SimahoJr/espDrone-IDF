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


static const char *TAG = "espDrone";



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
	xTaskCreate(&i2c_mpu_init,"MPU init", 2048,NULL, 10, NULL);
	vTaskDelay(100 / portTICK_RATE_MS);
	xTaskCreate(&pwm_task,"PWM Signals", 2048,NULL, 10, NULL);
	xTaskCreate(&mpu_task, "disp_task", 2048, NULL, 10, NULL);
	printf("Restarting now.\n");
	fflush(stdout);
//	esp_restart();
}

