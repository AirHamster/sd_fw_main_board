/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#include <stdint.h>
#include "ch.h"
#include "hal.h"
#include "rt_test_root.h"
#include "oslib_test_root.h"

#include "shell.h"
#include "chprintf.h"

#include "MPU9250.h"
#include "neo-m8.h"

uint8_t neo_rx_buff1[256];
uint8_t neo_rx_buff2[256];
struct minmea_sentence_rmc rmc;
/*
 * Maximum speed SPI configuration (18MHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig spi2_cfg = {
  false,
  NULL,
  GPIOC,
  GPIOC_MCU_CS,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_CPOL | SPI_CR1_CPHA,
//  0,
  0
};

static const SPIConfig spi2_cfg_neo = {
  false,
  NULL,
  GPIOC,
  GPIOC_MCU_CS,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0,
//  0,
  0
};
/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palSetLine(LINE_ORANGE_LED);
    chThdSleepMilliseconds(50);
   // palSetLine(LINE_GREEN_LED);
    chThdSleepMilliseconds(50);
    palSetLine(LINE_RED_LED);
    chThdSleepMilliseconds(200);
    palClearLine(LINE_ORANGE_LED);
    chThdSleepMilliseconds(50);
   // palClearLine(LINE_GREEN_LED);
    chThdSleepMilliseconds(50);
    palClearLine(LINE_RED_LED);
    chThdSleepMilliseconds(200);
  }
}

/*
 *  This is thread that serve spi2 bus and provide communication with modules
 *	MPU9250 and UBLOX NEO-M8P
 */

static THD_WORKING_AREA(spi_thread_1_wa, 4096);
static THD_FUNCTION(spi_thread_1, p) {

  (void)p;
  int16_t accel_data[3];
  int16_t gyro_data[3];
  int16_t mag_data[3];
  char rxbuf[401];
   //txbuf[0] = 0xF5;
  //txbuf[1] = 0xFF;
  int i = 0;
   chRegSetThreadName("SPI thread 1");
  while (true) {
        chThdSleepMilliseconds(300);
        palSetLine(LINE_GREEN_LED);    /* LED ON.                          */

 //   mpu_read_accel_data(&accel_data[0]);
 //   mpu_read_gyro_data(&gyro_data[0]);
    //mpu_read_mag_data(&mag_data[0]);
    neo_read_bytes(&SPID2, 400, rxbuf);
    rxbuf[401] = '\0';
    palClearLine(LINE_GREEN_LED);    /* LED OFF.                          */
    chThdSleepMilliseconds(100);
    if (minmea_parse_rmc(&rmc, rxbuf)){
    	//chprintf((BaseSequentialStream*)&SD1, "RMC found\n\r");
        	//chprintf((BaseSequentialStream*)&SD1, "RMC time:           %d:%d:%d:%d\n\r", rmc.time.hours, rmc.time.minutes, rmc.time.seconds, rmc.time.microseconds);
        	//chprintf((BaseSequentialStream*)&SD1, "RMC valid:          %d\n\r", rmc.valid);
        	//chprintf((BaseSequentialStream*)&SD1, "RMC raw latitude:   %d %d\n\r", rmc.latitude.value, rmc.latitude.scale);
        	//chprintf((BaseSequentialStream*)&SD1, "RMC raw longtitude: %d %d\n\r", rmc.longitude.value, rmc.longitude.scale);
        	//chprintf((BaseSequentialStream*)&SD1, "RMC raw speed:      %d %d\n\n\r", rmc.speed.value, rmc.speed.scale);
        	/*chprintf((BaseSequentialStream*)&SD1, "RMC fp latitude:    %d\n\r", minmea_rescale(&rmc.latitude, 1000));
        	chprintf((BaseSequentialStream*)&SD1, "RMC fp longtitude:  %d\n\r", minmea_rescale(&rmc.longitude, 1000));
        	chprintf((BaseSequentialStream*)&SD1, "RMC fp speed:       %d\n\r", minmea_rescale(&rmc.speed, 1000));
        	chprintf((BaseSequentialStream*)&SD1, "RMC float latitude: %d\n\r", minmea_tocoord(&rmc.latitude));
        	chprintf((BaseSequentialStream*)&SD1, "RMC float longtit:  %d\n\r", minmea_tocoord(&rmc.longitude));
        	chprintf((BaseSequentialStream*)&SD1, "RMC float speed:    %d\n\n\r", minmea_tocoord(&rmc.speed));*/

        }
   // chprintf((BaseSequentialStream*)&SD1, "\nACCEL X: %d ACCEL_Y: %d ACCEL_Z: %d \r\nGYRO_X: %d GYRO_Y: %d GYRO_Z: %d \r\nMAG_X: %d MAG_Y: %d MAG_Z: %d \r\n\n",
    //									accel_data[0], accel_data[1], accel_data[2], gyro_data[0], gyro_data[1], gyro_data[2], mag_data[0], mag_data[1], mag_data[2]);
    chprintf((BaseSequentialStream*)&SD1, "\n\rSPI:  \n\r");
    for (i = 0; i < 400; i++){
    	chprintf((BaseSequentialStream*)&SD1, "%c", rxbuf[i]);
    }
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);
  spiStart(&SPID2, &spi2_cfg_neo);       /* Setup transfer parameters.       */


  //spiStart(&SPID2, &spi2_cfg);
  /*
   * Shell manager initialization.
   */
#ifdef USE_SD_SHELL
  //shellInit();
  //chprintf((BaseSequentialStream*)&SD1, "Shell initialized\r\n");
#endif

	mpu9250_init();
	chThdSleepMilliseconds(100);
	float destination[3];
	initAK8963(&destination[0]);
	chprintf((BaseSequentialStream*)&SD1, "Hello World %dst test!\r\n", 1);
	chThdSleepMilliseconds(1000);
  /*
   * Creates the example thread.
   */

  chThdCreateStatic(spi_thread_1_wa, sizeof(spi_thread_1_wa),
                      NORMALPRIO + 1, spi_thread_1, NULL);
 // chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);
  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
   // if (palReadLine(LINE_BUTTON)) {
   //   test_execute((BaseSequentialStream *)&SD3, &rt_test_suite);
   //   test_execute((BaseSequentialStream *)&SD3, &oslib_test_suite);
   // }
    chThdSleepMilliseconds(500);
  }
}
