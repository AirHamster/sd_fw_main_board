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
#include "xbee.h"
#include "sd_shell_cmds.h"



/*
 * Maximum speed SPI configuration (3.3MHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig spi1_cfg = {
  false,
  NULL,
  GPIOA,
  GPIOA_RF_868_CS,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0,	//FPCLK1 is 54 MHZ. XBEE support 3.5 max, so divide it by 16
//  0,
  0
};

/*
 * Maximum speed SPI configuration (18MHz, CPHA=1, CPOL=1, MSb first).
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
   // palSetLine(LINE_RED_LED);
    chThdSleepMilliseconds(200);
    palClearLine(LINE_ORANGE_LED);
    chThdSleepMilliseconds(50);
   // palClearLine(LINE_GREEN_LED);
    chThdSleepMilliseconds(50);
   // palClearLine(LINE_RED_LED);
    chThdSleepMilliseconds(200);
  }
}

/*
 * This is a thread that serve SPI1 bus with Xbee module
 * and second CPU
 */

static THD_WORKING_AREA(spi1_thread_wa, 512);
static THD_FUNCTION(spi1_thread, p){
	(void)p;

	chRegSetThreadName("spi1_thread");
	while(true){
		xbee_check_attn();
		chThdSleepMilliseconds(2000);
	}
}


/*
 *  This is thread that serve spi2 bus and provide communication with modules
 *	MPU9250 and UBLOX NEO-M8P
 */

static THD_WORKING_AREA(spi2_thread_wa, 512);
static THD_FUNCTION(spi2_thread, p) {
  (void)p;
  int16_t accel_data[3];
  int16_t gyro_data[3];
  int16_t mag_data[3];
  uint8_t rxbuf[101];
  int i = 0;
   chRegSetThreadName("SPI2 thread");
  while (true) {
        chThdSleepMilliseconds(100);
        palSetLine(LINE_GREEN_LED);
    //mpu_read_accel_data(&accel_data[0]);
    //mpu_read_gyro_data(&gyro_data[0]);
    //mpu_read_mag_data(&mag_data[0]);
    neo_read_bytes(&SPID2, 100, rxbuf);
    palClearLine(LINE_GREEN_LED);
    chThdSleepMilliseconds(100);
    //chprintf((BaseSequentialStream*)&SD1, "\nACCEL X: %d ACCEL_Y: %d ACCEL_Z: %d \r\nGYRO_X: %d GYRO_Y: %d GYRO_Z: %d \r\nMAG_X: %d MAG_Y: %d MAG_Z: %d \n\r",
    //									accel_data[0], accel_data[1], accel_data[2], gyro_data[0], gyro_data[1], gyro_data[2], mag_data[0], mag_data[1], mag_data[2]);
    chprintf(IFACE1, "SPI:  ");
    for (i = 0; i<100; i++){
    	chprintf(IFACE1, "%x ", rxbuf[i]);
    }
    chprintf(IFACE1, "\n\r");
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

  spiStart(&SPID1, &spi1_cfg);
  spiStart(&SPID2, &spi2_cfg);


  //spiStart(&SPID2, &spi2_cfg);
  /*
   * Shell manager initialization.
   */
#ifdef USE_SD_SHELL

  sdStart(&SD1, NULL);
  shellInit();
#else
  sdStart(&SD1, NULL);
#endif

	mpu9250_init();
	chThdSleepMilliseconds(100);
	float destination[3];
	initAK8963(&destination[0]);
	chprintf((BaseSequentialStream*)&SD1, "Hello World %dst test!\r\n", 1);
	chThdSleepMilliseconds(1000);

  chThdCreateStatic(spi1_thread_wa, sizeof(spi1_thread_wa),
                    NORMALPRIO , spi1_thread, NULL);
  //chThdCreateStatic(spi2_thread_wa, sizeof(spi2_thread_wa),
   //                     NORMALPRIO + 1, spi2_thread, NULL);
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  //cmd_init();
  while(true) {
	  thread_t *shelltp = cmd_init();
	        chThdWait(shelltp);               /* Waiting termination.             */
	        chThdSleepMilliseconds(1000);
      }

  while (true) {
   // if (palReadLine(LINE_BUTTON)) {
   //   test_execute((BaseSequentialStream *)&SD3, &rt_test_suite);
   //   test_execute((BaseSequentialStream *)&SD3, &oslib_test_suite);
   // }
    chThdSleepMilliseconds(500);
  }
}
