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

static uint8_t txbuf[512];

static uint8_t rxbuf[512];
//static uint8_t send_message = WHO_AM_I_MPU9250 | 0x80;
static uint8_t send_message = 0xFF;
static uint8_t read_buff = 2;
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

static THD_WORKING_AREA(spi_thread_1_wa, 512);
static THD_FUNCTION(spi_thread_1, p) {

  (void)p;
  txbuf[0] = 0xF5;
  txbuf[1] = 0xFF;
  chRegSetThreadName("SPI thread 1");
  while (true) {
   // spiAcquireBus(&SPID2);              /* Acquire ownership of the bus.    */
    palSetLine(LINE_GREEN_LED);    /* LED ON.                          */
    chThdSleepMilliseconds(100);
   palClearLine(LINE_MPU_CS);

    //spiSend(&SPID2, 1, &send_message);	/* send request       */
    //spiSelect(&SPID2);                  /* Slave Select assertion.          */
    spiExchange(&SPID2, 2,
    		txbuf, rxbuf);          /* Atomic transfer operations.      */
  //  spiUnselect(&SPID2);                /* Slave Select de-assertion.       */
    //spiReceive(&SPID2, 1, &read_buff);
    spiReleaseBus(&SPID2);              /* Ownership release.               */
    palSetLine(LINE_MPU_CS);
    palClearLine(LINE_GREEN_LED);    /* LED OFF.                          */
    chThdSleepMilliseconds(100);
    chprintf((BaseSequentialStream*)&SD1, "MPU ans: %d %d \r\n", rxbuf[0], rxbuf[1]);
  }
}
/*
 *  This is thread that serve spi2 bus and provide communication with modules
 *	MPU9250 and UBLOX NEO-M8P
 */

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
  spiStart(&SPID2, &spi2_cfg);       /* Setup transfer parameters.       */
  //spiStart(&SPID2, &spi2_cfg);
  /*
   * Shell manager initialization.
   */
#ifdef USE_SD_SHELL
  shellInit();
#endif
  chprintf((BaseSequentialStream*)&SD1, "Hello World %dst test!\r\n", 1);
  /*
   * Creates the example thread.
   */

  chThdCreateStatic(spi_thread_1_wa, sizeof(spi_thread_1_wa),
                      NORMALPRIO + 1, spi_thread_1, NULL);
  //chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);
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
