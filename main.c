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

#include <string.h>

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "config.h"
#ifdef USE_UBLOX_GPS_MODULE
#include "neo-m8.h"
extern ubx_nav_pvt_t *pvt_box;
#endif
#ifdef USE_SD_SHELL
#include "sd_shell_cmds.h"
extern output_t *output;
#endif
#ifdef USE_BNO055_MODULE
#include "bno055_i2c.h"
extern bno055_t *bno055;
#endif
#ifdef USE_MICROSD_MODULE
#include "microsd.h"
#endif
#ifdef USE_WINDSENSOR_MODULE
#include "windsensor.h"
extern windsensor_t *wind;
#endif
#ifdef USE_BLE_MODULE
#include "nina-b3.h"
#endif
#include "eeprom.h"
struct ch_semaphore usart1_semaph;
struct ch_semaphore spi2_semaph;
static const WDGConfig wdgcfg = {
  STM32_IWDG_PR_64,
  STM32_IWDG_RL(1000),
  0xFFF					//Windowed watchdog workaround
};

/**
 * Executes the BKPT instruction that causes the debugger to stop.
 * If no debugger is attached, this will be ignored
 */


/*===========================================================================*/
/* Application code.                                                         */
/*===========================================================================*/

void fill_memory(void){
#ifdef USE_BNO055_MODULE
	bno055 = calloc(1, sizeof(bno055_t));
	pvt_box = calloc(1, sizeof(ubx_nav_pvt_t));
#endif
#ifdef USE_WINDSENSOR_MODULE
	wind = calloc(1, sizeof(windsensor_t));
#endif
#ifdef USE_SD_SHELL
	output = calloc(1, sizeof(output_t));
#endif
}
/*
 * Application entry point.
 */
int main(void) {
	uint32_t *ACTLR = (uint32_t *) 0xE000E008;
	*ACTLR |= 2;
	thread_t *sh = NULL;

	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	//wdgReset(&WDGD1);
	halInit();
	chSysInit();
	wdgStart(&WDGD1, &wdgcfg);
	fill_memory();
#if (__CORTEX_M == 0x03 || __CORTEX_M == 0x04)
    chSysLock();
    // enable UsageFault, BusFault, MemManageFault
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk |
                  SCB_SHCSR_MEMFAULTENA_Msk;
    // enable fault on division by zero
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
    chSysUnlock();
#endif

	chSemObjectInit(&usart1_semaph, 1);
	chSemObjectInit(&spi2_semaph, 1);
	palClearLine(LINE_RF_868_RST);
#ifdef USE_SD_SHELL
	sdStart(&SD1, NULL);
	shellInit();
	chSemWait(&usart1_semaph);
	sh = cmd_init();
	chSemSignal(&usart1_semaph);
	wdgReset(&WDGD1);
#else
	sdStart(&SD1, NULL);
#endif
	chThdSleepMilliseconds(30);
#ifdef USE_MICROSD_MODULE
	start_microsd_module();
	chThdSleepMilliseconds(15);
#endif
	wdgReset(&WDGD1);
#ifdef USE_WINDSENSOR_MODULE
	start_windsensor_module();
	chThdSleepMilliseconds(15);
#endif
	wdgReset(&WDGD1);
#ifdef USE_UBLOX_GPS_MODULE
	start_gps_module();
	chThdSleepMilliseconds(15);
#endif
	wdgReset(&WDGD1);
#ifdef USE_BNO055_MODULE
	start_bno055_module();
	chThdSleepMilliseconds(100);
#endif

#ifdef USE_BLE_MODULE
//	start_ble_module();
#endif

#ifdef USE_SD_SHELL
	start_json_module();
	chThdSleepMilliseconds(15);
#endif
	start_eeprom_module();
	eeprom_write_hw_version();
	chThdSleepMilliseconds(15);
	eeprom_read_hw_version();
	//eeprom_check_i2c_bus();
	//toggle_test_output();

	/*
	 * Normal main() thread activity, in this demo it does nothing.
	 */
	while (true) {
#ifdef USE_SD_SHELL
		if (!sh)
			sh = cmd_init();
		else if (chThdTerminatedX(sh)) {
			chThdRelease(sh);
			sh = NULL;
		}
#endif
		chThdSleepMilliseconds(500);
	}
}
