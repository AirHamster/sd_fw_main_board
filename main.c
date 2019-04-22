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
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include "rt_test_root.h"
#include "oslib_test_root.h"
#include "shell.h"

#include "MPU9250.h"
#include "sd_shell_cmds.h"
#include "xbee.h"

#include "chprintf.h"
#include "neo-m8.h"


uint8_t payload[256];
extern ubx_nav_pvt_t *pvt_box;
extern ubx_cfg_rate_t *rate_box;
extern ubx_cfg_nav5_t *nav5_box;
extern xbee_struct_t *xbee;
extern neo_struct_t *neo;
extern mpu_struct_t *mpu;
extern output_struct_t *output;
struct ch_semaphore usart1_semaph;
struct ch_semaphore spi2_semaph;
extern float calib[];
#define MAX_FILLER 11
#define FLOAT_PRECISION 9

static void gpt9cb(GPTDriver *gptp);
static void gpt11cb(GPTDriver *gptp);
static void gpt12cb(GPTDriver *gptp);
static void gpt14cb(GPTDriver *gptp);

void insert_dot(char *str){
	uint8_t str2[20];
	str2[0] = str[0];
	str2[1] = str[1];
	str2[2] = '.';
	memcpy(&str2[3], &str[2], 7);
	memcpy(str, str2, 8);
}



static GPTConfig gpt14cfg =
{
		20000,      // Timer clock
		gpt14cb,        // Callback function
		0,
		0
};

static GPTConfig gpt12cfg =
{
		20000,      // Timer clock
		gpt12cb,        // Callback function
		0,
		0
};

static GPTConfig gpt11cfg =
{
		20000,      // Timer clock
		gpt11cb,        // Callback function
		0,
		0
};

static GPTConfig gpt9cfg =
{
		20000,      // Timer clock
		gpt9cb,        // Callback function
		0,
		0
};

/*
 * Maximum speed SPI configuration (3.3MHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig spi1_cfg = {
		false,
		NULL,
		GPIOA,
		GPIOA_RF_868_CS,
		SPI_CR1_BR_1 | SPI_CR1_BR_0,	//FPCLK1 is 54 MHZ. XBEE support 3.5 max, so divide it by 16
		//  0,
		0
};

const SPIConfig neo_spi_cfg = {
		false,
		NULL,
		GPIOC,
		GPIOC_MCU_CS,
		SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0,
		//0,
		0
};

const SPIConfig mpu_spi_cfg = {
		false,
		NULL,
		GPIOC,
		GPIOC_MCU_CS,
		SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_CPOL,
		//0,
		0
};

thread_reference_t xbee_poll_trp = NULL;

static THD_WORKING_AREA(xbee_poll_thread_wa, 1024);
static THD_FUNCTION(xbee_poll_thread, p){
	(void)p;
	msg_t msg;
	uint8_t i;
	uint8_t txbuff[20];
	//uint8_t rxbuff[20];
	//uint8_t megabuff[32+24];	//Max payload lenth + headers etc
	//uint8_t crc;
	//msg_t msg;
	chRegSetThreadName("XBee polling thd");
	//memset(megabuff, 0x00, 32+24);
	//memset(txbuff, 0x00, 20);
	while (true){
		chSysLock();
		if (xbee->poll_suspend_state) {
			msg = chThdSuspendS(&xbee_poll_trp);
		}
		chSysUnlock();
		palToggleLine(LINE_RED_LED);
		if(!palReadLine(LINE_RF_868_SPI_ATTN)){
		xbee_polling();
		}
/*
		for (i = 0; i < 30; i++){
			xbee_read_no_cs(&SPID1, 1, &rxbuff[0]);
			if ( rxbuff[0] == 0x7E ){
				xbee_read_no_cs(&SPID1, 2, &rxbuff[1]);
				len = (rxbuff[1] << 8) | rxbuff[2];
				megabuff[0] = rxbuff[0];
				megabuff[1] = rxbuff[1];
				megabuff[2] = rxbuff[2];
				xbee_send(&SPID1, &megabuff[3], len + 1);	//len + CRC
				i = 30;
				crc = xbee_calc_CRC(&megabuff[3], len);
				if (crc == megabuff[3+len]){
					switch (megabuff[3]){

					case XBEE_AT_RESPONSE_FRAME:
						xbee_process_at_response(megabuff);
						break;
					case XBEE_TRANSMIT_STAT_FRAME:
						xbee_process_tx_stat(megabuff);
						break;
					case XBEE_RECEIVE_PACKET_FRAME:
						xbee_process_recieve(megabuff);
						break;
					case XBEE_NODE_ID_FRAME:
						xbee_process_node_id(megabuff);
						break;
					}
				}

			}
		} */
	}
}

thread_reference_t xbee_trp = NULL;

static THD_WORKING_AREA(xbee_thread_wa, 1024);
static THD_FUNCTION(xbee_thread, p){
	(void)p;
	msg_t msg;
	uint8_t at[] = {'S', 'L'};
	uint8_t rxbuf[15];
	chRegSetThreadName("XBee Thread");
	while (true) {
		chSysLock();
		if (xbee->suspend_state) {
			msg = chThdSuspendS(&xbee_trp);
		}
		chSysUnlock();

		/* Perform processing here.*/
		//chprintf((BaseSequentialStream*)&SD1, "thd_xbee\n\r");
		switch (msg){
		case XBEE_GET_OWN_ADDR:
			xbee_read_own_addr(xbee);
			break;
		case XBEE_GET_RSSI:
			xbee->rssi = xbee_read_last_rssi(xbee);
			chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*)&SD1, "RSSI: %d\r\n", xbee->rssi);
			chSemSignal(&usart1_semaph);
			break;
		case XBEE_GET_PACKET_PAYLOAD:
			xbee->packet_payload = xbee_get_packet_payload(xbee);
			chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*)&SD1, "Packet payload: %d\r\n", xbee->packet_payload);
			chSemSignal(&usart1_semaph);
			break;
		case XBEE_GET_STAT:
			xbee->bytes_transmitted = xbee_get_bytes_transmitted(xbee);
			xbee->good_packs_res = xbee_get_good_packets_res(xbee);
			xbee->rec_err_count = xbee_get_received_err_count(xbee);
			xbee->trans_errs = xbee_get_transceived_err_count(xbee);
			xbee->unicast_trans_count = xbee_get_unicast_trans_count(xbee);
			xbee->rssi = xbee_read_last_rssi(xbee);
			chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*)&SD1, "Bytes transmitted:     %d\r\n", xbee->bytes_transmitted);
			chprintf((BaseSequentialStream*)&SD1, "Good packets received: %d\r\n", xbee->good_packs_res);
			chprintf((BaseSequentialStream*)&SD1, "Received errors count: %d\r\n", xbee->rec_err_count);
			chprintf((BaseSequentialStream*)&SD1, "Transceiver errors:    %d\r\n", xbee->trans_errs);
			chprintf((BaseSequentialStream*)&SD1, "Unicast transmittions: %d\r\n", xbee->unicast_trans_count);
			chprintf((BaseSequentialStream*)&SD1, "RSSI:                  %d\r\n", xbee->rssi);
			chSemSignal(&usart1_semaph);
			break;
		case XBEE_GET_PING:
			chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*)&SD1, "Ping hello message\r\n");
			chSemSignal(&usart1_semaph);
			xbee_send_ping_message(xbee);
		}

		xbee->suspend_state = 1;
	}
}
/*
static THD_WORKING_AREA(spi2_thread_wa, 1024);
static THD_FUNCTION(spi2_thread, p) {
	int32_t spdi = 0;
	char lon[20];
	char lat[20];
	double spd;
	(void)p;

	chRegSetThreadName("SPI2 thread");
	while (true) {
		chThdSleepMilliseconds(500);
		//chprintf((BaseSequentialStream*)&SD1, "thd2\n\r");
		memset(lon, '\0',20);
		memset(lat, '\0',20);

		neo_poll();
		//neo_poll_nav_pvt();
		//chprintf((BaseSequentialStream*)&SD1, "thd2_2\n\r");
		itoa(pvt_box->lat, lat, 10);
		itoa(pvt_box->lon, lon, 10);
		insert_dot(lat);
		insert_dot(lon);
		spd = (float)(pvt_box->gSpeed * 0.0036);
		spdi = (int32_t)(spd);
		/*

    chprintf((BaseSequentialStream*)&SD1, "%s;", lat);
    chprintf((BaseSequentialStream*)&SD1, "%s;", lon);
    chprintf((BaseSequentialStream*)&SD1, "%d:", pvt_box->hour);
    chprintf((BaseSequentialStream*)&SD1, "%d:", pvt_box->min);
    chprintf((BaseSequentialStream*)&SD1, "%d;", pvt_box->sec);
    chprintf((BaseSequentialStream*)&SD1, "%d;", pvt_box->numSV);
    chprintf((BaseSequentialStream*)&SD1, "%d",  spdi);
    chprintf((BaseSequentialStream*)&SD1, "\r\n");


    chprintf((BaseSequentialStream*)&SD1, "YEAR: %d\n\r", pvt_box->year);
    chprintf((BaseSequentialStream*)&SD1, "MONT: %d\n\r", pvt_box->month);
    chprintf((BaseSequentialStream*)&SD1, "DAY:  %d\n\r", pvt_box->day);
    chprintf((BaseSequentialStream*)&SD1, "HOUR: %d\n\r", pvt_box->hour);
    chprintf((BaseSequentialStream*)&SD1, "MIN:  %d\n\r", pvt_box->min);
    chprintf((BaseSequentialStream*)&SD1, "SEC:  %d\n\r", pvt_box->sec);
    chprintf((BaseSequentialStream*)&SD1, "LONG: %s\n\r", lon);
    chprintf((BaseSequentialStream*)&SD1, "LATT: %s\n\r", lat);
    chprintf((BaseSequentialStream*)&SD1, "VAL:  %x\n\r", pvt_box->valid);
    chprintf((BaseSequentialStream*)&SD1, "FIXT:  %d\n\r", pvt_box->fixType);
    chprintf((BaseSequentialStream*)&SD1, "SV:   %d\n\r", pvt_box->numSV);
    chprintf((BaseSequentialStream*)&SD1, "SPD:  %d\n\r", spdi);
    chprintf((BaseSequentialStream*)&SD1, "\n\n\r");

		// ftoa(ptr,ax,5);
		//chprintf((BaseSequentialStream*)&SD1, "ACCEL X: %d ACCEL_Y: %d ACCEL_Z: %d \r\nGYRO_X: %d GYRO_Y: %d GYRO_Z: %d \r\nMAG_X: %d MAG_Y: %d MAG_Z: %d \r\n\n",
		//									accel_data[0], accel_data[1], accel_data[2], gyro_data[0], gyro_data[1], gyro_data[2], mag_data[0], mag_data[1], mag_data[2]);
		//chprintf((BaseSequentialStream*)&SD1, "ACCEL X: %d ACCEL_Y: %d ACCEL_Z: %d \r\nGYRO_X: %d GYRO_Y: %d GYRO_Z: %d \r\nMAG_X: %d MAG_Y: %d MAG_Z: %d \r\n\n",
		//   								(int32_t)ax, (int32_t)ay, (int32_t)az, (int32_t)gx, (int32_t)gy, (int32_t)gz, (int32_t)mx, (int32_t)my, (int32_t)mz);
		//chprintf((BaseSequentialStream*)&SD1, "Yaw: %d, Pitch: %d, Roll: %d\n\r", (int32_t)yaw, (int32_t)pitch, (int32_t)roll);
	}
}
*/

/*
 * Thread to process data collection and filtering from NEO-M8P
 */
thread_reference_t coords_trp = NULL;
static THD_WORKING_AREA(coords_thread_wa, 512);
static THD_FUNCTION(coords_thread, arg) {

	(void)arg;
	msg_t msg;
	chRegSetThreadName("GPS Parse Thread");
	while (true) {
		chSysLock();
		if (mpu->suspend_state) {
			msg = chThdSuspendS(&coords_trp);
		}
		chSysUnlock();
		neo_create_poll_request(UBX_NAV_CLASS, UBX_NAV_PVT_ID);
		chThdSleepMilliseconds(50);
		neo_poll();

	}
}

/*
 * Thread to process data collection and filtering from MPU9250
 */
thread_reference_t mpu_trp = NULL;
static THD_WORKING_AREA(mpu_thread_wa, 512);
static THD_FUNCTION(mpu_thread, arg) {

	(void)arg;
	msg_t msg;
	chRegSetThreadName("MPU9250 Thread");
	while (true) {
		chSysLock();
		if (mpu->suspend_state) {
			msg = chThdSuspendS(&mpu_trp);
		}
		chSysUnlock();
		switch(msg){
		case MPU_GET_GYRO_DATA:

			mpu_get_gyro_data();

			break;
		}
	}
}
/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
thread_reference_t shell_trp = NULL;
static THD_WORKING_AREA(shell_thread_wa, 512);
static THD_FUNCTION(shell_thread, arg) {

	(void)arg;
		msg_t msg;
		chRegSetThreadName("Shell Thread");
		while (true) {
			chSysLock();
		//	if (shell->suspend_state) {
				msg = chThdSuspendS(&shell_trp);
		//	}
			chSysUnlock();

			/* Perform processing here.*/
			//chprintf((BaseSequentialStream*)&SD1, "thd_xbee\n\r");
			switch (msg){
			case SHELL_UBX_COG_STATUS:
				neo_create_poll_request(UBX_CFG_CLASS, UBX_CFG_NAV5_ID);
				chThdSleepMilliseconds(100);
				neo_poll();
				chThdSleepMilliseconds(50);
				neo_poll();
				chThdSleepMilliseconds(50);
				break;
			case SHELL_UBX_RATE_STATUS:
				neo_create_poll_request(UBX_CFG_CLASS, UBX_CFG_RATE_ID);
				chThdSleepMilliseconds(50);
				neo_poll();
				chThdSleepMilliseconds(50);
				neo_poll();
				break;
			case SHELL_UBX_RATE_SET:
				neo_create_poll_request(UBX_CFG_CLASS, UBX_CFG_RATE_ID);
					chThdSleepMilliseconds(50);
					neo_poll();
					rate_box->measRate = 250;
					chThdSleepMilliseconds(50);
					neo_write_struct((uint8_t *)rate_box, UBX_CFG_CLASS, UBX_CFG_RATE_ID, sizeof(ubx_cfg_rate_t));
					chThdSleepMilliseconds(50);
					neo_poll();
					chThdSleepMilliseconds(50);
					break;
			default:
				break;
			//xbee->suspend_state = 1;
		}
		}
}

/*
 * Thread that outputs debug data which is needed
 */
thread_reference_t output_trp = NULL;
static THD_WORKING_AREA(output_thread_wa, 1024*2);
static THD_FUNCTION(output_thread, arg) {
	(void)arg;
	int32_t spdi = 0;
	char lon[20];
	char lat[20];
	double spd;
	msg_t msg;
	chRegSetThreadName("Data output Thread");
	while (true) {
		chSysLock();
		if (output->suspend_state) {
			msg = chThdSuspendS(&output_trp);
		}
		chSysUnlock();
		palToggleLine(LINE_GREEN_LED);
		//chprintf((BaseSequentialStream*)&SD1, "output is %d\n\r", output->test);
		if (output->test){
			chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*)&SD1, "Test output\n\r");
			chSemSignal(&usart1_semaph);
		}else{
			if (output->gps){

				itoa(pvt_box->lat, lat, 10);
				itoa(pvt_box->lon, lon, 10);
				insert_dot(lat);
				insert_dot(lon);
				spd = (float)(pvt_box->gSpeed * 0.0036);
				spdi = (int32_t)(spd);
				chSemWait(&usart1_semaph);
				//chprintf((BaseSequentialStream*)&SD1, "GPS output\n\r");
			    chprintf((BaseSequentialStream*)&SD1, "%s;", lat);
			    chprintf((BaseSequentialStream*)&SD1, "%s;", lon);
			    chprintf((BaseSequentialStream*)&SD1, "%d:", pvt_box->hour);
			    chprintf((BaseSequentialStream*)&SD1, "%d:", pvt_box->min);
			    chprintf((BaseSequentialStream*)&SD1, "%d;", pvt_box->sec);
			    chprintf((BaseSequentialStream*)&SD1, "%d;", pvt_box->numSV);
			    chprintf((BaseSequentialStream*)&SD1, "%d",  spdi);
			    chprintf((BaseSequentialStream*)&SD1, "\r\n");
				chSemSignal(&usart1_semaph);
			}else if (output->ypr){
				chSemWait(&usart1_semaph);
				chprintf((BaseSequentialStream*)&SD1, "Yaw: %f, Pitch: %f, Roll: %f\n\r",
															mpu->yaw, mpu->pitch, mpu->roll);
				chSemSignal(&usart1_semaph);

			}else if (output->gyro){
				chSemWait(&usart1_semaph);
				chprintf((BaseSequentialStream*)&SD1, "AX: %d, AY: %d, AZ: %d  GX: %d, GY: %d, GZ: %d  MX: %d, MY: %d, MZ: %d\r\n",
						mpu->accelCount[0], mpu->accelCount[1], mpu->accelCount[2],
						mpu->gyroCount[0], mpu->gyroCount[1], mpu->gyroCount[2],
						mpu->magCount[0], mpu->magCount[1], mpu->magCount[2]);
				chSemSignal(&usart1_semaph);
			}
		}
		output->suspend_state = 1;

	}
}

static void gpt9cb(GPTDriver *gptp){
	(void)gptp;

		chSysLockFromISR();
		chThdResumeI(&xbee_poll_trp, (msg_t)MPU_GET_GYRO_DATA);  /* Resuming the thread with message.*/
		chSysUnlockFromISR();

}

static void gpt11cb(GPTDriver *gptp){
	(void)gptp;

		chSysLockFromISR();
		chThdResumeI(&mpu_trp, (msg_t)MPU_GET_GYRO_DATA);  /* Resuming the thread with message.*/
		chSysUnlockFromISR();

}

static void gpt12cb(GPTDriver *gptp){
	(void)gptp;

		chSysLockFromISR();
		chThdResumeI(&coords_trp, (msg_t)0x1137);  /* Resuming the thread with message.*/
		chSysUnlockFromISR();

}

/*
 * GPT14  callback.
 */
static void gpt14cb(GPTDriver *gptp)
{
	(void)gptp;

	chSysLockFromISR();
	chThdResumeI(&output_trp, (msg_t)0x1137);  /* Resuming the thread with message.*/
	chSysUnlockFromISR();
}


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
		chThdSleepMilliseconds(500);
		palClearLine(LINE_ORANGE_LED);
		chThdSleepMilliseconds(500);
	}
}

void init_modules(void){
	chSysLock();
	neo_switch_to_ubx();
	chThdSleepMilliseconds(50);
	neo_poll();


	chThdSleepMilliseconds(50);
	neo_set_pvt_1hz();
	chThdSleepMilliseconds(50);
	neo_poll();
	chThdSleepMilliseconds(50);
	neo_create_poll_request(UBX_CFG_CLASS, UBX_CFG_RATE_ID);
	chThdSleepMilliseconds(50);
	neo_poll();
	rate_box->measRate = 250;
	chThdSleepMilliseconds(50);
	neo_write_struct((uint8_t *)rate_box, UBX_CFG_CLASS, UBX_CFG_RATE_ID, sizeof(ubx_cfg_rate_t));
	chThdSleepMilliseconds(50);
	neo_poll();
	chThdSleepMilliseconds(50);
	neo_create_poll_request(UBX_CFG_CLASS, UBX_CFG_RATE_ID);
	chThdSleepMilliseconds(50);
	neo_poll();
	chThdSleepMilliseconds(100);

	neo_create_poll_request(UBX_CFG_CLASS, UBX_CFG_NAV5_ID);
	chThdSleepMilliseconds(100);
	neo_poll();
	chThdSleepMilliseconds(50);
	neo_poll();
	chThdSleepMilliseconds(50);
	//neo_write_struct((uint8_t *)nav5_box, UBX_CFG_CLASS, UBX_CFG_NAV5_ID, sizeof(ubx_cfg_nav5_t));
	//chThdSleepMilliseconds(50);
	//neo_poll();
	chSysUnlock();
}

/*
 * Application entry point.
 */
int main(void) {
	thread_t *sh = NULL;
	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	chSysInit();
	chSemObjectInit(&usart1_semaph, 1);
	chSemObjectInit(&spi2_semaph, 1);
	/*
	 * Activates the serial driver 1 using the driver default configuration.
	 */
	sdStart(&SD1, NULL);
	spiStart(&SPID1, &spi1_cfg);
	spiStart(&SPID2, &neo_spi_cfg);

	xbee->suspend_state = 1;

	/*
     B* Shell manager initialization.
	 */
#ifdef USE_SD_SHELL

	sdStart(&SD1, NULL);
	shellInit();
	chSemWait(&usart1_semaph);
	sh = cmd_init();
	chSemSignal(&usart1_semaph);
#else
	sdStart(&SD1, NULL);
#endif


	mpu9250_init();
	chThdSleepMilliseconds(100);

	chThdSleepMilliseconds(100);


	initAK8963(&mpu->magCalibration[0]);
	chSemWait(&usart1_semaph);
	chprintf((BaseSequentialStream*)&SD1, "MAG_Calibration: %f, %f, %f\r\n", mpu->magCalibration[0], mpu->magCalibration[1], mpu->magCalibration[2]);
	chSemSignal(&usart1_semaph);

	output->suspend_state = 1;
	xbee->suspend_state = 1;
	xbee->poll_suspend_state = 1;
	neo->suspend_state = 1;
	mpu->suspend_state = 1;
	// set up the timer
	gptStart(&GPTD9, &gpt9cfg);
	gptStart(&GPTD11, &gpt11cfg);
	gptStart(&GPTD12, &gpt12cfg);
	gptStart(&GPTD14, &gpt14cfg);
	//chSysLock();
	//palSetLine(LINE_ORANGE_LED);
	/*
	 * Creates threads.
	 */

	chThdCreateStatic(xbee_thread_wa, sizeof(xbee_thread_wa), NORMALPRIO + 1, xbee_thread, NULL);
	chThdCreateStatic(xbee_poll_thread_wa, sizeof(xbee_poll_thread_wa), NORMALPRIO + 2, xbee_poll_thread, NULL);
	chThdCreateStatic(shell_thread_wa, sizeof(shell_thread_wa), NORMALPRIO + 3, shell_thread, NULL);
	chThdCreateStatic(output_thread_wa, sizeof(output_thread_wa), NORMALPRIO + 3, output_thread, NULL);
	chThdCreateStatic(mpu_thread_wa, sizeof(mpu_thread_wa), HIGHPRIO, mpu_thread, NULL);
	chThdCreateStatic(coords_thread_wa, sizeof(coords_thread_wa), NORMALPRIO + 4, coords_thread, NULL);
	palEnableLineEventI(LINE_RF_868_SPI_ATTN, PAL_EVENT_MODE_FALLING_EDGE);
	palSetLineCallbackI(LINE_RF_868_SPI_ATTN, xbee_attn_event, NULL);

	//chSysLock();
	gptStartContinuous(&GPTD9, 2000);
	chThdSleepMilliseconds(100);
	gptStartContinuous(&GPTD11, 50);
	chThdSleepMilliseconds(100);
	gptStartContinuous(&GPTD12, 5000);
	chThdSleepMilliseconds(100);
	gptStartContinuous(&GPTD14, 5000);
	//chSysUnlock();
	//xbee_thread_execute(XBEE_GET_OWN_ADDR);


	//chprintf((BaseSequentialStream*)&SD1, "Init\n\r");
	chThdSleepMilliseconds(3000);
	//xbee_thread_execute(XBEE_GET_PACKET_PAYLOAD);
	chSysLock();
		//init_modules();
		chSysUnlock();
	neo_switch_to_ubx();
	chThdSleepMilliseconds(50);
	// configure the timer to fire after 25 timer clock tics
	//   The clock is running at 200,000Hz, so each tick is 50uS,
	//   so 200,000 / 25 = 8,000Hz


	/*
	 * Normal main() thread activity, in this demo it does nothing except
	 * sleeping in a loop and check the button state.
	 */
	while (true) {

		if (!sh)
		      sh = cmd_init();
		    else if (chThdTerminatedX(sh)) {
		      chThdRelease(sh);
		      sh = NULL;
		    }
		//chThdWait(shelltp);               /* Waiting termination.*/
		chThdSleepMilliseconds(1000);

	}
}
