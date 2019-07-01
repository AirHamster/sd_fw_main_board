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

//#define TRAINER_MODULE

uint8_t payload[256];
uint8_t read_pvt = 1;
extern ubx_nav_pvt_t *pvt_box;
extern ubx_cfg_rate_t *rate_box;
extern ubx_cfg_nav5_t *nav5_box;
extern ubx_cfg_odo_t *cfg_odo_box;
extern ubx_nav_odo_t *odo_box;
extern xbee_struct_t *xbee;
extern neo_struct_t *neo;
extern mpu_struct_t *mpu;
extern tx_box_t *tx_box;

extern output_struct_t *output;
struct ch_semaphore usart1_semaph;
struct ch_semaphore spi2_semaph;
extern float calib[];
extern const I2CConfig i2ccfg;
#define MAX_FILLER 11
#define FLOAT_PRECISION 9

void send_data(uint8_t stream);
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

/*
 * Watchdog deadline set to more than one second (LSI=40000 / (64 * 1000)).
 */
/*static const WDGConfig wdgcfg = {
  STM32_IWDG_PR_64,
  STM32_IWDG_RL(512),
  STM32_IWDG_WIN_DISABLED
};
*/
static SerialConfig sd7cfg =
{
		115200
};

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
		25600,      // Timer clock
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
		SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0,	//FPCLK1 is 54 MHZ. XBEE support 3.5 max, so divide it by 16
		//  0,
		0
};

const SPIConfig neo_spi_cfg = {
		false,
		NULL,
		GPIOC,
		GPIOC_MCU_CS,
		SPI_CR1_BR_1 | SPI_CR1_BR_0,
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
	chRegSetThreadName("XBee polling thd");
	gptStop(&GPTD9);
	gptStart(&GPTD9, &gpt9cfg);
//	gptStartContinuous(&GPTD9, 2000);
	while (true){
		chSysLock();
		if (xbee->poll_suspend_state) {
			msg = chThdSuspendS(&xbee_poll_trp);
		}
		chSysUnlock();
		//palToggleLine(LINE_RED_LED);
		while(!palReadLine(LINE_RF_868_SPI_ATTN)){
			xbee_polling();
		}
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
			break;
		case XBEE_GET_CHANNELS:
			xbee->channels = xbee_read_channels(xbee);
			chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*)&SD1, "Channels settings: %x\r\n", xbee->channels);
			chSemSignal(&usart1_semaph);
			break;
		}


		xbee->suspend_state = 1;
	}
}

/*
 * Thread to process data collection and filtering from NEO-M8P
 */
thread_reference_t coords_trp = NULL;
static THD_WORKING_AREA(coords_thread_wa, 4096);
static THD_FUNCTION(coords_thread, arg) {

	(void)arg;
	msg_t msg;
	chRegSetThreadName("GPS Parse");
	gptStop(&GPTD12);
#ifndef TRAINER_MODULE
	gptStart(&GPTD12, &gpt12cfg);
	gptStartContinuous(&GPTD12, 10000);
#endif
	while (true) {
		chSysLock();
		if (neo->suspend_state) {
			msg = chThdSuspendS(&coords_trp);
		}
		chSysUnlock();
		if (read_pvt == 1){
			chSemWait(&spi2_semaph);
			neo_create_poll_request(UBX_NAV_CLASS, UBX_NAV_PVT_ID);
					chThdSleepMilliseconds(5);
					neo_poll();
					chSemSignal(&spi2_semaph);
					read_pvt = 0;
		}else{
			chSemWait(&spi2_semaph);
			neo_create_poll_request(UBX_NAV_CLASS, UBX_NAV_ODO_ID);
					chThdSleepMilliseconds(5);
					neo_poll();
					chSemSignal(&spi2_semaph);
					read_pvt = 1;
		}
		chThdSleepMilliseconds(25);

		palToggleLine(LINE_RED_LED);
		neo->suspend_state = 1;

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
	gptStop(&GPTD11);
#ifndef TRAINER_MODULE
	gptStart(&GPTD11, &gpt11cfg);
	gptStartContinuous(&GPTD11, 200);
#endif
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
 */msg_t msg;
thread_reference_t shell_trp = NULL;
static THD_WORKING_AREA(shell_thread_wa, 512);
static THD_FUNCTION(shell_thread, arg) {

	(void)arg;
	msg_t msg;
	chRegSetThreadName("Shell Thread");
	while (true) {
		chSysLock();
		msg = chThdSuspendS(&shell_trp);
		chSysUnlock();

		/* Perform processing here.*/
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
		}
	}
}

/*
 * Thread that works with UBLOX NINA Bluetooth
 */
thread_reference_t bt_trp = NULL;
static THD_WORKING_AREA(bt_thread_wa, 256);
static THD_FUNCTION(bt_thread, arg){
	(void)arg;
	msg_t msg;
	chRegSetThreadName("BT Thd");
	while (true) {
		chSysLock();
		if (output->suspend_state) {
			msg = chThdSuspendS(&bt_trp);
		}
		chSysUnlock();
	/*	switch (msg){
		case NINA_GET_DISCOVERABLE:
			nina_get_discoverable_status();
			break;
*/
		/*	event_listener_t elSerData;
			eventmask_t flags;
			chEvtRegisterMask((EventSource *)chnGetEventSource(&SD7), &elSerData, EVENT_MASK(1));

			while (TRUE)
			{
				chEvtWaitOneTimeout(EVENT_MASK(1), MS2ST(10));
				chSysLock();
				flags = chEvtGetAndClearFlags(&elSerData);
				chSysUnlock();
				if (flags & CHN_INPUT_AVAILABLE)
				{
					msg_t charbuf;
					do
					{
						charbuf = chnGetTimeout(&SD1, TIME_IMMEDIATE);
						if ( charbuf != Q_TIMEOUT )
						{
							chSequentialStreamPut(&SD1, charbuf);
						}
					}
					while (charbuf != Q_TIMEOUT);
				}
			}

		}*/
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
	chRegSetThreadName("Data output");
	gptStop(&GPTD14);
	gptStart(&GPTD14, &gpt14cfg);
	gptStartContinuous(&GPTD14, 5000);
	while (true) {
		chSysLock();
		if (output->suspend_state) {
			msg = chThdSuspendS(&output_trp);
		}
		chSysUnlock();
		palToggleLine(LINE_GREEN_LED);
		//chprintf((BaseSequentialStream*)&SD1, "output is %d\n\r", output->test);
		if (output->test){
			//chSemWait(&usart1_semaph);
			//chprintf((BaseSequentialStream*)&SD1, "Test output\n\r");
			//chSemSignal(&usart1_semaph);
			send_data(OUTPUT_XBEE);
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
				chprintf((BaseSequentialStream*)&SD1, "%s,", lat);
				chprintf((BaseSequentialStream*)&SD1, "%s,", lon);
				chprintf((BaseSequentialStream*)&SD1, "%d,", pvt_box->hour);
				chprintf((BaseSequentialStream*)&SD1, "%d,", pvt_box->min);
				chprintf((BaseSequentialStream*)&SD1, "%d,", pvt_box->sec);
				chprintf((BaseSequentialStream*)&SD1, "%d,", pvt_box->numSV);
				chprintf((BaseSequentialStream*)&SD1, "%d",  spdi);
				chprintf((BaseSequentialStream*)&SD1, "\r\n");
				chSemSignal(&usart1_semaph);
			}else if (output->ypr){
				chSemWait(&usart1_semaph);
				chprintf((BaseSequentialStream*)&SD1, "Yaw: %f, Pitch: %f, Roll: %f\n\r",
						mpu->yaw, mpu->pitch, mpu->roll);
				//chprintf((BaseSequentialStream*)&SD1, "%f;%f;%f\n\r", mpu->mx, mpu->my, mpu->mz);
				chSemSignal(&usart1_semaph);
			}else if (output->gyro){
				chSemWait(&usart1_semaph);
				/*chprintf((BaseSequentialStream*)&SD1, "AX: %d, AY: %d, AZ: %d  GX: %d, GY: %d, GZ: %d  MX: %d, MY: %d, MZ: %d\r\n",
						mpu->accelCount[0], mpu->accelCount[1], mpu->accelCount[2],
						mpu->gyroCount[0], mpu->gyroCount[1], mpu->gyroCount[2],
						mpu->magCount[0], mpu->magCount[1], mpu->magCount[2]);*/
				chprintf((BaseSequentialStream*)&SD1, "AX: %f, AY: %f, AZ: %f  GX: %f, GY: %f, GZ: %f  MX: %f, MY: %f, MZ: %f\r\n",
										mpu->ax, mpu->ay, mpu->az,
										mpu->gx, mpu->gy, mpu->gz,
										mpu->mx, mpu->my, mpu->mz);
				chSemSignal(&usart1_semaph);
			}else if(output->xbee){
				send_data(OUTPUT_XBEE);
			}
		}
		output->suspend_state = 1;

	}
}

void send_data(uint8_t stream){
	uint8_t databuff[23];
	int32_t spdi = 0;
	double spd;
	double dlat, dlon;
	spd = (float)(pvt_box->gSpeed * 0.0036);
	spdi = (int32_t)(spd);
	//tx_box->lat_cel = (int16_t)pvt_box->lat;
	//modf(pvt_box->lat, &dlat);
	tx_box->lat = pvt_box->lat;
	//tx_box->lat_drob = (uint16_t)((pvt_box->lat - tx_box->lat_cel)*10000000);
	tx_box->lon = pvt_box->lon;
	//modf(pvt_box->lon, &dlon);
	//tx_box->lon_cel = (int16_t)dlon;
	//tx_box->lon_drob = (uint16_t)((pvt_box->lon - tx_box->lon_cel)*10000000);
	tx_box->hour = pvt_box->hour;
	tx_box->min = pvt_box->min;
	tx_box->sec = pvt_box->sec;
	tx_box->dist = (uint16_t)odo_box->distance;
	tx_box->sat = pvt_box->numSV;
	tx_box->speed = spdi;
	tx_box->headMot = pvt_box->headMot;
	tx_box->headVeh = pvt_box->headVeh;
	//if(stream == OUTPUT_USART){
		//chSemWait(&usart1_semaph);
	/*	chprintf((BaseSequentialStream*)&SD1, "%d.%d;", tx_box->lat_cel, tx_box->lat_drob);
		    chprintf((BaseSequentialStream*)&SD1, "%d.%d;", tx_box->lon_cel, tx_box->lon_drob);
		    chprintf((BaseSequentialStream*)&SD1, "%d:", tx_box->hour);
		    chprintf((BaseSequentialStream*)&SD1, "%d:", tx_box->min);
		    chprintf((BaseSequentialStream*)&SD1, "%d;", tx_box->sec);
		    chprintf((BaseSequentialStream*)&SD1, "%d;", tx_box->sat);
		    chprintf((BaseSequentialStream*)&SD1, "%d",  tx_box->dist);
		    chprintf((BaseSequentialStream*)&SD1, "%d",  tx_box->speed);
		    chprintf((BaseSequentialStream*)&SD1, "\r\n");
		*/
	//	chprintf((BaseSequentialStream*)&SD1, "%d;%d;%d:%d:%d:%d:%d:%d:\r\n",
		//		tx_box->lat, tx_box->lon, tx_box->hour,
		//		tx_box->min, tx_box->sec, tx_box->sat, tx_box->dist, tx_box->speed);
		//chSemSignal(&usart1_semaph);
//	}else if (stream == OUTPUT_XBEE){
	//	wdgReset(&WDGD1);
		//memcpy(&tx_box->lat, &databuff[0], sizeof(float));
		databuff[0] = RF_GPS_PACKET;
		databuff[1] = (uint8_t)(tx_box->lat >> 24);
		databuff[2] = (uint8_t)(tx_box->lat >> 16 );
		databuff[3] = (uint8_t)(tx_box->lat >> 8);
		databuff[4] = (uint8_t)(tx_box->lat);
		//memcpy(&tx_box->lon, &databuff[4], sizeof(float));
		databuff[5] = (uint8_t)(tx_box->lon >> 24);
		databuff[6] = (uint8_t)(tx_box->lon >> 16);
		databuff[7] = (uint8_t)(tx_box->lon >> 8);
		databuff[8] = (uint8_t)(tx_box->lon);
		databuff[9] = tx_box->hour;
		databuff[10] = tx_box->min;
		databuff[11] = tx_box->sec;
		databuff[12] = tx_box->sat;
		databuff[13] = (uint8_t)(tx_box->dist >> 8);
		databuff[14] = (uint8_t)(tx_box->dist);
		databuff[15] = (uint8_t)(tx_box->speed);
		databuff[16] = (uint8_t)(tx_box->yaw >> 8);
		databuff[17] = (uint8_t)(tx_box->yaw);
		databuff[18] = (uint8_t)(tx_box->pitch >> 8);
		databuff[19] = (uint8_t)(tx_box->pitch);
		databuff[20] = (uint8_t)(tx_box->roll >> 8);
		databuff[21] = (uint8_t)(tx_box->roll);
		databuff[22] = tx_box->bat;

		databuff[23] = (uint8_t)(tx_box->headMot >> 24);
		databuff[24] = (uint8_t)(tx_box->headMot >> 16);
		databuff[25] = (uint8_t)(tx_box->headMot >> 8);
		databuff[26] = (uint8_t)(tx_box->headMot);

		databuff[27] = (uint8_t)(tx_box->headVeh >> 24);
		databuff[28] = (uint8_t)(tx_box->headVeh >> 16);
		databuff[29] = (uint8_t)(tx_box->headVeh >> 8);
		databuff[30] = (uint8_t)(tx_box->headVeh);

		xbee_send_rf_message(xbee, databuff, 31);
	//}
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
	chThdResumeI(&coords_trp, (msg_t)0x01);  /* Resuming the thread with message.*/
	chSysUnlockFromISR();

}

/*
 * GPT14  callback.
 */
static void gpt14cb(GPTDriver *gptp)
{
	(void)gptp;

	chSysLockFromISR();
	chThdResumeI(&output_trp, (msg_t)0x01);  /* Resuming the thread with message.*/
	chSysUnlockFromISR();
}

void init_modules(void){
	chSysLock();
	neo_switch_to_ubx();
	chThdSleepMilliseconds(50);
	neo_poll();


	chThdSleepMilliseconds(50);
	//neo_set_pvt_1hz();
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

extern void chSysHalt(const char*);
void _unhandled_exception(void) {
  //chSysHalt("UNDEFINED IRQ");
	return;
}

/*
 * Application entry point.
 */
int main(void) {
	thread_t *sh = NULL;
	float mag_scaling[3];
	float mag_offset[3];
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

	spiStart(&SPID1, &spi1_cfg);
	spiStart(&SPID2, &neo_spi_cfg);
	//i2cStart(&I2CD1, &i2ccfg);
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
	//sdStart(&SD7, &sd7cfg);
	//chprintf((BaseSequentialStream*)&SD7, "AT+CPWROFF\r");
	mpu9250_init();

	chThdSleepMilliseconds(100);


	initAK8963(&mpu->magCalibration[0]);
	chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*)&SD1, "MPU\r\n");
			chSemSignal(&usart1_semaph);


	output->suspend_state = 1;
	xbee->suspend_state = 1;
	xbee->poll_suspend_state = 1;
	xbee->tx_ready = 1;
	neo->suspend_state = 1;
	mpu->suspend_state = 1;

	palClearLine(LINE_RF_868_RST);
	chThdSleepMilliseconds(100);
	palSetLine(LINE_RF_868_RST);
	  /*
	   * Starting the watchdog driver.
	   */
	//wdgStart(&WDGD1, &wdgcfg);

	// set up the timer




	//wdgReset(&WDGD1);

	/*
	 * Creates threads.
	 */
	chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*)&SD1, "THD\r\n");
		chSemSignal(&usart1_semaph);

		neo_switch_to_ubx();
				chThdSleepMilliseconds(50);
			//	neo_set_pvt_1hz();
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

					neo_create_poll_request(UBX_CFG_CLASS, UBX_CFG_ODO_ID);
					chThdSleepMilliseconds(50);
					neo_poll();
					cfg_odo_box->flags = 1 << 0;
					chThdSleepMilliseconds(50);
					neo_write_struct((uint8_t *)cfg_odo_box, UBX_CFG_CLASS, UBX_CFG_ODO_ID, sizeof(ubx_cfg_odo_t));
					chThdSleepMilliseconds(50);
					neo_poll();

	chThdCreateStatic(xbee_thread_wa, sizeof(xbee_thread_wa), NORMALPRIO + 1, xbee_thread, NULL);
	chThdCreateStatic(xbee_poll_thread_wa, sizeof(xbee_poll_thread_wa), NORMALPRIO + 6, xbee_poll_thread, NULL);
	chThdCreateStatic(shell_thread_wa, sizeof(shell_thread_wa), NORMALPRIO + 3, shell_thread, NULL);
	chThdCreateStatic(output_thread_wa, sizeof(output_thread_wa), NORMALPRIO + 3, output_thread, NULL);
	chThdCreateStatic(coords_thread_wa, sizeof(coords_thread_wa), NORMALPRIO + 5, coords_thread, NULL);
	chThdCreateStatic(mpu_thread_wa, sizeof(mpu_thread_wa), NORMALPRIO + 4, mpu_thread, NULL);

	palEnableLineEventI(LINE_RF_868_SPI_ATTN, PAL_EVENT_MODE_FALLING_EDGE);
	palSetLineCallbackI(LINE_RF_868_SPI_ATTN, xbee_attn_event, NULL);

	chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*)&SD1, "MAG_Calibration: %f, %f, %f\r\n", mpu->magCalibration[0], mpu->magCalibration[1], mpu->magCalibration[2]);
		chSemSignal(&usart1_semaph);
/*
	neo_switch_to_ubx();
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
*/

	//	xbee_get_attn_pin_cfg(xbee);

	//	chThdSleepMilliseconds(300);
	//	chprintf((BaseSequentialStream*)&SD7, "AT+UBTDM?\r");

		xbee_set_10kbs_rate();
		//eeprom_write_hw_version();
		chThdSleepMilliseconds(100);
		//eeprom_read_hw_version();
		//xbee_read_baudrate(xbee);
		//chThdSleepMilliseconds(100);
	//	xbee_read_channels(xbee);
//	chThdSleepMilliseconds(3000);
		// configure the timer to fire after 25 timer clock tics
	//   The clock is running at 200,000Hz, so each tick is 50uS,
	//   so 200,000 / 25 = 8,000Hz
		//chThdSleepMilliseconds(1000);
		//mag_calibration(&mag_offset[0], &mag_scaling[0]);
		toggle_test_output();
		//toggle_ypr_output();
		//toggle_gyro_output();
	/*
	 * Normal main() thread activity, in this demo it does nothing except
	 * sleeping in a loop and check the button state.
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
		//chThdWait(shelltp);               /* Waiting termination.*/
		chThdSleepMilliseconds(1000);

	}
}
