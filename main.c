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
#include "quaternionFilters.h"
#include "chprintf.h"
#include "neo-m8.h"
float PI = CONST_PI;
float GyroMeasError = CONST_GME; // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = CONST_beta;  // compute beta
float GyroMeasDrift = CONST_GMD; // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = CONST_zeta; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
int16_t accel_data[3];
int16_t gyro_data[3];
int16_t mag_data[3];
float calib[3];
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
float temperature;
float SelfTest[6];
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float pitch, yaw, roll;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

nav_pvt_t pvt;
nav_pvt_t *pvt_box = &pvt;
xbee_struct_t xbee_struct;
xbee_struct_t *xbee = &xbee_struct;

#define MAX_FILLER 11
#define FLOAT_PRECISION 9
static const long pow10[FLOAT_PRECISION] = {
    10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000
};


void insert_dot(char *str){
	uint8_t len = strlen(str);
	uint8_t str2[20];
	str2[0] = str[0];
	str2[1] = str[1];
	str2[2] = '.';
	memcpy(&str2[3], &str[2], 7);
	memcpy(str, str2, 8);
}

/*
 * GPT14  callback.
 */
static void gpt14cb(GPTDriver *gptp)
{
	(void)gptp;
	 palToggleLine(LINE_GREEN_LED);


    /* perform some function that needs to be done on a regular basis */
}

static GPTConfig gpt14cfg =
{
  20000,      // Timer clock
  gpt14cb        // Callback function
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

static const SPIConfig spi2_cfg = {
  false,
  NULL,
  GPIOC,
  GPIOC_MCU_CS,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0,
  //0,
  0
};

thread_reference_t trp = NULL;

static THD_WORKING_AREA(xbee_thread_wa, 128);
static THD_FUNCTION(xbee_thread, p){
	(void)p;
	msg_t msg;
    uint8_t at[] = {'S', 'L'};
    uint8_t rxbuf[15];
	 while (true) {

		chSysLock();
		   if (xbee->suspend_state) {
		       	msg = chThdSuspendS(&trp);
		    }
		chSysUnlock();

	    /* Perform processing here.*/
	    chprintf((BaseSequentialStream*)&SD1, "thd_xbee\n\r");
	    switch (msg){
	    case XBEE_GET_OWN_ADDR:
	    	xbee_read_own_addr(xbee);
 		    break;
	    case XBEE_GET_RSSI:
	    	xbee->rssi = xbee_read_last_rssi(xbee);
	    	chprintf((BaseSequentialStream*)&SD1, "RSSI: %d\r\n", xbee->rssi);
	    	break;
	    case XBEE_GET_PACKET_PAYLOAD:
	    	xbee->packet_payload = xbee_get_packet_payload(xbee);
	    	chprintf((BaseSequentialStream*)&SD1, "Packet payload: %d\r\n", xbee->packet_payload);
	    	break;
	    case XBEE_GET_STAT:
	    	xbee->bytes_transmitted = xbee_get_bytes_transmitted(xbee);
	    	xbee->good_packs_res = xbee_get_good_packets_res(xbee);
	    	xbee->rec_err_count = xbee_get_received_err_count(xbee);
	    	xbee->trans_errs = xbee_get_transceived_err_count(xbee);
	    	xbee->unicast_trans_count = xbee_get_unicast_trans_count(xbee);
	    	xbee->rssi = xbee_read_last_rssi(xbee);
	    	chprintf((BaseSequentialStream*)&SD1, "Bytes transmitted:     %d\r\n", xbee->bytes_transmitted);
	    	chprintf((BaseSequentialStream*)&SD1, "Good packets received: %d\r\n", xbee->good_packs_res);
	    	chprintf((BaseSequentialStream*)&SD1, "Received errors count: %d\r\n", xbee->rec_err_count);
	    	chprintf((BaseSequentialStream*)&SD1, "Transceiver errors:    %d\r\n", xbee->trans_errs);
	    	chprintf((BaseSequentialStream*)&SD1, "Unicast transmittions: %d\r\n", xbee->unicast_trans_count);
	    	chprintf((BaseSequentialStream*)&SD1, "RSSI:                  %d\r\n", xbee->rssi);
	    	break;
	    }

	    xbee->suspend_state = 1;
  }
}

/*
 * This is a thread that serve SPI1 bus with Xbee module
 * and second CPU
 */

static THD_WORKING_AREA(spi1_thread_wa, 512);
static THD_FUNCTION(spi1_thread, p){
	(void)p;
	uint8_t i;

	chRegSetThreadName("spi1_thread");
	while(true){
		chThdSleepMilliseconds(1000);
	}
}


static THD_WORKING_AREA(spi2_thread_wa, 1024);
static THD_FUNCTION(spi2_thread, p) {
	char *ptr;
	float deltat = 0.2f;
	uint8_t rxbuf[200];
	int32_t spdi = 0;
	char lon[20];
	char lat[20];
	double spd;
  (void)p;

  chRegSetThreadName("SPI thread 1");
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
*/
    /*
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
*/
    //mpu_read_accel_data(&accelCount[0]);
    //mpu_read_gyro_data(&gyroCount[0]);
    //mpu_read_mag_data(&magCount[0]);
/*
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];
    az = (float)accelCount[2]*aRes - accelBias[2];

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes - gyroBias[1];
    gz = (float)gyroCount[2]*gRes - gyroBias[2];

    mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];
    mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];

    MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz, deltat);
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;
    yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll  *= 180.0f / PI;
    chThdSleepMilliseconds(100); */
   // ftoa(ptr,ax,5);
   //chprintf((BaseSequentialStream*)&SD1, "ACCEL X: %d ACCEL_Y: %d ACCEL_Z: %d \r\nGYRO_X: %d GYRO_Y: %d GYRO_Z: %d \r\nMAG_X: %d MAG_Y: %d MAG_Z: %d \r\n\n",
    //									accel_data[0], accel_data[1], accel_data[2], gyro_data[0], gyro_data[1], gyro_data[2], mag_data[0], mag_data[1], mag_data[2]);
    //chprintf((BaseSequentialStream*)&SD1, "ACCEL X: %d ACCEL_Y: %d ACCEL_Z: %d \r\nGYRO_X: %d GYRO_Y: %d GYRO_Z: %d \r\nMAG_X: %d MAG_Y: %d MAG_Z: %d \r\n\n",
     //   								(int32_t)ax, (int32_t)ay, (int32_t)az, (int32_t)gx, (int32_t)gy, (int32_t)gz, (int32_t)mx, (int32_t)my, (int32_t)mz);
    //chprintf((BaseSequentialStream*)&SD1, "Yaw: %d, Pitch: %d, Roll: %d\n\r", (int32_t)yaw, (int32_t)pitch, (int32_t)roll);
  }
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
  spiStart(&SPID1, &spi1_cfg);
  spiStart(&SPID2, &spi2_cfg);

  xbee->suspend_state = 1;

  /*
     * Shell manager initialization.
     */
  #ifdef USE_SD_SHELL

    sdStart(&SD1, NULL);
    shellInit();
  #else
    sdStart(&SD1, NULL);
    //shellInit();
  #endif

   // calibrateMPU9250(gyroBias, accelBias);
  //	mpu9250_init();
  	chThdSleepMilliseconds(100);


  	getAres(); // Get accelerometer sensitivity
  	getGres(); // Get gyro sensitivity
  	getMres(); // Get magnetometer sensitivity
  	magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
  	magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
  	magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
  	//initAK8963(&calib[0]);

  	palToggleLine(LINE_GREEN_LED);

    // set up the timer
//    gptStart(&GPTD14, &gpt14cfg);
    neo_switch_to_ubx();
      chThdSleepMilliseconds(1000);
      neo_set_pvt_1hz();
  /*
   * Creates threads.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  chThdCreateStatic(xbee_thread_wa, sizeof(xbee_thread_wa), NORMALPRIO + 1, xbee_thread, NULL);
  chThdCreateStatic(spi1_thread_wa, sizeof(spi1_thread_wa), NORMALPRIO + 1, spi1_thread, NULL);
  chThdCreateStatic(spi2_thread_wa, sizeof(spi2_thread_wa), NORMALPRIO + 2, spi2_thread, NULL);


  xbee_thread_execute(XBEE_GET_OWN_ADDR);



  //neo_switch_to_ubx();


  //chprintf((BaseSequentialStream*)&SD1, "Init\n\r");
  chThdSleepMilliseconds(1000);
  xbee_thread_execute(XBEE_GET_PACKET_PAYLOAD);

  // configure the timer to fire after 25 timer clock tics
    //   The clock is running at 200,000Hz, so each tick is 50uS,
    //   so 200,000 / 25 = 8,000Hz
   // gptStartContinuous(&GPTD14, 20000);
  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
	  thread_t *shelltp = cmd_init();
	      chThdWait(shelltp);               /* Waiting termination.*/
	  chThdSleepMilliseconds(5000);

  }
}
