
/*
 * config.h
 *
 *  Created on: Aug 12, 2019
 *      Author: a-h
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define USE_DEBUG_SHELL
#define USE_SD_SHELL
//#define USE_MPU_9250_MODULE
#define USE_UBLOX_GPS_MODULE
//#define USE_IRIDIUM_9603_MODULE
//#define USE_XBEE_868_MODULE
//#define USE_EEPROM_MODULE
#define USE_BNO055_MODULE
#define USE_MICROSD_MODULE
#define USE_WINDSENSOR_MODULE
#define SHELL_CONFIG_FILE
#define USE_BLE_MODULE


#define GPS_IF		SPID4
#define GYRO_IF		I2CD2
#define MICROSD_IF	SPID3
#define BLE_IF		SD7
#define WIND_IF		UARTD8
#endif /* CONFIG_H_ */
