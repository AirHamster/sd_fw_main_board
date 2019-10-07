
/*
 * config.h
 *
 *  Created on: Aug 12, 2019
 *      Author: a-h
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define SD_MODULE_TRAINER
#define SHELL_CONFIG_FILE
#define USE_DEBUG_SHELL
#define USE_SD_SHELL
#define USE_SERVICE_MODE
//#define USE_MPU_9250_MODULE
#define USE_UBLOX_GPS_MODULE
//#define USE_IRIDIUM_9603_MODULE
//#define USE_XBEE_868_MODULE
#define USE_EEPROM_MODULE
#define USE_HMC5883_MODULE
#define USE_BNO055_MODULE
#define USE_MICROSD_MODULE
#define USE_WINDSENSOR_MODULE
#define SHELL_CONFIG_FILE
#define USE_BLE_MODULE
#define USE_MATH_MODULE

#define LAG_ADDR		CCF95781688F
#define RUDDER_ADDR		CCF957816647


/**
 * @brief   Magic number for jumping to bootloader.
 */

#define SYMVAL(sym) (uint32_t)(((uint8_t *)&(sym)) - ((uint8_t *)0))
//#if !defined(MAGIC_BOOTLOADER_NUMBER) || defined(__DOXYGEN__)
#define MAGIC_BOOTLOADER_NUMBER 0xDEADBEEF
//#endif

#define SHELL_SD		SD1
#define SHELL_IFACE		(BaseSequentialStream*)&SD1
#define EEPROM_IF		I2CD1
#define GPS_IF			SPID4
#define GYRO_IF			I2CD2
#define MICROSD_IF		SPID3
#define BLE_IF			SD7
#define NINA_IF			SD7
#define NINA_IFACE		(BaseSequentialStream*)&SD7
#define WIND_IF		UARTD8
#endif /* CONFIG_H_ */
