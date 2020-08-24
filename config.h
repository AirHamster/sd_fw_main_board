
/*
 * config.h
 *
 *  Created on: Aug 12, 2019
 *      Author: a-h
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define SD_MAIN_MODULE
//#define SD_MODULE_TRAINER
//#define SD_MODULE_BUOY
#define SD_MODULE_SPORTSMAN

//#define SPORTSMAN_1
#define SPORTSMAN_2

//Common modules
#define SHELL_CONFIG_FILE
#define USE_DEBUG_SHELL
#define USE_SD_SHELL
#define USE_SERVICE_MODE
#define USE_UBLOX_GPS_MODULE
#define USE_XBEE_MODULE
#define USE_EEPROM_MODULE
#define USE_MCU_MCU_MODULE

#define USE_BNO055_MODULE
#define USE_BMX160_MODULE

#define USE_MICROSD_MODULE
#define USE_RUDDER_MODULE
#define USE_ADC_MODULE

#ifdef SD_MAIN_MODULE
#define USE_WINDSENSOR_MODULE
#define USE_BLE_MODULE
#define USE_MATH_MODULE
#endif

//#define USE_MPU_9250_MODULE
//#define USE_IRIDIUM_9603_MODULE
//#define USE_HMC5883_MODULE
//#define USE_HMC6343_MODULE

// In case that we keep calibrations on main modules
#define RAW_BLE_SENSOR_DATA

#define LAG_ADDR		D4CA6EB91DD3
//#define RUDDER_ADDR		CCF957816647
#define RUDDER_ADDR		D4CA6EBAFDA0

/*
#define TENSO_1
#define TENSO_2
#define TENSO_3
#define TENSO_4
*/

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
#define XBEE_IF			SPID1
#define GPS_IF			SPID4
#define GYRO_IF			I2CD2
#define MICROSD_IF		SPID3
#define BLE_IF			SD7
#define NINA_IF			SD7
#define NINA_IFACE		(BaseSequentialStream*)&SD7
#define WIND_IF			UARTD8
#endif /* CONFIG_H_ */
