/**
 * @file app.h
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief For application specific includes and definitions
 *        Will be included from main.h
 * @version 0.1
 * @date 2021-04-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef APP_H
#define APP_H

#include <Arduino.h>
/** Add you required includes after Arduino.h */
#include <Wire.h>
#include "TinyGPS++.h" //http://librarymanager/All#TinyGPSPlus
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h> // Click to install library: http://librarymanager/All#Adafruit_BME680

// Debug output set to 0 to disable app debug output
#ifndef MY_DEBUG
#define MY_DEBUG 1
#endif

// Enable GNSS Sections
//#define ENABLE_GNSS

// Enable ENV monitoring sections
#define ENABLE_ENV_MON

// Enable RS232 Renogy sections
#define ENABLE_RS232 

/** Examples for application events */
#define ACC_TRIGGER 0b1000000000000000
#define N_ACC_TRIGGER 0b0111111111111111

#ifdef NRF52_SERIES
#if MY_DEBUG > 0
#define MYLOG(tag, ...)                     \
	do                                      \
	{                                       \
		if (tag)                            \
			PRINTF("[%s] ", tag);           \
		PRINTF(__VA_ARGS__);                \
		PRINTF("\r\n");                       \
		if (g_ble_uart_is_connected)        \
		{                                   \
			g_ble_uart.printf(__VA_ARGS__); \
			g_ble_uart.printf("\r\n");        \
		}                                   \
	} while (0)
#else
#define MYLOG(...)
#endif
#endif
#ifdef ARDUINO_ARCH_RP2040
#if MY_DEBUG > 0
#define MYLOG(tag, ...)                  \
	do                                   \
	{                                    \
		if (tag)                         \
			Serial.printf("[%s] ", tag); \
		Serial.printf(__VA_ARGS__);      \
		Serial.printf("\n");             \
	} while (0)
#else
#define MYLOG(...)
#endif
#endif

/** Application function definitions */
void setup_app(void);
bool init_app(void);
void app_event_handler(void);
void ble_data_handler(void) __attribute__((weak));
void lora_data_handler(void);

/** Temperature + Humidity stuff */
int init_shtc3(void);
void shtc3_read_data(void);

/** Accelerometer stuff */
bool init_acc(void);
void clear_acc_int(void);
uint16_t *read_acc(void);

/** GNSS functions **/
bool init_gnss(void);
bool poll_gnss(void);

/** Renogy RS232 functions **/
void init_renogy_rs232(void);
void renogyPollRs232(void);
const char *renogyDecodeErrorStatus(void);
void renogyPrintStatus(void);

// LoRaWan functions
/** Include the WisBlock-API */
#include <WisBlock-API.h> // Click to install library: http://librarymanager/All#WisBlock-API

struct tracker_data_s
{
#ifdef ENABLE_GNSS
    uint8_t data_flag1 = 0x01;	// 1
	uint8_t data_flag2 = 0x88;	// 2
	uint8_t lat_1 = 0;			// 3
	uint8_t lat_2 = 0;			// 4
	uint8_t lat_3 = 0;			// 5
	uint8_t long_1 = 0;			// 6
	uint8_t long_2 = 0;			// 7
	uint8_t long_3 = 0;			// 8
	uint8_t alt_1 = 0;			// 9
	uint8_t alt_2 = 0;			// 10
	uint8_t alt_3 = 0;			// 11
#endif
	uint8_t data_flag3 = 0x08;	// 12
	uint8_t data_flag4 = 0x02;	// 13
	uint8_t batt_1 = 0;			// 14
	uint8_t batt_2 = 0;			// 15
#ifdef ENABLE_ENV_MON
 	uint8_t data_flag7 = 0x07;	// 24
	uint8_t data_flag8 = 0x68;	// 25
	uint8_t humid_1 = 0;		// 26
	uint8_t data_flag9 = 0x02;	// 27
	uint8_t data_flag10 = 0x67; // 28
	uint8_t temp_1 = 0;			// 29
	uint8_t temp_2 = 0;			// 30
#endif
};

extern tracker_data_s g_tracker_data;
#define TRACKER_DATA_LEN sizeof(tracker_data_s)

#ifdef ENABLE_RS232

struct renogy_data_s
{
#ifdef ENABLE_RS232
	uint8_t data_flag1 = 0x0C;   // 1
	uint8_t data_flag2 = 0x02;   // 2
	uint8_t panel_voltage_1 = 0; // 3
	uint8_t panel_voltage_2 = 0; // 4
	uint8_t panel_current_1 = 0; // 5
	uint8_t panel_current_2 = 0; // 6
	uint8_t panel_power_1 = 0;   // 7
	uint8_t panel_power_2 = 0;   // 8
	uint8_t batt_voltage_1 = 0;  // 9
	uint8_t batt_voltage_2 = 0;  // 10
	uint8_t batt_current_1 = 0;  // 11
	uint8_t batt_current_2 = 0;  // 12
	uint8_t batt_percent_1 = 0;  // 13
	uint8_t batt_percent_2 = 0;  // 14
	uint8_t load_voltage_1 = 0;  // 15
	uint8_t load_voltage_2 = 0;  // 16
	uint8_t load_current_1 = 0;  // 17
	uint8_t load_current_2 = 0;  // 18
	uint8_t load_power_1 = 0;    // 19
	uint8_t load_power_2 = 0;    // 20
	uint8_t load_status_1 = 0;   // 21
	uint8_t load_status_2 = 0;   // 22
	uint8_t error_status_1 = 0;  // 23
	uint8_t error_status_2 = 0;  // 24
	uint8_t recvd_downlink = 0;  // 25
#endif
};

extern renogy_data_s g_renogy_data;
#define RENOGY_DATA_LEN sizeof(renogy_data_s)
#endif // ENABLE_RS232

/** Battery level uinion */
union batt_s
{
	uint16_t batt16 = 0;
	uint8_t batt8[2];
};
/** Latitude/Longitude value union */
union latLong_s
{
	uint32_t val32;
	uint8_t val8[4];
};
/** Renogy value union */
union renogy_s 
{
	uint16_t val16;
	uint8_t val8[2];
};

#endif