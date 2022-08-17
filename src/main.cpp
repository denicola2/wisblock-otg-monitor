/**
   @file api-test.ino
   @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
   @brief Simple example how to use the SX126x-API
   @version 0.1
   @date 2021-09-10

   @copyright Copyright (c) 2021

*/
#include <Arduino.h>
/** Add you required includes after Arduino.h */
#include <Wire.h>
#include "credentials.h"
#include "app.h"

#ifdef NRF52_SERIES
/** Timer to wakeup task frequently and send message */
SoftwareTimer delayed_sending;
#endif
#ifdef ARDUINO_ARCH_RP2040
/** Timer for periodic sending */
TimerEvent_t delayed_sending;
#endif

/** Include the WisBlock-API */
#include <WisBlock-API.h> // Click to install library: http://librarymanager/All#WisBlock-API

/** Define the version of your SW */
#define SW_VERSION_1 1 // major version increase on API change / not backwards compatible
#define SW_VERSION_2 0 // minor version increase on API change / backward compatible
#define SW_VERSION_3 0 // patch version increase on bugfix, no affect on API

/** Application function definitions */
void setup_app(void);
bool init_app(void);
void app_event_handler(void);
void ble_data_handler(void) __attribute__((weak));
void lora_data_handler(void);

/** Application stuff */

/** Set the device name, max length is 10 characters */
char g_ble_dev_name[10] = "IoTA-MON";

/** Location data as byte array */
tracker_data_s g_tracker_data;

/** Flag showing if TX cycle is ongoing */
bool lora_busy = false;

/** Send Fail counter **/
uint8_t send_fail = 0;

/** Timer since last position message was sent */
time_t last_pos_send = 0;
/** Timer for delayed sending to keep duty cycle */

/** Battery level uinion */
batt_s batt_level;

/** Flag if delayed sending is already activated */
bool delayed_active = false;

/** Minimum delay between sending new locations, set to 45 seconds */
time_t min_delay = 45000;

#ifdef NRF52_SERIES
/**
 * @brief Timer function used to avoid sending packages too often.
 *       Delays the next package by 10 seconds
 * 
 * @param unused 
 *      Timer handle, not used
 */
void send_delayed(TimerHandle_t unused)
{
	api_wake_loop(STATUS);
}
#endif
#ifdef ARDUINO_ARCH_RP2040
/**
 * @brief Timer function used to avoid sending packages too often.
 *      Delays the next package by 10 seconds
 * 
 */
void send_delayed(void)
{
	api_wake_loop(STATUS);
}
#endif

/**
   @brief Application specific setup functions

*/
void setup_app(void)
{
	Serial.begin(115200);
	time_t serial_timeout = millis();
	// On nRF52840 the USB serial is not available immediately
	while (!Serial)
	{
		if ((millis() - serial_timeout) < 5000)
		{
			delay(100);
			digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
		}
		else
		{
			break;
		}
	}
	digitalWrite(LED_GREEN, LOW);

	MYLOG("APP", "Setup IoT-Anywhere Enclosure Monitor");

#ifdef NRF52_SERIES
	// Enable BLE
	g_enable_ble = true;
#endif

	// Set firmware version
	api_set_version(SW_VERSION_1, SW_VERSION_2, SW_VERSION_3);

	// Optional
	// Setup LoRaWAN credentials hard coded
	// It is strongly recommended to avoid duplicated node credentials
	// Options to setup credentials are
	// -over USB with AT commands
	// -over BLE with My nRF52 Toolbox

	// Read LoRaWAN settings from flash
	api_read_credentials();

	// Modify credentials to be unique in credentials.h
	uint8_t node_device_eui[8] = NODE_DEVICE_EUI;
	uint8_t node_app_eui[8] = NODE_APP_EUI;
	uint8_t node_app_key[16] = NODE_APP_KEY;
	uint8_t node_nws_key[16] = NODE_NWS_KEY;
	uint8_t node_apps_key[16] = NODE_APPS_KEY;

	// Change LoRaWAN settings
	g_lorawan_settings.lorawan_enable = true;
	g_lorawan_settings.auto_join = true;							// Flag if node joins automatically after reboot
	g_lorawan_settings.otaa_enabled = true;							// Flag for OTAA or ABP
	memcpy(g_lorawan_settings.node_device_eui, node_device_eui, 8); // OTAA Device EUI MSB
	memcpy(g_lorawan_settings.node_app_eui, node_app_eui, 8);		// OTAA Application EUI MSB
	memcpy(g_lorawan_settings.node_app_key, node_app_key, 16);		// OTAA Application Key MSB
	memcpy(g_lorawan_settings.node_nws_key, node_nws_key, 16);		// ABP Network Session Key MSB
	memcpy(g_lorawan_settings.node_apps_key, node_apps_key, 16);	// ABP Application Session key MSB
	g_lorawan_settings.node_dev_addr = 0x26021FB4;					// ABP Device Address MSB
	g_lorawan_settings.send_repeat_time = 900000;					// Send repeat time in milliseconds: 15 * 60 * 1000 => 15 minutes
	g_lorawan_settings.adr_enabled = false;							// Flag for ADR on or off
	g_lorawan_settings.public_network = true;						// Flag for public or private network
	g_lorawan_settings.duty_cycle_enabled = false;					// Flag to enable duty cycle (validity depends on Region)
	g_lorawan_settings.join_trials = 10;							// Number of join retries
	g_lorawan_settings.tx_power = TX_POWER_0;						// TX power 0 .. 15 (validity depends on Region)
	g_lorawan_settings.data_rate = DR_3;							// Data rate 0 .. 15 (validity depends on Region)
	g_lorawan_settings.lora_class = CLASS_A;						// LoRaWAN class 0: A, 2: C, 1: B is not supported
	g_lorawan_settings.subband_channels = 2;						// Subband channel selection 1 .. 9
	g_lorawan_settings.app_port = LORAWAN_APP_PORT;					// Data port to send data
	g_lorawan_settings.confirmed_msg_enabled = LMH_UNCONFIRMED_MSG;	// Flag to enable confirmed messages
	g_lorawan_settings.resetRequest = true;							// Command from BLE to reset device
	g_lorawan_settings.lora_region = LORAMAC_REGION_US915;			// LoRa region

	// Save LoRaWAN settings
	api_set_credentials();
}

/**
   @brief Application specific initializations

   @return true Initialization success
   @return false Initialization failure
*/
bool init_app(void)
{
	bool init_result = true;
	MYLOG("APP", "init_app");

	pinMode(WB_IO2, OUTPUT);
	digitalWrite(WB_IO2, HIGH);

	// Start the I2C bus
	Wire.begin();
	Wire.setClock(400000);

#ifdef ENABLE_GNSS
	// Initialize GNSS module
	MYLOG("APP", "Initialize uBlox GNSS");
	init_result = init_gnss();
	MYLOG("APP", "Result %s", init_result ? "success" : "failed");
#endif
#ifdef ENABLE_ENV_MON
	// Initialize Temp & Humi sensor
	MYLOG("APP", "Initialize shtc3 Temp & Humi");
	init_result |= init_shtc3();
	MYLOG("APP", "Result %s", init_result ? "success" : "failed");
#endif
#ifdef ENABLE_RS232
	// Initialize Serial (for RS232 to Renogy)
	MYLOG("APP", "Initialize Serial1 for RS232 to Renogy");
	init_renogy_rs232();
	MYLOG("APP", "Result %s", Serial1 ? "success" : "failed");
	//Serial1.write("Hello to renogy!");
#endif
	if (g_lorawan_settings.send_repeat_time != 0)
	{
		// Set delay for sending to scheduled sending time
		min_delay = g_lorawan_settings.send_repeat_time / 2;
	}
	else
	{
		// Send repeat time is 0, set delay to 30 seconds
		min_delay = 30000;
	}
	// Set to 1/2 of programmed send interval or 30 seconds
#ifdef NRF52_SERIES
	delayed_sending.begin(g_lorawan_settings.send_repeat_time, send_delayed);
#endif
#ifdef ARDUINO_ARCH_RP2040
	delayed_sending.oneShot = false;
	delayed_sending.ReloadValue = g_lorawan_settings.send_repeat_time;
	TimerInit(&delayed_sending, send_delayed);
	TimerSetValue(&delayed_sending, g_lorawan_settings.send_repeat_time);
#endif

	// Power down GNSS module
	// pinMode(WB_IO2, OUTPUT);
	// digitalWrite(WB_IO2, LOW);
	return init_result;
}

/**
   @brief Application specific event handler
          Requires as minimum the handling of STATUS event
          Here you handle as well your application specific events
*/
void app_event_handler(void)
{
	// Timer triggered event
	if ((g_task_event_type & STATUS) == STATUS)
	{
		g_task_event_type &= N_STATUS;
		MYLOG("APP", "Timer wakeup");

#ifdef NRF52_SERIES
		// If BLE is enabled, restart Advertising
		if (g_enable_ble)
		{
			restart_advertising(15);
		}
#endif
		if (lora_busy)
		{
			MYLOG("APP", "LoRaWAN TX cycle not finished, skip this event");
		}
		else
		{
#ifdef ENABLE_GNSS
			// Check location and populate payload
			if (poll_gnss())
			{
				MYLOG("APP", "Valid GNSS position");
			}
			else
			{
				MYLOG("APP", "No valid GNSS position");
			}
#endif
#ifdef ENABLE_ENV_MON
			// Read temp & humi data and populate payload
			shtc3_read_data();
#endif
			// Get battery level
			// g_tracker_data.batt = mv_to_percent(read_batt());
			batt_level.batt16 = read_batt() / 10;
			g_tracker_data.batt_1 = batt_level.batt8[1];
			g_tracker_data.batt_2 = batt_level.batt8[0];	

			// Remember last time sending
			last_pos_send = millis();

			// Just in case
			delayed_active = false;	

			uint8_t *packet = (uint8_t *)&g_tracker_data;
#if MY_DEBUG == 1
			Serial.println("Packet 1 prepared for uplink:");
			for (int idx = 0; idx < (int)TRACKER_DATA_LEN; idx++)
			{
				Serial.printf("%02X", packet[idx]);
			}
			Serial.println("");
#endif

			//Sending first uplink data: Battery, GNSS, ENV data
			lmh_error_status result = send_lora_packet((uint8_t *)&g_tracker_data, TRACKER_DATA_LEN);
			switch (result)
			{
			case LMH_SUCCESS:
				MYLOG("APP", "Packet 1 enqueued");
				// Set a flag that TX cycle is running
				lora_busy = true;
				break;
			case LMH_BUSY:
				MYLOG("APP", "LoRa transceiver is busy");
				break;
			case LMH_ERROR:
				MYLOG("APP", "Packet 1 error, too big to send with current DR");
				break;
			}

#ifdef ENABLE_RS232
			if (Serial1)
			{
				// Read Renogy Solar Controller and populate payload
				renogyPollRs232Data();
				renogyPollRs232Errors();
#if MY_DEBUG == 1
				renogyPrintStatus();
#endif
				packet = (uint8_t *)&g_renogy_data;
#if MY_DEBUG == 1
                Serial.println("Packet 2 prepared for uplink:");
				for (int idx = 0; idx < (int)RENOGY_DATA_LEN; idx++)
				{
					Serial.printf("%02X", packet[idx]);
				}
				Serial.println("");
#endif
				//MYLOG("RS232", "Renogy Error Status: %s", renogyDecodeErrorStatus());

				// Sending second uplink data: Renogy Solar Charge Controller readings
				delay(1); // Wait for a second to allow lorawan to send the first packet
				result = send_lora_packet((uint8_t *)&g_renogy_data, RENOGY_DATA_LEN);
				switch (result)
				{
				case LMH_SUCCESS:
					MYLOG("APP", "Packet 2 enqueued");
					// Set a flag that TX cycle is running
					lora_busy = true;
					break;
				case LMH_BUSY:
					MYLOG("APP", "LoRa transceiver is busy");
					break;
				case LMH_ERROR:
					MYLOG("APP", "Packet 2 error, too big to send with current DR");
					break;
				}
			}
			else
			{
				MYLOG("RS232", "RS232 interface not initialized, skipping Renogy Poll Cycle");
			}
#endif // RS232_ENABLED
		}
	}
}

#ifdef NRF52_SERIES
/**
   @brief Handle BLE UART data

*/
void ble_data_handler(void)
{
	if (g_enable_ble)
	{
		/**************************************************************/
		/**************************************************************/
		/// \todo BLE UART data arrived
		/// \todo or forward them to the AT command interpreter
		/// \todo parse them here
		/**************************************************************/
		/**************************************************************/
		if ((g_task_event_type & BLE_DATA) == BLE_DATA)
		{
			MYLOG("AT", "RECEIVED BLE");
			// BLE UART data arrived
			// in this example we forward it to the AT command interpreter
			g_task_event_type &= N_BLE_DATA;

			while (g_ble_uart.available() > 0)
			{
				at_serial_input(uint8_t(g_ble_uart.read()));
				delay(5);
			}
			at_serial_input(uint8_t('\n'));
		}
	}
}
#endif

/**
   @brief Handle received LoRa Data

*/
void lora_data_handler(void)
{
	// LoRa Join finished handling
	if ((g_task_event_type & LORA_JOIN_FIN) == LORA_JOIN_FIN)
	{
		g_task_event_type &= N_LORA_JOIN_FIN;
		if (g_join_result)
		{
			MYLOG("APP", "Successfully joined network");
		}
		else
		{
			MYLOG("APP", "Join network failed");
			/// \todo here join could be restarted.
			// lmh_join();
		}
	}

	// LoRa data handling
	if ((g_task_event_type & LORA_DATA) == LORA_DATA)
	{
		/**************************************************************/
		/**************************************************************/
		/// \todo LoRa data arrived
		/// \todo parse them here
		/**************************************************************/
		/**************************************************************/
		g_task_event_type &= N_LORA_DATA;
		MYLOG("APP", "Received package over LoRa");
		char log_buff[g_rx_data_len * 3] = {0};
		uint8_t log_idx = 0;
		for (int idx = 0; idx < g_rx_data_len; idx++)
		{
			sprintf(&log_buff[log_idx], "%02X ", g_rx_lora_data[idx]);
			log_idx += 3;
		}
		lora_busy = false;
		MYLOG("APP", "%s", log_buff);
	}

	// LoRa TX finished handling
	if ((g_task_event_type & LORA_TX_FIN) == LORA_TX_FIN)
	{
		g_task_event_type &= N_LORA_TX_FIN;

		MYLOG("APP", "LPWAN TX cycle %s", g_rx_fin_result ? "finished ACK" : "failed NAK");

		if (!g_rx_fin_result)
		{
			// Increase fail send counter
			send_fail++;

			if (send_fail == 10)
			{
				// Too many failed sendings, reset node and try to rejoin
				delay(100);
				api_reset();
			}
		}

		// Clear the LoRa TX flag
		lora_busy = false;
	}
}