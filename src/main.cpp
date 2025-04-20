/**
 * @file main.cpp
 * @author Bernd Giesecke (bernd@giesecke.tk)
 * @brief Low power test
 * @version 0.1
 * @date 2023-02-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <Arduino.h>
#include <WisBlock-API-V2.h>   //http://librarymanager/All#WisBlock-API-V2
#include <RAK12022_MAX31865.h> //http://librarymanager/All#RAK12022-MAX31865

// Debug output set to 0 to disable app debug output
#ifndef MY_DEBUG
#define MY_DEBUG 1
#endif

#if MY_DEBUG > 0
#define MYLOG(tag, ...)                     \
	do                                      \
	{                                       \
		if (tag)                            \
			PRINTF("[%s] ", tag);           \
		PRINTF(__VA_ARGS__);                \
		PRINTF("\n");                       \
		if (g_ble_uart_is_connected)        \
		{                                   \
			g_ble_uart.printf(__VA_ARGS__); \
			g_ble_uart.printf("\n");        \
		}                                   \
	} while (0)
#else
#define MYLOG(...)
#endif

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

/** Send Fail counter **/
uint8_t send_fail = 0;

/** Set the device name, max length is 10 characters */
char g_ble_dev_name[10] = "RAK-PT100";

/** Payload */
WisCayenne payload(255);

// use hardware SPI,just pass in the CS pin
const int Max_CS = SS;
MAX31865 maxTemp;
bool has_rak12022 = false;

/**
 * @brief Initial setup of the application (before LoRaWAN and BLE setup)
 *
 */
void setup_app(void)
{
	Serial.begin(115200);
	Serial1.begin(9500);
	
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

	// Set firmware version
	api_set_version(SW_VERSION_1, SW_VERSION_2, SW_VERSION_3);

	MYLOG("APP", "Setup application");
	g_enable_ble = true;
}

/**
 * @brief Final setup of application  (after LoRaWAN and BLE setup)
 *
 * @return true
 * @return false
 */
bool init_app(void)
{
	MYLOG("APP", "Initialize application");
	pinMode(WB_IO2, OUTPUT);
	digitalWrite(WB_IO2, HIGH);
	pinMode(WB_IO6, INPUT);
	delay(300);
	restart_advertising(30);

	// configure rtd sensor
	has_rak12022 = maxTemp.begin(Max_CS, MAX31865_3WIRE, MAX31865_PT100); // if use 2-wire,choose MAX31865_2WIRE (MAX31865_2WIRE,MAX31865_3WIRE，MAX31865_4WIRE)
	if (has_rak12022)
	{
		MYLOG("APP", "Found MAX31865");
		maxTemp.MAX31865_SetLowFaultThreshold(29.0);  // Set the low fault threshold to 29 degrees C
		maxTemp.MAX31865_SetHighFaultThreshold(34.0); // Set the high fault threshold to 34 degrees C

		bool dready_stat = true;
		time_t start_wait = millis();
		while (digitalRead(WB_IO6) == HIGH)
		{
			delay(100);
			if ((millis() - start_wait) > 5000)
			{
				dready_stat = false;
				break;
			}
		}

		if (!dready_stat)
		{
			MYLOG("APP", "DRDY timeout");
		}
		float mTemp, mResistance;
		uint8_t mStatus = 0;
		maxTemp.MAX31865_GetTemperatureAndStatus(mTemp, mResistance, mStatus);

		MYLOG("APP", "PT100 temperature: %.2f res: %.2f stat: %d", mTemp, mResistance, mStatus);

	}
	else
	{
		MYLOG("APP", "MAX31865 is not connected, Please check your connections\r\n");
	}
	RAK_SPI::SPIend();
	pinMode(MOSI, INPUT_PULLDOWN);
	pinMode(MISO, INPUT_PULLDOWN);
	pinMode(SCK, INPUT_PULLDOWN);
	pinMode(SS, INPUT_PULLDOWN);
	digitalWrite(WB_IO2, LOW);

	return true;
}

/**
 * @brief Handle events
 * 		Events can be
 * 		- timer (setup with AT+SENDINT=xxx)
 * 		- interrupt events
 * 		- wake-up signals from other tasks
 */
void app_event_handler(void)
{
	// Timer triggered event
	if ((g_task_event_type & STATUS) == STATUS)
	{
		g_task_event_type &= N_STATUS;
		MYLOG("APP", "Timer wakeup");
		digitalWrite(WB_IO2, HIGH);
		delay(200);

		// Prepare payload
		payload.reset();

		// Get Battery status
		float batt_level_f = 0.0;
		for (int readings = 0; readings < 10; readings++)
		{
			batt_level_f += read_batt();
		}
		batt_level_f = batt_level_f / 10;
		payload.addVoltage(LPP_CHANNEL_BATT, batt_level_f / 1000.0);

		if (has_rak12022)
		{
			// Re-init sensor after power down
			maxTemp.begin(Max_CS, MAX31865_3WIRE, MAX31865_PT100);
			maxTemp.MAX31865_SetLowFaultThreshold(25.0);  // Set the low fault threshold to 29 degrees C
			maxTemp.MAX31865_SetHighFaultThreshold(34.0); // Set the high fault threshold to 34 degrees C

			// Wait for data ready
			MYLOG("APP", "DRDY = %d", digitalRead(WB_IO6));
			bool dready_stat = true;
			time_t start_wait = millis();
			while (digitalRead(WB_IO6) == HIGH)
			{
				delay(100);
				if ((millis()-start_wait) > 5000)
				{
					dready_stat = false;
					break;
				}
			}

			if (!dready_stat)
			{
				MYLOG("APP", "DRDY timeout");
			}
			float mTemp, mResistance;
			uint8_t mStatus = 0;
			maxTemp.MAX31865_GetTemperatureAndStatus(mTemp, mResistance, mStatus);

			MYLOG("APP", "PT100 temperature: %.2f res: %.2f stat: %d", mTemp, mResistance, mStatus);
			if (mResistance != 0)
			{
				payload.addTemperature(LPP_CHANNEL_TEMP, mTemp);
			}
			if (mStatus & MAX31865_FAULT_TEMP_HIGH)
			{
				MYLOG("APP", "RTD High Threshold");
			}
			if (mStatus & MAX31865_FAULT_TEMP_LOW)
			{
				MYLOG("APP", "RTD Low Threshold");
			}
			if (mStatus & MAX31865_FAULT_REFIN_HIGH)
			{
				MYLOG("APP", "REFIN- > 0.85 x Bias");
			}
			if (mStatus & MAX31865_FAULT_REFIN_LOW_OPEN)
			{
				MYLOG("APP", "REFIN- < 0.85 x Bias - FORCE- open");
			}
			if (mStatus & MAX31865_FAULT_RTDIN_LOW_OPEN)
			{
				MYLOG("APP", "RTDIN- < 0.85 x Bias - FORCE- open");
			}
			if (mStatus & MAX31865_FAULT_VOLTAGE_OOR)
			{
				MYLOG("APP", "Voltage out of range fault");
			}

			RAK_SPI::SPIend();
			pinMode(MOSI, INPUT_PULLDOWN);
			pinMode(MISO, INPUT_PULLDOWN);
			pinMode(SCK, INPUT_PULLDOWN);
			pinMode(SS, INPUT_PULLDOWN);
		}

		if (g_lorawan_settings.lorawan_enable)
		{
			if (g_lpwan_has_joined)
			{

				lmh_error_status result = send_lora_packet(payload.getBuffer(), payload.getSize(), 2);
				switch (result)
				{
				case LMH_SUCCESS:
					MYLOG("APP", "Packet enqueued");
					break;
				case LMH_BUSY:
					MYLOG("APP", "LoRa transceiver is busy");
					break;
				case LMH_ERROR:
					MYLOG("APP", "Packet error, too big to send with current DR");
					break;
				}
			}
			else
			{
				MYLOG("APP", "Network not joined, skip sending");
			}
		}
		else
		{
			send_p2p_packet(payload.getBuffer(), payload.getSize());
		}
		digitalWrite(WB_IO2, LOW);
	}
}

/**
 * @brief Handle BLE events
 *
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

/**
 * @brief Handle LoRa events
 *
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
			AT_PRINTF("+EVT:JOINED");
		}
		else
		{
			MYLOG("APP", "Join network failed");
			AT_PRINTF("+EVT:JOIN_FAILED_TX_TIMEOUT");
			/// \todo here join could be restarted.
			lmh_join();
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
		MYLOG("APP", "RSSI %d SNR %d", g_last_rssi, g_last_snr);

		char log_buff[g_rx_data_len * 3] = {0};
		uint8_t log_idx = 0;
		for (int idx = 0; idx < g_rx_data_len; idx++)
		{
			sprintf(&log_buff[log_idx], "%02X ", g_rx_lora_data[idx]);
			log_idx += 3;
		}
		MYLOG("APP", "%s", log_buff);

		log_idx = 0;
		for (int idx = 0; idx < g_rx_data_len; idx++)
		{
			sprintf(&log_buff[log_idx], "%02X", g_rx_lora_data[idx]);
			log_idx += 2;
		}

		if (g_lorawan_settings.lorawan_enable)
		{
			AT_PRINTF("+EVT:RX_1:%d:%d:UNICAST:%d:%s", g_last_rssi, g_last_snr, g_last_fport, log_buff);
		}
		else
		{
			AT_PRINTF("+EVT:RXP2P:%d:%d:%s", g_last_rssi, g_last_snr, log_buff);
		}
	}

	// LoRa TX finished handling
	if ((g_task_event_type & LORA_TX_FIN) == LORA_TX_FIN)
	{
		g_task_event_type &= N_LORA_TX_FIN;

		if (g_lorawan_settings.lorawan_enable)
		{
			MYLOG("APP", "LoRa TX cycle %s", g_rx_fin_result ? "finished ACK" : "failed NAK");

			if (g_lorawan_settings.lorawan_enable)
			{
				if (g_lorawan_settings.confirmed_msg_enabled == LMH_UNCONFIRMED_MSG)
				{
					AT_PRINTF("+EVT:TX_DONE");
				}
				else
				{
					AT_PRINTF("+EVT:%s", g_rx_fin_result ? "SEND_CONFIRMED_OK" : "SEND_CONFIRMED_FAILED");
				}
			}
			else
			{
				AT_PRINTF("+EVT:TXP2P_DONE");
			}
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
		}
		else
		{
			MYLOG("APP", "P2P TX finished");
			AT_PRINTF("+EVT:TXP2P_DONE\n");
		}
	}
}
