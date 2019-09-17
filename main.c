/***************************************************************************//**
 * @file
 * @brief Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 **************************************************************************************************/

/* Standard library headers */
#include <stdio.h>
#include <string.h>

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
//#include "boards.h"
#include "ble-configuration.h"
#include "board_features.h"

#include "gpiointerrupt.h"
//#include "graphics.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
//#include "aat.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include "em_rtcc.h"

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

#define NODISPLAY

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 1
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

/* ---- Application macros ---- */

/* GENERAL MACROS */
#define SOFT_TIMER_DISPLAY_REFRESH_HANDLE		0	// Handle for the display refresh
#define SOFT_TIMER_FIXED_TRANSFER_TIME_HANDLE	1 	// Handle for stopping fixed time transfer
#define COEX_COUNTER_UPDATE                     2

#define DATA_SIZE			255					// Size of the arrays for sending and receiving data

#define DATA_TRANSFER_SIZE_INDICATIONS		0 // If == 0 or > MTU-3 then it will send MTU-3 bytes of data, otherwise it will use this value
#define DATA_TRANSFER_SIZE_NOTIFICATIONS	0 // If == 0 or > MTU-3 then it will calculate the data amount to send for maximum over-the-air packet usage, otherwise it will use this value

#define PHY_1M				(0x01)
#define PHY_2M				(0x02)
#define PHY_S8				(0x04)
#define PHY_S2				(0x08)

#define TX_POWER			(-50)

//#define USE_LED_FOR_CONNECTION_SIGNALING		// Define this so that LED0 is ON when connection is established and OFF when it's disconnected
//#define USE_LED_FOR_DATA_SENDING_SIGNALING 	// Define this so that LED1 is ON when data is being send

/* SLAVE SIDE MACROS */
#define NOTIFICATIONS_START				(uint32)(1 << 0)  	// Bit flag to external signal command
#define NOTIFICATIONS_END				(uint32)(1 << 1)	// Bit flag to external signal command
#define INDICATIONS_START				(uint32)(1 << 2)	// Bit flag to external signal command
#define INDICATIONS_END					(uint32)(1 << 3)	// Bit flag to external signal command
#define ADV_INTERVAL_MAX				160					// 160 * 0.625us = 100ms
#define ADV_INTERVAL_MIN				160					// 160 * 0.625us = 100ms
//#define SEND_FIXED_TRANSFER_COUNT		1000				// Uncomment this if you want to send a fixed amount of indications/notifications on each button press
//#define SEND_FIXED_TRANSFER_TIME		(32768*5)				// Uncomment this if you want to send indications/notifications for a fixed amount of time (in 32.768Hz clock ticks) on each button press

/* MASTER SIDE MACROS */
#define PHY_CHANGE						(uint32)(1 << 4)	// Bit flag to external signal command
#define WRITE_NO_RESPONSE_START			(uint32)(1 << 5)	// Bit flag to external signal command
#define WRITE_NO_RESPONSE_END			(uint32)(1 << 6)	// Bit flag to external signal command
#define CONN_INTERVAL_1MPHY_MAX			40					// 40 * 1.25ms = 50ms
#define CONN_INTERVAL_1MPHY_MIN			40					// 40 * 1.25ms = 50ms
#define SLAVE_LATENCY_1MPHY				0					// How many connection intervals can the slave skip if no data is to be sent
#define SUPERVISION_TIMEOUT_1MPHY		100					// 100 * 10ms = 1000ms
#define CONN_INTERVAL_2MPHY_MAX			20  				// 20 * 1.25ms = 25ms
#define CONN_INTERVAL_2MPHY_MIN			20					// 20 * 1.25ms = 25ms
#define SLAVE_LATENCY_2MPHY				0					// How many connection intervals can the slave skip if no data is to be sent
#define SUPERVISION_TIMEOUT_2MPHY		100					// 100 * 10ms = 1000ms
#define CONN_INTERVAL_125KPHY_MAX		160					// 160 * 1.25ms = 200ms
#define CONN_INTERVAL_125KPHY_MIN		160					// 160 * 1.25ms = 200ms
#define SLAVE_LATENCY_125KPHY			0					// How many connection intervals can the slave skip if no data is to be sent
#define SUPERVISION_TIMEOUT_125KPHY		200					// 200 * 10ms = 2000ms
#define SCAN_INTERVAL					16					// 16 * 0.625 = 10ms
#define SCAN_WINDOW						16					// 16 * 0.625 = 10ms
#define ACTIVE_SCANNING					1					// 1 = active scanning (sends scan requests), 0 = passive scanning (doesn't send scan requests)
/* -------------------- */

//#ifdef FEATURE_PTI_SUPPORT
//static const RADIO_PTIInit_t ptiInit = RADIO_PTI_INIT;
//#endif

#if (CONN_INTERVAL_125KPHY_MAX < 32) || (CONN_INTERVAL_125KPHY_MIN < 32)
#error "Minimum connection interval for LE Coded PHY must be above 40ms according to set_phy command description in API Ref."
#endif

#if defined(SEND_FIXED_TRANSFER_COUNT) && defined(SEND_FIXED_TRANSFER_TIME)
#error "These are mutually exclusive options, you either do a fixed amount of transfers of transfer over a fixed amount of time."
#endif

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

#define NODISPLAY

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 1
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

/* ---- Application macros ---- */

/* GENERAL MACROS */
#define SOFT_TIMER_DISPLAY_REFRESH_HANDLE		0	// Handle for the display refresh
#define SOFT_TIMER_FIXED_TRANSFER_TIME_HANDLE	1 	// Handle for stopping fixed time transfer
#define COEX_COUNTER_UPDATE                     2

#define DATA_SIZE			255					// Size of the arrays for sending and receiving data

#define DATA_TRANSFER_SIZE_INDICATIONS		0 // If == 0 or > MTU-3 then it will send MTU-3 bytes of data, otherwise it will use this value
#define DATA_TRANSFER_SIZE_NOTIFICATIONS	0 // If == 0 or > MTU-3 then it will calculate the data amount to send for maximum over-the-air packet usage, otherwise it will use this value

#define PHY_1M				(0x01)
#define PHY_2M				(0x02)
#define PHY_S8				(0x04)
#define PHY_S2				(0x08)

#define TX_POWER			(-50)

//#define USE_LED_FOR_CONNECTION_SIGNALING		// Define this so that LED0 is ON when connection is established and OFF when it's disconnected
//#define USE_LED_FOR_DATA_SENDING_SIGNALING 	// Define this so that LED1 is ON when data is being send

/* SLAVE SIDE MACROS */
#define NOTIFICATIONS_START				(uint32)(1 << 0)  	// Bit flag to external signal command
#define NOTIFICATIONS_END				(uint32)(1 << 1)	// Bit flag to external signal command
#define INDICATIONS_START				(uint32)(1 << 2)	// Bit flag to external signal command
#define INDICATIONS_END					(uint32)(1 << 3)	// Bit flag to external signal command
#define ADV_INTERVAL_MAX				160					// 160 * 0.625us = 100ms
#define ADV_INTERVAL_MIN				160					// 160 * 0.625us = 100ms
//#define SEND_FIXED_TRANSFER_COUNT		1000				// Uncomment this if you want to send a fixed amount of indications/notifications on each button press
//#define SEND_FIXED_TRANSFER_TIME		(32768*5)				// Uncomment this if you want to send indications/notifications for a fixed amount of time (in 32.768Hz clock ticks) on each button press

/* MASTER SIDE MACROS */
#define PHY_CHANGE						(uint32)(1 << 4)	// Bit flag to external signal command
#define WRITE_NO_RESPONSE_START			(uint32)(1 << 5)	// Bit flag to external signal command
#define WRITE_NO_RESPONSE_END			(uint32)(1 << 6)	// Bit flag to external signal command
#define CONN_INTERVAL_1MPHY_MAX			40					// 40 * 1.25ms = 50ms
#define CONN_INTERVAL_1MPHY_MIN			40					// 40 * 1.25ms = 50ms
#define SLAVE_LATENCY_1MPHY				0					// How many connection intervals can the slave skip if no data is to be sent
#define SUPERVISION_TIMEOUT_1MPHY		100					// 100 * 10ms = 1000ms
#define CONN_INTERVAL_2MPHY_MAX			20  				// 20 * 1.25ms = 25ms
#define CONN_INTERVAL_2MPHY_MIN			20					// 20 * 1.25ms = 25ms
#define SLAVE_LATENCY_2MPHY				0					// How many connection intervals can the slave skip if no data is to be sent
#define SUPERVISION_TIMEOUT_2MPHY		100					// 100 * 10ms = 1000ms
#define CONN_INTERVAL_125KPHY_MAX		160					// 160 * 1.25ms = 200ms
#define CONN_INTERVAL_125KPHY_MIN		160					// 160 * 1.25ms = 200ms
#define SLAVE_LATENCY_125KPHY			0					// How many connection intervals can the slave skip if no data is to be sent
#define SUPERVISION_TIMEOUT_125KPHY		200					// 200 * 10ms = 2000ms
#define SCAN_INTERVAL					16					// 16 * 0.625 = 10ms
#define SCAN_WINDOW						16					// 16 * 0.625 = 10ms
#define ACTIVE_SCANNING					1					// 1 = active scanning (sends scan requests), 0 = passive scanning (doesn't send scan requests)
/* -------------------- */

//#ifdef FEATURE_PTI_SUPPORT
//static const RADIO_PTIInit_t ptiInit = RADIO_PTI_INIT;
//#endif

#if (CONN_INTERVAL_125KPHY_MAX < 32) || (CONN_INTERVAL_125KPHY_MIN < 32)
#error "Minimum connection interval for LE Coded PHY must be above 40ms according to set_phy command description in API Ref."
#endif

#if defined(SEND_FIXED_TRANSFER_COUNT) && defined(SEND_FIXED_TRANSFER_TIME)
#error "These are mutually exclusive options, you either do a fixed amount of transfers of transfer over a fixed amount of time."
#endif

uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

/* Bluetooth stack configuration parameters (see "UG136: Silicon Labs Bluetooth C Application Developer's Guide" for details on each parameter) */
static gecko_configuration_t config = {
  .config_flags = 0,                                   /* Check flag options from UG136 */
#if 0 //defined(FEATURE_LFXO)
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,        /* Sleep is enabled */
#else
  .sleep.flags = 0,
#endif // LFXO
  .bluetooth.max_connections = MAX_CONNECTIONS,        /* Maximum number of simultaneous connections */
  .bluetooth.max_advertisers = 1,        /* Maximum number of advertisement sets */
  .bluetooth.heap = bluetooth_stack_heap,              /* Bluetooth stack memory for connection management */
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap), /* Bluetooth stack memory for connection management */
  .bluetooth.sleep_clock_accuracy = 100,               /* Accuracy of the Low Frequency Crystal Oscillator in ppm. *
                                                       * Do not modify if you are using a module                  */
  .gattdb = &bg_gattdb_data,                           /* Pointer to GATT database */
  .ota.flags = 0,                                      /* Check flag options from UG136 */
  .ota.device_name_len = 3,                            /* Length of the device name in OTA DFU mode */
  .ota.device_name_ptr = "OTA",                        /* Device name in OTA DFU mode */
#if (HAL_PA_ENABLE)
  .pa.config_enable = 1,                               /* Set this to be a valid PA config */
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT,               /* Configure PA input to VBAT */
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,               /* Configure PA input to DCDC */
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
#endif // (HAL_PA_ENABLE)
  .rf.flags = GECKO_RF_CONFIG_ANTENNA,                 /* Enable antenna configuration. */
  .rf.antenna = GECKO_RF_ANTENNA,                      /* Select antenna path! */
};

enum states_enum {idle, transmitting, result};
/* ---- Application variables ---- */
enum states_enum state = idle;							// Variable to keep track of states in master side
uint32_t time_elapsed;									// Variable to calculate time during which there was data tranmission
uint32_t testTime = 60;                      //Variable that is tied to gattdb_TestTime and holds the Time for the Notifications test (In Seconds)
const uint8_t displayRefreshOn = 1;						// Turn ON display refresh on master side
const uint8_t displayRefreshOff = 0;					// Turn OFF display refresh on master side
uint8_t boot_to_dfu = 0; 								// Flag indicating if device should boot into DFU mode
uint16_t mtuSize = 0;  									// Variable to hold the MTU size once a new connection is formed
uint16_t pduSize = 0;									// Variable to hold the PDU size once a new connection is formed
uint16_t maxDataSizeIndications = 0;
uint16_t maxDataSizeNotifications = 0;					// Variable to calculate maximum data size for optimum throughput
uint8_t connection = 0; 								// Variable to hold the connection handle
uint16_t phyInUse = PHY_1M;								// Variable to hold the PHY in use
uint16_t phyToUse = 0;									// Variable to hold the next PHY to use when changing to and from LE Coded Phy
bool notifications_enabled = false; 					// Flag to check if notifications are enabled or not
bool indications_enabled = false; 						// Flag to check if indications are enabled or not
bool roleIsSlave; 										// Flag to check if role is slave or master (based on PB0 being pressed or not during boot)
bool sendNotifications = false; 						// Flag to trigger sending of notifications
bool sendIndications = false; 							// Flag to trigger sending of indications
bool sendWriteNoResponse = false;						// Flag to trigger sending of write no response
bool notification_accepted = true;						// Flag to check if previous notification command was accepted and generate new data for the next one
uint8 throughput_array_notifications[DATA_SIZE] = {0}; 	// Array to hold data payload to be sent over notifications
uint8 throughput_array_indications[DATA_SIZE] = {0}; 	// Array to hold data payload to be sent over indications
uint32 bitsSent = 0; 									// Variable to increment the amount of data sent and received and display the throughput
uint32 throughput = 0;									// Variable to hold throughput calculation
uint32 operationCount = 0;								// Variable to count how many GATT operations have occurred from both sides
uint8_t enableNotificationsIndications = 0;				// Variable to control enabling notifications and indications in master mode
uint8_t invalidData = 0;								// Variable to register how many notifications were not received by the application
#ifdef SEND_FIXED_TRANSFER_COUNT
uint32_t transferCount = 0;
#endif
char throughputString[] = "TH:           \n";			// Char array to print the bitsSent variable on the display every second, so this will be throughput
char mtuSizeString[] = "MTU:     "; 				// Char array to print MTU size on the display
char connIntervalString[] = "INTRV:      ";		// Char array to print connection interval on the display
char pduSizeString[] = "PDU:     ";				// Char array to print PTU size on the display
char deviceNameString[] = "Throughput Tester";			// Char array to with device name to match against scan results
char phyInUseString[] = "PHY:    ";						// Char array to print PHY in use on the display
char maxDataSizeNotificationsString[] = "DATA SIZE:     ";
char invalidDataString[] = "INVALD:     ";
char operationCountString[] = "CNT:       \n";
const char roleSlaveString[] = {"ROLE: Slave\n"};
const char roleMasterString[] = {"ROLE: Master\n"};
char statusConnectedString[] = {"RSSI:     \n"};
const char statusDisconnectedString[] = {"STATUS: Discon\n"};
const char notifyEnabledString[] = {"NOTIFY: Yes\n"};
const char notifyDisabledString[] = {"NOTIFY: No\n"};
const char indicateEnabledString[] = {"INDICATE: Yes\n"};
const char indicateDisabledString[] = {"INDICATE: No\n"};
char* roleString;
char* statusString = (char*)statusDisconnectedString;
char* notifyString = (char*)notifyDisabledString;
char* indicateString = (char*)indicateDisabledString;
/* -------------------- */

/**************************************************************************//**
* @brief Routine to refresh the info on the display based on the Bluetooth link status
*****************************************************************************/
void displayRefresh()
{

#ifndef NODISPLAY

	GRAPHICS_Clear();

	GRAPHICS_AppendString(roleString);
	GRAPHICS_AppendString(statusString);
	GRAPHICS_AppendString(connIntervalString);
	GRAPHICS_AppendString(pduSizeString);
	GRAPHICS_AppendString(mtuSizeString);
	GRAPHICS_AppendString(maxDataSizeNotificationsString);
	GRAPHICS_AppendString(phyInUseString);
	GRAPHICS_AppendString(notifyString);
	GRAPHICS_AppendString(indicateString);

	sprintf(throughputString+4, "%07lu", throughput);
	  throughputString[11] = ' ';
	  throughputString[12] = 'b';
	  throughputString[13] = 'p';
	  throughputString[14] = 's';
	GRAPHICS_AppendString(throughputString);
	sprintf(operationCountString+5, "%09lu", operationCount);
	GRAPHICS_AppendString(operationCountString);
	//sprintf(invalidDataString+8, "%04u", invalidData);
	//GRAPHICS_AppendString(invalidDataString);

	GRAPHICS_Update();

#endif
}


/**************************************************************************//**
* @brief Function to Enable TX and RX active pins for Coex debugging
*****************************************************************************/
#define COEX_DEBUG 1
#ifdef COEX_DEBUG

// Enable TX_ACT signal through GPIO PD10
#define _PRS_CH_CTRL_SOURCESEL_RAC2						0x00000020UL
#define PRS_CH_CTRL_SOURCESEL_RAC2						(_PRS_CH_CTRL_SOURCESEL_RAC2 << 8)
#define _PRS_CH_CTRL_SIGSEL_RACPAEN						0x00000004UL
#define PRS_CH_CTRL_SIGSEL_RACPAEN						(_PRS_CH_CTRL_SIGSEL_RACRX << 0)
#define TX_ACTIVE_PRS_SOURCE							PRS_CH_CTRL_SOURCESEL_RAC2
#define TX_ACTIVE_PRS_SIGNAL							PRS_CH_CTRL_SIGSEL_RACPAEN

#define TX_ACTIVE_PRS_CHANNEL 							5
#define TX_ACTIVE_PRS_LOCATION 							0
#define TX_ACTIVE_PRS_PORT 								gpioPortD
#define TX_ACTIVE_PRS_PIN 								10
#define TX_ACTIVE_PRS_ROUTELOC_REG 						ROUTELOC1
#define TX_ACTIVE_PRS_ROUTELOC_MASK 					(~0x00003F00UL)
#define TX_ACTIVE_PRS_ROUTELOC_VALUE					PRS_ROUTELOC1_CH5LOC_LOC0 // PD10
#define TX_ACTIVE_PRS_ROUTEPEN PRS_ROUTEPEN_CH5PEN

// Enable RX_ACT signal through GPIO PD11
#define _PRS_CH_CTRL_SOURCESEL_RAC2						0x00000020UL
#define PRS_CH_CTRL_SOURCESEL_RAC2						(_PRS_CH_CTRL_SOURCESEL_RAC2 << 8)
#define _PRS_CH_CTRL_SIGSEL_RACRX						0x00000002UL
#define PRS_CH_CTRL_SIGSEL_RACRX						(_PRS_CH_CTRL_SIGSEL_RACRX << 0)
#define RX_ACTIVE_PRS_SOURCE PRS_CH_CTRL_SOURCESEL_RAC2
#define RX_ACTIVE_PRS_SIGNAL PRS_CH_CTRL_SIGSEL_RACRX

#define RX_ACTIVE_PRS_CHANNEL							6
#define RX_ACTIVE_PRS_LOCATION							13
#define RX_ACTIVE_PRS_PORT								gpioPortD
#define RX_ACTIVE_PRS_PIN								11
#define RX_ACTIVE_PRS_ROUTELOC_REG						ROUTELOC1
#define RX_ACTIVE_PRS_ROUTELOC_MASK						(~0x003F0000UL)
#define RX_ACTIVE_PRS_ROUTELOC_VALUE					PRS_ROUTELOC1_CH6LOC_LOC13 // PD11
#define RX_ACTIVE_PRS_ROUTEPEN							PRS_ROUTEPEN_CH6PEN

void initTxRXActive()
{

	CMU_ClockEnable(cmuClock_PRS, true); // enable clock to PRS

	// Setup PRS input as TX_ACTIVE signal
	PRS_SourceAsyncSignalSet(TX_ACTIVE_PRS_CHANNEL, TX_ACTIVE_PRS_SOURCE, TX_ACTIVE_PRS_SIGNAL);

	// enable TX_ACTIVE output pin with initial value of 0
	GPIO_PinModeSet(TX_ACTIVE_PRS_PORT, TX_ACTIVE_PRS_PIN, gpioModePushPull, 0);

	// Route PRS CH/LOC to TX Active GPIO output
	PRS->TX_ACTIVE_PRS_ROUTELOC_REG = (PRS->TX_ACTIVE_PRS_ROUTELOC_REG &
	TX_ACTIVE_PRS_ROUTELOC_MASK) | TX_ACTIVE_PRS_ROUTELOC_VALUE;
	PRS->ROUTEPEN |= TX_ACTIVE_PRS_ROUTEPEN;

	// Setup PRS input as RX_ACTIVE signal
	PRS_SourceAsyncSignalSet(RX_ACTIVE_PRS_CHANNEL, RX_ACTIVE_PRS_SOURCE, RX_ACTIVE_PRS_SIGNAL);

	// enable RX_ACTIVE output pin with initial value of 0
	GPIO_PinModeSet(RX_ACTIVE_PRS_PORT, RX_ACTIVE_PRS_PIN, gpioModePushPull, 0);

	// Route PRS CH/LOC to RX Active GPIO output
	PRS->RX_ACTIVE_PRS_ROUTELOC_REG = (PRS->RX_ACTIVE_PRS_ROUTELOC_REG & RX_ACTIVE_PRS_ROUTELOC_MASK) | RX_ACTIVE_PRS_ROUTELOC_VALUE;
	PRS->ROUTEPEN |= RX_ACTIVE_PRS_ROUTEPEN;


	GPIO_PinModeSet(BSP_COEX_REQ_PORT, BSP_COEX_REQ_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(BSP_COEX_GNT_PORT, BSP_COEX_GNT_PIN, gpioModeInput, 0);
	GPIO_PinModeSet(BSP_COEX_PRI_PORT, BSP_COEX_PRI_PIN, gpioModePushPull, 0);


}
#endif


/**************************************************************************//**
* @brief Function to handle buttons press and release actions
*****************************************************************************/
void handle_button_change(uint8_t pin)
{

	if(pin == BSP_BUTTON0_PIN)
	{
		if (!GPIO_PinInGet(BSP_BUTTON0_PORT,BSP_BUTTON0_PIN))
		{
			if(roleIsSlave)
			{
				/* PB0 pressed */
				gecko_external_signal(NOTIFICATIONS_START);
			}
			else
			{
				gecko_external_signal(PHY_CHANGE);
			}
		}
		else
		{
			if(roleIsSlave) {
				/* PB0 released */
				gecko_external_signal(NOTIFICATIONS_END);
			}
		}
	}
	else if (pin == BSP_BUTTON1_PIN)
	{
		if (!GPIO_PinInGet(BSP_BUTTON1_PORT,BSP_BUTTON1_PIN))
		{
			if(roleIsSlave) {
				/* PB1 pressed */
				gecko_external_signal(INDICATIONS_START);
			}
			else
			{
				gecko_external_signal(WRITE_NO_RESPONSE_START);
			}
		}
		else
		{
			if(roleIsSlave) {
				/* PB1 released */
				gecko_external_signal(INDICATIONS_END);
			}
			else
			{
				gecko_external_signal(WRITE_NO_RESPONSE_END);
			}
		}
	}
}


/**************************************************************************//**
* @brief Function to generate circular data (0-255) in the data payload
*****************************************************************************/
void generate_data_notifications(void){

	throughput_array_notifications[0] = throughput_array_notifications[maxDataSizeNotifications-1] + 1;

	for(int i = 1; i<maxDataSizeNotifications; i++)
	{
		throughput_array_notifications[i] = throughput_array_notifications[i-1] + 1;
	}
}


/**************************************************************************//**
* @brief Function to generate circular data (0-255) in the data payload
*****************************************************************************/
void generate_data_indications(void){

	throughput_array_indications[0] = throughput_array_indications[maxDataSizeIndications-1] + 1;

	for(int i = 1; i<maxDataSizeIndications; i++)
	{
		throughput_array_indications[i] = throughput_array_indications[i-1] + 1;
	}
}

/**************************************************************************//**
* @brief Processes advertisement packets looking for "Throughput Tester" device name
*****************************************************************************/
int process_scan_response(struct gecko_msg_le_gap_scan_response_evt_t *pResp)
{
	/* Decoding advertising packets is done here. The list of AD types can be found
	 * at: https://www.bluetooth.com/specifications/assigned-numbers/Generic-Access-Profile */

    int i = 0;
    int ad_match_found = 0;
	int ad_len;
    int ad_type;

    while (i < (pResp->data.len - 1))
    {
        ad_len  = pResp->data.data[i];
        ad_type = pResp->data.data[i+1];

        if (ad_type == 0x09)
        {
            /* type 0x09 = Complete Local Name */

        	/* Check if device name is Throughput Tester */
        	if(memcmp(pResp->data.data+i+2, deviceNameString, 17) == 0)
			{
        		ad_match_found = 1;
        		break;
			}
        }

        /* Jump to next AD record */
        i = i + ad_len + 1;
    }

    return(ad_match_found);
}

/**************************************************************************//**
* @brief Does a few things before initiating data transmissions. Read RTCC, disable
* display refresh in master side and turn ON LED indicating data transmission
*****************************************************************************/
void dataTransmissionStart(void)
{
	bitsSent = 0;
	throughput = 0;
	time_elapsed = RTCC_CounterGet();

	/* Turn OFF Display refresh on master side */
	gecko_cmd_gatt_write_characteristic_value_without_response(connection, gattdb_display_refresh, 1, &displayRefreshOff);

	/* Stop display refresh */
	gecko_cmd_hardware_set_soft_timer(0, SOFT_TIMER_DISPLAY_REFRESH_HANDLE, 0);

#ifdef USE_LED_FOR_DATA_SENDING_SIGNALING
	/* Turn ON data LED */
	GPIO_PinOutSet(BSP_LED1_PORT,BSP_LED1_PIN);
#endif
}

/**************************************************************************//**
* @brief Does a few after data transmissions ended. Calculate transmission time,
* enable display refresh in master side and turn OFF LED indicating data transmission
*****************************************************************************/
void dataTransmissionEnd(void)
{
	time_elapsed = RTCC_CounterGet() - time_elapsed;

	/* Turn ON Display on master side - stack is probably still busy pushing the last few notifications out so we need to check output */
	while(gecko_cmd_gatt_write_characteristic_value_without_response(connection, gattdb_display_refresh, 1, &displayRefreshOn)->result!=0);

	/* Resume display refresh - stack is probably still busy pushing the last few notifications out so we need to check output */
	while(gecko_cmd_hardware_set_soft_timer(32768, SOFT_TIMER_DISPLAY_REFRESH_HANDLE, 0)->result != 0);

#ifdef USE_LED_FOR_DATA_SENDING_SIGNALING
	/* Turn ON data LED */
	GPIO_PinOutClear(BSP_LED1_PORT,BSP_LED1_PIN);
#endif
	/* Calculate throughput */
	throughput = (uint32_t)((float)bitsSent / (float)((float)time_elapsed / (float)32768));
}

/**
 * @brief  Main function
 */
void main(void)
{
  struct gecko_msg_system_get_counters_rsp_t *getCounters;

  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();
 // initTxRXActive();

  // Initialize stack
  gecko_init(&config);

#ifdef USE_LED_FOR_CONNECTION_SIGNALING
  /* Configure LED0 to indicate if connection is established or not */
  GPIO_PinModeSet(BSP_LED0_PORT, BSP_LED0_PIN, gpioModePushPull, 0);
#endif

#ifdef USE_LED_FOR_DATA_SENDING_SIGNALING
  /* Configure LED0 to indicate if connection is established or not */
  GPIO_PinModeSet(BSP_LED1_PORT, BSP_LED1_PIN, gpioModePushPull, 0);
#endif

  /* Configure PB0 and PB1 as inputs on the WSTK */
  GPIO_PinModeSet(BSP_BUTTON0_PORT,BSP_BUTTON0_PIN,gpioModeInputPullFilter,1);
  GPIO_PinModeSet(BSP_BUTTON1_PORT,BSP_BUTTON1_PIN,gpioModeInputPullFilter,1);

  if(GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN))
  {
	  roleIsSlave = true;
	  roleString = (char*)roleSlaveString;
  } else {
	  roleIsSlave = false;
	  roleString = (char*)roleMasterString;
  }


#ifndef NODISPLAY
  /* Initialize interrupts */
  GPIOINT_Init();

  /* Configuring push buttons PB0 and PB1 on the WSTK to generate interrupt */
  GPIO_ExtIntConfig(BSP_BUTTON0_PORT,BSP_BUTTON0_PIN,6,true,true,true);
  GPIOINT_CallbackRegister(BSP_BUTTON0_PIN,handle_button_change);

  GPIO_ExtIntConfig(BSP_BUTTON1_PORT,BSP_BUTTON1_PIN,7,true,true,true);
  GPIOINT_CallbackRegister(BSP_BUTTON1_PIN,handle_button_change);

  GRAPHICS_Init();
#endif

  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    if(notifications_enabled && sendNotifications)
    {
    	evt = gecko_peek_event();

    	if(gecko_cmd_gatt_server_send_characteristic_notification(connection, gattdb_throughput_notifications, maxDataSizeNotifications, throughput_array_notifications)->result == 0)
		{
    		bitsSent += (maxDataSizeNotifications*8);
    		operationCount++;
    		generate_data_notifications();
#ifdef SEND_FIXED_TRANSFER_COUNT
    		if(++transferCount == SEND_FIXED_TRANSFER_COUNT) {
    			dataTransmissionEnd();
    			/* Stop sending notifications */
    			sendNotifications = false;
    		}
#endif
		}

    }
    else if(sendWriteNoResponse)
    {
    	evt = gecko_peek_event();

    	if(gecko_cmd_gatt_write_characteristic_value_without_response(connection, gattdb_throughput_write_no_response, maxDataSizeNotifications, throughput_array_notifications)->result == 0)
		{
    		bitsSent += (maxDataSizeNotifications*8);
    		operationCount++;
    		generate_data_notifications();
#ifdef SEND_FIXED_TRANSFER_COUNT
    		if(++transferCount == SEND_FIXED_TRANSFER_COUNT) {
    			dataTransmissionEnd();
    			/* Stop sending notifications */
    			sendWriteNoResponse = false;
    		}
#endif
		}
    }
    else
    {
    	/* Check for stack event. */
    	evt = gecko_wait_event();
    }

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {

      /* This boot event is generated when the system boots up after reset.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:

    	  RETARGET_SerialInit();
    	  gecko_cmd_hardware_set_soft_timer(3*32768,COEX_COUNTER_UPDATE,0);
			sprintf(connIntervalString+7, "%04u", 0);
			sprintf(phyInUseString+5, "%s", "1M");
			sprintf(mtuSizeString+5, "%03u", mtuSize);
			sprintf(pduSizeString+5, "%03u", pduSize);
			sprintf(maxDataSizeNotificationsString+11, "%03u", maxDataSizeNotifications);
			sprintf(invalidDataString+9, "%03u", invalidData);

			//gecko_cmd_coex_set_options(GECKO_COEX_OPTION_ENABLE, 1);
			gecko_cmd_gatt_server_write_attribute_value(gattdb_display_refresh, 0, 1, &displayRefreshOn);

			gecko_cmd_gatt_set_max_mtu(250);

			gecko_cmd_system_set_tx_power(TX_POWER);

			if(roleIsSlave) {

		/* Set advertising parameters. 100ms advertisement interval.
				 * The first parameter is advertising set handle
				 * The next two parameters are minimum and maximum advertising interval, both in
				 * units of (milliseconds * 1.6).
				 * The last two parameters are duration and maxevents left as default. */

				gecko_cmd_le_gap_set_advertise_timing(0, ADV_INTERVAL_MIN, ADV_INTERVAL_MAX, 0, 0);

		        /* Start general advertising and enable connections. */
		        gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
			} else {

				gecko_cmd_le_gap_set_conn_timing_parameters(CONN_INTERVAL_1MPHY_MIN, CONN_INTERVAL_1MPHY_MAX, SLAVE_LATENCY_1MPHY, SUPERVISION_TIMEOUT_1MPHY, 0, 0);

				/* Set scan parameters and start scanning */

				gecko_cmd_le_gap_set_discovery_timing(1, SCAN_INTERVAL, SCAN_WINDOW);

				gecko_cmd_le_gap_set_discovery_type(1, ACTIVE_SCANNING);

				gecko_cmd_le_gap_start_discovery(1, le_gap_discover_generic);
			}

			gecko_cmd_hardware_set_soft_timer(32768, SOFT_TIMER_DISPLAY_REFRESH_HANDLE, 0);

        break;

      case gecko_evt_le_connection_opened_id:
#ifdef USE_LED_FOR_CONNECTION_SIGNALING
    	  /* Turn ON connection LED */
    	  GPIO_PinOutSet(BSP_LED0_PORT,BSP_LED0_PIN);
#endif
    	  break;

      case gecko_evt_le_connection_closed_id:
#ifdef USE_LED_FOR_CONNECTION_SIGNALING
    	  	/* Turn off connection LED */
    	    GPIO_PinOutClear(BSP_LED0_PORT,BSP_LED0_PIN);
#endif

			/* Clear all flags and relevant parameters */
			connection = 0;
			mtuSize = 0;
			pduSize = 0;
			maxDataSizeNotifications = 0;
			invalidData = 0;
			operationCount = 0;
			indications_enabled = false;
			notifications_enabled = false;
			throughput = 0;
			enableNotificationsIndications = 0;
			phyInUse = PHY_1M;
			phyToUse = 0;

			sprintf(connIntervalString+7, "%04u", 0);
			sprintf(phyInUseString+5, "%s", "1M");
			sprintf(mtuSizeString+5, "%03u", mtuSize);
			sprintf(pduSizeString+5, "%03u", pduSize);
			sprintf(maxDataSizeNotificationsString+11, "%03u", maxDataSizeNotifications);
			sprintf(invalidDataString+9, "%03u", invalidData);

			statusString = (char*)statusDisconnectedString;
			notifyString = (char*)notifyDisabledString;
			indicateString = (char*)indicateDisabledString;

			/* Reset data */
			memset(throughput_array_notifications, 0, DATA_SIZE);
			memset(throughput_array_indications, 0, DATA_SIZE);

			if(roleIsSlave) {
				/* Check if need to boot to dfu mode */
				if (boot_to_dfu) {
					/* Enter to DFU OTA mode */
					gecko_cmd_system_reset(2);
				}
				else {
					/* Restart advertising after client has disconnected */

					gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_undirected_connectable);

				}
			} else {
				/* Back to scanning */

				gecko_cmd_le_gap_start_discovery(1, le_gap_discover_generic);
			}
        break;

      case gecko_evt_gatt_server_characteristic_status_id:

		  if(evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_throughput_notifications)
		  {
			  if(evt->data.evt_gatt_server_characteristic_status.status_flags == gatt_server_client_config &&
				 evt->data.evt_gatt_server_characteristic_status.client_config_flags == gatt_notification)
			  {
				  notifications_enabled = true;
				  notifyString = (char*)notifyEnabledString;
			  }

			  if(evt->data.evt_gatt_server_characteristic_status.status_flags == gatt_server_client_config &&
				 evt->data.evt_gatt_server_characteristic_status.client_config_flags == gatt_disable)
			  {
				  notifications_enabled = false;
				  notifyString = (char*)notifyDisabledString;
			  }

		  }

		  if(evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_throughput_indications)
		  {
			  if(evt->data.evt_gatt_server_characteristic_status.status_flags == gatt_server_client_config &&
				 evt->data.evt_gatt_server_characteristic_status.client_config_flags == gatt_indication)
			  {
				  indications_enabled = true;
				  indicateString = (char*)indicateEnabledString;
			  }

			  if(evt->data.evt_gatt_server_characteristic_status.status_flags == gatt_server_client_config &&
				 evt->data.evt_gatt_server_characteristic_status.client_config_flags == gatt_disable)
			  {
				  indications_enabled = false;
				  indicateString = (char*)indicateDisabledString;
			  }

			  if(evt->data.evt_gatt_server_characteristic_status.status_flags == gatt_server_confirmation)
			  {
				  /* Last indicate operation was acknowledged, send more data */
				  bitsSent += ((maxDataSizeIndications)*8);
				  operationCount++;
				  generate_data_indications();
#ifdef SEND_FIXED_TRANSFER_COUNT
				  if(++transferCount == SEND_FIXED_TRANSFER_COUNT) {
					  dataTransmissionEnd();
					  // Stop sending indications
					  sendIndications = false;
				  }
#endif
				  if(indications_enabled && sendIndications)
				  {
					  while(gecko_cmd_gatt_server_send_characteristic_notification(connection, gattdb_throughput_indications, maxDataSizeIndications, throughput_array_indications)->result != 0);
				  }
			  }
		  }

    	  break;

      case gecko_evt_gatt_characteristic_value_id:

    	  /* Data received */
    	  if(evt->data.evt_gatt_characteristic_value.att_opcode == gatt_handle_value_indication) {
    		  gecko_cmd_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);
    	  }

    	  bitsSent += (evt->data.evt_gatt_characteristic_value.value.len*8);
    	  operationCount++;

    	  /* Validate the data */
    	  for(int i=1; i<evt->data.evt_gatt_characteristic_value.value.len; i++)
    	  {
    		  if(evt->data.evt_gatt_characteristic_value.value.data[i] != (uint8)((evt->data.evt_gatt_characteristic_value.value.data[i-1])+1))
    		  {
				  /* Data is not what we expected */
				  invalidData++;
    		  }
    	  }

    	  break;

      case gecko_evt_gatt_server_attribute_value_id:

    	  if(evt->data.evt_gatt_server_attribute_value.attribute == gattdb_display_refresh)
    	  {
			  /* Display ON/OFF state changes */
			  if(evt->data.evt_gatt_server_attribute_value.value.data[0] == 0)
			  {
				  bitsSent = 0;
				  throughput = 0;
				  time_elapsed = RTCC_CounterGet();
				  /* Disable display refresh */
				  gecko_cmd_hardware_set_soft_timer(0, SOFT_TIMER_DISPLAY_REFRESH_HANDLE, 0);
				  /* Turn ON data LED */
				  GPIO_PinOutSet(BSP_LED1_PORT,BSP_LED1_PIN);

			  }
			  else
			  {
				  time_elapsed = RTCC_CounterGet() - time_elapsed;
				  /* Enable display refresh */
				  gecko_cmd_hardware_set_soft_timer(32768, SOFT_TIMER_DISPLAY_REFRESH_HANDLE, 0);
				  /* Turn OFF data LED */
				  GPIO_PinOutClear(BSP_LED1_PORT,BSP_LED1_PIN);
				  /* Calculate throughput */
				  throughput = (uint32_t)((float)bitsSent / (float)((float)time_elapsed / (float)32768));
				  //throughput = bitsSent / (time_elapsed / 32768);

			  }
    	  }

    	  if(evt->data.evt_gatt_server_attribute_value.attribute == gattdb_throughput_write_no_response)
    	  {
        	  bitsSent += (evt->data.evt_gatt_server_attribute_value.value.len*8);
        	  operationCount++;

        	  /* Validate the data */
        	  for(int i=1; i<evt->data.evt_gatt_server_attribute_value.value.len; i++)
        	  {
        		  if(evt->data.evt_gatt_server_attribute_value.value.data[i] != (uint8)((evt->data.evt_gatt_server_attribute_value.value.data[i-1])+1))
        		  {
    				  /* Data is not what we expected */
    				  invalidData++;
        		  }
        	  }
    	  }
    	  break;

      case gecko_evt_hardware_soft_timer_id:

    	  switch(evt->data.evt_hardware_soft_timer.handle)
    	  {
    	  struct gecko_msg_coex_get_counters_rsp_t *coex_counter_rsp;
			  case SOFT_TIMER_DISPLAY_REFRESH_HANDLE:

		    	  if(gecko_cmd_le_connection_get_rssi(connection)->result != 0) {
		    		  // Command didn't go through, most likely out of memory error
		    		  //sprintf(statusConnectedString+6, "ERR");
		    	  }

#ifndef NODISPLAY
		    	  displayRefresh();
#endif
	    		  //bitsSent = 0;
				  break;

			  case SOFT_TIMER_FIXED_TRANSFER_TIME_HANDLE:
				  dataTransmissionEnd();
				  sendNotifications = false;
				  sendIndications = false;
				  sendWriteNoResponse = false;
				  break;
			  case COEX_COUNTER_UPDATE:
				  coex_counter_rsp = gecko_cmd_coex_get_counters(1);
				  printf("lp requests %d, hp requests %d, lp denials %d, hp denials %d\r\n ",
						  coex_counter_rsp->counters.data[0],
						  coex_counter_rsp->counters.data[4],
						  coex_counter_rsp->counters.data[8],
						  coex_counter_rsp->counters.data[12]);
				  break;
			  default:
				  break;
    	  }
	  break;

	  case gecko_evt_le_connection_rssi_id:
		 // sprintf(statusConnectedString+6, "%03d", evt->data.evt_le_connection_rssi.rssi);
		  break;

#if 1
      case gecko_evt_le_connection_phy_status_id:
    	  	  phyToUse = 0;
    	  	  phyInUse = evt->data.evt_le_connection_phy_status.phy;
    		  switch(phyInUse) {
				  case PHY_1M:
					  sprintf(phyInUseString+5, "%s", "1M");
					  break;
				  case PHY_2M:
					  sprintf(phyInUseString+5, "%s", "2M");
					  break;
				  case PHY_S8:
					  sprintf(phyInUseString+5, "%s", "S8");
					  break;
				  case PHY_S2:
					  sprintf(phyInUseString+5, "%s", "S2");
					  break;
				  default:
					  break;
			  }
    	  break;

      case gecko_evt_gatt_mtu_exchanged_id:

    	  mtuSize = evt->data.evt_gatt_mtu_exchanged.mtu;

    	  sprintf(mtuSizeString+5, "%03u", mtuSize);

    	  connection = evt->data.evt_gatt_mtu_exchanged.connection;

    	  if(DATA_TRANSFER_SIZE_INDICATIONS == 0 || DATA_TRANSFER_SIZE_INDICATIONS > (mtuSize-3))
    	  {
    		  maxDataSizeIndications = mtuSize-3;
    	  }
    	  else
    	  {
    		  maxDataSizeIndications = DATA_TRANSFER_SIZE_INDICATIONS;
    	  }

    	  if(DATA_TRANSFER_SIZE_NOTIFICATIONS == 0 || DATA_TRANSFER_SIZE_NOTIFICATIONS > (mtuSize-3))
    	  {
			  if(pduSize!=0 && mtuSize!=0) {
				  if(pduSize <= mtuSize)
				  {
					  maxDataSizeNotifications = (pduSize - 7) + ((mtuSize - 3 - pduSize + 7) / pduSize * pduSize);
				  }
				  else
				  {

					  if(pduSize-mtuSize<=4)
					  {
						  maxDataSizeNotifications = pduSize - 7;
					  } else {
						  maxDataSizeNotifications = mtuSize - 3;
					  }
				  }
			  }
    	  }
    	  else
    	  {
    		  maxDataSizeNotifications = DATA_TRANSFER_SIZE_NOTIFICATIONS;
    	  }
    	  sprintf(maxDataSizeNotificationsString+11, "%03u", maxDataSizeNotifications);

    	  if(!roleIsSlave) {
			  /* For the sake of simplicity we'll just assume that the CCCD handle for the indication
			   * and notification characteristics is the characteristic handle + 1
			   */
			  enableNotificationsIndications = 1;
			  gecko_cmd_gatt_write_descriptor_value(connection, gattdb_throughput_notifications+1, 1, &enableNotificationsIndications);
    	  }
    	  break;
#endif
      case gecko_evt_gatt_procedure_completed_id:

    	  if(enableNotificationsIndications == 1) {
    		  notifications_enabled = 1;
    		  enableNotificationsIndications = 2;
    		  gecko_cmd_gatt_write_descriptor_value(connection, gattdb_throughput_indications+1, 1, &enableNotificationsIndications);
    	  }

    	  if(enableNotificationsIndications == 2) {
    		  indications_enabled = 2;
    	  }
    	  break;

      case gecko_evt_le_connection_parameters_id:

    	  pduSize = evt->data.evt_le_connection_parameters.txsize;
    	  sprintf(pduSizeString+5, "%03u", pduSize);
    	  sprintf(connIntervalString+7, "%04u", (unsigned int)((float)evt->data.evt_le_connection_parameters.interval*1.25));
    	  statusString = (char*)statusConnectedString;


    	  if(DATA_TRANSFER_SIZE_NOTIFICATIONS == 0 || DATA_TRANSFER_SIZE_NOTIFICATIONS > (mtuSize-3))
    	  {
			  if(pduSize!=0 && mtuSize!=0) {
				  if(pduSize <= mtuSize)
				  {
					  maxDataSizeNotifications = (pduSize - 7) + ((mtuSize - 3 - pduSize + 7) / pduSize * pduSize);
				  }
				  else
				  {
					  if(pduSize-mtuSize<=4)
					  {
						  maxDataSizeNotifications = pduSize - 7;
					  } else {
						  maxDataSizeNotifications = mtuSize - 3;
					  }
				  }
			  }
    	  }
    	  else
		  {
			  maxDataSizeNotifications = DATA_TRANSFER_SIZE_NOTIFICATIONS;
		  }
    	  sprintf(maxDataSizeNotificationsString+11, "%03u", maxDataSizeNotifications);

    	  /* Change phy if request */
    	  if(phyToUse) {
    		  gecko_cmd_le_connection_set_preferred_phy(connection, phyToUse, phyToUse);
    	  }
    	  break;

      case gecko_evt_system_external_signal_id:

    	  switch (evt->data.evt_system_external_signal.extsignals)
    	  {
    	  	  case NOTIFICATIONS_START:

    	  		  dataTransmissionStart();
    	  		  sendNotifications = true;
    	  		  generate_data_notifications();
#if defined(SEND_FIXED_TRANSFER_COUNT)
    	  		  transferCount = 0;
#elif defined(SEND_FIXED_TRANSFER_TIME)
    	  		  gecko_cmd_hardware_set_soft_timer(SEND_FIXED_TRANSFER_TIME, SOFT_TIMER_FIXED_TRANSFER_TIME_HANDLE, 1);
#endif
    	  		  getCounters = gecko_cmd_system_get_counters(1);
    	  		  break;

    	  	  case NOTIFICATIONS_END:

#if !defined(SEND_FIXED_TRANSFER_COUNT) && !defined(SEND_FIXED_TRANSFER_TIME)
    	  		  dataTransmissionEnd();
    	  		  sendNotifications = false;
    	  		  getCounters = gecko_cmd_system_get_counters(1);

#endif
    	  		  break;

    	  	  case WRITE_NO_RESPONSE_START:
    	  		  dataTransmissionStart();
    	  		  sendWriteNoResponse = true;
    	  		  generate_data_notifications();
#if defined(SEND_FIXED_TRANSFER_COUNT)
    	  		  transferCount = 0;
#elif defined(SEND_FIXED_TRANSFER_TIME)
    	  		  gecko_cmd_hardware_set_soft_timer(SEND_FIXED_TRANSFER_TIME, SOFT_TIMER_FIXED_TRANSFER_TIME_HANDLE, 1);
#endif
    	  		  break;

    	  	  case WRITE_NO_RESPONSE_END:
#if !defined(SEND_FIXED_TRANSFER_COUNT) && !defined(SEND_FIXED_TRANSFER_TIME)
    	  		  dataTransmissionEnd();
    	  		  sendWriteNoResponse = false;

#endif
				  break;

    	  	  case INDICATIONS_START:

    	  		  dataTransmissionStart();
    	  		  sendIndications = true;
#if defined(SEND_FIXED_TRANSFER_COUNT)
    	  		  transferCount = 0;
#elif defined(SEND_FIXED_TRANSFER_TIME)
    	  		  gecko_cmd_hardware_set_soft_timer(SEND_FIXED_TRANSFER_TIME, SOFT_TIMER_FIXED_TRANSFER_TIME_HANDLE, 1);
#endif
    	  		  if(indications_enabled)
    	  		  {
    	  			  generate_data_indications();
        	  		  while(gecko_cmd_gatt_server_send_characteristic_notification(connection, gattdb_throughput_indications, maxDataSizeIndications, throughput_array_indications)->result != 0);
    	  		  }
    	  		  break;

    	  	  case INDICATIONS_END:

#if !defined(SEND_FIXED_TRANSFER_COUNT) && !defined(SEND_FIXED_TRANSFER_TIME)
    	  		  dataTransmissionEnd();
    	  		  sendIndications = false;
#endif
    	  		  break;

    	  	  case PHY_CHANGE:
    	  		  switch(phyInUse) {
    	  		  case PHY_1M:
#if defined(_SILICON_LABS_32B_SERIES_1_CONFIG_2) || defined(_SILICON_LABS_32B_SERIES_1_CONFIG_3)
    	  			  /* We're on 1M PHY, go to 2M PHY - only supported by xG12 and xG13 */
    	  			  phyToUse = PHY_2M;
    	  			  /* Change connection parameters for 2MPHY */
    	  			gecko_cmd_le_connection_set_timing_parameters(connection, CONN_INTERVAL_2MPHY_MIN, CONN_INTERVAL_2MPHY_MAX, SLAVE_LATENCY_2MPHY, SUPERVISION_TIMEOUT_2MPHY,0,0);
#endif
    	  			  break;

    	  		  case PHY_2M:
#if defined(_SILICON_LABS_32B_SERIES_1_CONFIG_3)
    	  			  /* We're on 2M PHY, go to 125kbit Coded PHY (S=8) - only supported by xG13 */
    	  			  phyToUse = PHY_S8;
    	  			  /* Change connection parameters according to set_phy command description
					   * in API Ref. Minimum connection interval for LE Coded PHY is 40ms */
    	  			gecko_cmd_le_connection_set_timing_parameters(connection, CONN_INTERVAL_125KPHY_MIN, CONN_INTERVAL_125KPHY_MAX, SLAVE_LATENCY_125KPHY, SUPERVISION_TIMEOUT_125KPHY,0,0);
#else
					  /* We're on 2MPHY but with xG12, go back to 1M PHY */
					  phyToUse = PHY_1M;
					  /* Change connection parameters back to the minimum */
					  gecko_cmd_le_connection_set_parameters(connection, CONN_INTERVAL_1MPHY_MIN, CONN_INTERVAL_1MPHY_MAX, SLAVE_LATENCY_1MPHY, SUPERVISION_TIMEOUT_1MPHY);
#endif
    	  			  break;

    	  		  case PHY_S8:
#if defined(_SILICON_LABS_32B_SERIES_1_CONFIG_3)
    	  			  /* We're on S8 PHY, go back to 1M PHY */
    	  			  phyToUse = PHY_1M;
    	  			  /* Change connection parameters back to the minimum */
    	  			gecko_cmd_le_connection_set_timing_parameters(connection, CONN_INTERVAL_1MPHY_MIN, CONN_INTERVAL_1MPHY_MAX, SLAVE_LATENCY_1MPHY, SUPERVISION_TIMEOUT_1MPHY,0,0);
#endif
    	  			  break;

    	  		  default:
    	  			  break;
    	  		  }
    	  		  break;
    	  	  default:
    	  		  break;
    	  }

    	break;

	  case gecko_evt_le_gap_scan_response_id:
			/* process scan responses: this function returns 1 if we found the "Throughput Tester" device name */
			if(process_scan_response(&(evt->data.evt_le_gap_scan_response)) > 0) {
				/* Match found - stop scanning and connect */
				gecko_cmd_le_gap_end_procedure();

				gecko_cmd_le_gap_connect(evt->data.evt_le_gap_scan_response.address, evt->data.evt_le_gap_scan_response.address_type, 1);
			}
		break;

	  case gecko_evt_gatt_server_user_read_request_id:
	  		if(evt->data.evt_gatt_server_user_read_request.characteristic==gattdb_TestTime)
	  		{
	  		  gecko_cmd_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection , evt->data.evt_gatt_server_user_read_request.characteristic , 0 , sizeof(testTime), (uint8_t const*)&testTime);
	  		}
	  		break;


		/* Events related to OTA upgrading
      ----------------------------------------------------------------------------- */

      /* Check if the user-type OTA Control Characteristic was written.
       * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
      case gecko_evt_gatt_server_user_write_request_id:

      if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_TestTime) {
        /* Change Global variable tied to Characteristic */
         testTime = (uint32_t) evt->data.evt_gatt_server_attribute_value.value.data[0];

        /* Send response to Write Request */
        gecko_cmd_gatt_server_send_user_write_response( evt->data.evt_gatt_server_user_write_request.connection, gattdb_TestTime, bg_err_success);

      }

        if(evt->data.evt_gatt_server_user_write_request.characteristic==gattdb_ota_control)
        {
          /* Set flag to enter to OTA mode */
          boot_to_dfu = 1;
          /* Send response to Write Request */
          gecko_cmd_gatt_server_send_user_write_response(
            evt->data.evt_gatt_server_user_write_request.connection,
            gattdb_ota_control,
            bg_err_success);

          /* Close connection to enter to DFU OTA mode */
          //gecko_cmd_endpoint_close(evt->data.evt_gatt_server_user_write_request.connection);
          gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
        }
        break;

      default:
        break;
    }
  }
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */

