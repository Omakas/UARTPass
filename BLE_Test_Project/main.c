/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* For socket communications (UDP) */
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <sys/types.h>

/* applibs_versions.h defines the API struct versions to use for applibs APIs. */
#include "applibs_versions.h"
#include "epoll_timerfd_utilities.h"

#include <applibs/log.h>
/* This is here due to the documenation from microsoft 
 * See URL https://docs.microsoft.com/en-us/azure-sphere/app-development/uart#feedback
 */
//#define UART_STRUCTS_VERSION 1
#include <applibs/uart.h>
#include <applibs/networking.h>

/* #include "mt3620_rdb.h */
#include "mt3620_avnet_dev.h"
#include "mt3620_rdb.h"

/* Uncomment the below define to enable debugging of the debug UART */
//#define DEBUG_DEBUG_UART

#define ERROR_RET		( -1 )
#define SUCCESS_RET		( 0 )

//--------- Support for LPS22HH added ---------//

// Support functions.
static int InitPeripheralsAndHandlers(void);
static void ClosePeripheralsAndHandlers(void);
static void SendUartMessage(int uartFd, const char *dataToSend);
static void uartEventDebugData(EventData *eventData);
static void uartEventBleData(EventData *eventData);

static char str[512];

// File descriptors - initialized to invalid value
static int epollFd = -1;
static int uartFd = -1;
static int uartBleFd = -1;

// Termination state
static volatile sig_atomic_t terminationRequired = false;

static void TerminationHandler(int signalNumber)
{
	// Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
	terminationRequired = true;
}

/// <summary>
///     Handle UART event: if there is incoming data, print it, and blink the LED.
/// </summary>
static void uartEventDebugData(EventData *eventData)
{
	const size_t receiveBufferSize = 256;
	uint8_t receiveBuffer[receiveBufferSize + 1]; // allow extra byte for string termination
	ssize_t bytesRead;

	// Read UART message
	bytesRead = read(uartFd, receiveBuffer, receiveBufferSize);
	if (bytesRead < 0) {
		Log_Debug("ERROR: Could not read UART: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (bytesRead > 0) {
		// Null terminate the buffer to make it a valid string, and print it
		receiveBuffer[bytesRead] = 0;

		/* Pass the data to the debug port */
		SendUartMessage(uartFd, ( char* )receiveBuffer);
	}
}

/// <summary>
///     Handle UART event: if there is incoming data, print it, and blink the LED.
/// </summary>
static void uartEventBleData(EventData *eventData)
{
	const size_t receiveBufferSize = 256;
	uint8_t receiveBuffer[receiveBufferSize + 1]; // allow extra byte for string termination
	ssize_t bytesRead;

	// Read UART message
	bytesRead = read(uartFd, receiveBuffer, receiveBufferSize);
	if (bytesRead < 0) {
		Log_Debug("ERROR: Could not read UART: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (bytesRead > 0) {
		// Null terminate the buffer to make it a valid string, and print it
		receiveBuffer[bytesRead] = 0;

		/* Pass the data to the Nordic */
		SendUartMessage(uartBleFd, (char*)receiveBuffer);
	}
}

/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int InitPeripheralsAndHandlers(void)
{
	struct sigaction action;
	memset(&action, 0, sizeof(struct sigaction));
	action.sa_handler = TerminationHandler;
	sigaction(SIGTERM, &action, NULL);

	// Create a UART_Config object, open the UART and set up UART event handler
	UART_Config uartConfig;
	UART_InitConfig(&uartConfig);
	uartConfig.baudRate = 115200;
	uartConfig.flowControl = UART_FlowControl_None;
	uartConfig.dataBits = UART_DataBits_Eight;
	
	UART_Config uartConfig1;
	UART_InitConfig(&uartConfig1);
	uartConfig1.baudRate = 115200;
	uartConfig1.dataBits = UART_DataBits_Eight;
	uartConfig1.flowControl = UART_FlowControl_None;

	uartFd = UART_Open(MT3620_UART_ISU0, &uartConfig);
	if (uartFd > 0)
	{
		SendUartMessage(uartFd, "Uart is up\n");
	}
	else
	{
		Log_Debug("Failed to bring up the debug UART\n");
		return ERROR_RET;
	}

	if (RegisterEventHandlerToEpoll(epollFd, uartFd, &uartEventDebugData, EPOLLIN) != 0) {
		return ERROR_RET;
	}

	//Enable the HW Flow control for the BLE module
	
	uartBleFd = UART_Open(MT3620_UART_ISU1, &uartConfig1);
	if (uartBleFd > 0)
	{
		SendUartMessage(uartFd, "BLE Uart is up\n");
	}
	else
	{
		SendUartMessage(uartFd, "Failed to bring up the BLE UART\n");
		return ERROR_RET;
	}

	if (RegisterEventHandlerToEpoll(epollFd, uartBleFd, &uartEventBleData, EPOLLIN) != 0) {
		return ERROR_RET;
	}

    epollFd = CreateEpollFd();
    if (epollFd < 0) 
	{
		SendUartMessage(uartFd, "Failed to bring up the epollFD\n");
        return ERROR_RET;
    }
    return SUCCESS_RET;
}

/// <summary>
///     Helper function to send a fixed message via the given UART.
/// </summary>
/// <param name="uartFd">The open file descriptor of the UART to write to</param>
/// <param name="dataToSend">The data to send over the UART</param>
static void SendUartMessage(int uartFd, const char *dataToSend)
{
	size_t totalBytesSent = 0;
	size_t totalBytesToSend = strlen(dataToSend);
	int sendIterations = 0;
	while (totalBytesSent < totalBytesToSend) {
		sendIterations++;

		// Send as much of the remaining data as possible
		size_t bytesLeftToSend = totalBytesToSend - totalBytesSent;
		const char *remainingMessageToSend = dataToSend + totalBytesSent;
		ssize_t bytesSent = write(uartFd, remainingMessageToSend, bytesLeftToSend);
		if (bytesSent < 0) {
			snprintf(str, sizeof(str), "ERROR: Could not write to UART: %s (%d).\n", strerror(errno), errno);
			Log_Debug( str );
			SendUartMessage(uartFd, str );
			terminationRequired = true;
			return;
		}
		totalBytesSent += (size_t)bytesSent;
	}
#if defined(DEBUG_DEBUG_UART)
	snprintf(str, sizeof(str), "Sent %zu bytes over UART in %d calls.\n", totalBytesSent, sendIterations );
	Log_Debug(str);
#endif
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
    Log_Debug("Closing file descriptors.\n");
	SendUartMessage(uartFd, "Closing file descriptors.\n");
	CloseFdAndPrintError(uartBleFd, "Uart1");
    CloseFdAndPrintError(epollFd, "Epoll");
	CloseFdAndPrintError(uartFd, "Uart");
}

/// <summary>
///     Main entry point for this application.
/// </summary>
int main(int argc, char *argv[])
{		
	Log_Debug("I2C accelerometer application starting.\n");
    if (InitPeripheralsAndHandlers() != 0) {
        terminationRequired = true;
    }

	SendUartMessage(uartFd, "I2C accelerometer application starting.\n");

    // Use epoll to wait for events and trigger handlers, until an error or SIGTERM happens
    while (!terminationRequired) {
        if (WaitForEventAndCallHandler(epollFd) != 0) {
            terminationRequired = true;
        }
    }

	Log_Debug("Application exiting.\n");
    ClosePeripheralsAndHandlers();
    return SUCCESS_RET;
}
