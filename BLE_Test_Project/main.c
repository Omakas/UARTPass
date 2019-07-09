/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

   /************************************************************************************************
   Name:		Avnet_LSM6DS0
   Sphere OS:	19.02
   This file contains the 'main' function. Program execution begins and ends there

   Author:
   Peter Fenn (Avnet Engineering & Technology)

   Purpose:
   Sphere Starter Kit accelerometer test 

   Description:
   ...

   *************************************************************************************************/

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
#include <applibs/uart.h>
#include <applibs/networking.h>

/* #include "mt3620_rdb.h" */
#include "mt3620_avnet_dev.h"
#include "mt3620_rdb.h"

//--------- Support for LPS22HH added ---------//

// Support functions.
static int InitPeripheralsAndHandlers(void);
static void ClosePeripheralsAndHandlers(void);
static void SendUartMessage(int uartFd, const char *dataToSend);

static char str[512];

// File descriptors - initialized to invalid value
static int epollFd = -1;
static int uartFd = -1;

// Termination state
static volatile sig_atomic_t terminationRequired = false;

static void TerminationHandler(int signalNumber)
{
	// Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
	terminationRequired = true;
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

	uartFd = UART_Open(MT3620_UART_ISU0, &uartConfig);
	SendUartMessage(uartFd, "Uart is up\n");

    epollFd = CreateEpollFd();
    if (epollFd < 0) {
        return -1;
    }
    return 0;
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
	snprintf(str, sizeof(str), "Sent %zu bytes over UART in %d calls.\n", totalBytesSent, sendIterations );
	Log_Debug(str);
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
    Log_Debug("Closing file descriptors.\n");
	SendUartMessage(uartFd, "Closing file descriptors.\n");
	CloseFdAndPrintError(uartFd, "Uart");
    CloseFdAndPrintError(epollFd, "Epoll");
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
    return 0;
}
