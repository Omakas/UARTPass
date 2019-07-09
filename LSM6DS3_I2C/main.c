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
#include <applibs/i2c.h>
#include <applibs/gpio.h>
#include <applibs/uart.h>
#include <applibs/networking.h>

/* #include "mt3620_rdb.h" */
#include "mt3620_avnet_dev.h"
#include "mt3620_rdb.h"

typedef struct {
	GPIO_Id gpio;
	int Fd;
	GPIO_Value currentValue;
} GPIO_STRUCT;

//--------- Support for LPS22HH added ---------//

#define US_TO_NANO(X)			( X * 1000 )

// I2C Device Address 8 bit format: if SA0=0 -> 0xB9 if SA0=1 -> 0xBB 
#define LPS22HB_I2C_ADD_H  ( 0xBBu )
#define LPS22HB_I2C_ADD_L  ( 0xB9u )
// Device Identification (Who am I) 
#define LPS22HB_WHO_AM_I   ( 0x0Fu )  /* register address */
#define LPS22HB_ID         ( 0xB3u ) /* register value */

#define GPIO_TOGGLE_SECONDS_TIME_SPAN		( 0 )
#define GPIO_TOGGLE_MSECONDS_TIME_SPAN		(US_TO_NANO(100))
#define GPIO_TOGGLE_NANO_SECONDS_TIME_SPAN	(GPIO_TOGGLE_MSECONDS_TIME_SPAN)

#define UDP_MESSAGE_SECONDS_TIME_SPAN		( 1u )
#define UDP_MESSAGE_NANO_SECONDS_TIME_SPAN	( 5000000L )

// This sample C application for the MT3620 Reference Development Board (Azure Sphere)
// uses the Azure Sphere I2C APIs to display data from an accelerometer connected via I2C.
//
// It uses the APIs for the following Azure Sphere application libraries:
// - log (messages shown in Visual Studio's Device Output window during debugging)
// - i2c (communicates with LSM6DS3 accelerometer)

// Support functions.
static void TerminationHandler(int signalNumber);
static void AccelTimerEventHandler(EventData *eventData);
static int ReadWhoAmI(void);
static bool CheckTransferSize(const char *desc, size_t expectedBytes, ssize_t actualBytes);
static int InitPeripheralsAndHandlers(void);
static void ClosePeripheralsAndHandlers(void);
static void SendUartMessage(int uartFd, const char *dataToSend);
static void gpioToggleEventHandler(EventData *eventData);
static int connectToServer(void);
static void UdpTxEventHandler(EventData *eventData);

/* NOTE: Not using the defines from the header since we are currently short on time */
static GPIO_STRUCT gpioArr[] = 
{
	{.gpio = 0,.Fd = -1,.currentValue  = GPIO_Value_Low },
	{.gpio = 1,.Fd = -1,.currentValue  = GPIO_Value_Low },
	{.gpio = 2,.Fd = -1,.currentValue  = GPIO_Value_Low },
	{.gpio = 3,.Fd = -1,.currentValue  = GPIO_Value_Low },
	{.gpio = 4,.Fd = -1,.currentValue  = GPIO_Value_Low },
	{.gpio = 5,.Fd = -1,.currentValue  = GPIO_Value_Low },
	{.gpio = 7,.Fd = -1,.currentValue  = GPIO_Value_Low },
	{.gpio = 8,.Fd = -1,.currentValue  = GPIO_Value_Low },
	{.gpio = 9,.Fd = -1,.currentValue  = GPIO_Value_Low },
	{.gpio = 10,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 12,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 13,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 15,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 16,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 17,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 18,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 19,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 20,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 21,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 22,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 23,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 31,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 32,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 33,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 34,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 35,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 41,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 42,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 43,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 44,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 56,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 57,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 58,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 59,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 60,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 66,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 67,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 68,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 69,.Fd = -1,.currentValue = GPIO_Value_Low },
	{.gpio = 70,.Fd = -1,.currentValue = GPIO_Value_Low }
};

static char str[512];
static const unsigned int GPIO_ARRAY_SIZE = sizeof(gpioArr) / sizeof(gpioArr[0]);

// event handler data structures. Only the event handler field needs to be populated.
static EventData accelEventData = { .eventHandler = &AccelTimerEventHandler };
static EventData GpioToggleEvent = { .eventHandler = &gpioToggleEventHandler };
static EventData UdpTxEvent = { .eventHandler = &UdpTxEventHandler };

// File descriptors - initialized to invalid value
static int epollFd = -1;
static int accelTimerFd = -1;
static int i2cFd = -1;
static int uartFd = -1;
static int gpioToggleTimerFd = -1;
static int UdpTimerFd = -1;
static int sockfd = -1;
static struct sockaddr_in serv_addr;
static struct hostent *server;

static const char UdpMessage[512] = "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Donec arcu lorem, volutpat et sollicitudin eu, consequat et magna. Sed et tincidunt ante. In hac habitasse platea dictumst. Quisque finibus dapibus metus, nec fermentum elit tempus vitae. Pellentesque habitant morbi tristique senectus et netus et malesuada fames ac turpis egestas. Nunc gravida aliquet pharetra. Proin malesuada sapien odio, non ullamcorper ante placerat eget. Morbi finibus pharetra odio ut fringilla. Etiam pulvinar feugiat semper amet";
static char IpAddress[20] = "192.168.1.64";
static int portNumber = 2000;
static bool isConnected = false;

// DocID026899 Rev 10, S6.1.1, I2C operation
// SDO is tied to ground so the least significant bit of the address is zero.
static const uint8_t lsm6ds3Address = 0x6Au;     /* Addr = 0x6A */
// static const uint8_t lsm6ds3Address = 0x29;  /* Addr = 0x52  */
// static const uint8_t lsm6ds3Address = 0x7E;

// Termination state
static volatile sig_atomic_t terminationRequired = false;

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    terminationRequired = true;
}

static void UdpTxEventHandler(EventData *eventData)
{
	if (ConsumeTimerFdEvent(UdpTimerFd) != 0) {
		terminationRequired = true;
		return;
	}

	if (!isConnected)
	{
		connectToServer();
		SendUartMessage(uartFd, "Not Conected\n");
		if (!isConnected)
		{
			SendUartMessage(uartFd, "Not Conected after tring\n");
			return;
		}
	}

	int n = write(sockfd, UdpMessage, sizeof(UdpMessage) / sizeof(UdpMessage[0]));
	if (n < 0)
	{
		isConnected = false;
		SendUartMessage(uartFd, "Failed to send\n");
		/* Close the socket and try to open it again on the next event */
		close(sockfd);
	}
}

static void gpioToggleEventHandler(EventData *eventData)
{
	if (ConsumeTimerFdEvent(gpioToggleTimerFd) != 0) {
		terminationRequired = true;
		return;
	}

	int index;
	for (index = 0; index < GPIO_ARRAY_SIZE; index++)
	{
		/* Toggle the current value so it will be different on this time around */
		if (gpioArr[index].currentValue == GPIO_Value_High)
		{
			gpioArr[index].currentValue = GPIO_Value_Low;
		}
		else
		{
			gpioArr[index].currentValue = GPIO_Value_High;
		}

		int ret = GPIO_SetValue(gpioArr[index].Fd, gpioArr[index].currentValue);
		if (ret == -1)
		{
			/* Something went wrong, output the debug message */
			snprintf(str, sizeof(str), "GPIO pin %d failed to set\n", gpioArr[index].gpio);
			SendUartMessage(uartFd, str);
		}
	}

}

/// <summary>
///     Print latest data from accelerometer.
/// </summary>
static void AccelTimerEventHandler(EventData *eventData)
{
    static unsigned int iter = 1;

    if (ConsumeTimerFdEvent(accelTimerFd) != 0) {
        terminationRequired = true;
        return;
    }

    // Status register describes whether accelerometer is available.
    // DocID026899 Rev 10, S9.26, STATUS_REG (1Eh); [0] = XLDA
    static const uint8_t statusRegId = 0x1Eu;
    uint8_t status;
    ssize_t transferredBytes = I2CMaster_WriteThenRead(
        i2cFd, lsm6ds3Address, &statusRegId, sizeof(statusRegId), &status, sizeof(status));
    if (!CheckTransferSize("I2CMaster_WriteThenRead (STATUS_REG)",
                           sizeof(statusRegId) + sizeof(status), transferredBytes)) {
        terminationRequired = true;
        return;
    }

    if ((status & 0x1) == 0) {
        snprintf(str, sizeof(str), "INFO: %u: No accelerometer data.\n", iter);
		Log_Debug(str);
		SendUartMessage(uartFd, str);
    } else {
        // Read two-byte Z-axis output register.
        // DocID026899 Rev 10, S9.38, OUTZ_L_XL (2Ch)
        static const uint8_t outZLXl = 0x2Cu;
        int16_t zRaw;
        transferredBytes = I2CMaster_WriteThenRead(i2cFd, lsm6ds3Address, &outZLXl, sizeof(outZLXl),
                                                   (uint8_t *)&zRaw, sizeof(zRaw));
        if (!CheckTransferSize("I2CMaster_WriteThenRead (OUTZ_L_XL)",
                               sizeof(outZLXl) + sizeof(zRaw), transferredBytes)) {
            terminationRequired = true;
            return;
        }

        // DocID026899 Rev 10, S4.1, Mechanical characteristics
        // These constants are specific to LA_So where FS = +/-4g, as set in CTRL1_X.
        double g = (zRaw * 0.122) / 1000.0;

		snprintf( str, sizeof(str), "INFO: %u: vertical acceleration: %.2lfg\n", iter, g);
		Log_Debug(str);
		SendUartMessage(uartFd, str);

    }

    ++iter;
}

// Demonstrates three ways of reading data from the attached device.
// This also works as a smoke test to ensure the MT3620 can talk to the I2C device.
static int ReadWhoAmI(void)
{
    // DocID026899 Rev 10, S9.11, WHO_AM_I (0Fh); has fixed value 0x69.
    static const uint8_t whoAmIRegId = LPS22HB_WHO_AM_I;
	// static const uint8_t whoAmIRegId = 0xC1;
	// static const uint8_t expectedWhoAmI = 0x69;
	static const uint8_t expectedWhoAmI = 0x6C;
	uint8_t actualWhoAmI = 0;

    // Read register value using AppLibs combination read and write API.
   
		ssize_t transferredBytes =
			I2CMaster_WriteThenRead(i2cFd, lsm6ds3Address, &whoAmIRegId, sizeof(whoAmIRegId),
				&actualWhoAmI, sizeof(actualWhoAmI));
		if (!CheckTransferSize("I2CMaster_WriteThenRead (WHO_AM_I)",
			sizeof(whoAmIRegId) + sizeof(actualWhoAmI), transferredBytes)) {
			return -1;
		}

    snprintf(str, sizeof(str), "*** INFO: WHO_AM_I=0x%02x (I2CMaster_WriteThenRead)\n", actualWhoAmI);
	Log_Debug(str);
	SendUartMessage(uartFd, str);
	snprintf(str, sizeof(str), "*** INFO: ACTUAL = 0x%02x, EXPECTED = 0x%02x\n", actualWhoAmI, expectedWhoAmI);
	Log_Debug(str);
	SendUartMessage(uartFd, str);
    if (actualWhoAmI != expectedWhoAmI) {
        Log_Debug("ERROR: Unexpected WHO_AM_I value.\n");
		SendUartMessage(uartFd, "ERROR: Unexpected WHO_AM_I value.\n");
        return -1;
    }

    // Read register value using AppLibs separate read and write APIs.
    transferredBytes = I2CMaster_Write(i2cFd, lsm6ds3Address, &whoAmIRegId, sizeof(whoAmIRegId));
    if (!CheckTransferSize("I2CMaster_Write (WHO_AM_I)", sizeof(whoAmIRegId), transferredBytes)) {
        return -1;
    }
    transferredBytes = I2CMaster_Read(i2cFd, lsm6ds3Address, &actualWhoAmI, sizeof(actualWhoAmI));
    if (!CheckTransferSize("I2CMaster_Read (WHO_AM_I)", sizeof(actualWhoAmI), transferredBytes)) {
        return -1;
    }
    Log_Debug("INFO: WHO_AM_I=0x%02x (I2CMaster_Write + I2CMaster_Read)\n", actualWhoAmI);
    if (actualWhoAmI != expectedWhoAmI) {
        Log_Debug("ERROR: Unexpected WHO_AM_I value.\n");
		SendUartMessage(uartFd, "ERROR: Unexpected WHO_AM_I value.\n");
        return -1;
    }

    // Read register value using POSIX APIs.
    // This uses the I2C target address which was set earlier with
    // I2CMaster_SetDefaultTargetAddress.
    transferredBytes = write(i2cFd, &whoAmIRegId, sizeof(whoAmIRegId));
    if (!CheckTransferSize("write (WHO_AM_I)", sizeof(whoAmIRegId), transferredBytes)) {
        return -1;
    }
    transferredBytes = read(i2cFd, &actualWhoAmI, sizeof(actualWhoAmI));
    if (!CheckTransferSize("read (WHO_AM_I)", sizeof(actualWhoAmI), transferredBytes)) {
        return -1;
    }
    Log_Debug("INFO: WHO_AM_I=0x%02x (POSIX read + write)\n", actualWhoAmI);
    if (actualWhoAmI != expectedWhoAmI) {
        Log_Debug("ERROR: Unexpected WHO_AM_I value.\n");
		SendUartMessage(uartFd, "ERROR: Unexpected WHO_AM_I value.\n");
        return -1;
    }

    return 0;
}

/// <summary>
///    Checks the number of transferred bytes for SPI functions and prints an error
///    message if the functions failed or if the number of bytes is different than
///    expected number of bytes to be transferred.
/// </summary>
/// <returns>true on success, or false on failure</returns>
static bool CheckTransferSize(const char *desc, size_t expectedBytes, ssize_t actualBytes)
{
    if (actualBytes < 0) {
		snprintf(str, sizeof(str), "ERROR: %s: errno=%d (%s)\n", desc, errno, strerror(errno));
		Log_Debug(str);
		SendUartMessage(uartFd, str);
        return false;
    }

    if (actualBytes != (ssize_t)expectedBytes) {
		snprintf( str, sizeof( str ), "ERROR: %s: transferred %zd bytes; expected %zd\n", desc, actualBytes, expectedBytes);
		Log_Debug(str);
		SendUartMessage(uartFd, str);
        return false;
    }

    return true;
}

/// <summary>
///     Resets the accelerometer and samples the vertical acceleration.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int ResetAndSampleLsm6ds3(void)
{
    // Reset device to put registers into default state.
    // DocID026899 Rev 10, S9.14, CTRL3_C (12h); [0] = SW_RESET
    static const uint8_t ctrl3cRegId = 0x12;
    const uint8_t resetCommand[] = {ctrl3cRegId, 0x01};
    ssize_t transferredBytes =
        I2CMaster_Write(i2cFd, lsm6ds3Address, resetCommand, sizeof(resetCommand));
    if (!CheckTransferSize("I2CMaster_Write (CTRL3_C)", sizeof(resetCommand), transferredBytes)) {
        return -1;
    }

    // Wait for device to come out of reset.
    uint8_t ctrl3c;
    do {
        transferredBytes = I2CMaster_WriteThenRead(i2cFd, lsm6ds3Address, &ctrl3cRegId,
                                                   sizeof(ctrl3cRegId), &ctrl3c, sizeof(ctrl3c));
    } while (!(transferredBytes == (sizeof(ctrl3cRegId) + sizeof(ctrl3c)) && (ctrl3c & 0x1) == 0));

    // Use sample range +/- 4g, with 12.5Hz frequency.
    // DocID026899 Rev 10, S9.12, CTRL1_XL (10h)
    static const uint8_t setCtrl1XlCommand[] = {0x10, 0x18};
    transferredBytes = I2CMaster_Write(i2cFd, lsm6ds3Address, setCtrl1XlCommand, sizeof(setCtrl1XlCommand));
    if (!CheckTransferSize("I2CMaster_Write (CTRL1_XL)", sizeof(setCtrl1XlCommand),
                           transferredBytes)) {
        return -1;
    }

    return 0;
}

static int connectToServer(void)
{
	memset((char *)&serv_addr, 0, sizeof(serv_addr));

	if (sockfd == -1)
	{
		sockfd = socket(AF_INET, SOCK_DGRAM, 0);
		if (sockfd == -1)
		{
			SendUartMessage(uartFd, "Error opening socket\n");
			return -1;
		}
	}

	server = gethostbyname(IpAddress);
	if (server == NULL)
	{
		SendUartMessage(uartFd, "ERROR, no such host\n");
		return 0;
	}

	memset((char *)&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	memcpy((char *)&serv_addr.sin_addr.s_addr, (char *)server->h_addr, server->h_length);
	serv_addr.sin_port = htons(portNumber);

	if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
	{
		SendUartMessage(uartFd, "ERROR connecting");
	}
	else
	{
		SendUartMessage(uartFd, "connected\n");
		isConnected = true;
	}

	//close(sockfd);
	return 0;
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

	struct timespec UdpPeriod = { .tv_sec = UDP_MESSAGE_SECONDS_TIME_SPAN,.tv_nsec = UDP_MESSAGE_NANO_SECONDS_TIME_SPAN };
	UdpTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &UdpPeriod, &UdpTxEvent, EPOLLIN);

	connectToServer();

	/* Loop though and init all the GPIO's we need for this test */
	int index;
	for (index = 0; index < GPIO_ARRAY_SIZE; index++)
	{
		gpioArr[index].Fd = GPIO_OpenAsOutput(gpioArr[index].gpio, GPIO_OutputMode_PushPull, gpioArr[index].currentValue);
		if (gpioArr[index].Fd == -1)
		{
			snprintf(str, sizeof(str), "ERROR: Could not open GPIO_%d: %s (%d).\n", gpioArr[index].gpio, strerror(errno), errno);
			Log_Debug(str);
			SendUartMessage(uartFd, str);
			snprintf(str, sizeof(str), "GPIO_%d: fd is %d.\n", gpioArr[index].gpio, gpioArr[index].Fd);
			Log_Debug(str);
			SendUartMessage(uartFd, str);
			/* For now lets not give up if we fail to open a GPIO pin since it is really only for toggleing pins */
		}
		
	}

	struct timespec gpioTogglePeriod = { .tv_sec = GPIO_TOGGLE_SECONDS_TIME_SPAN,.tv_nsec = GPIO_TOGGLE_NANO_SECONDS_TIME_SPAN };
	gpioToggleTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &gpioTogglePeriod, &GpioToggleEvent, EPOLLIN);

    // Print accelerometer data every second.
    struct timespec accelReadPeriod = {.tv_sec = 1, .tv_nsec = 0};
    
    accelTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &accelReadPeriod, &accelEventData, EPOLLIN);
    if (accelTimerFd < 0) {
		SendUartMessage(uartFd, "Failed to create epoll\n");
        return -1;
    }

    i2cFd = I2CMaster_Open(MT3620_RDB_HEADER4_ISU2_I2C);
    if (i2cFd < 0) {
        
		snprintf( str, sizeof(str), "ERROR: I2CMaster_Open: errno=%d (%s)\n", errno, strerror(errno));
		Log_Debug(str);
		SendUartMessage(uartFd, str);
        return -1;
    }

    int result = I2CMaster_SetBusSpeed(i2cFd, I2C_BUS_SPEED_STANDARD);
    if (result != 0) {
		snprintf(str, sizeof(str), "ERROR: I2CMaster_SetBusSpeed: errno=%d (%s)\n", errno, strerror(errno));
		Log_Debug(str);
		SendUartMessage(uartFd, str);
        return -1;
    }

    result = I2CMaster_SetTimeout(i2cFd, 100);
    if (result != 0) {
        snprintf(str, sizeof(str), "ERROR: I2CMaster_SetTimeout: errno=%d (%s)\n", errno, strerror(errno));
		Log_Debug(str);
		SendUartMessage(uartFd, str);
        return -1;
    }

    // This default address is used for POSIX read and write calls.  The AppLibs APIs take a target
    // address argument for each read or write.
    result = I2CMaster_SetDefaultTargetAddress(i2cFd, lsm6ds3Address);
    if (result != 0) {
        Log_Debug("ERROR: I2CMaster_SetDefaultTargetAddress: errno=%d (%s)\n", errno,
                  strerror(errno));
		Log_Debug(str);
		SendUartMessage(uartFd, str);
        return -1;
    }

    result = ReadWhoAmI();
    if (result != 0) {
		SendUartMessage(uartFd, "Failed the who am i\n");
        return -1;
    }

    result = ResetAndSampleLsm6ds3();
    if (result != 0) {
		SendUartMessage(uartFd, "Failed the reset of the accel\n");
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
    CloseFdAndPrintError(i2cFd, "i2c");
	CloseFdAndPrintError(uartFd, "Uart");
    CloseFdAndPrintError(accelTimerFd, "accelTimer");
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

    ClosePeripheralsAndHandlers();
    Log_Debug("Application exiting.\n");
    return 0;
}
