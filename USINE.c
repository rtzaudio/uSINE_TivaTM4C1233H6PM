/* ============================================================================
 *
 * uSIGN USB Signal Generator
 *
 * Copyright (C) 2020, RTZ Professional Audio, LLC
 * All Rights Reserved
 *
 * RTZ is registered trademark of RTZ Professional Audio, LLC
 *
 * ============================================================================
 *
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================ */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Gate.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <inc/hw_ints.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* USB Driver files */
#include <usblib/usblib.h>
#include <usblib/usb-ids.h>
#include <usblib/device/usbdevice.h>
#include <usblib/device/usbdbulk.h>

/* Standard Includes */
#include <file.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

#include <USINE.h>

/* uSINE Board Header file */
#include "Board.h"
#include "AD9837.h"
#include "AD5293.h"
#include "CLITask.h"
#include "usb_bulk_structs.h"

// Local Constants
#define COMMAND_PACKET_RECEIVED     0x00000001
#define COMMAND_STATUS_UPDATE       0x00000002

// Global System Data
SYSDAT g_sys;

// Global flag indicating that a USB configuration has been set.
volatile bool g_bUSBConfigured = false;
volatile uint32_t g_ui32TxCount = 0;
volatile uint32_t g_ui32RxCount = 0;
volatile uint32_t g_ui32Flags = 0;
char *g_pcStatus;

static uint32_t ui32RxCount = 0;
static uint32_t ui32TxCount = 0;

static Hwi_Struct usbHwiStruct;

/* Function Prototypes */
void USB_init(void);
void DSS_init(void);
uint32_t RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData);
uint32_t TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData);

//*****************************************************************************
// Main Entry Point
//*****************************************************************************

int main(void)
{
	Task_Params taskParams;
    Error_Block eb;

    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initI2C();
    Board_initSPI();
    Board_initUART();
    Board_initUSB(Board_USBDEVICE);
    // Board_initWatchdog();

    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.stackSize = 1496;
    taskParams.priority  = 10;

    if (Task_create(MainControlTask, &taskParams, &eb) == NULL)
        System_abort("MainControlTask!\n");

    /* Start BIOS */
    BIOS_start();

    return (0);
}

//*****************************************************************************
//
//*****************************************************************************

Void MainControlTask(UArg arg0, UArg arg1)
{
    char serialnum[64];

    /* Set initial LED states */
    GPIO_write(Board_LED_ACTIVE, PIN_HIGH);
    GPIO_write(Board_LED_STATUS, PIN_HIGH);

    /* Read the globally unique serial number from EPROM. We are also
     * reading the 6-byte MAC address from the AT24MAC serial EPROM.
     */
    if (!ReadGUIDS(g_sys.ui8SerialNumber, g_sys.ui8MAC))
    {
        System_printf("Read Serial Number Failed!\n");
        System_flush();
    }

    /* Format the 64-bit GUID as a string */
    GetHexStr(serialnum, g_sys.ui8SerialNumber, 16);
    System_printf("Serial#: %s\n", serialnum);
    System_flush();

    /* Initialize the digipot SPI interface */
    AD5293_init();

    /* Initialize the DSS waveform generator */
    DSS_init();

    /* Initialize the USB library for device mode */
    USB_init();

    /* Now startup the RS232 CLI command line interface task */
    CLI_init();
    CLI_startup();

    /* Now begin the main program command task processing loop */
    GPIO_write(Board_LED_STATUS, PIN_LOW);

    while (true)
    {
        GPIO_toggle(Board_LED_STATUS);

        Task_sleep(1000);
    }
}

//*****************************************************************************
//
// Handles bulk driver notifications related to the receive channel (data from
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************

uint32_t
RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    //
    // Which event are we being sent?
    //
    switch(ui32Event)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
        {
            g_bUSBConfigured = true;
            g_pcStatus = "Host connected.";
            g_ui32Flags |= COMMAND_STATUS_UPDATE;

            //
            // Flush our buffers.
            //
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);

            break;
        }

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
        {
            g_bUSBConfigured = false;
            g_pcStatus = "Host disconn.";
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            break;
        }

        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
            tUSBDBulkDevice *psDevice;

            //
            // Get a pointer to our instance data from the callback data
            // parameter.
            //
            psDevice = (tUSBDBulkDevice *)pvCBData;

            //
            // Read the new packet and echo it back to the host.
            //
            //return(EchoNewDataToHost(psDevice, pvMsgData, ui32MsgValue));
            break;
        }

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        //
        // Ignore all other events and return 0.
        //
        default:
            break;
    }

    return 0;
}

//*****************************************************************************
//
// Handles bulk driver notifications related to the transmit channel (data to
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************

uint32_t
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    //
    // We are not required to do anything in response to any transmit event
    // in this example. All we do is update our transmit counter.
    //
    if(ui32Event == USB_EVENT_TX_COMPLETE)
    {
        g_ui32TxCount += ui32MsgValue;
    }

    //
    // Dump a debug message.
    //
    //DEBUG_PRINT("TX complete %d\n", ui32MsgValue);

    return 0;
}

//*****************************************************************************
// Initialize the DSS waveform signal generator chip
//*****************************************************************************

void DSS_init(void)
{
    /* Initialize the DSS programmable waveform generator */
    AD9837_init();
    AD9837_reset();

    g_sys.frequency = 1000.0f;

    /* Calculate the 32-bit frequency divisor */
    uint32_t freqCalc = AD9837_freqCalc(g_sys.frequency);

    /* Program the DSS ref clock with new value */
    AD9837_adjustFreqMode32(FREQ0, FULL, freqCalc);
    AD9837_adjustFreqMode32(FREQ1, FULL, freqCalc);
}

void DSS_SetFrequency(float freq)
{
    g_sys.frequency = freq;

    /* Calculate the 32-bit frequency divisor */
    uint32_t freqCalc = AD9837_freqCalc(freq);

    /* Program the DSS ref clock with new value */
    AD9837_adjustFreqMode32(FREQ0, FULL, freqCalc);
    AD9837_adjustFreqMode32(FREQ1, FULL, freqCalc);
}

//*****************************************************************************
// Initialize the USB for device mode
//*****************************************************************************

void USB_init(void)
{
    Error_Block eb;
    Hwi_Params  hwiParams;

    // Tell the user what we are up to.
    System_printf("Configuring USB\n");

    // Initialize the transmit and receive buffers.
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);

    /* Setup USB0 HWI interrupt handler */
    Error_init(&eb);
    Hwi_Params_init(&hwiParams);
    Hwi_construct(&(usbHwiStruct), INT_USB0, (ti_sysbios_interfaces_IHwi_FuncPtr)USB0DeviceIntHandler, &hwiParams, &eb);
    if (Error_check(&eb)) {
        System_abort("Couldn't construct USB error hwi");
    }

    // Initialize the USB stack for device mode.
    //USBStackModeSet(0, eUSBModeDevice, 0);
    // Forcing device mode so that the VBUS and ID pins are not used or monitored by the USB controller.
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    // Pass our device information to the USB library and place the device
    // on the bus.
    USBDBulkInit(0, &g_sBulkDevice);
}

//*****************************************************************************
// This function reads the unique 128-serial number and 48-bit MAC address
// via I2C from the AT24MAC402 serial EPROM.
//*****************************************************************************

int ReadGUIDS(uint8_t ui8SerialNumber[16], uint8_t ui8MAC[6])
{
    bool            ret;
    uint8_t         txByte;
    I2C_Handle      handle;
    I2C_Params      params;
    I2C_Transaction i2cTransaction;

    /* default is all FF's  in case read fails*/
    memset(ui8SerialNumber, 0xFF, 16);
    memset(ui8MAC, 0xFF, 6);

    I2C_Params_init(&params);
    params.transferCallbackFxn = NULL;
    params.transferMode = I2C_MODE_BLOCKING;
    params.bitRate = I2C_100kHz;

    handle = I2C_open(Board_I2C_AT24MAC402, &params);

    if (!handle) {
        System_printf("I2C did not open\n");
        System_flush();
        return 0;
    }

    /* Note the Upper bit of the word address must be set
     * in order to read the serial number. Thus 80H would
     * set the starting address to zero prior to reading
     * this sixteen bytes of serial number data.
     */

    txByte = 0x80;

    i2cTransaction.slaveAddress = AT24MAC_EPROM_EXT_ADDR;
    i2cTransaction.writeBuf     = &txByte;
    i2cTransaction.writeCount   = 1;
    i2cTransaction.readBuf      = ui8SerialNumber;
    i2cTransaction.readCount    = 16;

    ret = I2C_transfer(handle, &i2cTransaction);

    if (!ret)
    {
        System_printf("Unsuccessful I2C transfer\n");
        System_flush();
    }

    /* Now read the 6-byte 48-bit MAC at address 0x9A. The EUI-48 address
     * contains six or eight bytes. The first three bytes of the  UI read-only
     * address field are called the Organizationally Unique Identifier (OUI)
     * and the IEEE Registration Authority has assigned FCC23Dh as the Atmel OUI.
     */

    txByte = 0x9A;

    i2cTransaction.slaveAddress = AT24MAC_EPROM_EXT_ADDR;
    i2cTransaction.writeBuf     = &txByte;
    i2cTransaction.writeCount   = 1;
    i2cTransaction.readBuf      = ui8MAC;
    i2cTransaction.readCount    = 6;

    ret = I2C_transfer(handle, &i2cTransaction);

    if (!ret)
    {
        System_printf("Unsuccessful I2C transfer\n");
        System_flush();
    }

    I2C_close(handle);

    return ret;
}

//*****************************************************************************
// Return GUID as hex string
//*****************************************************************************

int GetHexStr(char* textbuf, uint8_t* databuf, int datalen)
{
    char fmt[8];
    uint32_t i;
    int32_t l;

    const uint32_t wordSize = 4;

    *textbuf = 0;
    strcpy(fmt, "%02X");

    for (i=0; i < datalen; i++)
    {
        l = sprintf(textbuf, fmt, *databuf++);
        textbuf += l;

        if (((i % wordSize) == (wordSize-1)) && (i != (datalen-1)))
        {
            l = sprintf(textbuf, "-");
            textbuf += l;
        }
    }

    return strlen(textbuf);
}

// End-Of-File
