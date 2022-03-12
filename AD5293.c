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
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

#include <usblib/usblib.h>
#include <usblib/usb-ids.h>
#include <usblib/device/usbdevice.h>
#include <usblib/device/usbdbulk.h>

#include <file.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdlib.h>

/* STC1200 Board Header file */

#include "USINE.h"
#include "Board.h"
#include "AD5293.h"

static SPI_Handle  s_spiHandle;

//*****************************************************************************
// Write a 24 bit command out to the digital potentiometer
//*****************************************************************************

static uint16_t _SPIWrite(uint16_t cmd, uint16_t data)
{
    uint16_t txData;
    uint16_t rxData;
    SPI_Transaction transaction;

    /* Lower 10 bits are data, bits 10-13 are command */
    txData = cmd | (data & 0x3FF);

    /* Setup the SPI data buffer pointers */
    transaction.count = 1;
    transaction.txBuf = (Ptr)&txData;
    transaction.rxBuf = (Ptr)&rxData;

    /* Send the SPI transaction */
    SPI_transfer(s_spiHandle, &transaction);

    return rxData;
}

//*****************************************************************************
// Open the SPI port to the AD5293 digipot.
//*****************************************************************************

int32_t AD5293_init(void)
{
    SPI_Params spiParams;

    /* Open for SPI mode 1 (POL=1, PHA=1) and 16-bit word size */
    SPI_Params_init(&spiParams);
    spiParams.transferMode  = SPI_MODE_BLOCKING;
    spiParams.mode          = SPI_MASTER;
    spiParams.frameFormat   = SPI_POL0_PHA1;    /* SPI mode 1       */
    spiParams.bitRate       = 100000;           /* 10 MHz clock     */
    spiParams.dataSize      = 16;               /* 16-bit word size */
    spiParams.transferCallbackFxn = NULL;

    /* Open the SPI port and save the handle to global variable */
    s_spiHandle = SPI_open(Board_SPI_AD5293, &spiParams);

    if (s_spiHandle == NULL)
        System_abort("Error initializing SPI0\n");

    /* On power-up, the serial data input register write command
     * for the RDAC register is disabled so we enable this here
     * so it's ready to accept RDAC commands.
     */
    AD5293_rdac_write_enable();

	return 0;
}

//*****************************************************************************
// AD5293 Digi-Pot Commands
//*****************************************************************************

uint16_t AD5293_nop(void)
{
    return _SPIWrite(AD5293_NOP, 0);
}

//*****************************************************************************
// This must be called initially to enable writes to the RDAC register
//*****************************************************************************

uint16_t AD5293_rdac_write_enable(void)
{
    return _SPIWrite(AD5293_WRITE_CTLREG, AD5293_CTLREG_ENABLE);
}

//*****************************************************************************
// Set the RDAC level from 0 - 1023
//*****************************************************************************

uint16_t AD5293_rdac_write(uint16_t level)
{
    if (level > AD5293_MAX_POS)
        level = AD5293_MAX_POS;
    /* Invert range */
    level -= AD5293_MAX_POS;
    /* Send the new level */
    return _SPIWrite(AD5293_WRITE_RDAC, level);
}

//*****************************************************************************
// Read the current RDAC level
//*****************************************************************************

uint16_t AD5293_rdac_read(void)
{
    uint16_t level;
    /* Send read RDAC command */
    _SPIWrite(AD5293_READ_RDAC, 0);
    /* Allow time for RDY */
    Task_sleep(5);
    /* Do dummy read to get results */
    level = _SPIWrite(AD5293_NOP, 0);
    /* Invert the range */
    level -= AD5293_MAX_POS;
    /* Return the current level */
    return level;
}

//*****************************************************************************
// Reset the RDAC and return wiper to mid scale
//*****************************************************************************

uint16_t AD5293_rdac_reset(void)
{
    return _SPIWrite(AD5293_RESET_RDAC, 0);
}

//*****************************************************************************
// Write control register bits to controller
//*****************************************************************************

uint16_t AD5293_ctrlreg_write(uint16_t data)
{
    return _SPIWrite(AD5293_WRITE_CTLREG, data);
}

//*****************************************************************************
// Read control register bits
//*****************************************************************************

uint16_t AD5293_ctrlreg_read(void)
{
    /* Send read read ctrl reg command */
    _SPIWrite(AD5293_READ_CTLREG, 0);
    /* Allow time for RDY */
    Task_sleep(5);
    /* Do dummy read to get results */
    return _SPIWrite(AD5293_NOP, 0);
}

//*****************************************************************************
// Set the controller power down state for power save mode.
//*****************************************************************************

uint16_t AD5293_power_down(bool flag)
{
    /* Send read read ctrl reg command */
    return _SPIWrite(AD5293_POWER_DOWN, (flag) ? 0x1 : 0x0);
}

/* End-Of-File */
