/* ============================================================================
 *
 * XMOD Data Capture and Telemetry Systems
 *
 * Copyright (C) 2021, RTZ Microsystems, LLC
 * All Rights Reserved
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

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <USINE_TM4C1233H6PM.h>

#define Board_initGeneral           USINE_initGeneral
#define Board_initGPIO              USINE_initGPIO
#define Board_initI2C               USINE_initI2C
#define Board_initPWM               USINE_initPWM
#define Board_initSPI               USINE_initSPI
#define Board_initUART              USINE_initUART
#define Board_initUSB               USINE_initUSB
#define Board_initWatchdog          USINE_initWatchdog

#define Board_USBHOST               USINE_USBHOST
#define Board_USBDEVICE             USINE_USBDEVICE

#define Board_I2C_AT24MAC402        USINE_I2C0

#define Board_SPI_AD9837            USINE_SPI0
#define Board_SPI_AD5293            USINE_SPI1
#define Board_SPI_LTC2420           USINE_SPI2

#define Board_RS232                 USINE_UART0

/* GPIO Pin Definitions */

//#define Board_AD9732_FSYNC          USINE_AD9732_FSYNC

#define Board_LED_ACTIVE            USINE_LED_ACTIVE
#define Board_LED_STATUS            USINE_LED_STATUS

#define Board_WATCHDOG0             USINE_WATCHDOG0

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
