/*
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
 */
/** ============================================================================
 *  @file       USINE.h
 *
 *  @brief      USINE Board Specific APIs
 *
 *  The USINE header file should be included in an application as follows:
 *  @code
 *  #include <USINE.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef __USINE_TM4C1233H6PMI_H
#define __USINE_TM4C1233H6PMI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/GPIO.h>

/* LEDs on USINE are active high. */
#define USINE_LED_OFF	( 0)
#define USINE_LED_ON	(~0)

#define PIN_LOW			( 0)
#define PIN_HIGH		(~0)

/* Board specific I2C addresses */
#define AT24MAC_EPROM_ADDR      (0xA0 >> 1)
#define AT24MAC_EPROM_EXT_ADDR  (0xB0 >> 1)

/*!
 *  @def    USINE_GPIOName
 *  @brief  Enum of LED names on the USINE dev board
 */
typedef enum USINE_GPIOName {
    //USINE_AD9732_FSYNC = 0,
    USINE_PC4,
    USINE_PC5,
    USINE_LED_ACTIVE,
    USINE_LED_STATUS,

    USINE_GPIOCOUNT
} USINE_GPIOName;

/*!
 *  @def    USINE_I2CName
 *  @brief  Enum of I2C names on the USINE dev board
 */
typedef enum USINE_I2CName {
    USINE_I2C0 = 0,

    USINE_I2CCOUNT
} USINE_I2CName;

/*!
 *  @def    USINE_PWMName
 *  @brief  Enum of PWM names on the USINE dev board
 */
typedef enum USINE_PWMName {
    USINE_PWM0 = 0,

    USINE_PWMCOUNT
} USINE_PWMName;

/*!
 *  @def    USINE_SPIName
 *  @brief  Enum of SPI names on the USINE dev board
 */
typedef enum USINE_SPIName {
    USINE_SPI0 = 0,
    USINE_SPI1,
    USINE_SPI2,

    USINE_SPICOUNT
} USINE_SPIName;

/*!
 *  @def    USINE_UARTName
 *  @brief  Enum of UARTs on the USINE dev board
 */
typedef enum USINE_UARTName {
    USINE_UART0 = 0,

    USINE_UARTCOUNT
} USINE_UARTName;

/*!
 *  @def    USINE_USBMode
 *  @brief  Enum of USB setup function on the USINE dev board
 */
typedef enum USINE_USBMode {
    USINE_USBDEVICE,
    USINE_USBHOST
} USINE_USBMode;

/*!
 *  @def    USINE_USBMSCHFatFsName
 *  @brief  Enum of USBMSCHFatFs names on the USINE dev board
 */
typedef enum USINE_USBMSCHFatFsName {
    USINE_USBMSCHFatFs0 = 0,

    USINE_USBMSCHFatFsCOUNT
} USINE_USBMSCHFatFsName;

/*
 *  @def    USINE_WatchdogName
 *  @brief  Enum of Watchdogs on the USINE dev board
 */
typedef enum USINE_WatchdogName {
    USINE_WATCHDOG0 = 0,

    USINE_WATCHDOGCOUNT
} USINE_WatchdogName;

/*!
 *  @def    USINE_WiFiName
 *  @brief  Enum of WiFi names on the USINE dev board
 */
typedef enum USINE_WiFiName {
    USINE_WIFI = 0,

    USINE_WIFICOUNT
} USINE_WiFiName;

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings. This include
 *     - Enable clock sources for peripherals
 */
extern void USINE_initGeneral(void);

/*!
 *  @brief  Initialize board specific GPIO settings
 *
 *  This function initializes the board specific GPIO settings and
 *  then calls the GPIO_init API to initialize the GPIO module.
 *
 *  The GPIOs controlled by the GPIO module are determined by the GPIO_config
 *  variable.
 */
extern void USINE_initGPIO(void);

/*!
 *  @brief  Initialize board specific I2C settings
 *
 *  This function initializes the board specific I2C settings and then calls
 *  the I2C_init API to initialize the I2C module.
 *
 *  The I2C peripherals controlled by the I2C module are determined by the
 *  I2C_config variable.
 */
extern void USINE_initI2C(void);

/*!
 *  @brief  Initialize board specific PWM settings
 *
 *  This function initializes the board specific PWM settings and then calls
 *  the PWM_init API to initialize the PWM module.
 *
 *  The PWM peripherals controlled by the PWM module are determined by the
 *  PWM_config variable.
 */
extern void USINE_initPWM(void);

/*!
 *  @brief  Initialize board specific SDSPI settings
 *
 *  This function initializes the board specific SDSPI settings and then calls
 *  the SDSPI_init API to initialize the SDSPI module.
 *
 *  The SDSPI peripherals controlled by the SDSPI module are determined by the
 *  SDSPI_config variable.
 */
extern void USINE_initSDSPI(void);

/*!
 *  @brief  Initialize board specific SPI settings
 *
 *  This function initializes the board specific SPI settings and then calls
 *  the SPI_init API to initialize the SPI module.
 *
 *  The SPI peripherals controlled by the SPI module are determined by the
 *  SPI_config variable.
 */
extern void USINE_initSPI(void);

/*!
 *  @brief  Initialize board specific UART settings
 *
 *  This function initializes the board specific UART settings and then calls
 *  the UART_init API to initialize the UART module.
 *
 *  The UART peripherals controlled by the UART module are determined by the
 *  UART_config variable.
 */
extern void USINE_initUART(void);

/*!
 *  @brief  Initialize board specific USB settings
 *
 *  This function initializes the board specific USB settings and pins based on
 *  the USB mode of operation.
 *
 *  @param      usbMode    USB mode of operation
 */
extern void USINE_initUSB(USINE_USBMode usbMode);

/*!
 *  @brief  Initialize board specific USBMSCHFatFs settings
 *
 *  This function initializes the board specific USBMSCHFatFs settings and then
 *  calls the USBMSCHFatFs_init API to initialize the USBMSCHFatFs module.
 *
 *  The USBMSCHFatFs peripherals controlled by the USBMSCHFatFs module are
 *  determined by the USBMSCHFatFs_config variable.
 */
extern void USINE_initUSBMSCHFatFs(void);

/*!
 *  @brief  Initialize board specific Watchdog settings
 *
 *  This function initializes the board specific Watchdog settings and then
 *  calls the Watchdog_init API to initialize the Watchdog module.
 *
 *  The Watchdog peripherals controlled by the Watchdog module are determined
 *  by the Watchdog_config variable.
 */
extern void USINE_initWatchdog(void);

/*!
 *  @brief  Initialize board specific WiFi settings
 *
 *  This function initializes the board specific WiFi settings and then calls
 *  the WiFi_init API to initialize the WiFi module.
 *
 *  The hardware resources controlled by the WiFi module are determined by the
 *  WiFi_config variable.
 */
extern void USINE_initWiFi(void);

#ifdef __cplusplus
}
#endif

#endif /* __USINE_TM4C1233H6PMI_H */
