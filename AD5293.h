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


#ifndef _AD5293_H_
#define _AD5293_H_

//*****************************************************************************
// Constants
//*****************************************************************************

// AD5293 RDAC and EEMEM Commands
#define AD5293_NOP              0x00            // Restore from EEMEM to RDAC
#define AD5293_WRITE_RDAC       (0x01 << 10)    // Write setting to RDAC
#define AD5293_READ_RDAC        (0x02 << 10)    // Read setting from RDAC
#define AD5293_RESET_RDAC       (0x04 << 10)    // Reset RDAC to mid scale
#define AD5293_WRITE_CTLREG     (0x06 << 10)    // Write contents serial reg to ctl reg
#define AD5293_READ_CTLREG      (0x07 << 10)    // Read contents serial reg to ctl reg
#define AD5293_POWER_DOWN       (0x08 << 10)    // Read contents serial reg to ctl reg

// Flags for AD5293_WRITE_CTLREG function 4 above
#define AD5293_CTLREG_ENABLE    0x02            // ctrl register C1 RDAC write enable
#define AD5293_CTLREG_NORMAL    0x04            // calibration enable 0=resistor
                                                // performance mode (default). 1= normal
// 1024 max wiper positions (0-1023)
#define AD5293_MAX_POS          0x3FF

//*****************************************************************************
// Data Types
//*****************************************************************************

//typedef struct _AD5293_DEVICE {
//    SPI_Handle  handle;
//	uint16_t    configReg;
//} AD5293_DEVICE;

//*****************************************************************************
// Function Prototypes
//*****************************************************************************

int32_t AD5293_init(void);
uint16_t AD5293_rdac_write_enable(void);
uint16_t AD5293_nop(void);
uint16_t AD5293_rdac_write(uint16_t level);
uint16_t AD5293_rdac_read(void);
uint16_t AD5293_rdac_reset(void);
uint16_t AD5293_ctrlreg_write(uint16_t data);
uint16_t AD5293_ctrlreg_read(void);
uint16_t AD5293_power_down(bool flag);


#endif  /* _AD5293_H_ */
