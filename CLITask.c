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

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Gate.h>
#include <xdc/runtime/Memory.h>

#include <ti/sysbios/BIOS.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

#include <file.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include <time.h>

#include <driverlib/sysctl.h>

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/hal/Seconds.h>

#include "uSINE.h"
#include "CLITask.h"
#include "Board.h"
#include "AD5293.h"

//*****************************************************************************
// Type Definitions
//*****************************************************************************

typedef union {
    char  *s;
    char   c;
    float  f;
} arg_t;

typedef struct {
    const char* name;
    void (*func)(arg_t*);
    const char* args;
    const char* doc;
} cmd_t;

#define MK_CMD(x) void cmd_ ## x (arg_t*)

//*****************************************************************************
// CLI Function Handle Declarations
//*****************************************************************************

MK_CMD(cls);
MK_CMD(help);
MK_CMD(about);
MK_CMD(sernum);
MK_CMD(level);
MK_CMD(freq);

/* The dispatch table */
#define CMD(func, params, help) {#func, cmd_ ## func, params, help}

cmd_t dispatch[] = {
    CMD(cls, "", "Clear the screen"),
    CMD(help, "", "Display this help"),
    CMD(about, "", "About the system"),
    CMD(sernum, "", "Display serial number"),
    CMD(level, "s", "Display or set output level (0-1023)"),
    CMD(freq, "s", "Set the generator frequency (10-30000)"),
};

#define NUM_CMDS    (sizeof(dispatch)/sizeof(cmd_t))

//*****************************************************************************
// Static and External Data Items
//*****************************************************************************

#define MAX_CHARS   80

/*** Static Data Items ***/
static UART_Handle s_handleUart;
static const char *s_delim = " |()\n";
static char s_cmdbuf[MAX_CHARS+3];
static char s_cmdprev[MAX_CHARS+3];

/*** Function Prototypes ***/
static void parse_cmd(char *buf);
static arg_t *args_parse(const char *s);

/*** External Data Items ***/
extern SYSDAT g_sys;
extern SYSCONFIG g_cfg;

//*****************************************************************************
//
//*****************************************************************************

int CLI_init(void)
{
    UART_Params uartParams;

    UART_Params_init(&uartParams);

    uartParams.readMode       = UART_MODE_BLOCKING;
    uartParams.writeMode      = UART_MODE_BLOCKING;
    uartParams.readTimeout    = 1000;                   // 1 second read timeout
    uartParams.writeTimeout   = BIOS_WAIT_FOREVER;
    uartParams.readCallback   = NULL;
    uartParams.writeCallback  = NULL;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.writeDataMode  = UART_DATA_TEXT;
    uartParams.readDataMode   = UART_DATA_BINARY;
    uartParams.readEcho       = UART_ECHO_OFF;
    uartParams.baudRate       = 115200;
    uartParams.stopBits       = UART_STOP_ONE;
    uartParams.parityType     = UART_PAR_NONE;

    s_handleUart = UART_open(Board_RS232, &uartParams);

    if (s_handleUart == NULL)
        System_abort("Error initializing UART\n");

    return 1;
}

//*****************************************************************************
//
//*****************************************************************************

Bool CLI_startup(void)
{
    Error_Block eb;
    Task_Params taskParams;

    Error_init(&eb);

    Task_Params_init(&taskParams);

    taskParams.stackSize = 2048;
    taskParams.priority  = 2;
    taskParams.arg0      = 0;
    taskParams.arg1      = 0;

    Task_create((Task_FuncPtr)CLITaskFxn, &taskParams, &eb);

    return TRUE;
}

//*****************************************************************************
//
//*****************************************************************************

void CLI_putc(int ch)
{
    UART_write(s_handleUart, &ch, 1);
}

void CLI_puts(char* s)
{
    int l = strlen(s);
    UART_write(s_handleUart, s, l);
}

void CLI_printf(const char *fmt, ...)
{
    va_list arg;
    static char buf[128];
    va_start(arg, fmt);
    vsnprintf(buf, sizeof(buf)-1, fmt, arg);
    va_end(arg);
    UART_write(s_handleUart, buf, strlen(buf));
}

void CLI_prompt(void)
{
    CLI_putc(CRET);
    CLI_putc(LF);
    CLI_putc('>');
    CLI_putc(' ');
}

//*****************************************************************************
//
//*****************************************************************************

Void CLITaskFxn(UArg arg0, UArg arg1)
{
    uint8_t ch;
    int cnt = 0;

    CLI_printf(VT100_HOME);
    CLI_printf(VT100_CLS);

    CLI_printf("uSINE v%d.%02d.%03d\n\n", FIRMWARE_VER, FIRMWARE_REV, FIRMWARE_BUILD);
    CLI_puts("Enter 'help' to view a list valid commands\n\n> ");

    while (true)
    {
        /* Read a character from the console */
        if (UART_read(s_handleUart, &ch, 1) == 1)
        {
            if (ch == CRET)
            {
                if (cnt)
                {
                    CLI_putc(CRET);
                    CLI_putc(LF);
                    /* save command for previous recall */
                    strcpy(s_cmdprev, s_cmdbuf);
                    /* parse new command and execute */
                    parse_cmd(s_cmdbuf);
                    /* reset the command buffer */
                    s_cmdbuf[0] = 0;
                    cnt = 0;
                }
                CLI_prompt();
            }
            else if (ch == BKSPC)
            {
                if (cnt)
                {
                    s_cmdbuf[--cnt] = 0;

                    CLI_putc(BKSPC);
                    CLI_putc(' ');
                    CLI_putc(BKSPC);
                }
            }
            else if (ch == CTL_Z)
            {
                /* restore previous command */
                strcpy(s_cmdbuf, s_cmdprev);
                cnt = strlen(s_cmdbuf);
                CLI_printf("%s", s_cmdbuf);
            }
            else
            {
                if (cnt < MAX_CHARS)
                {
                    if (isalnum((int)ch) || strchr(s_delim, (int)ch) || (ch == '.'))
                    {
                        s_cmdbuf[cnt++] = tolower(ch);
                        s_cmdbuf[cnt] = 0;
                        CLI_putc((int)ch);
                    }
                }
            }
        }
    }
}

//*****************************************************************************
//
//*****************************************************************************

void parse_cmd(char *buf)
{
    const char* tok = strtok(buf, s_delim);

    if (!tok)
        return;

    int i = NUM_CMDS;

    while(i--)
    {
        cmd_t cur = dispatch[i];

        if (!strncmp(tok, cur.name, strlen(tok)))
        {
            arg_t *args = args_parse(cur.args);

            //if (args == NULL && strlen(cur.args))
            //    return;//Error in argument parsing

            cur.func(args);

            if (args)
                free(args);
            return;
        }
    }

    CLI_puts("Command Not Found\n");
}

#define ESCAPE { free(args); return NULL; }

arg_t *args_parse(const char *s)
{
    int argc = strlen(s);

    arg_t *args = malloc(sizeof(arg_t)*argc);

    int i;

    for(i=0; i < argc; ++i)
    {
        char *tok;

        switch(s[i])
        {
            case 's':
                args[i].s = strtok(NULL,s_delim);
                if (!args[i].s)
                    ESCAPE;
                break;

            case 'c':
                tok = strtok(NULL,s_delim);
                if (!tok)
                    ESCAPE;
                args[i].c = tok[0];
                if (!islower(args[i].c))
                    ESCAPE;
                break;

            case 'f':
                tok = strtok(NULL,s_delim);
                if (sscanf(tok,"%f", &args[i].f)!=1)
                    ESCAPE;
                break;
        }
    }

    return args;
}
#undef ESCAPE

//*****************************************************************************
// CLI Command Handlers
//*****************************************************************************

void cmd_help(arg_t *args)
{
    char tmp[100];
    int i = NUM_CMDS;

    CLI_puts("\nAvailable Commands:\n\n");

    while(i--)
    {
        cmd_t cmd=dispatch[i];
        System_snprintf(tmp, 100, "%s(%s)", cmd.name, cmd.args);
        CLI_printf("%10s\t %s\n", tmp, cmd.doc);
    }
}

void cmd_about(arg_t *args)
{
    CLI_printf("STC-1200 v%d.%02d.%03d\n", FIRMWARE_VER, FIRMWARE_REV, FIRMWARE_BUILD);
    CLI_puts("Copyright (C) 2020, RTZ Professional Audio, LLC.\n");
}

void cmd_cls(arg_t *args)
{
    CLI_puts(VT100_CLS);
    CLI_puts(VT100_HOME);
}

void cmd_sernum(arg_t *args)
{
    char serialnum[64];
    /*  Format the 64 bit GUID as a string */
    GetHexStr(serialnum, g_sys.ui8SerialNumber, 16);
    CLI_printf("%s\n", serialnum);
}

void cmd_level(arg_t *args)
{
    int level = 0;

    if (!args)
    {
        level = (int)AD5293_rdac_read();
        CLI_printf("current level %d\n", level);
    }
    else
    {
        level = atoi(args->s);

        if ((level < 0) || (level > AD5293_MAX_POS))
        {
            CLI_printf("ERROR: level must be in range of (0-%d)\n", AD5293_MAX_POS);
        }
        else
        {
            AD5293_rdac_write((uint16_t)level);

            CLI_printf("level set to %d\n", level);
        }
    }
}

void cmd_freq(arg_t *args)
{
    if (!args)
    {
        CLI_printf("current frequency %.2f\n", g_sys.frequency);
    }
    else
    {
        float freq = atof(args->s);

        if ((freq >= 10.0f) && (freq <= 30000.0f))
        {
            DSS_SetFrequency(freq);

            CLI_printf("frequency set to %.2f\n", freq);
        }
    }
}

// End-Of-File
