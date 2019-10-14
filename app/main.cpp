/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Hello World application
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 *
 * @}
 */


#define TESTING_LARGE_IMAGES


#include "stdio.h"
#include "stddef.h"
#include "string.h"
//#include "stm32f4xx.h"
#include "periph/cpuid.h"
#include "board.h"
#include "cat.hpp"



int main( void)
{
    uint32_t cpuid = *(uint32_t *)(0xE000ED00);
    char cpuname[4], part[12];
    if(( cpuid & 0xFF000000) == 0x41000000)
        strcpy( cpuname, "ARM");
    else
        strcpy( cpuname, "-");

    int var = ( cpuid & 0x00F00000) >> 20;

    int pt = ( cpuid & 0x0000FFF0) >> 4;
    if( pt == 0xC24)
       strcpy( part, "Cortex-M4");
    else if( pt == 0xC27)
       strcpy( part, "Cortex-M7");
    else
       strcpy( part, "unknown");

    int pat = ( cpuid & 0x0000000F);

    printf("You are running RIOT on a(n) %s board.\n", "discovery" /* RIOT_BOARD */);
    printf("This board features a(n) %s MCU.\n", "STM32f429" /*RIOT_MCU*/);
    printf("CPU id: %s %s rev %Xp%X\n", cpuname, part, var, pat);

    // test SDRAM???
#if 0
    uint32_t Buffer[32];
    SDRAM_ReadBuffer( Buffer, 0xD0000000, 32);

    for(i = 0xD0000000; i < 0xD0000000 + 0x50000; i += 2)
        *(uint32_t *)i = (i - 0xD0000000);

    SDRAM_ReadBuffer( Buffer, 0, 32);

    Buffer[0] = 0x5599;
    Buffer[1] = 0xA0A0;
    Buffer[2] = 0x5353;

    SDRAM_WriteBuffer( Buffer, 0, 4);

    SDRAM_ReadBuffer( Buffer, 0, 4);
#endif


    CAT_main();
} // endfunc main


/*************************** End of file ****************************/
//=================================================================================================
// End of file
//=================================================================================================


