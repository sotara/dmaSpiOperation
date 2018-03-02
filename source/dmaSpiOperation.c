/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    dmaSpiOperation.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_dmamux.h"
#include "fsl_dspi.h"
#include "fsl_ftm.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"

#define PRINTF printf
#define BUFF_LENGTH 4U


/**
 * IRQ handler for Pilot Cntl oscillation
 *
 * @TODO @@@TSB when the pin is moved to a FTM allocated pin, this IRQ handler will not be needed
 */
void IRQ_PILOT_CNTL_FTM_HANDLER(void)
{
#define FTM_CHANNEL_FLAG kFTM_Chnl0Flag

    if ((FTM_GetStatusFlags(PILOT_CNTL_FTM_BASEADDR) & FTM_CHANNEL_FLAG) == FTM_CHANNEL_FLAG)
    {
        /* Clear interrupt flag.*/
        FTM_ClearStatusFlags(PILOT_CNTL_FTM_BASEADDR, FTM_CHANNEL_FLAG);
    }

    GPIO_TogglePinsOutput(PILOT_CNTL_GPIO, 1U << PILOT_CNTL_GPIO_PIN); /**< Toggle the Pilot Cntl pin */
}

void IRQ_MCP3911_OSC_FTM_HANDLER(void)
{
#define FTM_CHANNEL_FLAG kFTM_Chnl0Flag

    if ((FTM_GetStatusFlags(MCP3911_OSC_FTM_BASEADDR) & FTM_CHANNEL_FLAG) == FTM_CHANNEL_FLAG)
    {
        /* Clear interrupt flag.*/
        FTM_ClearStatusFlags(MCP3911_OSC_FTM_BASEADDR, FTM_CHANNEL_FLAG);
    }

}

edma_handle_t g_EDMA_Handle;
volatile bool g_Transfer_Done = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

/* User callback function for EDMA transfer. */
void EDMA_Callback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    if (transferDone)
    {
        g_Transfer_Done = true;
    }
}

static void master_task(void *pvParameters)
{
    uint32_t srcAddr[BUFF_LENGTH] = {0x01, 0x02, 0x03, 0x04};
    uint32_t destAddr[BUFF_LENGTH] = {0x00, 0x00, 0x00, 0x00};

    edma_transfer_config_t transferConfig;
    edma_config_t userConfig;

    /* Configure DMAMUX */
    DMAMUX_Init(DMAMUX0);
    DMAMUX_SetSource(DMAMUX0, 0, 63);
    DMAMUX_EnableChannel(DMAMUX0, 0);
    /* Configure EDMA one shot transfer */
    /*
     * userConfig.enableRoundRobinArbitration = false;
     * userConfig.enableHaltOnError = true;
     * userConfig.enableContinuousLinkMode = false;
     * userConfig.enableDebugMode = false;
     */
    EDMA_GetDefaultConfig(&userConfig);
    EDMA_Init(DMA0, &userConfig);
    EDMA_CreateHandle(&g_EDMA_Handle, DMA0, 0);
    EDMA_SetCallback(&g_EDMA_Handle, EDMA_Callback, NULL);
    EDMA_PrepareTransfer(&transferConfig, srcAddr, sizeof(srcAddr[0]), destAddr, sizeof(destAddr[0]),
                         sizeof(srcAddr[0]), sizeof(srcAddr), kEDMA_MemoryToMemory);
    EDMA_SubmitTransfer(&g_EDMA_Handle, &transferConfig);
    EDMA_StartTransfer(&g_EDMA_Handle);
    /* Wait for EDMA transfer finish */
    while (g_Transfer_Done != true)
    {
    }


	while(1)
	{
	    printf("master task\n");
		vTaskDelay( 1000 ); // delay

	}
}

/*
 * @brief   Application entry point.
 */
int main(void) {

  	/* Init board hardware. */
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    printf("Hello World\n");

    if (xTaskCreate(master_task, "Master_task", configMINIMAL_STACK_SIZE + 64, NULL, configMAX_PRIORITIES-1, NULL) !=
        pdPASS)
    {
        PRINTF("Failed to create master task");
        vTaskSuspend(NULL);
    }

    vTaskStartScheduler();
    for (;;)
        ;
    return 0;
}
