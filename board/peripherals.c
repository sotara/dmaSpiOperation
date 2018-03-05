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

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v1.0
* BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/**
 * @file    peripherals.c
 * @brief   Peripherals initialization file.
 */
 
/* This is a template for board specific configuration created by MCUXpresso IDE Project Wizard.*/

#include "FreeRTOSConfig.h"
#include "board.h"
#include "pin_mux.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_port.h"
#include "fsl_dspi.h"
#include "fsl_gpio.h"
#include "fsl_ftm.h"
#include "fsl_rtc.h"
#include "priorityLevels.h"
#include "peripherals.h"

/**
 * @brief Initialize SPI0 Peripheral
 *
 * @todo @@@TSB need to add full details on how why SPI0 is being initalized like it is
 */
static void InitializeSPI0()
{
#define TRANSFER_BAUDRATE (4000000U) /*! Transfer baudrate - 4000k */
	uint32_t srcClock_Hz;
	dspi_master_config_t masterConfig;
    gpio_pin_config_t mcp3911Reset = {
        kGPIO_DigitalOutput, 0,
    };
    gpio_pin_config_t mcp3911DataReady = {
    		kGPIO_DigitalInput, 0,
    };

    /* Master config */
    masterConfig.whichCtar = kDSPI_Ctar0;
    masterConfig.ctarConfig.baudRate = TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.bitsPerFrame = 8;
    masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveLow;
    masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseSecondEdge;
    masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
    masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;

    masterConfig.whichPcs = kDSPI_Pcs0;
    masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

    masterConfig.enableContinuousSCK = false;
    masterConfig.enableRxFifoOverWrite = false;
    masterConfig.enableModifiedTimingFormat = false;
    masterConfig.samplePoint = kDSPI_SckToSin0Clock;

    srcClock_Hz = CLOCK_GetFreq(DSPI0_CLK_SRC);
    DSPI_MasterInit(MCP3911_DSPI_BASEADDR, &masterConfig, srcClock_Hz);
    GPIO_PinInit(MCP3911_RESET_GPIO, MCP3911_RESET_GPIO_PIN, &mcp3911Reset);
    GPIO_PinInit(MCP3911_DATA_READY_GPIO, MCP3911_DATA_READY_GPIO_PIN, &mcp3911DataReady);

    GPIO_SetPinsOutput(MCP3911_RESET_GPIO, 1U << MCP3911_RESET_GPIO_PIN); /**< set reset pin high to take part out of reset */

    /*
     * set the SPI0 IRQ priority
     */
//    NVIC_SetPriority(SPI0_IRQn, MCP3911_SPI0_IRQ_PRIORITY_LEVEL);
//    /* Enable the NVIC for DSPI peripheral. */
//    EnableIRQ(SPI0_IRQn);
}

static void InitializeRTC()
{
    rtc_config_t rtcConfig;
    /* Init RTC */
    /*
     * rtcConfig.wakeupSelect = false;
     * rtcConfig.updateMode = false;
     * rtcConfig.supervisorAccess = false;
     * rtcConfig.compensationInterval = 0;
     * rtcConfig.compensationTime = 0;
     */
    RTC_GetDefaultConfig(&rtcConfig);
    RTC_Init(RTC, &rtcConfig);
    /* Select RTC clock source */
    /* Enable the RTC 32KHz oscillator */
    RTC->CR |= RTC_CR_OSCE_MASK;
}

/**
 * @brief Initialize FTM0 for Pilot Control
 *
 */
static void InitializeFTM0()
{
#define MCP3911_OSC_RATE (4000000U) // 4MHz clock rate
    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam;
    ftm_pwm_level_select_t pwmLevel = kFTM_LowTrue;

    /* Configure ftm params with frequency 24kHZ */
    ftmParam.chnlNumber = MCP3911_OSC_FTM_CHANNEL;
    ftmParam.level = pwmLevel;
    ftmParam.dutyCyclePercent = 50U;
    ftmParam.firstEdgeDelayPercent = 0U;

    FTM_GetDefaultConfig(&ftmInfo);
    /* Initialize FTM module */
    FTM_Init(MCP3911_OSC_FTM_BASEADDR, &ftmInfo);

    if ( FTM_SetupPwm(MCP3911_OSC_FTM_BASEADDR, &ftmParam, 1U, kFTM_EdgeAlignedPwm, MCP3911_OSC_RATE, CLOCK_GetFreq(kCLOCK_BusClk)) )
    {
    	while(1);
    }

//    /* Enable channel interrupt flag.*/
//    FTM_EnableInterrupts(PILOT_CNTL_FTM_BASEADDR, FTM_CHANNEL_INTERRUPT_ENABLE);
//
//    /* Enable at the NVIC */
//    NVIC_SetPriority(IRQ_PILOT_CNTL_FTM_HANDLER_VECTOR, PILOT_CNTL_IRQ_PRIORITY_LEVEL);
//    EnableIRQ(IRQ_PILOT_CNTL_FTM_HANDLER_VECTOR);

    FTM_StartTimer(MCP3911_OSC_FTM_BASEADDR, kFTM_SystemClock);


}

/**
 * @brief Initialize FTM0 for Pilot Control
 *
 */
static void InitializeFTM2()
{
#define PILOT_CNTL_RATE (400U) // 400Hz clock rate
    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam;
    ftm_pwm_level_select_t pwmLevel = kFTM_LowTrue;

    gpio_pin_config_t pilotCntlPinConfig = {
        kGPIO_DigitalOutput, 0,
    };

    /* Configure ftm params with frequency 24kHZ */
    ftmParam.chnlNumber = PILOT_CNTL_FTM_CHANNEL;
    ftmParam.level = pwmLevel;
    ftmParam.dutyCyclePercent = 50U;
    ftmParam.firstEdgeDelayPercent = 0U;

    FTM_GetDefaultConfig(&ftmInfo);
    /* Initialize FTM module */
    ftmInfo.prescale = kFTM_Prescale_Divide_8; // must select prescale to allow for slower PWM rate.
    FTM_Init(PILOT_CNTL_FTM_BASEADDR, &ftmInfo);

    if (FTM_SetupPwm(PILOT_CNTL_FTM_BASEADDR, &ftmParam, 1U, kFTM_EdgeAlignedPwm, PILOT_CNTL_RATE, CLOCK_GetFreq(kCLOCK_BusClk)) )
    {
    	while(1);
    }

    /* Enable channel interrupt flag.*/
    FTM_EnableInterrupts(PILOT_CNTL_FTM_BASEADDR, FTM_CHANNEL_INTERRUPT_ENABLE);

    /* Enable at the NVIC */
    NVIC_SetPriority(IRQ_PILOT_CNTL_FTM_HANDLER_VECTOR, PILOT_CNTL_IRQ_PRIORITY_LEVEL);
    EnableIRQ(IRQ_PILOT_CNTL_FTM_HANDLER_VECTOR);

    FTM_StartTimer(PILOT_CNTL_FTM_BASEADDR, kFTM_SystemClock);

    GPIO_PinInit(PILOT_CNTL_GPIO, PILOT_CNTL_GPIO_PIN, &pilotCntlPinConfig);

}


/**
 * @brief Set up and initialize all required blocks and functions related to the peripherals hardware.
 */
void BOARD_InitBootPeripherals(void)
{

	InitializeSPI0();
	InitializeRTC();
	InitializeFTM0();
//	InitializeFTM2();
}
