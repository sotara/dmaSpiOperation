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
#include "fsl_dspi_edma.h"
#include "fsl_port.h"

#define USING_fsl_dspi_edma (1)

dspi_master_handle_t g_m_handle;

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef enum _AdcTaskCommands_e
{
	eAdcCmdNull,
	eAdcCmdConfigure,
	eAdcCmdStartSampleData,
	eAdcCmdStopSampleData,
	eAdcCmdWriteRegCommand,
	eAdcCmdReadRegCommand,
}eAdcTaskCommands_t;

#define ADC_Printf printf
#define NUM_BUFFERS (2) // number of memory buffers to read from
#define BUFFER_SIZE (256)
#define MCP3911_GPIOC_DATA_READY_IRQ_PRIORITY_LEVEL (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY +1)

#define MCP3911_DSPI_MASTER_BASEADDR SPI0
#define MCP3911_DSPI_MASTER_DMA_MUX_BASE DMAMUX_BASE
#define MCP3911_DSPI_MASTER_DMA_BASE DMA_BASE
#define MCP3911_DSPI_MASTER_DMA_RX_REQUEST_SOURCE kDmaRequestMux0SPI0Rx
#define MCP3911_DSPI_MASTER_DMA_TX_REQUEST_SOURCE kDmaRequestMux0SPI0Tx
#define MCP3911_DSPI_MASTER_DMA_BASEADDR ((DMA_Type *)(MCP3911_DSPI_MASTER_DMA_BASE))
#define MCP3911_DSPI_MASTER_PCS_FOR_TRANSFER kDSPI_MasterPcs0

#define MCP3911_DSPI_MASTER_DMA_MUX_BASEADDR ((DMAMUX_Type *)(MCP3911_DSPI_MASTER_DMA_MUX_BASE))

#define WRITE (1)
#define READ  (0)

#define ADC_DIFF_CH0_BASE ADC0
#define ADC_DIFF_CH0_GROUP 0U

#define ADC_DIFF_CH1_BASE ADC1
#define ADC_DIFF_CH1_GROUP 0U


/*******************************************************************************
 * Variables
 ******************************************************************************/
typedef struct ADCOperation_s
{
		volatile bool DataReadyFlag; /**< flag to indicate that a Data Ready has been detected */
		volatile bool DMADoneFlag; /**< indicate that the DMA of SPI data has completed */
		volatile int32_t Value;
		volatile int32_t LastValue;
		volatile int32_t Delta;
		volatile uint32_t Counter;
		uint16_t dataIdx;
		uint8_t bufferIdx;
		uint32_t adc0[NUM_BUFFERS][BUFFER_SIZE];
		uint32_t adc1[NUM_BUFFERS][BUFFER_SIZE];
}sADCOperation_t;

sADCOperation_t qs_ADCOperation;

typedef struct SampleStats_s
{
	uint32_t max;
	uint32_t min;
}sSampleStats_t;

enum _MCP3911WriteCmd_e
{
	eMCP3911RegAddr,
	eMCP3911DataByte1,
	eMCP3911DataByte2,
	eMCP3911NumberDataBytes,
	eMCP3911WriteCmdSize
};
static uint8_t tl_MCP3911WriteCmd[eMCP3911WriteCmdSize];
static dspi_master_edma_handle_t g_dspi_edma_m_handle;
volatile bool isTransferCompleted = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

/**
 * @note This is to allow for communication from the FRDM-K64 to the MCP3911
 * development board.
 * http://www.microchip.com/DevelopmentTools/ProductDetails.aspx?PartNO=ard00385
 * ( MCP3911 Single Phase Energy Meter Ref Design )
 *
 * The project will use the K64 SP0 bus (PTD0,PTD1,PTD2,PTD3) to jumper to
 * the MCP3911 boards Test Points TP2,TP3,TP4,TP5 as follows:
 *   CS - PTD0 <---> TP5
 *  CLK - PTD1 <---> TP4
 * MOSI - PTD2 <---> TP2
 * MISO - PTD3 <---> TP3
 * The series resistors have to be lifted (R58...R61)
 */

#define TRANSFER_SIZE 32U        /*!< SPI bus Transfer dataSize */
uint8_t masterRxData[TRANSFER_SIZE] = {0U}; /**< SPI bus receive buffer */
uint8_t masterTxData[TRANSFER_SIZE] = {0U}; /**< SPI bus transmit buffer */
dspi_transfer_t masterXfer;

/**<
 * @brief structure used to handle the SPI bus communication tranfers
 */
typedef struct MCP3911SpiTransfer_s
{
	uint16_t trxSize; /**< number of bytes to transfer */
	uint32_t txCount; /**< number of tx bytes written to SPI FIFO */
	uint32_t rxCount; /**< number fo Rx bytes received by BUS */
	uint8_t trxComplete; /**< flag to signal the SPI transfer has been completed */
	uint8_t dmaRequest; /**< @todo remove, debug flag is used to signal a dma request is to be made */
	uint8_t fault; /**< @todo remove, debug flag is used signal fault problems, */
}sMCP3911SpiTransfer_t;

/** @TODO replace static definition here, this was made global to help debug
static volatile sMCP3911SpiTransfer_t ts_Spi0Tranfer; */
volatile sMCP3911SpiTransfer_t ts_MCP3911SpiTranfer = {0,0,0,1,0,0};
static volatile int32_t ts_SingleCmdCompleteFlag = 0;

int32_t retStatusValues[25];
int32_t retStatusValuesidx = 0;

static void DSPI_MasterTrigger()
{
#define MCP3911_ADC_READ_SIZE (13)
	uint32_t primaskReg;
	int32_t status;

	primaskReg = DisableGlobalIRQ();
	if( ts_MCP3911SpiTranfer.trxComplete == 1)
	{
		masterTxData[0] = 0x1;
		masterTxData[1] = 0x0;
		masterTxData[2] = 0x0;
		masterTxData[3] = 0x0;
		masterTxData[4] = 0x0;

		ts_MCP3911SpiTranfer.trxComplete = 0;
		masterXfer.txData = masterTxData;
		masterXfer.rxData = masterRxData;
		masterXfer.dataSize = MCP3911_ADC_READ_SIZE;
		masterXfer.configFlags = kDSPI_MasterCtar0 | MCP3911_DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;

		GPIO_ClearPinsOutput(PROFILE_GPIO, 1U << PROFILE_PIN_PTC3); /**< set pin low */
#if USING_fsl_dspi_edma
		status = DSPI_MasterTransferEDMA(MCP3911_DSPI_MASTER_BASEADDR, &g_dspi_edma_m_handle, &masterXfer);
#else
		status = DSPI_MasterTransferNonBlocking(MCP3911_DSPI_MASTER_BASEADDR, &g_m_handle, &masterXfer);
#endif
	    GPIO_SetPinsOutput(PROFILE_GPIO, 1U << PROFILE_PIN_PTC3); /**< set pin high */
		if( status )
		{
			if( retStatusValuesidx < 25)
			{
				retStatusValues[retStatusValuesidx++] = status;
			}
			else
			{
				retStatusValuesidx = 0;
				retStatusValues[retStatusValuesidx++] = status;
			}
		}
		ts_MCP3911SpiTranfer.dmaRequest = 1; /* set flag to indicate that we're making a request */
	}
	else
	{
		ts_MCP3911SpiTranfer.fault = 1;
	}
	EnableGlobalIRQ(primaskReg);

}


/**
 * @brief MCP3911 Data Ready IRQ Handler
 *
 * @details This IRQ handler will be called every time the MCP3911 Data Ready IRQ event occurs.
 * This method will determine if MCP3911 Data Ready GPIO line has detected a dropping event.
 * This event indicates that the MCP3911 ADC data is ready to read.   On this
 * event, the handler will trigger a SPI0 read request to read the two 16 bit data
 * samples.
 *
 * @note Target operation would be to have the DMA operation handle this.
 */
void IRQ_MCP3911_DATA_READY_HANDLER(void)
{
#define MCP3911_ADC_READ_SIZE (13)
	uint32_t irqFlag;

	irqFlag = GPIO_GetPinsInterruptFlags(MCP3911_DATA_READY_GPIO);
	if( irqFlag & (1U << MCP3911_DATA_READY_GPIO_PIN ))
	{
		/* Clear external interrupt flag. */
		GPIO_ClearPinsInterruptFlags(MCP3911_DATA_READY_GPIO, 1U << MCP3911_DATA_READY_GPIO_PIN);

		DSPI_MasterTrigger();
	}

}


/**
 * @brief Callback method for eDMA transfer
 *
 * @details Method called as the eDMA complete IRQ handler.
 * This method does a buffer ping pong operations to allow for task code to copy data off before
 * being overwritten.
 *
 */
#if USING_fsl_dspi_edma
void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData)
#else
void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData)
#endif
{
	enum _AdcReadArray_e
	{
		eReadCommand,
		eADC0lb,
		eADC0ub,
		eADC1lb,
		eADC1ub,
		eModReg,
		ePhaseRegub,
		ePhaseReglb,
		eGainReg,
		eComRegub,
		eComReglb,
		eCnfgRegub,
		eCnfgReglb,
	};

	union channelEndian
	{
			char charData[4];
			uint32_t sampleData;
	}adcData;

    assert(qs_ADCOperation.bufferIdx < NUM_BUFFERS);
    assert(qs_ADCOperation.dataIdx < BUFFER_SIZE);

	/*
	 * will first verify that the data bytes were ready to read
	 */
	if( (masterTxData[eReadCommand] == 0x1)  && !(masterRxData[eComRegub] & 0x3) )
	{
		/*
		 * if reading all registers then were expecting the MCP3911 to be running
		 * from DataReady pulses.
		 * if the Data Ready Status is not zero, then the two
		 * ADC channels are not ready to read.  Don't bother reading, wait for next
		 * Data Ready pulse
		 */

	    /* copy channel 0 ADC sample */
	    adcData.sampleData = 0;
	    adcData.charData[1] = masterRxData[eADC0lb]; /* need to endian flip here */
	    adcData.charData[0] = masterRxData[eADC0ub];
		qs_ADCOperation.adc0[qs_ADCOperation.bufferIdx][qs_ADCOperation.dataIdx] = adcData.sampleData;

	    /* copy channel 1 ADC sample */
		adcData.sampleData = 0;
	    adcData.charData[1] = masterRxData[eADC1lb]; /* need to endian flip here */
	    adcData.charData[0] = masterRxData[eADC1ub];
		qs_ADCOperation.adc1[qs_ADCOperation.bufferIdx][qs_ADCOperation.dataIdx] = adcData.sampleData;

		qs_ADCOperation.dataIdx++;

		if( qs_ADCOperation.dataIdx == BUFFER_SIZE)
		{
			/*
			 * set flag to indicate buffer is full
			 */
			qs_ADCOperation.DMADoneFlag = true;
			qs_ADCOperation.dataIdx = 0;
			qs_ADCOperation.bufferIdx++;

			if(qs_ADCOperation.bufferIdx == NUM_BUFFERS)
			{
				qs_ADCOperation.bufferIdx = 0;
			}
		}
	}
	else
	{
		ts_MCP3911SpiTranfer.fault |= 2;
	}

    ts_MCP3911SpiTranfer.trxComplete = 1; /* set flag to indicate that the spi trx has completed */
	ts_MCP3911SpiTranfer.dmaRequest = 0; /* clear flag to indicate that dma request is done */
    ts_SingleCmdCompleteFlag = 1;

}



/**
 * @brief print data read from the SPI0 peripheral
 */
static void PrintReadResults()
{
	uint16_t idx;
	ADC_Printf("spi read: %x ",tl_MCP3911WriteCmd[eMCP3911RegAddr]);
	for(idx=1; idx<tl_MCP3911WriteCmd[eMCP3911NumberDataBytes]; idx++)
	{
		ADC_Printf("%x ",masterRxData[idx]);
	}
	ADC_Printf("\\n\n");
}


/**
 * @brief trigger a single MCP3911 TRX operation
 *
 * @params WriteFlag if set write operation else read operation
 * @return void
 */
static void SingleMCP3911TRX(uint8_t WriteFlag)
{
	uint16_t idx;
	int32_t status;
	uint32_t primaskReg;

	primaskReg = DisableGlobalIRQ();
	if ( ts_MCP3911SpiTranfer.trxComplete )
	{

		ts_MCP3911SpiTranfer.trxSize = 0;
		ts_MCP3911SpiTranfer.txCount = 0;
		ts_MCP3911SpiTranfer.rxCount = 0;

		for(idx=0; idx < tl_MCP3911WriteCmd[eMCP3911NumberDataBytes]; idx++)
		{
			if(idx == 0)
			{
				if(WriteFlag == READ)
				{
					masterTxData[ts_MCP3911SpiTranfer.trxSize++] = tl_MCP3911WriteCmd[idx] | 0x1; /* set read bit */
				}
				else
				{
					masterTxData[ts_MCP3911SpiTranfer.trxSize++] = tl_MCP3911WriteCmd[idx];
				}
			}
			else
			{
				masterTxData[ts_MCP3911SpiTranfer.trxSize++] = tl_MCP3911WriteCmd[idx];
			}
		}

		//    spiRequestTranfer();

		ts_MCP3911SpiTranfer.trxComplete = 0;
		masterXfer.txData = masterTxData;
		masterXfer.rxData = masterRxData;
		masterXfer.dataSize = tl_MCP3911WriteCmd[eMCP3911NumberDataBytes];
		masterXfer.configFlags = kDSPI_MasterCtar0 | MCP3911_DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;

		ts_SingleCmdCompleteFlag = 0;
#if USING_fsl_dspi_edma
		status = DSPI_MasterTransferEDMA(MCP3911_DSPI_MASTER_BASEADDR, &g_dspi_edma_m_handle, &masterXfer);
#else
		status = DSPI_MasterTransferNonBlocking(MCP3911_DSPI_MASTER_BASEADDR, &g_m_handle, &masterXfer);

#endif
		assert( !status );
		EnableGlobalIRQ(primaskReg);

	}
	else
	{
		EnableGlobalIRQ(primaskReg);
		ADC_Printf("ERROR SPI port not available for reqeust\r\n");
	}

}

/**
 * @brief set variables needed for a single MCP3911 write command
 *
 * @params RegAddr Internal register address
 * @params DataByte1 data byte to write
 * @params DataByte2 data byte to write
 * @params NumDataBytes number of data bytes to write
 * @return void
 */
static void SingleMCP3911Operation(uint8_t RegAddr, uint8_t DataByte1, uint8_t DataByte2, uint8_t NumDataBytes)
{
	tl_MCP3911WriteCmd[eMCP3911RegAddr] = RegAddr << 1; /** note register address must be shifted left 1 bit */
	tl_MCP3911WriteCmd[eMCP3911DataByte1] = DataByte1;
	tl_MCP3911WriteCmd[eMCP3911DataByte2] = DataByte2;
	tl_MCP3911WriteCmd[eMCP3911NumberDataBytes] = NumDataBytes+1;
}

/**
 * @brief perform power up configuration of the MCP3911
 *
 * @todo this configuration values may be updated as testing is done.
 *
 */
static void ConfigureMCP3911()
{
    // write the configuration register 0x0C
	SingleMCP3911Operation(0xC, 0x27, 0x02, 0x02);
	SingleMCP3911TRX(WRITE);

	while(ts_SingleCmdCompleteFlag == 0)
    {
		__asm("NOP"); /* delay */
    }

	// write the configuration register 0x0C
	SingleMCP3911Operation(0xC, 0x1F, 0x02, 0x02);
	SingleMCP3911TRX(WRITE);

	while(ts_SingleCmdCompleteFlag == 0)
    {
		__asm("NOP"); /* delay */
    }

	// read the configuration register 0x0C
	SingleMCP3911Operation(0xC, 0xFF, 0xFF, 0x02);
	SingleMCP3911TRX(READ);

	while(ts_SingleCmdCompleteFlag == 0)
    {
		__asm("NOP"); /* delay */
    }

	PrintReadResults();

    // write the phase register 0x07
	SingleMCP3911Operation(0x7, 0x00, 0x03, 0x02);
	SingleMCP3911TRX(WRITE);

	while(ts_SingleCmdCompleteFlag == 0)
    {
		__asm("NOP"); /* delay */
    }

    // read the phase register 0x07
	SingleMCP3911Operation(0x7, 0xff, 0xff, 0x02);
	SingleMCP3911TRX(READ);

	while(ts_SingleCmdCompleteFlag == 0)
    {
		__asm("NOP"); /* delay */
    }
	PrintReadResults();

    // write the Gain register 0x09
	SingleMCP3911Operation(0x9, 0x00, 0x01, 0x01);
	SingleMCP3911TRX(WRITE);

	while(ts_SingleCmdCompleteFlag == 0)
    {
		__asm("NOP"); /* delay */
    }
    // Read the Gain register 0x09
	SingleMCP3911Operation(0x9, 0xFF, 0xff, 0x01);
	SingleMCP3911TRX(READ);

	while(ts_SingleCmdCompleteFlag == 0)
    {
		__asm("NOP"); /* delay */
    }
	PrintReadResults();

    // write the Status Com register 0x0A
	SingleMCP3911Operation(0xA, 0x13, 0xE0, 0x02);
	SingleMCP3911TRX(WRITE);

	while(ts_SingleCmdCompleteFlag == 0)
    {
		__asm("NOP"); /* delay */
    }

	// Read the Status Com register 0x0A
	SingleMCP3911Operation(0xA, 0xff, 0xff, 0x02);
	SingleMCP3911TRX(READ);

	while(ts_SingleCmdCompleteFlag == 0)
    {
		__asm("NOP"); /* delay */
    }
	PrintReadResults();

}




static void mcp3911Task(void *pvParameters)
{
#define NUMBER_DATA_BUFFERS (10) /**< number of data buffers we will hold data to send to UDP port */
	uint32_t adcCmd = 0,adcState=0;
	uint32_t adcTaskState;
    uint32_t *adc0DataBuffer,*adc1DataBuffer,adcDataBufferIdx;
#if USING_fsl_dspi_edma
    edma_handle_t dspiEdmaMasterRxRegToRxDataHandle;
    edma_handle_t dspiEdmaMasterTxDataToIntermediaryHandle;
    edma_handle_t dspiEdmaMasterIntermediaryToTxRegHandle;
    uint32_t masterRxChannel, masterIntermediaryChannel, masterTxChannel;

    masterRxChannel = 0U;
    masterIntermediaryChannel = 1U;
    masterTxChannel = 2U;
#endif

    adc0DataBuffer = pvPortMalloc(BUFFER_SIZE*NUMBER_DATA_BUFFERS*sizeof(uint32_t));
    adc1DataBuffer = pvPortMalloc(BUFFER_SIZE*NUMBER_DATA_BUFFERS*sizeof(uint32_t));
    adcDataBufferIdx = 0;

    qs_ADCOperation.DMADoneFlag = false;
    qs_ADCOperation.Counter = 0;
    qs_ADCOperation.dataIdx = 0;
    qs_ADCOperation.bufferIdx = 0;

#if USING_fsl_dspi_edma
    /* DMA MUX init */
    DMAMUX_Init(MCP3911_DSPI_MASTER_DMA_MUX_BASEADDR);


    DMAMUX_SetSource(MCP3911_DSPI_MASTER_DMA_MUX_BASEADDR, masterRxChannel, MCP3911_DSPI_MASTER_DMA_RX_REQUEST_SOURCE);
    DMAMUX_EnableChannel(MCP3911_DSPI_MASTER_DMA_MUX_BASEADDR, masterRxChannel);

    DMAMUX_SetSource(MCP3911_DSPI_MASTER_DMA_MUX_BASEADDR, masterTxChannel, MCP3911_DSPI_MASTER_DMA_TX_REQUEST_SOURCE);
    DMAMUX_EnableChannel(MCP3911_DSPI_MASTER_DMA_MUX_BASEADDR, masterTxChannel);

    /* Set up dspi master */
    memset(&(dspiEdmaMasterRxRegToRxDataHandle), 0, sizeof(dspiEdmaMasterRxRegToRxDataHandle));
    memset(&(dspiEdmaMasterTxDataToIntermediaryHandle), 0, sizeof(dspiEdmaMasterTxDataToIntermediaryHandle));
    memset(&(dspiEdmaMasterIntermediaryToTxRegHandle), 0, sizeof(dspiEdmaMasterIntermediaryToTxRegHandle));

    EDMA_CreateHandle(&(dspiEdmaMasterRxRegToRxDataHandle), MCP3911_DSPI_MASTER_DMA_BASEADDR, masterRxChannel);
    EDMA_CreateHandle(&(dspiEdmaMasterTxDataToIntermediaryHandle), MCP3911_DSPI_MASTER_DMA_BASEADDR,
                      masterIntermediaryChannel);
    EDMA_CreateHandle(&(dspiEdmaMasterIntermediaryToTxRegHandle), MCP3911_DSPI_MASTER_DMA_BASEADDR, masterTxChannel);

//    NVIC_SetPriority(DMA0_IRQn, 1U);
    DSPI_MasterTransferCreateHandleEDMA(MCP3911_DSPI_MASTER_BASEADDR, &g_dspi_edma_m_handle, DSPI_MasterUserCallback,
										NULL, &dspiEdmaMasterRxRegToRxDataHandle,
										&dspiEdmaMasterTxDataToIntermediaryHandle,
										&dspiEdmaMasterIntermediaryToTxRegHandle);
#else
    DSPI_MasterTransferCreateHandle(MCP3911_DSPI_BASEADDR, &g_m_handle, DSPI_MasterUserCallback, NULL);

#endif
	/* Configure the PTC12 to generate IRQ on rising edge. */
	PORT_SetPinInterruptConfig(MCP3911_DATA_READY_PORT, MCP3911_DATA_READY_GPIO_PIN, kPORT_InterruptRisingEdge);
//	PORT_SetPinInterruptConfig(MCP3911_DATA_READY_PORT, MCP3911_DATA_READY_GPIO_PIN, kPORT_DMARisingEdge);
    NVIC_SetPriority(IRQ_MCP3911_DATA_READY_HANDLER_VECTOR, MCP3911_GPIOC_DATA_READY_IRQ_PRIORITY_LEVEL);


    while (1)
    {
//    	if( xTaskNotifyWait(0x00, 0xFFFFFFFF, &adcCmd, 1) == pdTRUE )
    	{
        	/**
        	 * @note Place breakpoint on the switch statement and provided the commands to configure and to start sampling.
        	 * In the actual code, a CLI command would be used here to issue these commands that would signal the xTaskNotifyWait
        	 * from above.
        	 * adcCmd == 1 for eAdcCmdConfigure
        	 * adcCmd == 2 for eAdcCmdStartSampleData
        	 */
    		switch(adcCmd)
    		{
    			case eAdcCmdConfigure:
    		    	ADC_Printf("eAdcCmdConfigure\r\n");
    		    	adcTaskState = eAdcCmdConfigure;
    				ConfigureMCP3911();
    				break;
    			case eAdcCmdStartSampleData:
    		    	ADC_Printf("eAdcCmdStartSampleData\r\n");
    			    EnableIRQ(IRQ_MCP3911_DATA_READY_HANDLER_VECTOR);
//    		    	ts_MCP3911SpiTranfer.dataReadyRequestFlag = 1;
//    		    	PORT_SetPinInterruptConfig(MCP3911_DATA_READY_PORT, MCP3911_DATA_READY_GPIO_PIN, kPORT_DMARisingEdge);
//    			    DMAMUX_SetSource(MCP3911_DSPI_MASTER_DMA_MUX_BASEADDR, masterTxChannel, kDmaRequestMux0PortB);
    			    DSPI_MasterTrigger();

//    				EDMA_PrepareTransfer(mcp3911TransferCnfg, masterTxData, sizeof(uint8_t), masterRxData, sizeof(uint8_t), MCP3911_ADC_READ_SIZE, MCP3911_ADC_READ_SIZE, kEDMA_MemoryToPeripheral);
//    				EDMA_SubmitTransfer()
//    				EDMA_StartTransfer(dspiEdmaMasterTxDataToIntermediaryHandle);
    			    adcTaskState = eAdcCmdStartSampleData;
    				break;
    			case eAdcCmdStopSampleData:
    		    	ADC_Printf("eAdcCmdStopSampleData\r\n");
    			    adcTaskState = eAdcCmdStopSampleData;
//    		    	ts_MCP3911SpiTranfer.dataReadyRequestFlag = 0;
    		    	PORT_SetPinInterruptConfig(MCP3911_DATA_READY_PORT, MCP3911_DATA_READY_GPIO_PIN, kPORT_InterruptRisingEdge);
//    			    DMAMUX_SetSource(MCP3911_DSPI_MASTER_DMA_MUX_BASEADDR, masterTxChannel, MCP3911_DSPI_MASTER_DMA_TX_REQUEST_SOURCE);
//    				DisableIRQ(IRQ_MCP3911_DATA_READY_HANDLER_VECTOR);
    				break;
    			case eAdcCmdWriteRegCommand:
    		    	ADC_Printf("eAdcCmdWriteRegCommand\r\n");
    		    	SingleMCP3911TRX(WRITE);
    				break;
    			case eAdcCmdReadRegCommand:
    			    adcTaskState = eAdcCmdReadRegCommand;
    		    	ADC_Printf("eAdcCmdReadRegCommand\r\n");
    		    	SingleMCP3911TRX(READ);
    				break;
    		}
    		if(adcCmd)
    		{
    			ADC_Printf("CLI cmd %x \n",adcCmd);
    			adcState = adcCmd;
    			adcCmd = 0;
    		}
    	}

    	if( adcState == eAdcCmdReadRegCommand)
    	{
    		if(ts_MCP3911SpiTranfer.trxComplete == 1)
    		{
    			PrintReadResults();
        		adcTaskState = eAdcCmdNull;
    		}
    	}

    	if( adcState == eAdcCmdStartSampleData)
    	{
    		if( qs_ADCOperation.DMADoneFlag )
    		{
    			qs_ADCOperation.DMADoneFlag = 0;

//    			memcpy(&adc0DataBuffer[adcDataBufferIdx*BUFFER_SIZE],qs_ADCOperation.adc0[qs_ADCOperation.bufferIdx],BUFFER_SIZE*sizeof(uint32_t));
//
//    			memcpy(&adc1DataBuffer[adcDataBufferIdx*BUFFER_SIZE],qs_ADCOperation.adc1[qs_ADCOperation.bufferIdx],BUFFER_SIZE*sizeof(uint32_t));

    			adcDataBufferIdx++;
    			if(adcDataBufferIdx >= NUMBER_DATA_BUFFERS)
    			{
    				adcDataBufferIdx = 0;
    			}
    		}
    	}
    	else
    	{
    		vTaskDelay( 100 ); // delay
    	}

    }

    vTaskDelete( NULL ); // End the task

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

    if (xTaskCreate(mcp3911Task, "mcp3911Task", configMINIMAL_STACK_SIZE + 64, NULL, configMAX_PRIORITIES-1, NULL) !=
        pdPASS)
    {
    	ADC_Printf("Failed to create master task");
        vTaskSuspend(NULL);
    }

    vTaskStartScheduler();
    for (;;)
        ;
    return 0;
}
