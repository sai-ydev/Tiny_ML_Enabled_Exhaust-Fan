/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "app.h"
#include "fsl_lpuart.h"
#include "fsl_lpi2c.h"
#include "fsl_utick.h"
#include "fsl_debug_console.h"
#include "WizFi360.h"
#include "emc2101.h"
#include "bme68x.h"
#include "common.h"
#include "bsec_integration.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Ring buffer size (Unit: Byte). */
#define DEMO_RING_BUFFER_SIZE 16
#define LPI2C_MASTER_CLOCK_FREQUENCY CLOCK_GetLPFlexCommClkFreq(2u)
#define LPI2C_BAUDRATE               100000U
#define UTICK_TIME_1MS (1000UL)
#define _1_MIN  UINT16_C(1000 * 60)
/*! @brief Ring buffer to save received data. */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
volatile uint32_t tick_count;
static void state_save(const uint8_t *state_buffer, uint32_t length);
static uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer);
static uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer);
static void tvoc_equivalent_calibration(void);
static void output_ready(char *outputs);
/*******************************************************************************
 * Variables
 ******************************************************************************/
WizFi360_t WizFi360;
static uint8_t dev_addr[NUM_OF_SENS];
static uint32_t counter_ms = 0;
static bool tvoc_disable_flag = false;
static bool otf_temp = true;
uint8_t bme_cs[NUM_OF_SENS];
extern uint8_t n_sensors;
extern uint8_t *bsecInstance[NUM_OF_SENS];
/*
  Ring buffer for data input and output, in this example, input data are saved
  to ring buffer in IRQ handler. The main function polls the ring buffer status,
  if there are new data, then send them out.
  Ring buffer full: (((rxIndex + 1) % DEMO_RING_BUFFER_SIZE) == txIndex)
  Ring buffer empty: (rxIndex == txIndex)
*/
uint8_t demoRingBuffer[DEMO_RING_BUFFER_SIZE];
volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */

/*******************************************************************************
 * Code
 ******************************************************************************/
static void UTickCallback(void)
{
   tick_count++;

}

static uint32_t UTick_GetTick(void)
{
	return tick_count;
}

void DEMO_LPUART_IRQHandler(void)
{
    uint8_t data;
    uint16_t tmprxIndex = rxIndex;
    uint16_t tmptxIndex = txIndex;

    /* If new data arrived. */
    if ((kLPUART_RxDataRegFullFlag)&LPUART_GetStatusFlags(DEMO_LPUART))
    {
        data = LPUART_ReadByte(DEMO_LPUART);

        /* If ring buffer is not full, add data to ring buffer. */
        if (((tmprxIndex + 1) % DEMO_RING_BUFFER_SIZE) != tmptxIndex)
        {
            demoRingBuffer[rxIndex] = data;
            rxIndex++;
            rxIndex %= DEMO_RING_BUFFER_SIZE;
        }
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Main function
 */
int main(void)
{
    lpuart_config_t config;
    lpi2c_master_config_t masterConfig;
    uint16_t tmprxIndex = rxIndex;
    uint16_t tmptxIndex = txIndex;
    uint8_t sock;
    bsec_version_t version;
    WizFi360_Connection_t* Connection;
    return_values_init ret = {BME68X_OK, BSEC_OK};
    char header[400];
    float ext_temp;
    int8_t int_temp;
    uint8_t duty_cycle;
    uint16_t fan_rpm;
    BOARD_InitHardware();

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kLPUART_ParityDisabled;
     * config.stopBitCount = kLPUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 0;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    LPUART_GetDefaultConfig(&config);
    LPI2C_MasterGetDefaultConfig(&masterConfig);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx     = true;
    config.enableRx     = true;

    LPUART_Init(DEMO_LPUART, &config, DEMO_LPUART_CLK_FREQ);
    LPUART_Init(LPUART1, &config, DEMO_LPUART_CLK_FREQ);

    PRINTF("WizFI360 Example\r\n");

    /* Enable RX interrupt. */
    LPUART_EnableInterrupts(LPUART1, kLPUART_RxDataRegFullInterruptEnable);
    EnableIRQ(LP_FLEXCOMM1_IRQn);

    /* Change the default baudrate configuration */
    masterConfig.baudRate_Hz = LPI2C_BAUDRATE;

    /* Initialize the LPI2C master peripheral */
    LPI2C_MasterInit(LPI2C2, &masterConfig, LPI2C_MASTER_CLOCK_FREQUENCY);

	UTICK_Init(UTICK0);

	UTICK_SetTick(UTICK0, kUTICK_Repeat, (UTICK_TIME_1MS - 1), UTickCallback);


    /* Init ESP module */
	while (WizFi360_Init(&WizFi360, 115200) != ESP_OK) {
		PRINTF("Problems with initializing module!\r\n");
	}

	/* Set mode to STA+AP */
	while (WizFi360_SetMode(&WizFi360, WizFi360_Mode_STA_AP) != ESP_OK);

	/* Enable server on port 80 */
	while (WizFi360_ServerEnable(&WizFi360, 80) != ESP_OK);

	/* Module is connected OK */
	PRINTF("Initialization finished!\r\n");

	/* Disconnect from wifi if connected */
	WizFi360_WifiDisconnect(&WizFi360);

	/* Get a list of all stations  */
	WizFi360_ListWifiStations(&WizFi360);

	/* Wait till finishes */
	WizFi360_WaitReady(&WizFi360);

	WizFi360_WifiConnect(&WizFi360, "", "");

	/* Wait till finish */
	WizFi360_WaitReady(&WizFi360);

	/* Get connected devices */
	WizFi360_WifiGetConnected(&WizFi360);

	WizFi360_Update(&WizFi360);

	emc2101_init();
	emc2101_enable_tach_input(true);
	emc2101_set_PWM_divisor(0);
	emc2101_set_duty_cycle(50);

	while(1){
		emc2101_get_external_temperature(&ext_temp);
		emc2101_get_internalTemperature(&int_temp);
		emc2101_get_duty_cycle(&duty_cycle);
		emc2101_get_fan_rpm(&fan_rpm);
		PRINTF("External Temperature is %3.2f\r\n", ext_temp);
		PRINTF("Internal Temperature is %d\r\n",int_temp);
		PRINTF("Duty Cycle is %u\r\n", duty_cycle);
		PRINTF("RPM is %u\r\n", fan_rpm);

		SDK_DelayAtLeastUs(1000000, CLOCK_GetFreq(kCLOCK_CoreSysClk));
	}

	ret = bsec_iot_init(SAMPLE_RATE, bme68x_interface_init, state_load, config_load);
	if (ret.bme68x_status != BME68X_OK) {
		PRINTF("ERROR while initializing BME69x: %d\r\n", ret.bme68x_status);
		return ret.bme68x_status;
	}
	if (ret.bsec_status < BSEC_OK) {
		PRINTF("\nERROR while initializing BSEC library: %d\r\n", ret.bsec_status);
		return ret.bsec_status;
	}
	else if (ret.bsec_status > BSEC_OK) {
		PRINTF("\nWARNING while initializing BSEC library: %d\r\n", ret.bsec_status);
	}

	ret.bsec_status = bsec_get_version(bsecInstance, &version);

	PRINTF("BSEC Version : %u.%u.%u.%u\r\n",version.major,version.minor,version.major_bugfix,version.minor_bugfix);
#if (OUTPUT_MODE == IAQ)
	sprintf(header, "Sensor_No, Time(ms), IAQ,  IAQ_accuracy, Static_IAQ, TVOC_Equivalent, TVOC_Equivalent_Accuracy, Raw_Temperature(degC), Raw_Humidity(%%rH), Comp_Temperature(degC),  Comp_Humidity(%%rH), Raw_pressure(Pa), Raw_Gas(ohms), Gas_percentage, CO2, bVOC, Stabilization_status, Run_in_status, Bsec_status\r\n");
#else
	sprintf(header, "Sensor_No, Time(ms), Class/Target_1_prediction, Class/Target_2_prediction, Class/Target_3_prediction, Class/Target_4_prediction, Prediction_accuracy_1, Prediction_accuracy_2, Prediction_accuracy_3, Prediction_accuracy_4, Raw_pressure(Pa), Raw_Temperature(degC),  Raw_Humidity(%%rH), Raw_Gas(ohm), Raw_Gas_Index(num), Bsec_status\r\n");
#endif

	PRINTF("%s", header);

	bsec_iot_loop(state_save, UTick_GetTick, output_ready, tvoc_equivalent_calibration);


    while (1)
    {
        /* Send data only when LPUART TX register is empty and ring buffer has data to send out. */
        while (kLPUART_TxDataRegEmptyFlag & LPUART_GetStatusFlags(DEMO_LPUART))
        {
            tmprxIndex = rxIndex;
            tmptxIndex = txIndex;
            if (tmprxIndex != tmptxIndex)
            {
                LPUART_WriteByte(DEMO_LPUART, demoRingBuffer[txIndex]);
                txIndex++;
                txIndex %= DEMO_RING_BUFFER_SIZE;
            }
        }
    }
}

static uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available,
    // otherwise return length of loaded state string.
    // ...
    return 0;
}

static uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{

    return 0;
}

static void output_ready(char *outputs)
{
    PRINTF("%s",outputs);
}

static void state_save(const uint8_t *state_buffer, uint32_t length)
{
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
}

/**
 * @brief Function to calibrate the TVOC equivalent by enabling and disabling the baseline adaptation.
 * Note : TVOC equivalent calibration is only possilbe in LP Mode.
 */
static void tvoc_equivalent_calibration(void)
{
    if (get_sample_rate() == BSEC_SAMPLE_RATE_LP)
    {
        if (counter_ms == 0)
        {
            set_tvoc_equivalent_baseline(true);
            tvoc_disable_flag = true;
        }
        else if ((counter_ms >= (_1_MIN * 30)) && tvoc_disable_flag)
        {
            set_tvoc_equivalent_baseline(false);
            tvoc_disable_flag = false;
        }
        counter_ms = UTick_GetTick();
    }
    else if (otf_temp)
    {
        PRINTF("[INFO] TVOC equivalent calibration not supported in current BSEC mode.\n");
        otf_temp = false;
    }
}

