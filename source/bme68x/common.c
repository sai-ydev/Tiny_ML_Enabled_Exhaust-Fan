/**
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "bme68x.h"
#include "common.h"
#include "fsl_lpi2c.h"
#include "fsl_common_arm.h"
#include "fsl_debug_console.h"
#include "app.h"

/******************************************************************************/
/*!                Static variable definition                                 */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    lpi2c_master_transfer_t read_transfer;

    read_transfer.slaveAddress = device_addr;
    read_transfer.direction = kLPI2C_Read;
    read_transfer.subaddress = reg_addr;
    read_transfer.subaddressSize = 1;
    read_transfer.data = reg_data;
    read_transfer.dataSize = len;
    read_transfer.flags = kLPI2C_TransferDefaultFlag;

    status_t status = LPI2C_MasterTransferBlocking(LPI2C2, &read_transfer);



    return (BME68X_INTF_RET_TYPE) status;
}

/*!
 * I2C write function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    lpi2c_master_transfer_t write_transfer;

	write_transfer.slaveAddress = device_addr;
	write_transfer.direction = kLPI2C_Write;
	write_transfer.subaddress = reg_addr;
	write_transfer.subaddressSize = 1;
	write_transfer.data = reg_data;
	write_transfer.dataSize = len;
	write_transfer.flags = kLPI2C_TransferDefaultFlag;

	status_t status = LPI2C_MasterTransferBlocking(LPI2C2, &write_transfer);



	return (BME68X_INTF_RET_TYPE) status;
}



/*!
 * Delay function map to COINES platform
 */
void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    SDK_DelayAtLeastUs(period, CLOCK_GetFreq(kCLOCK_CoreSysClk));

}

void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:

            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            PRINTF("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
        	PRINTF("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
        	PRINTF("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
        	PRINTF("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
        	PRINTF("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
        	PRINTF("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            break;
        default:
        	PRINTF("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf)
{
    int8_t rslt = BME68X_OK;

    if (bme != NULL)
    {


#if defined(PC)
        setbuf(stdout, NULL);
#endif

        /* Bus configuration : I2C */
        if (intf == BME68X_I2C_INTF)
        {
        	PRINTF("I2C Interface\n");
            dev_addr = BME68X_I2C_ADDR_HIGH;
            bme->read = bme68x_i2c_read;
            bme->write = bme68x_i2c_write;
            bme->intf = BME68X_I2C_INTF;

        }


        bme->delay_us = bme68x_delay_us;
        bme->intf_ptr = &dev_addr;
        bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
    }
    else
    {
        rslt = BME68X_E_NULL_PTR;
    }

    return rslt;
}


