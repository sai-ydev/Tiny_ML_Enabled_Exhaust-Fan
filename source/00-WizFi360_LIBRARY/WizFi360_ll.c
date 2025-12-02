/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2015
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#include "WizFi360_ll.h"
#include "fsl_lpuart.h"
#include "app.h"
/**************************************************************************/
/**************************************************************************/
/**************************************************************************/
/*                                                                        */
/*    Edit file name to WizFi360_ll.c and edit values for your platform    */
/*                                                                        */
/**************************************************************************/
/**************************************************************************/
/**************************************************************************/

uint8_t WizFi360_LL_USARTInit(uint32_t baudrate) {
	/* Init USART */


	/* Return 0 = Successful */
	return 0;
}

uint8_t WizFi360_LL_USARTSend(uint8_t* data, uint16_t count) {
	/* Send data via USART */
	LPUART_WriteBlocking(LPUART1, data, count);
	/* Return 0 = Successful */
	return 0;
}

void LP_FLEXCOMM1_IRQHandler(void)
{
	uint8_t data;
	
	if ((kLPUART_RxDataRegFullFlag) & LPUART_GetStatusFlags(LPUART1))
	{
		data = LPUART_ReadByte(LPUART1);
		WizFi360_DataReceived(&data, 1);
	}
	SDK_ISR_EXIT_BARRIER;
}

