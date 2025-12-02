/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2016
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
#include "WizFi360.h"
#include "fsl_debug_console.h"

/* Commands list */
#define WizFi360_COMMAND_IDLE           0
#define WizFi360_COMMAND_CWQAP          1
#if WizFi360_USE_APSEARCH
#define WizFi360_COMMAND_CWLAP          2
#endif
#define WizFi360_COMMAND_CWJAP          3
#if WizFi360_USE_FIRMWAREUPDATE
#define WizFi360_COMMAND_CIUPDATE       4
#endif
#define WizFi360_COMMAND_CWMODE         5
#define WizFi360_COMMAND_CIPSERVER      6
#define WizFi360_COMMAND_CIPDINFO       7
#define WizFi360_COMMAND_SEND           8
#define WizFi360_COMMAND_CLOSE          9
#define WizFi360_COMMAND_CIPSTART       10
#define WizFi360_COMMAND_CIPMUX         11
#define WizFi360_COMMAND_CWSAP          12
#define WizFi360_COMMAND_ATE            13
#define WizFi360_COMMAND_AT             14
#define WizFi360_COMMAND_RST            15
#define WizFi360_COMMAND_RESTORE        16
#define WizFi360_COMMAND_UART           17
#define WizFi360_COMMAND_CWJAP_GET      19
#define WizFi360_COMMAND_SLEEP          20
#define WizFi360_COMMAND_GSLP           21
#define WizFi360_COMMAND_CIPSTA         22
#define WizFi360_COMMAND_CIPAP          23
#define WizFi360_COMMAND_CIPSTAMAC      24
#define WizFi360_COMMAND_CIPAPMAC       25
#define WizFi360_COMMAND_CIPSTO         26
#define WizFi360_COMMAND_CWLIF          27
#define WizFi360_COMMAND_CIPSTATUS      28
#define WizFi360_COMMAND_SENDDATA       29

#if WizFi360_USE_PING
#define WizFi360_COMMAND_PING           18
#endif

#define WizFi360_DEFAULT_BAUDRATE       115200 /*!< Default WizFi360 baudrate */
#define WizFi360_TIMEOUT                30000  /*!< Timeout value is milliseconds */

/* Debug */
#define WizFi360_DEBUG(x)               PRINTF("%s", x)

/* Maximum number of return data size in one +IPD from WizFi360 module */
#define ESP8255_MAX_BUFF_SIZE          5842

/* Temporary buffer */
static BUFFER_t TMP_Buffer;
static BUFFER_t USART_Buffer;
static uint8_t TMPBuffer[WizFi360_TMPBUFFER_SIZE];
static uint8_t USARTBuffer[WizFi360_USARTBUFFER_SIZE];

#if WizFi360_USE_APSEARCH
/* AP list */
static WizFi360_APs_t WizFi360_APs;
#endif

/* Create data array for connections */
#if WizFi360_USE_SINGLE_CONNECTION_BUFFER == 1
static char ConnectionData[WizFi360_CONNECTION_BUFFER_SIZE]; /*!< Data array */
#endif

/* Private functions */
#if WizFi360_USE_APSEARCH
static void ParseCWLAP(WizFi360_t* WizFi360, char* Buffer);
#endif
static void ParseCIPSTA(WizFi360_t* WizFi360, char* Buffer);
static void ParseCWSAP(WizFi360_t* WizFi360, char* Buffer);
static void ParseCWJAP(WizFi360_t* WizFi360, char* Buffer);
static void ParseCWLIF(WizFi360_t* WizFi360, char* Buffer);
static void ParseIP(char* ip_str, uint8_t* arr, uint8_t* cnt);
static void ParseMAC(char* ptr, uint8_t* arr, uint8_t* cnt);
static void ParseReceived(WizFi360_t* WizFi360, char* Received, uint8_t from_usart_buffer, uint16_t bufflen);
void CopyCharacterUSART2TMP(WizFi360_t* WizFi360);
static char* EscapeString(char* str);
char* ReverseEscapeString(char* str);
static WizFi360_Result_t SendCommand(WizFi360_t* WizFi360, uint8_t Command, char* CommandStr, char* StartRespond);
static WizFi360_Result_t SendUARTCommand(WizFi360_t* WizFi360, uint32_t baudrate, char* cmd);
static WizFi360_Result_t SendMACCommand(WizFi360_t* WizFi360, uint8_t* addr, char* cmd, uint8_t command);
static void CallConnectionCallbacks(WizFi360_t* WizFi360);
static void ProcessSendData(WizFi360_t* WizFi360);
void* mem_mem(void* haystack, size_t haystacksize, void* needle, size_t needlesize);

#define CHARISNUM(x)    ((x) >= '0' && (x) <= '9')
static uint8_t CHARISHEXNUM(char a);
#define CHAR2NUM(x)     ((x) - '0')
static uint8_t Hex2Num(char a);
static int32_t ParseNumber(char* ptr, uint8_t* cnt);
static uint32_t ParseHexNumber(char* ptr, uint8_t* cnt);

/* List of possible WizFi360 baudrates */
static uint32_t WizFi360_Baudrate[] = {
	9600, 57600, 115200, 921600
};

/* Check IDLE */
#define WizFi360_CHECK_IDLE(WizFi360)                         \
do {                                                        \
	if (                                                    \
		(WizFi360)->ActiveCommand != WizFi360_COMMAND_IDLE    \
	) {                                                     \
		WizFi360_Update(WizFi360);                            \
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_BUSY);        \
	}                                                       \
} while (0);

/* Check if device is connected to WIFI */
#define WizFi360_CHECK_WIFICONNECTED(WizFi360)                \
do {                                                        \
	if (                                                    \
		!(WizFi360)->Flags.F.WifiConnected                   \
	) {                                                     \
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_WIFINOTCONNECTED); \
	}                                                       \
} while (0);

/* Return from function with desired status */
#define WizFi360_RETURNWITHSTATUS(WizFi360, status)           \
do {                                                        \
	(WizFi360)->Result = status;                             \
	return status;                                          \
} while (0); 

/* Reset ESP connection */
#define WizFi360_RESETCONNECTION(WizFi360, conn)              \
do {                                                        \
	(conn)->Active = 0;                                     \
	(conn)->Client = 0;                                     \
	(conn)->FirstPacket = 0;                                \
	(conn)->HeadersDone = 0;                                \
} while (0);                                                \

/* Reset all connections */
#define WizFi360_RESET_CONNECTIONS(WizFi360)                  memset(WizFi360->Connection, 0, sizeof(WizFi360->Connection));

/******************************************/
/*          Basic AT commands Set         */
/******************************************/
WizFi360_Result_t WizFi360_Init(WizFi360_t* WizFi360, uint32_t baudrate) {
	uint8_t i;
	
	/* Save settings */
	WizFi360->Timeout = 0;
	
	/* Init temporary buffer */
	if (BUFFER_Init(&TMP_Buffer, WizFi360_TMPBUFFER_SIZE, TMPBuffer)) {
		/* Return from function */
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_NOHEAP);
	}
	
	/* Init USART working */
	if (BUFFER_Init(&USART_Buffer, WizFi360_USARTBUFFER_SIZE, USARTBuffer)) {
		/* Return from function */
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_NOHEAP);
	}

	/* Init RESET pin */
	WizFi360_RESET_INIT;
	
	/* Set pin low */
	WizFi360_RESET_LOW;
	
	/* Delay for while */
	WizFi360_DELAYMS(100);
	
	/* Set pin high */
	WizFi360_RESET_HIGH;
	
	/* Delay for while */
	WizFi360_DELAYMS(100);
	
	/* Save current baudrate */
	WizFi360->Baudrate = baudrate;
	
	/* Init USART */
	WizFi360_LL_USARTInit(WizFi360->Baudrate);
	
	/* Set allowed timeout */
	WizFi360->Timeout = 1000;
	
	/* Reset device */
	SendCommand(WizFi360, WizFi360_COMMAND_RST, "AT+RST\r\n", "ready\r\n");
	
	/* Wait till idle */
	WizFi360_WaitReady(WizFi360);

	/* Check status */
	if (!WizFi360->Flags.F.LastOperationStatus) {
		/* Check for baudrate, try with predefined baudrates */
		for (i = 0; i < sizeof(WizFi360_Baudrate) / sizeof(WizFi360_Baudrate[0]); i++) {
			/* Init USART */
			WizFi360_LL_USARTInit(WizFi360->Baudrate);
			
			/* Set allowed timeout */
			WizFi360->Timeout = 1000;
			
			/* Reset device */
			SendCommand(WizFi360, WizFi360_COMMAND_RST, "AT+RST\r\n", "ready\r\n");
			
			/* Wait till idle */
			WizFi360_WaitReady(WizFi360);
		
			/* Check status */
			if (WizFi360->Flags.F.LastOperationStatus) {
				/* Save current baudrate */
				WizFi360->Baudrate = WizFi360_Baudrate[i];
				
				break;
			}
		}
	}
	
	/* Check status */
	if (!WizFi360->Flags.F.LastOperationStatus) {		
		/* Device is not connected */
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_DEVICENOTCONNECTED);
	}
	
	/* Set allowed timeout to 30sec */
	WizFi360->Timeout = 10;
	
	/* Test device */
	SendCommand(WizFi360, WizFi360_COMMAND_AT, "AT\r\n", "OK\r\n");
	
	/* Wait till idle */
	WizFi360_WaitReady(WizFi360);
	
	/* Check status */
	if (!WizFi360->Flags.F.LastOperationStatus) {		
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_DEVICENOTCONNECTED);
	}
	
	/* Enable echo if not already */
	SendCommand(WizFi360, WizFi360_COMMAND_ATE, "ATE1\r\n", "ATE1");
	
	/* Wait till idle */
	WizFi360_WaitReady(WizFi360);
	
	/* Enable multiple connections */
	while (WizFi360_SetMux(WizFi360, 1) != ESP_OK);
	
	/* Enable IP and PORT to be shown on +IPD statement */
	while (WizFi360_Setdinfo(WizFi360, 1) != ESP_OK);
	
	/* Get station MAC */
	while (WizFi360_GetSTAMAC(WizFi360) != ESP_OK);
	
	/* Get softAP MAC */
	while (WizFi360_GetAPMAC(WizFi360) != ESP_OK);
	
	/* Get softAP MAC */
	while (WizFi360_GetAPIP(WizFi360) != ESP_OK);
	
	/* Return OK */
	return WizFi360_WaitReady(WizFi360);
}

WizFi360_Result_t WizFi360_DeInit(WizFi360_t* WizFi360) {
	/* Clear temporary buffer */
	BUFFER_Free(&TMP_Buffer);
	
	/* Return OK from function */
	WizFi360_RETURNWITHSTATUS(WizFi360, ESP_OK);
}

WizFi360_Result_t WizFi360_RestoreDefault(WizFi360_t* WizFi360) {
	/* Send command */
	if (SendCommand(WizFi360, WizFi360_COMMAND_RESTORE, "AT+RESTORE\r\n", "ready\r\n") != ESP_OK) {
		return WizFi360->Result;
	}
	
	/* Little delay */
	WizFi360_DELAYMS(2);
	
	/* Reset USART to default ESP baudrate */
	WizFi360_LL_USARTInit(WizFi360_DEFAULT_BAUDRATE);
	
	/* Wait till ready, ESP will send data in default baudrate after reset */
	WizFi360_WaitReady(WizFi360);
	
	/* Reset USART buffer */
	BUFFER_Reset(&USART_Buffer);
	
	/* Return OK */
	WizFi360_RETURNWITHSTATUS(WizFi360, ESP_OK);
}

#if WizFi360_USE_FIRMWAREUPDATE
WizFi360_Result_t WizFi360_FirmwareUpdate(WizFi360_t* WizFi360) {
	/* Check connected */
	WizFi360_CHECK_WIFICONNECTED(WizFi360);
	
	/* Send command if possible */
	return SendCommand(WizFi360, WizFi360_COMMAND_CIUPDATE, "AT+CIUPDATE\r\n", "+CIPUPDATE:");
}
#endif

WizFi360_Result_t WizFi360_SetUART(WizFi360_t* WizFi360, uint32_t baudrate) {
	/* Set current baudrate */
	return SendUARTCommand(WizFi360, baudrate, "AT+UART_CUR");
}

WizFi360_Result_t WizFi360_SetUARTDefault(WizFi360_t* WizFi360, uint32_t baudrate) {
	/* Set default baudrate */
	return SendUARTCommand(WizFi360, baudrate, "AT+UART_DEF");
}

WizFi360_Result_t WizFi360_SetSleepMode(WizFi360_t* WizFi360, WizFi360_SleepMode_t SleepMode) {
	char tmp[20];
	
	/* Check idle mode */
	WizFi360_CHECK_IDLE(WizFi360);
	
	/* Format command */
	sprintf(tmp, "AT+SLEEP=%d\r\n", SleepMode);
	
	/* Send command */
	SendCommand(WizFi360, WizFi360_COMMAND_SLEEP, tmp, "+SLEEP");
	
	/* Wait ready */
	return WizFi360_WaitReady(WizFi360);
}

WizFi360_Result_t WizFi360_Sleep(WizFi360_t* WizFi360, uint32_t Milliseconds) {
	char tmp[20];
	
	/* Check idle mode */
	WizFi360_CHECK_IDLE(WizFi360);
	
	/* Format command */
	sprintf(tmp, "AT+GSLP=%u\r\n", Milliseconds);
	
	/* Send command */
	SendCommand(WizFi360, WizFi360_COMMAND_GSLP, tmp, NULL);
	
	/* Wait ready */
	return WizFi360_WaitReady(WizFi360);
}

WizFi360_Result_t WizFi360_Update(WizFi360_t* WizFi360) {
	char Received[256];
	char ch;
	uint8_t lastcmd;
	uint16_t stringlength;
	
	/* If timeout is set to 0 */
	if (WizFi360->Timeout == 0) {
		WizFi360->Timeout = 1000;
	}
	
	/* Check timeout */
	if ((WizFi360->Time - WizFi360->StartTime) > WizFi360->Timeout) {
		/* Save temporary active command */
		lastcmd = WizFi360->ActiveCommand;
		
		/* Timeout reached, reset command */
		WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
		
		/* Timeout reached */
		if (lastcmd == WizFi360_COMMAND_CIPSTART) {
			/* We get timeout on cipstart */
			WizFi360_RESETCONNECTION(WizFi360, &WizFi360->Connection[WizFi360->StartConnectionSent]);
			
			/* Call user function */
			WizFi360_Callback_ClientConnectionTimeout(WizFi360, &WizFi360->Connection[WizFi360->StartConnectionSent]);
		}
	}
	
	/* We are waiting to send data */
	if (WizFi360_COMMAND_SENDDATA) {
		/* Check what we are searching for */
		if (WizFi360->Flags.F.WaitForWrapper) {
			int16_t found;
			uint8_t dummy[2];
			/* Wait for character */
			if ((found = BUFFER_Find(&USART_Buffer, (uint8_t *)"> ", 2)) >= 0) {
				if (found == 0) {
					/* Make a dummy read */
					BUFFER_Read(&USART_Buffer, dummy, 2);
				}
			} else if ((found = BUFFER_Find(&TMP_Buffer, (uint8_t *)"> ", 0)) >= 0) {
				if (found == 0) {
					/* Make 2 dummy reads */
					BUFFER_Read(&TMP_Buffer, dummy, 2);
				}
			}
			if (found >= 0) {
				/* Send data */
				ProcessSendData(WizFi360);
			}
		}
	}
	
	/* If AT+UART command was used, only check if "OK" exists in buffer */
	if (
		WizFi360->ActiveCommand == WizFi360_COMMAND_UART && /*!< Active command is UART change */
		!WizFi360->IPD.InIPD                               /*!< We are not in IPD mode */
	) {
		/* Check for "OK\r" */
		if (BUFFER_Find(&USART_Buffer, (uint8_t *)"OK\r", 3) >= 0) {
			/* Clear buffer */
			BUFFER_Reset(&USART_Buffer);
			
			/* We are OK here */
			WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
			
			/* Last command is OK */
			WizFi360->Flags.F.LastOperationStatus = 1;
			
			/* Return from function */
			//return ESP_OK;
		}
	}
	
	/* Get string from USART buffer if we are not in IPD mode */
	while (
		!WizFi360->IPD.InIPD &&                                                             /*!< Not in IPD mode */
		//!WizFi360->Flags.F.WaitForWrapper &&
		(stringlength = BUFFER_ReadString(&USART_Buffer, Received, sizeof(Received))) > 0 /*!< Something in USART buffer */
	) {		
		/* Parse received string */
		PRINTF("Received : %s \r\n", Received);
		ParseReceived(WizFi360, Received, 1, stringlength);
	}
	
	/* Get string from TMP buffer when no command active */
	while (
		!WizFi360->IPD.InIPD &&                                                             /*!< Not in IPD mode */
		//!WizFi360->Flags.F.WaitForWrapper &&
		WizFi360->ActiveCommand == WizFi360_COMMAND_IDLE &&                                  /*!< We are in IDLE mode */
		(stringlength = BUFFER_ReadString(&TMP_Buffer, Received, sizeof(Received))) > 0 /*!< Something in TMP buffer */
	) {
		/* Parse received string */
		ParseReceived(WizFi360, Received, 0, stringlength);
	}
	
	/* If we are in IPD mode */
	if (WizFi360->IPD.InIPD) {
		BUFFER_t* buff;
		/* Check for USART buffer */
		if (WizFi360->IPD.USART_Buffer) {
			buff = &USART_Buffer;
		} else {
			buff = &TMP_Buffer;
		}
		
		/* If anything received */
		while (
			WizFi360->IPD.InPtr < WizFi360->Connection[WizFi360->IPD.ConnNumber].BytesReceived && /*!< Still not everything received */
			BUFFER_GetFull(buff) > 0                                                    /*!< Data are available in buffer */
		) {
			/* Read from buffer */
			BUFFER_Read(buff, (uint8_t *)&ch, 1);
			
			/* Add from USART buffer */
			WizFi360->Connection[WizFi360->IPD.ConnNumber].Data[WizFi360->IPD.InPtr] = ch;
			
			/* Increase pointers */
			WizFi360->IPD.InPtr++;
			WizFi360->IPD.PtrTotal++;
			
#if WizFi360_CONNECTION_BUFFER_SIZE < ESP8255_MAX_BUFF_SIZE
			/* Check for pointer */
			if (WizFi360->IPD.InPtr >= WizFi360_CONNECTION_BUFFER_SIZE && WizFi360->IPD.InPtr != WizFi360->Connection[WizFi360->IPD.ConnNumber].BytesReceived) {
				/* Set connection buffer size */
				WizFi360->Connection[WizFi360->IPD.ConnNumber].DataSize = WizFi360->IPD.InPtr;
				WizFi360->Connection[WizFi360->IPD.ConnNumber].LastPart = 0;
				PRINTF(" receive data : %s\r\n", WizFi360->Connection[WizFi360->IPD.ConnNumber].Data);
				/* Buffer is full, call user function */
//				if (WizFi360->Connection[WizFi360->IPD.ConnNumber].Client) {
//					WizFi360_Callback_ClientConnectionDataReceived(WizFi360, &WizFi360->Connection[WizFi360->IPD.ConnNumber], WizFi360->Connection[WizFi360->IPD.ConnNumber].Data);
//				} else {
//					WizFi360_Callback_ServerConnectionDataReceived(WizFi360, &WizFi360->Connection[WizFi360->IPD.ConnNumber], WizFi360->Connection[WizFi360->IPD.ConnNumber].Data);
//				}
				
				
				/* Reset input pointer */
				WizFi360->IPD.InPtr = 0;
			}
#endif
		}
		
		/* Check if everything received */
		if (WizFi360->IPD.PtrTotal >= WizFi360->Connection[WizFi360->IPD.ConnNumber].BytesReceived) {
			char* ptr;
			
			/* Not in IPD anymore */
			WizFi360->IPD.InIPD = 0;
			
			/* Set package data size */
			WizFi360->Connection[WizFi360->IPD.ConnNumber].DataSize = WizFi360->IPD.InPtr;
			WizFi360->Connection[WizFi360->IPD.ConnNumber].LastPart = 1;
			
			/* We have data, lets see if Content-Length exists and save it */
			if (
				WizFi360->Connection[WizFi360->IPD.ConnNumber].FirstPacket &&
				(ptr = strstr(WizFi360->Connection[WizFi360->IPD.ConnNumber].Data, "Content-Length: ")) != NULL
			) {
				/* Increase pointer and parse number */
				ptr += 16;
				
				/* Parse content length */
				WizFi360->Connection[WizFi360->IPD.ConnNumber].ContentLength = ParseNumber(ptr, NULL);
			}
			
			/* Set flag to trigger callback for data received */
			WizFi360->Connection[WizFi360->IPD.ConnNumber].CallDataReceived = 1;
		}
	}
	
	/* Call user functions on connections if needed */
	CallConnectionCallbacks(WizFi360);
	
	/* Return OK */
	WizFi360_RETURNWITHSTATUS(WizFi360, ESP_OK);
}

void WizFi360_TimeUpdate(WizFi360_t* WizFi360, uint32_t time_increase) {
	/* Increase time */
	WizFi360->Time += time_increase;
}

WizFi360_Result_t WizFi360_WaitReady(WizFi360_t* WizFi360) {
	/* Do job */
	do {
		/* Check for wrapper */
		if (WizFi360->Flags.F.WaitForWrapper) {
			/* We have found it, stop execution here */
			if (BUFFER_Find(&USART_Buffer, (uint8_t *)"> ", 2) >= 0) {
				WizFi360->Flags.F.WaitForWrapper = 0;
				break;
			}
		}
		/* Update device */
		WizFi360_Update(WizFi360);
	} while (WizFi360->ActiveCommand != WizFi360_COMMAND_IDLE);
	
	/* Return OK */
	WizFi360_RETURNWITHSTATUS(WizFi360, ESP_OK);
}

WizFi360_Result_t WizFi360_IsReady(WizFi360_t* WizFi360) {
	/* Check IDLE */
	WizFi360_CHECK_IDLE(WizFi360);
	
	/* Return OK */
	WizFi360_RETURNWITHSTATUS(WizFi360, ESP_OK);
}

WizFi360_Result_t WizFi360_SetMode(WizFi360_t* WizFi360, WizFi360_Mode_t Mode) {
	char command[30];
	
	/* Format command */
	sprintf(command, "AT+CWMODE_CUR=%d\r\n", (uint8_t)Mode);
	
	/* Send command */
	if (SendCommand(WizFi360, WizFi360_COMMAND_CWMODE, command, "AT+CWMODE") != ESP_OK) {
		return WizFi360->Result;
	}
	
	/* Save mode we sent */
	WizFi360->SentMode = Mode;

	/* Wait till command end */
	WizFi360_WaitReady(WizFi360);
	
	/* Check status */
	if (WizFi360->Mode != Mode) {
		/* Return error */
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_ERROR);
	}
	
	/* Now get settings for current AP mode */
	return WizFi360_GetAP(WizFi360);
}

WizFi360_Result_t WizFi360_RequestSendData(WizFi360_t* WizFi360, WizFi360_Connection_t* Connection) {
	char command[30];
	
	/* Check idle state */
	WizFi360_CHECK_IDLE(WizFi360);
	
	/* Format command */
	sprintf(command, "AT+CIPSEND=%d,2048\r\n", Connection->Number);

	/* Send command */
	if (SendCommand(WizFi360, WizFi360_COMMAND_SEND, command, "AT+CIPSEND") != ESP_OK) {
		return WizFi360->Result;
	}
	
	/* We are waiting for "> " response */
	Connection->WaitForWrapper = 1;
	WizFi360->Flags.F.WaitForWrapper = 1;
	
	/* Save connection pointer */
	WizFi360->SendDataConnection = Connection;
	
	/* Return from function */
	return WizFi360->Result;
}


WizFi360_Result_t WizFi360_CloseConnection(WizFi360_t* WizFi360, WizFi360_Connection_t* Connection) {
	char tmp[30];
	
	/* Format connection */
	sprintf(tmp, "AT+CIPCLOSE=%d\r\n", Connection->Number);
	
	/* Send command */
	return SendCommand(WizFi360, WizFi360_COMMAND_CLOSE, tmp, "AT+CIPCLOSE");
}

WizFi360_Result_t WizFi360_CloseAllConnections(WizFi360_t* WizFi360) {
	/* Send command */
	return SendCommand(WizFi360, WizFi360_COMMAND_CLOSE, "AT+CIPCLOSE=5\r\n", "AT+CIPCLOSE");
}

WizFi360_Result_t WizFi360_AllConectionsClosed(WizFi360_t* WizFi360) {
	uint8_t i;
	
	/* Go through all connections */
	for (i = 0; i < WizFi360_MAX_CONNECTIONS; i++) {
		/* Check if active */
		if (WizFi360->Connection[i].Active) {
			WizFi360_RETURNWITHSTATUS(WizFi360, ESP_ERROR);
		}
	}
	
	/* Return OK */
	WizFi360_RETURNWITHSTATUS(WizFi360, ESP_OK);
}

WizFi360_Result_t WizFi360_SetMux(WizFi360_t* WizFi360, uint8_t mux) {
	char tmp[30];
	
	/* Format command */
	sprintf(tmp, "AT+CIPMUX=%d\r\n", mux);
	
	/* Send command */
	if (SendCommand(WizFi360, WizFi360_COMMAND_CIPMUX, tmp, "AT+CIPMUX") != ESP_OK) {
		return WizFi360->Result;
	}
	
	/* Wait till command end */
	WizFi360_WaitReady(WizFi360);
	
	/* Check last status */
	if (!WizFi360->Flags.F.LastOperationStatus) {
		/* Return error */
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_ERROR);
	}
	
	/* Return OK */
	WizFi360_RETURNWITHSTATUS(WizFi360, ESP_OK);
}

WizFi360_Result_t WizFi360_Setdinfo(WizFi360_t* WizFi360, uint8_t info) {
	char tmp[30];
	
	/* Format string */
	sprintf(tmp, "AT+CIPDINFO=%d\r\n", info);

	/* Send command and wait */
	if (SendCommand(WizFi360, WizFi360_COMMAND_CIPDINFO, tmp, "AT+CIPDINFO") != ESP_OK) {
		return WizFi360->Result;
	}

	/* Wait till command end */
	WizFi360_WaitReady(WizFi360);
	
	/* Check last status */
	if (!WizFi360->Flags.F.LastOperationStatus) {
		/* Return error */
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_ERROR);
	}
	
	/* Return OK */
	WizFi360_RETURNWITHSTATUS(WizFi360, ESP_OK);
}

WizFi360_Result_t WizFi360_ServerEnable(WizFi360_t* WizFi360, uint16_t port) {
	char tmp[30];
	
	/* Format string */
	sprintf(tmp, "AT+CIPSERVER=1,%d\r\n", port);

	/* Send command and wait */
	if (SendCommand(WizFi360, WizFi360_COMMAND_CIPSERVER, tmp, "AT+CIPSERVER") != ESP_OK) {
		return WizFi360->Result;
	}

	/* Wait till command end */
	WizFi360_WaitReady(WizFi360);

	/* Check last status */
	if (!WizFi360->Flags.F.LastOperationStatus) {
		/* Return error */
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_ERROR);
	}
	
	/* Return OK */
	WizFi360_RETURNWITHSTATUS(WizFi360, ESP_OK);
}

WizFi360_Result_t WizFi360_ServerDisable(WizFi360_t* WizFi360) {
	/* Send command and wait */
	if (SendCommand(WizFi360, WizFi360_COMMAND_CIPSERVER, "AT+CIPSERVER=0\r\n", "AT+CIPSERVER") != ESP_OK) {
		return WizFi360->Result;
	}

	/* Wait till command end */
	WizFi360_WaitReady(WizFi360);

	/* Check last status */
	if (!WizFi360->Flags.F.LastOperationStatus) {
		/* Return error */
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_ERROR);
	}
	
	/* Return OK */
	WizFi360_RETURNWITHSTATUS(WizFi360, ESP_OK);
}

WizFi360_Result_t WizFi360_SetServerTimeout(WizFi360_t* WizFi360, uint16_t timeout) {
	char tmp[20];
	
	/* Format string */
	sprintf(tmp, "AT+CIPSTO=%d\r\n", timeout);

	/* Send command and wait */
	if (SendCommand(WizFi360, WizFi360_COMMAND_CIPSTO, tmp, NULL) != ESP_OK) {
		return WizFi360->Result;
	}

	/* Wait till command end */
	WizFi360_WaitReady(WizFi360);

	/* Check last status */
	if (!WizFi360->Flags.F.LastOperationStatus) {
		/* Return error */
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_ERROR);
	}
	
	/* Return OK */
	WizFi360_RETURNWITHSTATUS(WizFi360, ESP_OK);
}

WizFi360_Result_t WizFi360_WifiDisconnect(WizFi360_t* WizFi360) {
	/* Send command */
	return SendCommand(WizFi360, WizFi360_COMMAND_CWQAP, "AT+CWQAP\r\n", "AT+CWQAP");
}

WizFi360_Result_t WizFi360_WifiConnect(WizFi360_t* WizFi360, char* ssid, char* pass) {
	char tmp[100];
	char s[50], p[50];
	
	/* Escape special characters for WizFi360 */
	strcpy(s, EscapeString(ssid));
	strcpy(p, EscapeString(pass));
	
	/* Format command */
	sprintf(tmp, "AT+CWJAP_CUR=\"%s\",\"%s\"\r\n", s, p);
	
	/* Send command */
	return SendCommand(WizFi360, WizFi360_COMMAND_CWJAP, tmp, "+CWJAP:");
}

WizFi360_Result_t WizFi360_WifiConnectDefault(WizFi360_t* WizFi360, char* ssid, char* pass) {
	char tmp[100];
	char s[50], p[50];
	
	/* Escape special characters for WizFi360 */
	strcpy(s, EscapeString(ssid));
	strcpy(p, EscapeString(pass));
	
	/* Format command */
	sprintf(tmp, "AT+CWJAP_DEF=\"%s\",\"%s\"\r\n", s, p);
	
	/* Send command */
	return SendCommand(WizFi360, WizFi360_COMMAND_CWJAP, tmp, "+CWJAP:");
}

WizFi360_Result_t WizFi360_WifiGetConnected(WizFi360_t* WizFi360) {	
	/* Send command */
	return SendCommand(WizFi360, WizFi360_COMMAND_CWJAP_GET, "AT+CWJAP_CUR?\r\n", "+CWJAP_CUR:");
}

WizFi360_Result_t WizFi360_GetSTAIPBlocking(WizFi360_t* WizFi360) {
	/* Send command */
	if (WizFi360_GetSTAIP(WizFi360) != ESP_OK) {
	
	}

	/* Wait till command end */
	WizFi360_WaitReady(WizFi360);
	
	/* Return OK */
	WizFi360_RETURNWITHSTATUS(WizFi360, ESP_OK);
}

WizFi360_Result_t WizFi360_GetAPIPBlocking(WizFi360_t* WizFi360) {
	/* Send command */
	if (WizFi360_GetAPIP(WizFi360) != ESP_OK) {
	
	}

	/* Wait till command end */
	return WizFi360_WaitReady(WizFi360);
}

WizFi360_Result_t WizFi360_GetSTAIP(WizFi360_t* WizFi360) {	
	/* Send command */
	SendCommand(WizFi360, WizFi360_COMMAND_CIPSTA, "AT+CIPSTA_CUR?\r\n", "+CIPSTA_CUR");
	
	/* Check status */
	if (WizFi360->Result == ESP_OK) {
		/* Reset flags */
		WizFi360->Flags.F.STAIPIsSet = 0;
		WizFi360->Flags.F.STANetmaskIsSet = 0;
		WizFi360->Flags.F.STAGatewayIsSet = 0;
	}
	
	/* Return status */
	return WizFi360->Result;
}

WizFi360_Result_t WizFi360_GetAPIP(WizFi360_t* WizFi360) {	
	/* Send command */
	SendCommand(WizFi360, WizFi360_COMMAND_CIPAP, "AT+CIPAP_CUR?\r\n", "+CIPAP_CUR");
	
	/* Check status */
	if (WizFi360->Result == ESP_OK) {
		/* Reset flags */
		WizFi360->Flags.F.APIPIsSet = 0;
		WizFi360->Flags.F.APNetmaskIsSet = 0;
		WizFi360->Flags.F.APGatewayIsSet = 0;
	}
	
	/* Return status */
	return WizFi360->Result;
}

/******************************************/
/*            MAC MANUPULATION            */
/******************************************/
WizFi360_Result_t WizFi360_GetSTAMAC(WizFi360_t* WizFi360) {	
	/* Send command */
	SendCommand(WizFi360, WizFi360_COMMAND_CIPSTAMAC, "AT+CIPSTAMAC?\r\n", "+CIPSTAMAC");
	
	/* Check status */
	if (WizFi360->Result == ESP_OK) {
		/* Reset flags */
		WizFi360->Flags.F.STAMACIsSet = 0;
	}
	
	/* Return stats */
	return WizFi360->Result;
}

WizFi360_Result_t WizFi360_SetSTAMAC(WizFi360_t* WizFi360, uint8_t* addr) {
	/* Send current MAC command */
	return SendMACCommand(WizFi360, addr, "AT+CIPSTAMAC_CUR", WizFi360_COMMAND_CIPSTAMAC);
}

WizFi360_Result_t WizFi360_SetSTAMACDefault(WizFi360_t* WizFi360, uint8_t* addr) {
	/* Send current MAC command */
	return SendMACCommand(WizFi360, addr, "AT+CIPSTAMAC_DEF", WizFi360_COMMAND_CIPSTAMAC);
}

WizFi360_Result_t WizFi360_GetAPMAC(WizFi360_t* WizFi360) {	
	/* Send command */
	SendCommand(WizFi360, WizFi360_COMMAND_CIPAPMAC, "AT+CIPAPMAC?\r\n", "+CIPAPMAC");
	
	/* Check status */
	if (WizFi360->Result == ESP_OK) {
		/* Reset flags */
		WizFi360->Flags.F.APMACIsSet = 0;
	}
	
	/* Return stats */
	return WizFi360->Result;
}

WizFi360_Result_t WizFi360_SetAPMAC(WizFi360_t* WizFi360, uint8_t* addr) {
	/* Send current MAC command */
	return SendMACCommand(WizFi360, addr, "AT+CIPAPMAC_CUR", WizFi360_COMMAND_CIPAPMAC);
}

WizFi360_Result_t WizFi360_SetAPMACDefault(WizFi360_t* WizFi360, uint8_t* addr) {
	/* Send current MAC command */
	return SendMACCommand(WizFi360, addr, "AT+CIPAPMAC_DEF", WizFi360_COMMAND_CIPAPMAC);
}

/******************************************/
/*                AP + STA                */
/******************************************/
#if WizFi360_USE_APSEARCH
WizFi360_Result_t WizFi360_ListWifiStations(WizFi360_t* WizFi360) {
	/* Reset pointer */
	WizFi360_APs.Count = 0;
	
	/* Send list command */
	return SendCommand(WizFi360, WizFi360_COMMAND_CWLAP, "AT+CWLAP\r\n", "+CWLAP");	
}
#endif

WizFi360_Result_t WizFi360_GetAP(WizFi360_t* WizFi360) {
	/* Send command to read current AP settings */
	if (SendCommand(WizFi360, WizFi360_COMMAND_CWSAP, "AT+CWSAP?\r\n", "+CWSAP") != ESP_OK) {
		return WizFi360->Result;
	}
	
	/* Wait till command end */
	return WizFi360_WaitReady(WizFi360);
}

WizFi360_Result_t WizFi360_SetAP(WizFi360_t* WizFi360, WizFi360_APConfig_t* WizFi360_Config) {
	char tmp[120];
	char pass[65], ssid[65];
	
	/* Check input values */
	if (
		strlen(WizFi360_Config->SSID) > 64 ||
		strlen(WizFi360_Config->Pass) > 64 ||
		(WizFi360_Config->Ecn != WizFi360_Ecn_OPEN && strlen(WizFi360_Config->Pass) < 8) ||
		WizFi360_Config->Ecn == WizFi360_Ecn_WEP ||
		WizFi360_Config->MaxConnections < 1 || WizFi360_Config->MaxConnections > 4
	) {
		/* Error */
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_ERROR);
	}
	
	/* Escape characters */
	strcpy(ssid, EscapeString(WizFi360_Config->SSID));
	strcpy(pass, EscapeString(WizFi360_Config->Pass));
	
	/* Format command */
	sprintf(tmp, "AT+CWSAP_CUR=\"%s\",\"%s\",%d,%d,%d,%d\r\n",
		ssid, pass,
		WizFi360_Config->Channel,
		(uint8_t)WizFi360_Config->Ecn,
		WizFi360_Config->MaxConnections,
		WizFi360_Config->Hidden
	);
	
	/* Send command */
	SendCommand(WizFi360, WizFi360_COMMAND_CWSAP, tmp, "AT+CWSAP");
	
	/* Return status */
	return WizFi360_Update(WizFi360);
}

WizFi360_Result_t WizFi360_SetAPDefault(WizFi360_t* WizFi360, WizFi360_APConfig_t* WizFi360_Config) {
	char tmp[120];
	char pass[65], ssid[65];
	
	/* Check input values */
	if (
		strlen(WizFi360_Config->SSID) > 64 ||
		strlen(WizFi360_Config->Pass) > 64 ||
		(WizFi360_Config->Ecn != WizFi360_Ecn_OPEN && strlen(WizFi360_Config->Pass) < 8) ||
		WizFi360_Config->Ecn == WizFi360_Ecn_WEP ||
		WizFi360_Config->MaxConnections < 1 || WizFi360_Config->MaxConnections > 4
	) {
		/* Error */
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_ERROR);
	}
	
	/* Escape characters */
	strcpy(ssid, EscapeString(WizFi360_Config->SSID));
	strcpy(pass, EscapeString(WizFi360_Config->Pass));
	
	/* Format command */
	sprintf(tmp, "AT+CWSAP_DEF=\"%s\",\"%s\",%d,%d,%d,%d\r\n",
		ssid, pass,
		WizFi360_Config->Channel,
		(uint8_t)WizFi360_Config->Ecn,
		WizFi360_Config->MaxConnections,
		WizFi360_Config->Hidden
	);
	
	/* Send command */
	SendCommand(WizFi360, WizFi360_COMMAND_CWSAP, tmp, "AT+CWSAP");
	
	/* Wait till command end */
	return WizFi360_WaitReady(WizFi360);
}

WizFi360_Result_t WizFi360_GetConnectedStations(WizFi360_t* WizFi360) {
	char resp[4];
	
	/* Format response, use first byte of IP as first string */
	sprintf(resp, "%d", WizFi360->APIP[0]);
	
	/* Try to send command */
	if (SendCommand(WizFi360, WizFi360_COMMAND_CWLIF, "AT+CWLIF\r\n", resp) != ESP_OK) {
		return WizFi360->Result;
	}
	
	/* Reset counters in structure */
	WizFi360->ConnectedStations.Count = 0;
	
	/* Return result */
	return WizFi360->Result;
}

/******************************************/
/*               TCP CLIENT               */
/******************************************/
WizFi360_Result_t WizFi360_StartClientConnection(WizFi360_t* WizFi360, char* name, char* location, uint16_t port, void* user_parameters) {
	int8_t conn = -1;
	uint8_t i = 0;
	
	/* Check if IDLE */
	WizFi360_CHECK_IDLE(WizFi360);
	
	/* Check if connected to network */
	WizFi360_CHECK_WIFICONNECTED(WizFi360);
	
	/* Find available connection */
	for (i = 0; i < WizFi360_MAX_CONNECTIONS; i++) {
		if (!WizFi360->Connection[i].Active) {
			/* Save */
			conn = i;
			
			break;
		}
	}
	
	/* Try it */
	if (conn != -1) {
		char tmp[100];
		/* Format command */
		sprintf(tmp, "AT+CIPSTART=%d,\"TCP\",\"%s\",%d\r\n", conn, location, port);
		
		/* Send command */
		if (SendCommand(WizFi360, WizFi360_COMMAND_CIPSTART, tmp, NULL) != ESP_OK) {
			return WizFi360->Result;
		}
		
		/* We are active now as client */
		WizFi360->Connection[i].Active = 1;
		WizFi360->Connection[i].Client = 1;
		WizFi360->Connection[i].TotalBytesReceived = 0;
		WizFi360->Connection[i].Number = conn;
#if WizFi360_USE_SINGLE_CONNECTION_BUFFER == 1
		WizFi360->Connection[i].Data = ConnectionData;
#endif
		WizFi360->StartConnectionSent = i;
		
		/* Copy values */
		strncpy(WizFi360->Connection[i].Name, name, sizeof(WizFi360->Connection[i].Name));
		WizFi360->Connection[i].UserParameters = user_parameters;
		
		/* Return OK */
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_OK);
	}
	
	/* Return error */
	WizFi360_RETURNWITHSTATUS(WizFi360, ESP_ERROR);
}


/******************************************/
/*               UDP CLIENT               */
/******************************************/
WizFi360_Result_t WizFi360_StartUDPConnection(WizFi360_t* WizFi360, char* name, char* location, uint16_t port, void* user_parameters) {
	int8_t conn = -1;
	uint8_t i = 0;
	
	/* Check if IDLE */
	WizFi360_CHECK_IDLE(WizFi360);
	
	/* Check if connected to network */
	WizFi360_CHECK_WIFICONNECTED(WizFi360);
	
	/* Find available connection */
	for (i = 0; i < WizFi360_MAX_CONNECTIONS; i++) {
		if (!WizFi360->Connection[i].Active) {
			/* Save */
			conn = i;
			
			break;
		}
	}
	
	/* Try it */
	if (conn != -1) {
		char tmp[100];
		/* Format command */
		sprintf(tmp, "AT+CIPSTART=%d,\"UDP\",\"%s\",%d\r\n", conn, location, port);
		
		/* Send command */
		if (SendCommand(WizFi360, WizFi360_COMMAND_CIPSTART, tmp, NULL) != ESP_OK) {
			return WizFi360->Result;
		}
		
		/* We are active now as client */
		WizFi360->Connection[i].Active = 1;
		WizFi360->Connection[i].Client = 1;
		WizFi360->Connection[i].TotalBytesReceived = 0;
		WizFi360->Connection[i].Number = conn;
#if WizFi360_USE_SINGLE_CONNECTION_BUFFER == 1
		WizFi360->Connection[i].Data = ConnectionData;
#endif
		WizFi360->StartConnectionSent = i;
		
		/* Copy values */
		strncpy(WizFi360->Connection[i].Name, name, sizeof(WizFi360->Connection[i].Name));
		WizFi360->Connection[i].UserParameters = user_parameters;
		
		/* Return OK */
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_OK);
	}
	
	/* Return error */
	WizFi360_RETURNWITHSTATUS(WizFi360, ESP_ERROR);
}


/******************************************/
/*              PING SUPPORT              */
/******************************************/
#if WizFi360_USE_PING
WizFi360_Result_t WizFi360_Ping(WizFi360_t* WizFi360, char* addr) {
	char tmp[100];
	
	/* Check idle */
	WizFi360_CHECK_IDLE(WizFi360);
	
	/* Device must be connected to wifi */
	WizFi360_CHECK_WIFICONNECTED(WizFi360);
	
	/* Save ping address information */
	strcpy(WizFi360->PING.Address, addr);
	
	/* Format command for pinging */
	sprintf(tmp, "AT+PING=\"%s\"\r\n", addr);
	
	/* Reset flag */
	WizFi360->PING.Success = 0;
	
	/* Send command */
	if (SendCommand(WizFi360, WizFi360_COMMAND_PING, tmp, "+") == ESP_OK) {
		/* Call user function */
		WizFi360_Callback_PingStarted(WizFi360, addr);
	}
	
	/* Return status */
	return WizFi360->Result;
}
#endif

uint16_t WizFi360_DataReceived(uint8_t* ch, uint16_t count) {
	/* Writes data to USART buffer */
	return BUFFER_Write(&USART_Buffer, ch, count);
}

/******************************************/
/*                CALLBACKS               */
/******************************************/
/* Called when ready string detected */
__weak void WizFi360_Callback_DeviceReady(WizFi360_t* WizFi360) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_DeviceReady could be implemented in the user file
	*/
}

/* Called when watchdog reset on WizFi360 is detected */
__weak void WizFi360_Callback_WatchdogReset(WizFi360_t* WizFi360) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_WatchdogReset could be implemented in the user file
	*/
}

/* Called when module disconnects from wifi network */
__weak void WizFi360_Callback_WifiDisconnected(WizFi360_t* WizFi360) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_WifiDisconnected could be implemented in the user file
	*/
}

/* Called when module connects to wifi network */
__weak void WizFi360_Callback_WifiConnected(WizFi360_t* WizFi360) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_WifiConnected could be implemented in the user file
	*/
}

/* Called when connection to wifi network fails */
__weak void WizFi360_Callback_WifiConnectFailed(WizFi360_t* WizFi360) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_WifiConnectFailed could be implemented in the user file
	*/
}

/* Called when module gets IP address */
__weak void WizFi360_Callback_WifiGotIP(WizFi360_t* WizFi360) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_WifiGotIP could be implemented in the user file
	*/
}

/* Called when IP is set */
__weak void WizFi360_Callback_WifiIPSet(WizFi360_t* WizFi360) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_WifiIPSet could be implemented in the user file
	*/
}

/* Called when wifi spot is detected */
__weak void WizFi360_Callback_WifiDetected(WizFi360_t* WizFi360, WizFi360_APs_t* WizFi360_AP) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_WifiDetected could be implemented in the user file
	*/
}

/* Called on DHCP timeout */
__weak void WizFi360_Callback_DHCPTimeout(WizFi360_t* WizFi360) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_DHCPTimeout could be implemented in the user file
	*/
}

/* Called when "x,CONNECT" is detected */
__weak void WizFi360_Callback_ServerConnectionActive(WizFi360_t* WizFi360, WizFi360_Connection_t* Connection) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_ServerConnectionActive could be implemented in the user file
	*/
}

/* Called when "x,CLOSED" is detected */
__weak void WizFi360_Callback_ServerConnectionClosed(WizFi360_t* WizFi360, WizFi360_Connection_t* Connection) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_ServerConnectionClosed could be implemented in the user file
	*/
}

/* Called when "+IPD..." is detected */
__weak void WizFi360_Callback_ServerConnectionDataReceived(WizFi360_t* WizFi360, WizFi360_Connection_t* Connection, char* Buffer) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_ServerConnectionDataReceived could be implemented in the user file
	*/
}

/* Called when user should fill data buffer to be sent with connection */
__weak uint16_t WizFi360_Callback_ServerConnectionSendData(WizFi360_t* WizFi360, WizFi360_Connection_t* Connection, char* Buffer, uint16_t max_buffer_size) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_ServerConnectionSendData could be implemented in the user file
	*/
	
	/* Return number of bytes in array */
	return 0;
}

/* Called when data are send successfully */
__weak void WizFi360_Callback_ServerConnectionDataSent(WizFi360_t* WizFi360, WizFi360_Connection_t* Connection) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_ServerConnectionDataSent could be implemented in the user file
	*/
}

/* Called when error returned trying to sent data */
__weak void WizFi360_Callback_ServerConnectionDataSentError(WizFi360_t* WizFi360, WizFi360_Connection_t* Connection) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_ServerConnectionDataSentError could be implemented in the user file
	*/
}

/* Called when user is connected to server as client */
__weak void WizFi360_Callback_ClientConnectionConnected(WizFi360_t* WizFi360, WizFi360_Connection_t* Connection) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_ClientConnectionConnected could be implemented in the user file
	*/
}

/* Called when user should fill data buffer to be sent with connection */
__weak uint16_t WizFi360_Callback_ClientConnectionSendData(WizFi360_t* WizFi360, WizFi360_Connection_t* Connection, char* Buffer, uint16_t max_buffer_size) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_ClientConnectionSendData could be implemented in the user file
	*/
	
	/* Return number of bytes in array */
	return 0;
}

/* Called when data are send successfully */
__weak void WizFi360_Callback_ClientConnectionDataSent(WizFi360_t* WizFi360, WizFi360_Connection_t* Connection) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_ClientConnectionDataSent could be implemented in the user file
	*/
}

/* Called when error returned trying to sent data */
__weak void WizFi360_Callback_ClientConnectionDataSentError(WizFi360_t* WizFi360, WizFi360_Connection_t* Connection) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_ClientConnectionDataSentError could be implemented in the user file
	*/
}

/* Called when server returns data back to client */
__weak void WizFi360_Callback_ClientConnectionDataReceived(WizFi360_t* WizFi360, WizFi360_Connection_t* Connection, char* Buffer) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_ClientConnectionDataReceived could be implemented in the user file
	*/
}

/* Called when ERROR is returned on AT+CIPSTART command */
__weak void WizFi360_Callback_ClientConnectionError(WizFi360_t* WizFi360, WizFi360_Connection_t* Connection) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_ClientConnectionError could be implemented in the user file
	*/
}

/* Called when timeout is reached on AT+CIPSTART command */
__weak void WizFi360_Callback_ClientConnectionTimeout(WizFi360_t* WizFi360, WizFi360_Connection_t* Connection) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_ClientConnectionTimeout could be implemented in the user file
	*/
}

/* Called when "x,CLOSED" is detected when connection was made as client */
__weak void WizFi360_Callback_ClientConnectionClosed(WizFi360_t* WizFi360, WizFi360_Connection_t* Connection) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_ClientConnectionClosed could be implemented in the user file
	*/
}

#if WizFi360_USE_PING
/* Called when pinging started */
__weak void WizFi360_Callback_PingStarted(WizFi360_t* WizFi360, char* address) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_PingStarted could be implemented in the user file
	*/
}

/* Called when PING command ends */
__weak void WizFi360_Callback_PingFinished(WizFi360_t* WizFi360, WizFi360_Ping_t* PING) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_PingFinished could be implemented in the user file
	*/
}
#endif

#if WizFi360_USE_FIRMWAREUPDATE
/* Called on status messages for network firmware update */
__weak void WizFi360_Callback_FirmwareUpdateStatus(WizFi360_t* WizFi360, WizFi360_FirmwareUpdate_t status) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_FirmwareUpdateStatus could be implemented in the user file
	*/
}

/* Called when firmware network update was successful */
__weak void WizFi360_Callback_FirmwareUpdateSuccess(WizFi360_t* WizFi360) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_FirmwareUpdateSuccess could be implemented in the user file
	*/
}

/* Called when firmware network error occurred */
__weak void WizFi360_Callback_FirmwareUpdateError(WizFi360_t* WizFi360) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_FirmwareUpdateError could be implemented in the user file
	*/
}
#endif

/* Called when AT+CWLIF returns OK */
__weak void WizFi360_Callback_ConnectedStationsDetected(WizFi360_t* WizFi360, WizFi360_ConnectedStations_t* Stations) {
	/* NOTE: This function Should not be modified, when the callback is needed,
           the WizFi360_Callback_ConnectedStationsDetected could be implemented in the user file
	*/
}

/******************************************/
/*           PRIVATE FUNCTIONS            */
/******************************************/
static void ParseCWSAP(WizFi360_t* WizFi360, char* Buffer) {
	char* ptr;
	uint8_t i, cnt;
	
	/* Find : in string */
	ptr = Buffer;
	while (*ptr) {
		if (*ptr == ':') {
			break;
		}
		ptr++;
	}
	
	/* Check if exists */
	if (ptr == 0) {
		return;
	}
	
	/* Go to '"' character */
	ptr++;
	
	/**** NEEDS IMPROVEMENT ****/
	/* If '"' character is inside SSID or password part, parser will fail */
	
	/***** SSID ****/
	WizFi360->AP.SSID[0] = 0;
	if (*ptr == '"') {
		ptr++;
	}
	
	/* Copy till "," which indicates end of SSID string and start of password part */
	i = 0;
	while (*ptr && (*ptr != '"' || *(ptr + 1) != ',' || *(ptr + 2) != '"')) {
		WizFi360->AP.SSID[i++] = *ptr++;
	}
	WizFi360->AP.SSID[i++] = 0;
	
	/* Increase pointer by 3, ignore "," part */
	ptr += 3;
	
	/* Copy till ", which indicates end of password string and start of number */
	i = 0;
	while (*ptr && (*ptr != '"' || *(ptr + 1) != ',')) {
		WizFi360->AP.Pass[i++] = *ptr++;
	}
	WizFi360->AP.Pass[i++] = 0;
	
	/* Increase pointer by 2 */
	ptr += 2;
	
	/* Get channel number */
	WizFi360->AP.Channel = ParseNumber(ptr, &cnt);
	
	/* Increase pointer and comma */
	ptr += cnt + 1;
	
	/* Get ecn value */
	WizFi360->AP.Ecn = (WizFi360_Ecn_t)ParseNumber(ptr, &cnt);
	
	/* Increase pointer and comma */
	ptr += cnt + 1;
	
	/* Get max connections value */
	WizFi360->AP.MaxConnections = ParseNumber(ptr, &cnt);
	
	/* Increase pointer and comma */
	ptr += cnt + 1;
	
	/* Get hidden value */
	WizFi360->AP.Hidden = ParseNumber(ptr, &cnt);
}

#if WizFi360_USE_APSEARCH
static void ParseCWLAP(WizFi360_t* WizFi360, char* Buffer) {
	uint8_t pos = 7, num = 0;
	char* ptr;
	
	/* Check if we have memory available first */
	if (WizFi360_APs.Count >= WizFi360_MAX_CONNECTIONS) {
		return;
	}
	
	/* Get start pointer */
	if (Buffer[pos] == '(') {
		pos++;
	}
	
	/* Get token */
	ptr = strtok(&Buffer[pos], ",");
	
	/* Do it until token != NULL */
	while (ptr != NULL) {
		/* Send debug */
		
		/* Get positions */
		switch (num++) {
			case 0: 
				WizFi360_APs.AP[WizFi360_APs.Count].Ecn = ParseNumber(ptr, NULL);
				break;
			case 1:
				/* Ignore first and last " */
				ptr++;
				ptr[strlen(ptr) - 1] = 0;
				strcpy(WizFi360_APs.AP[WizFi360_APs.Count].SSID, ptr);
				break;
			case 2: 
				WizFi360_APs.AP[WizFi360_APs.Count].RSSI = ParseNumber(ptr, NULL);
				break;
			case 3:
				/* Ignore first and last " */
				ptr++;
				ptr[strlen(ptr) - 1] = 0;
			
				/* Parse MAC address */
				ParseMAC(ptr, WizFi360_APs.AP[WizFi360_APs.Count].MAC, NULL);
			
				break;
			case 4: 
				WizFi360_APs.AP[WizFi360_APs.Count].Channel = ParseNumber(ptr, NULL);
				break;
			case 5: 
				WizFi360_APs.AP[WizFi360_APs.Count].Offset = ParseNumber(ptr, NULL);
				break;
			case 6: 
				WizFi360_APs.AP[WizFi360_APs.Count].Calibration = ParseNumber(ptr, NULL);
				break;
			default: break;
		}
		
		/* Get new token */
		ptr = strtok(NULL, ",");
	}
	
	/* Increase count */
	WizFi360_APs.Count++;
}
#endif

static void ParseCIPSTA(WizFi360_t* WizFi360, char* Buffer) {
	uint8_t pos, s;
	uint8_t command = 0;
	
	/* Get positions */
	if (strncmp("+CIPSTA_CUR:ip", Buffer, 14) == 0) {
		pos = 14;
		s = 1;
		command = WizFi360_COMMAND_CIPSTA;
	} else if (strncmp("+CIPSTA_CUR:netmask", Buffer, 19) == 0) {
		pos = 19;
		s = 2;
		command = WizFi360_COMMAND_CIPSTA;
	} else if (strncmp("+CIPSTA_CUR:gateway", Buffer, 19) == 0) {
		pos = 19;
		s = 3;
		command = WizFi360_COMMAND_CIPSTA;
	} else if (strncmp("+CIPSTA:ip", Buffer, 10) == 0) {
		pos = 10;
		s = 1;
		command = WizFi360_COMMAND_CIPSTA;
	} else if (strncmp("+CIPSTA:netmask", Buffer, 15) == 0) {
		pos = 15;
		s = 2;
		command = WizFi360_COMMAND_CIPSTA;
	} else if (strncmp("+CIPSTA:gateway", Buffer, 15) == 0) {
		pos = 15;
		s = 3;
		command = WizFi360_COMMAND_CIPSTA;
	} else if (strncmp("+CIPAP_CUR:ip", Buffer, 13) == 0) {
		pos = 13;
		s = 1;
	} else if (strncmp("+CIPAP_CUR:netmask", Buffer, 18) == 0) {
		pos = 18;
		s = 2;
	} else if (strncmp("+CIPAP_CUR:gateway", Buffer, 18) == 0) {
		pos = 18;
		s = 3;
	} else if (strncmp("+CIPAP:ip", Buffer, 9) == 0) {
		pos = 9;
		s = 1;
	} else if (strncmp("+CIPAP:netmask", Buffer, 14) == 0) {
		pos = 14;
		s = 2;
	} else if (strncmp("+CIPAP:gateway", Buffer, 14) == 0) {
		pos = 14;
		s = 3;
	}
	
	/* Copy content */
	if (command == WizFi360_COMMAND_CIPSTA) {
		switch (s) {
			case 1:
				/* Parse IP string */
				ParseIP(&Buffer[pos + 2], WizFi360->STAIP, NULL);
				WizFi360->Flags.F.STAIPIsSet = 1;
				break;
			case 2:
				ParseIP(&Buffer[pos + 2], WizFi360->STANetmask, NULL);
				WizFi360->Flags.F.STANetmaskIsSet = 1;
				break;
			case 3:
				ParseIP(&Buffer[pos + 2], WizFi360->STAGateway, NULL);
				WizFi360->Flags.F.STAGatewayIsSet = 1;
				break;
			default: break;
		}
	} else {
		switch (s) {
			case 1:
				/* Parse IP string */
				ParseIP(&Buffer[pos + 2], WizFi360->APIP, NULL);
				WizFi360->Flags.F.APIPIsSet = 1;
				break;
			case 2:
				ParseIP(&Buffer[pos + 2], WizFi360->APNetmask, NULL);
				WizFi360->Flags.F.APNetmaskIsSet = 1;
				break;
			case 3:
				ParseIP(&Buffer[pos + 2], WizFi360->APGateway, NULL);
				WizFi360->Flags.F.APGatewayIsSet = 1;
				break;
			default: break;
		}		
	}
}

static void ParseCWLIF(WizFi360_t* WizFi360, char* Buffer) {
	uint8_t cnt;
	
	/* Check if memory available */
	if (WizFi360->ConnectedStations.Count >= WizFi360_MAX_CONNECTEDSTATIONS) {
		return;
	}
	
	/* Parse IP */
	ParseIP(Buffer, WizFi360->ConnectedStations.Stations[WizFi360->ConnectedStations.Count].IP, &cnt);
	
	/* Parse MAC */
	ParseMAC(&Buffer[cnt + 1], WizFi360->ConnectedStations.Stations[WizFi360->ConnectedStations.Count].MAC, NULL);
	
	/* Increase counter */
	WizFi360->ConnectedStations.Count++;
}

static void ParseCWJAP(WizFi360_t* WizFi360, char* Buffer) {
	char* ptr = Buffer;
	uint8_t i, cnt;
	
	/* Check for existance */
	if (!strstr(Buffer, "+CWJAP_")) {
		return;
	}
	
	/* Find first " character */
	while (*ptr && *ptr != '"') {
		ptr++;
	}
	
	/* Remove first " for SSID */
	ptr++;
	
	/* Parse SSID part */
	i = 0;
	while (*ptr && (*ptr != '"' || *(ptr + 1) != ',' || *(ptr + 2) != '"')) {
		WizFi360->ConnectedWifi.SSID[i++] = *ptr++;
	}
	WizFi360->ConnectedWifi.SSID[i++] = 0;
	
	/* Increase pointer by 3, ignore "," part */
	ptr += 3;
	
	/* Get MAC */
	ParseMAC(ptr, WizFi360->ConnectedWifi.MAC, &cnt);
	
	/* Increase counter by elements in MAC address and ", part */
	ptr += cnt + 2;
	
	/* Get channel */
	WizFi360->ConnectedWifi.Channel = ParseNumber(ptr, &cnt);
	
	/* Increase position */
	ptr += cnt + 1;
	
	/* Get RSSI */
	WizFi360->ConnectedWifi.RSSI = ParseNumber(ptr, &cnt);
}

static uint8_t Hex2Num(char a) {
	if (a >= '0' && a <= '9') {
		return a - '0';
	} else if (a >= 'a' && a <= 'f') {
		return (a - 'a') + 10;
	} else if (a >= 'A' && a <= 'F') {
		return (a - 'A') + 10;
	}
	
	return 0;
}

static uint8_t CHARISHEXNUM(char a) {
	if (a >= '0' && a <= '9') {
		return 1;
	} else if (a >= 'a' && a <= 'f') {
		return 1;
	} else if (a >= 'A' && a <= 'F') {
		return 1;
	}
	
	return 0;
}

static int32_t ParseNumber(char* ptr, uint8_t* cnt) {
	uint8_t minus = 0;
	int32_t sum = 0;
	uint8_t i = 0;
	
	/* Check for minus character */
	if (*ptr == '-') {
		minus = 1;
		ptr++;
		i++;
	}
	
	/* Parse number */
	while (CHARISNUM(*ptr)) {
		sum = 10 * sum + CHAR2NUM(*ptr);
		ptr++;
		i++;
	}
	
	/* Save number of characters used for number */
	if (cnt != NULL) {
		*cnt = i;
	}
	
	/* Minus detected */
	if (minus) {
		return 0 - sum;
	}
	
	/* Return number */
	return sum;
}

static uint32_t ParseHexNumber(char* ptr, uint8_t* cnt) {
	uint32_t sum = 0;
	uint8_t i = 0;
	
	/* Parse number */
	while (CHARISHEXNUM(*ptr)) {
		sum = 16 * sum + Hex2Num(*ptr);
		ptr++;
		i++;
	}
	
	/* Save number of characters used for number */
	if (cnt != NULL) {
		*cnt = i;
	}
	
	/* Return number */
	return sum;
}

static void ParseIP(char* ip_str, uint8_t* arr, uint8_t* cnt) {
	char* token;
	uint8_t i = 0;
	uint8_t x = 0;
	uint8_t c;
	char Data[16];
	
	/* Make a string copy first */
	memcpy(Data, ip_str, sizeof(Data) - 1);
	Data[sizeof(Data) - 1] = 0;
	
	/* Parse numbers, skip :" */
	token = strtok(Data, ".");
	
	/* We expect 4 loops */
	while (token != NULL) {
		/* Parse number */
		arr[x++] = ParseNumber(token, &c);
		
		/* Save number of characters used for IP string */
		i += c;
		
		/* Max 4 loops */
		if (x >= 4) {
			break;
		}
		
		/* Increase number of characters, used for "." (DOT) */
		i++;
		
		/* Get new token */
		token = strtok(NULL, ".");
	}
	
	/* Save number of characters */
	if (cnt != NULL) {
		*cnt = i;
	}
}

static void ParseMAC(char* ptr, uint8_t* arr, uint8_t* cnt) {
	char* hexptr;
	uint8_t hexnum = 0;
	uint8_t tmpcnt = 0, sum = 0;
	
	/* Get token */
	hexptr = strtok(ptr, ":");

	/* Do it till NULL */
	while (hexptr != NULL) {
		/* Parse hex */
		arr[hexnum++] = ParseHexNumber(hexptr, &tmpcnt);
		
		/* Add to sum value */
		sum += tmpcnt;
		
		/* We have 6 characters */
		if (hexnum >= 6) {
			break;
		}
		
		/* Increase for ":" */
		sum++;
		
		/* Get new token */
		hexptr = strtok(NULL, ":");
	}
	
	/* Save value */
	if (cnt) {
		*cnt = sum;
	}
}

static void ParseReceived(WizFi360_t* WizFi360, char* Received, uint8_t from_usart_buffer, uint16_t bufflen) {
	char* ch_ptr;
	uint8_t bytes_cnt;
	uint32_t ipd_ptr = 0;
	WizFi360_Connection_t* Conn;
	
	/* Update last activity */
	WizFi360->LastReceivedTime = WizFi360->Time;
	
	/* Check for empty new line */
	if (bufflen == 2 && Received[0] == '\r' && Received[1] == '\n') {
		return;
	}
	
	/* First check, if any command is active */
	if (WizFi360->ActiveCommand != WizFi360_COMMAND_IDLE && from_usart_buffer == 1) {
		/* Check if string does not belong to this command */
		if (
			strcmp(Received, "OK\r\n") != 0 &&
			strcmp(Received, "SEND OK\r\n") != 0 &&
			strcmp(Received, "ERROR\r\n") != 0 &&
			strcmp(Received, "ready\r\n") != 0 &&
			strcmp(Received, "busy p...\r\n") != 0 &&
			strncmp(Received, "+IPD", 4) != 0 &&
			strncmp(Received, WizFi360->ActiveCommandResponse[0], strlen(WizFi360->ActiveCommandResponse[0])) != 0
		) {
			/* Save string to temporary buffer, because we received a string which does not belong to this command */
			BUFFER_WriteString(&TMP_Buffer, Received);
			
			/* Return from function */
			return;
		}
	}
	
	/* Device is ready */
	if (strcmp(Received, "ready\r\n") == 0) {
		WizFi360_Callback_DeviceReady(WizFi360);
	}
	
	/* Device WDT reset */
	if (strcmp(Received, "wdt reset\r\n") == 0) {
		WizFi360_Callback_WatchdogReset(WizFi360);
	}
	
	/* Call user callback functions if not already */
	CallConnectionCallbacks(WizFi360);
	
	/* We are connected to Wi-Fi */
	if (strcmp(Received, "WIFI CONNECTED\r\n") == 0) {
		/* Set flag */
		WizFi360->Flags.F.WifiConnected = 1;
		
		/* Call user callback function */
		WizFi360_Callback_WifiConnected(WizFi360);
	} else if (strcmp(Received, "WIFI DISCONNECT\r\n") == 0) {
		/* Clear flags */
		WizFi360->Flags.F.WifiConnected = 0;
		WizFi360->Flags.F.WifiGotIP = 0;
		
		/* Reset connected wifi structure */
		memset((uint8_t *)&WizFi360->ConnectedWifi, 0, sizeof(WizFi360->ConnectedWifi));
		
		/* Reset all connections */
		WizFi360_RESET_CONNECTIONS(WizFi360);
		
		/* Call user callback function */
		WizFi360_Callback_WifiDisconnected(WizFi360);
	} else if (strcmp(Received, "WIFI GOT IP\r\n") == 0) {
		/* Wifi got IP address */
		WizFi360->Flags.F.WifiGotIP = 1;
		
		/* Call user callback function */
		WizFi360_Callback_WifiGotIP(WizFi360);
	}
			
	/* In case data were send */
	if (strstr(Received, "SEND OK\r\n") != NULL) {
		uint8_t cnt;
		
		/* Reset active command so user will be able to call new command in callback function */
		WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
		
		for (cnt = 0; cnt < WizFi360_MAX_CONNECTIONS; cnt++) {
			/* Check for data sent */
			if (WizFi360->Connection[cnt].WaitingSentRespond) {
				/* Reset flag */
				WizFi360->Connection[cnt].WaitingSentRespond = 0;
				
				/* Call user function according to connection type */
				if (WizFi360->Connection[cnt].Client) {
					/* Client mode */
					WizFi360_Callback_ClientConnectionDataSent(WizFi360, &WizFi360->Connection[cnt]);
				} else {
					/* Server mode */
					WizFi360_Callback_ServerConnectionDataSent(WizFi360, &WizFi360->Connection[cnt]);
				}
			}
		}
	}
	/* Check if +IPD was received with incoming data */
	if (strncmp(Received, "+IPD", 4) == 0) {		
		/* If we are not in IPD mode already */
		/* Go to IPD mode */
		WizFi360->IPD.InIPD = 1;
		WizFi360->IPD.USART_Buffer = from_usart_buffer;
		
		/* Reset pointer */
		ipd_ptr = 5;
		
		/* Get connection number from IPD statement */
		WizFi360->IPD.ConnNumber = CHAR2NUM(Received[ipd_ptr]);
		
		/* Set working buffer for this connection */
#if WizFi360_USE_SINGLE_CONNECTION_BUFFER == 1
		WizFi360->Connection[WizFi360->IPD.ConnNumber].Data = ConnectionData;
#endif
		
		/* Save connection number */
		WizFi360->Connection[WizFi360->IPD.ConnNumber].Number = WizFi360->IPD.ConnNumber;
		
		/* Increase pointer by 2 */
		ipd_ptr += 2;
		
		/* Save number of received bytes */
		WizFi360->Connection[WizFi360->IPD.ConnNumber].BytesReceived = ParseNumber(&Received[ipd_ptr], &bytes_cnt);
		
		/* First time */
		if (WizFi360->Connection[WizFi360->IPD.ConnNumber].TotalBytesReceived == 0) {
			/* Reset flag */
			WizFi360->Connection[WizFi360->IPD.ConnNumber].HeadersDone = 0;
			
			/* This is first packet of data */
			WizFi360->Connection[WizFi360->IPD.ConnNumber].FirstPacket = 1;
		} else {
			/* This is not first packet */
			WizFi360->Connection[WizFi360->IPD.ConnNumber].FirstPacket = 0;
		}
		
		/* Save total number of bytes */
		WizFi360->Connection[WizFi360->IPD.ConnNumber].TotalBytesReceived += WizFi360->Connection[WizFi360->IPD.ConnNumber].BytesReceived;
		
		/* Increase global number of bytes received from WizFi360 module to stack */
		WizFi360->TotalBytesReceived += WizFi360->Connection[WizFi360->IPD.ConnNumber].BytesReceived;
		
		/* Increase pointer for number of characters for number and for comma */
		ipd_ptr += bytes_cnt + 1;
		
		/* Save IP */
		ParseIP(&Received[ipd_ptr], WizFi360->Connection[WizFi360->IPD.ConnNumber].RemoteIP, &bytes_cnt);
		
		/* Increase pointer for number of characters for IP string and for comma */
		ipd_ptr += bytes_cnt + 1;
		
		/* Save PORT */
		WizFi360->Connection[WizFi360->IPD.ConnNumber].RemotePort = ParseNumber(&Received[ipd_ptr], &bytes_cnt);
		
		/* Increase pointer */
		ipd_ptr += bytes_cnt + 1;
		
		/* Find : element where real data starts */
		ipd_ptr = 0;
		while (Received[ipd_ptr] != ':') {
			ipd_ptr++;
		}
		ipd_ptr++;
		
		/* Copy content to beginning of buffer */
		memcpy((uint8_t *)WizFi360->Connection[WizFi360->IPD.ConnNumber].Data, (uint8_t *)&Received[ipd_ptr], bufflen - ipd_ptr);
		/* Check for length */
		if ((bufflen - ipd_ptr) > WizFi360->Connection[WizFi360->IPD.ConnNumber].BytesReceived) {
			/* Add zero at the end of string */
			WizFi360->Connection[WizFi360->IPD.ConnNumber].Data[WizFi360->Connection[WizFi360->IPD.ConnNumber].BytesReceived] = 0;
		}
		
		/* Calculate remaining bytes */
		WizFi360->IPD.InPtr = WizFi360->IPD.PtrTotal = bufflen - ipd_ptr;
		
		/* Check remaining data */
		if (ipd_ptr >= WizFi360->Connection[WizFi360->IPD.ConnNumber].BytesReceived) {
			/* Not in IPD anymore */
			WizFi360->IPD.InIPD = 0;
			
			/* Set package data size */
			WizFi360->Connection[WizFi360->IPD.ConnNumber].DataSize = ipd_ptr;
			WizFi360->Connection[WizFi360->IPD.ConnNumber].LastPart = 1;
			
			/* Enable flag to call received data callback */
			WizFi360->Connection[WizFi360->IPD.ConnNumber].CallDataReceived = 1;
		}
	}
	
	/* Check if we have a new connection */
	if ((ch_ptr = (char *)mem_mem(Received, bufflen, ",CONNECT\r\n", 10)) != NULL) {
		/* New connection has been made */
		Conn = &WizFi360->Connection[CHAR2NUM(*(ch_ptr - 1))];
		Conn->Active = 1;
		Conn->Number = CHAR2NUM(*(ch_ptr - 1));
		
		/* Call user function according to connection type (client, server) */
		if (Conn->Client) {			
			/* Reset current connection */
			if (WizFi360->ActiveCommand == WizFi360_COMMAND_CIPSTART) {
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
			}
			
			/* Connection started as client */
			WizFi360_Callback_ClientConnectionConnected(WizFi360, Conn);
		} else {
			/* Connection started as server */
			WizFi360_Callback_ServerConnectionActive(WizFi360, Conn);
		}
	}
	
	/* Check if already connected */
	if (strstr(Received, "ALREADY CONNECTED\r\n") != NULL) {
		PRINTF("Connection %d already connected!\r\n", WizFi360->StartConnectionSent);
	}
	
	/* Check if we have a closed connection */
	if ((ch_ptr = (char *)mem_mem(Received, bufflen, ",CLOSED\r\n", 9)) != NULL && Received != ch_ptr) {
		uint8_t client, active;
		
		/* Check if CLOSED statement is on beginning, if not, write it to temporary buffer and leave here */
		/* If not on beginning of string, probably ,CLOSED was returned after +IPD statement */
		/* Make string standalone */
		if (ch_ptr == (Received + 1)) {
			/* Save values */
			client = WizFi360->Connection[CHAR2NUM(*(ch_ptr - 1))].Client;
			active = WizFi360->Connection[CHAR2NUM(*(ch_ptr - 1))].Active;
			
			/* Connection closed, reset flags now */
			WizFi360_RESETCONNECTION(WizFi360, &WizFi360->Connection[CHAR2NUM(*(ch_ptr - 1))]);
			
			/* Call user function */
			if (active) {
				if (client) {
					/* Client connection closed */
					WizFi360_Callback_ClientConnectionClosed(WizFi360, &WizFi360->Connection[CHAR2NUM(*(ch_ptr - 1))]);
				} else {
					/* Server connection closed */
					WizFi360_Callback_ServerConnectionClosed(WizFi360, &WizFi360->Connection[CHAR2NUM(*(ch_ptr - 1))]);
				}
			}
		} else {
			/* Write to temporary buffer */
			BUFFER_Write(&TMP_Buffer, (uint8_t *)(ch_ptr - 1), 10);
		}
	}
	
	/* Check if we have a new connection */
	if ((ch_ptr = strstr(Received, ",CONNECT FAIL\r\n")) != NULL) {
		/* New connection has been made */
		Conn = &WizFi360->Connection[CHAR2NUM(*(ch_ptr - 1))];
		WizFi360_RESETCONNECTION(WizFi360, Conn);
		Conn->Number = CHAR2NUM(*(ch_ptr - 1));
		
		/* Call user function according to connection type (client, server) */
		if (Conn->Client) {
			/* Reset current connection */
			if (WizFi360->ActiveCommand == WizFi360_COMMAND_CIPSTART) {
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
			}
			
			/* Connection failed */
			WizFi360_Callback_ClientConnectionError(WizFi360, Conn);
		}
	}
	
	/* Check commands we have sent */
	switch (WizFi360->ActiveCommand) {
		/* Check wifi disconnect response */
		case WizFi360_COMMAND_CWQAP:
			if (strcmp(Received, "OK\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
			}
			break;
		case WizFi360_COMMAND_CWJAP:			
			/* We send command and we have error response */
			if (strncmp(Received, "+CWJAP:", 7) == 0) {
				/* We received an error, wait for "FAIL" string for next time */
				strcpy(WizFi360->ActiveCommandResponse[0], "FAIL\r\n");
				
				/* Check reason */
				WizFi360->WifiConnectError = (WizFi360_WifiConnectError_t)CHAR2NUM(Received[7]);
			}
			
			if (strcmp(Received, "OK\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
			}
			
			if (strcmp(Received, "FAIL\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
				
				/* Call user function */
				WizFi360_Callback_WifiConnectFailed(WizFi360);
			}
			break;
		case WizFi360_COMMAND_CWJAP_GET:
			/* We sent command to get current connected AP */
			if (strncmp(Received, "+CWJAP_CUR:", 11) == 0) {
				/* Parse string */
				ParseCWJAP(WizFi360, Received);
			}
			if (strcmp(Received, "OK\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
			}
			break;
#if WizFi360_USE_APSEARCH
		case WizFi360_COMMAND_CWLAP:
			/* CWLAP received, parse it */
			if (strncmp(Received, "+CWLAP:", 7) == 0) {
				/* Parse CWLAP */
				ParseCWLAP(WizFi360, Received);
			}
			if (strcmp(Received, "OK\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
				
				/* Call user function */
				WizFi360_Callback_WifiDetected(WizFi360, &WizFi360_APs);
			}
			break;
#endif
		case WizFi360_COMMAND_CWSAP:
			/* CWLAP received, parse it */
			if (strncmp(Received, "+CWSAP", 6) == 0) {
				/* Parse CWLAP */
				ParseCWSAP(WizFi360, Received);
			}
			if (strcmp(Received, "OK\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
			}
			break;
		case WizFi360_COMMAND_CIPSTA:
			/* CIPSTA detected */
			if (strncmp(Received, "+CIPSTA", 7) == 0) {
				/* Parse CIPSTA */
				ParseCIPSTA(WizFi360, Received);
			}
		
			if (strcmp(Received, "OK\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
				
				/* Callback function */
				WizFi360_Callback_WifiIPSet(WizFi360);
			}
			break;
		case WizFi360_COMMAND_CIPAP:
			/* CIPAP detected */
			if (strncmp(Received, "+CIPAP", 6) == 0) {
				/* Parse CIPAP (or CIPSTA) */
				ParseCIPSTA(WizFi360, Received);
			}
		
			if (strcmp(Received, "OK\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
			}
			break;
		case WizFi360_COMMAND_CWMODE:
			if (strcmp(Received, "OK\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
				
				/* Save mode */
				WizFi360->Mode = WizFi360->SentMode;
			}
			break;
		case WizFi360_COMMAND_CIPSERVER:
			if (strcmp(Received, "OK\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
			}
			break;
		case WizFi360_COMMAND_SEND:
			if (strcmp(Received, "OK\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_SENDDATA;
				
				/* Do not reset command, instead, wait for wrapper command! */
				WizFi360->Flags.F.WaitForWrapper = 1;
				
				/* We are now waiting for SEND OK */
				strcpy(WizFi360->ActiveCommandResponse[0], "SEND OK");
			}
			break;
		case WizFi360_COMMAND_SENDDATA:
			break;
		case WizFi360_COMMAND_CIPSTART:
			if (strcmp(Received, "OK\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
			}
			if (strcmp(Received, "ERROR\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
				
				/* Reset connection */
				WizFi360_RESETCONNECTION(WizFi360, &WizFi360->Connection[WizFi360->StartConnectionSent]);
				
				/* Call user function */
				WizFi360_Callback_ClientConnectionError(WizFi360, &WizFi360->Connection[WizFi360->StartConnectionSent]);
			}
			break;
		case WizFi360_COMMAND_CIPMUX:
		case WizFi360_COMMAND_CIPDINFO:
		case WizFi360_COMMAND_AT:
		case WizFi360_COMMAND_UART:
		case WizFi360_COMMAND_CLOSE:
		case WizFi360_COMMAND_SLEEP:
		case WizFi360_COMMAND_GSLP:
		case WizFi360_COMMAND_CIPSTO:
		case WizFi360_COMMAND_RESTORE:
			if (strcmp(Received, "OK\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
			}
			break;
		case WizFi360_COMMAND_RST:
			if (strcmp(Received, "ready\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
				
				/* Set flag */
				WizFi360->Flags.F.LastOperationStatus = 1;
			}
			break;
#if WizFi360_USE_PING
		case WizFi360_COMMAND_PING:
			/* Check if data about ping milliseconds are received */
			if (Received[0] == '+') {
				/* Parse number for pinging */
				WizFi360->PING.Time = ParseNumber(&Received[1], NULL);
			}
			if (strcmp(Received, "OK\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
				
				/* Set status */
				WizFi360->PING.Success = 1;
				
				/* Error callback */
				WizFi360_Callback_PingFinished(WizFi360, &WizFi360->PING);
			} else if (strcmp(Received, "ERROR\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
				
				/* Set status */
				WizFi360->PING.Success = 0;
				
				/* Error callback */
				WizFi360_Callback_PingFinished(WizFi360, &WizFi360->PING);
			}
			break;
#endif
		case WizFi360_COMMAND_CIPSTAMAC:
			/* CIPSTA detected */
			if (strncmp(Received, "+CIPSTAMAC", 10) == 0) {
				/* Parse CIPSTA */
				ParseMAC(&Received[12], WizFi360->STAMAC, NULL);
			}
		
			if (strcmp(Received, "OK\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
			}
			break;
		case WizFi360_COMMAND_CIPAPMAC:
			/* CIPSTA detected */
			if (strncmp(Received, "+CIPAPMAC", 9) == 0) {
				/* Parse CIPSTA */
				ParseMAC(&Received[11], WizFi360->APMAC, NULL);
			}
		
			if (strcmp(Received, "OK\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
			}
			break;
#if WizFi360_USE_FIRMWAREUPDATE
		case WizFi360_COMMAND_CIUPDATE:
			/* Check for strings for update */
			if (strncmp(Received, "+CIPUPDATE:", 11) == 0) {
				/* Get current number */
				uint8_t num = CHAR2NUM(Received[11]);
				
				/* Check step */
				if (num == 4) {
					/* We are waiting last step, increase timeout */
					WizFi360->Timeout = 10 * WizFi360_TIMEOUT;
				}
				
				/* Call user function */
				WizFi360_Callback_FirmwareUpdateStatus(WizFi360, (WizFi360_FirmwareUpdate_t)num);
			}
		
			if (strcmp(Received, "OK\r\n") == 0 || strcmp(Received, "ready\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
				
				/* Reset timeout */
				WizFi360->Timeout = WizFi360_TIMEOUT;
				
				/* Call user function */
				WizFi360_Callback_FirmwareUpdateSuccess(WizFi360);
			}
			
			if (strcmp(Received, "ERROR\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
				
				/* Reset timeout */
				WizFi360->Timeout = WizFi360_TIMEOUT;
				
				/* Call user function */
				WizFi360_Callback_FirmwareUpdateError(WizFi360);
			}
			break;
#endif
		case WizFi360_COMMAND_CWLIF:
			/* Check if first character is number */
			if (CHARISNUM(Received[0])) {
				/* Parse response */
				ParseCWLIF(WizFi360, Received);
			}
		
			if (strcmp(Received, "OK\r\n") == 0) {
				/* Reset active command */
				WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
				
				/* Call user function */
				WizFi360_Callback_ConnectedStationsDetected(WizFi360, &WizFi360->ConnectedStations);
			}
			break;
		default:
			/* No command was used to send, data received without command */
			break;
	}
	
	/* Set flag for last operation status */
	if (strcmp(Received, "OK\r\n") == 0) {
		WizFi360->Flags.F.LastOperationStatus = 1;
		
		/* Reset active command */
		/* TODO: Check if OK here */
		if (WizFi360->ActiveCommand != WizFi360_COMMAND_SEND) {
			/* We are waiting for "> " string */
			WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
		}
	}
	if (strcmp(Received, "ERROR\r\n") == 0 || strcmp(Received, "busy p...\r\n") == 0) {
		WizFi360->Flags.F.LastOperationStatus = 0;
		
		/* Reset active command */
		/* TODO: Check if ERROR here */
		WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
	}
	if (strcmp(Received, "SEND OK\r\n") == 0) {
		/* Force IDLE when we are in SEND mode and SEND OK is returned. Do not wait for "> " wrapper */
		WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
		
		/* Clear flag */
		WizFi360->Flags.F.WaitForWrapper = 0;
	}
}

static WizFi360_Result_t SendCommand(WizFi360_t* WizFi360, uint8_t Command, char* CommandStr, char* StartRespond) {
	/* Check idle mode */
	WizFi360_CHECK_IDLE(WizFi360);
	
	/* Clear buffer */
	if (Command == WizFi360_COMMAND_UART) {
		/* Reset USART buffer */
		BUFFER_Reset(&USART_Buffer);
	}
	
	/* Clear buffer and send command */
	WizFi360_LL_USARTSend((uint8_t *)CommandStr, strlen(CommandStr));
	
	/* Save current active command */
	WizFi360->ActiveCommand = Command;
	WizFi360->ActiveCommandResponse[0][0] = 0;
	strcpy(WizFi360->ActiveCommandResponse[0], StartRespond);
	
	/* Set command start time */
	WizFi360->StartTime = WizFi360->Time;
	
	/* Return OK */
	WizFi360_RETURNWITHSTATUS(WizFi360, ESP_OK);
}

static char* EscapeString(char* str) {
	static char buff[100];
	char* str_ptr = buff;
	
	/* Go through string */
	while (*str) {
		/* Check for special character */
		if (*str == ',' || *str == '"' || *str == '\\') {
			*str_ptr++ = '/';
		}
		/* Copy and increase pointers */
		*str_ptr++ = *str++;
	}
	
	/* Add zero to the end */
	*str_ptr = 0;
	
	/* Return buffer */
	return buff;
}

char* ReverseEscapeString(char* str) {
	static char buff[100];
	char* str_ptr = buff;
	
	/* Go through string */
	while (*str) {
		/* Check for special character */
		if (*str == '/') {
			/* Check for next string after '/' */
			if (*(str + 1) == ',' || *(str + 1) == '"' || *(str + 1) == '\\') {
				/* Ignore '/' */
				str++;
			}
		}
		
		/* Copy and increase pointers */
		*str_ptr++ = *str++;
	}
	
	/* Add zero to the end */
	*str_ptr = 0;
	
	/* Return buffer */
	return buff;
}

static WizFi360_Result_t SendUARTCommand(WizFi360_t* WizFi360, uint32_t baudrate, char* cmd) {
	char command[35];
	
	/* Check idle */
	WizFi360_CHECK_IDLE(WizFi360);
	
	/* Format command */
	sprintf(command, "%s=%d,8,1,0,0\r\n", cmd, baudrate);
	
	/* Send command */
	if (SendCommand(WizFi360, WizFi360_COMMAND_UART, command, "AT+UART") != ESP_OK) {
		return WizFi360->Result;
	}
	
	/* Wait till command end */
	WizFi360_WaitReady(WizFi360);
	
	/* Check for success */
	if (!WizFi360->Flags.F.LastOperationStatus) {
		WizFi360_RETURNWITHSTATUS(WizFi360, ESP_ERROR);
	}
	
	/* Save baudrate */
	WizFi360->Baudrate = baudrate;
	
	/* Delay a little, wait for all bytes from ESP are received before we delete them from buffer */
	WizFi360_DELAYMS(5);
	
	/* Set new UART baudrate */
	WizFi360_LL_USARTInit(WizFi360->Baudrate);
	
	/* Clear buffer */
	BUFFER_Reset(&USART_Buffer);
	
	/* Delay a little */
	WizFi360_DELAYMS(5);
	
	/* Reset command */
	WizFi360->ActiveCommand = WizFi360_COMMAND_IDLE;
	
	/* Return OK */
	WizFi360_RETURNWITHSTATUS(WizFi360, ESP_OK);
}

static WizFi360_Result_t SendMACCommand(WizFi360_t* WizFi360, uint8_t* addr, char* cmd, uint8_t command) {
	char tmp[40];
	
	/* Check idle */
	WizFi360_CHECK_IDLE(WizFi360);
	
	/* Format string */
	sprintf(tmp, "%s=\"%02x:%02x:%02x:%02x:%02x:%02x\"", cmd, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
	
	/* Send command */
	SendCommand(WizFi360, command, tmp, NULL);
	
	/* Wait ready */
	WizFi360_WaitReady(WizFi360);
	
	/* Check last operation */
	if (WizFi360->Flags.F.LastOperationStatus) {
		/* MAC set was OK, copy MAC address */
		memcpy(command == WizFi360_COMMAND_CIPSTAMAC ? &WizFi360->STAMAC : &WizFi360->APMAC, addr, 6);
	} else {
		/* Check status */
		if (WizFi360->Result == ESP_OK) {
			/* Reset flags */
			if (command == WizFi360_COMMAND_CIPSTAMAC) {
				WizFi360->Flags.F.STAMACIsSet = 0;
			} else {
				WizFi360->Flags.F.APMACIsSet = 0;
			}
		}
	}
	
	/* Return stats */
	return WizFi360->Result;
}

static void CallConnectionCallbacks(WizFi360_t* WizFi360) {
	uint8_t conn_number;
	
	/* Check if there are any pending data to be sent to connection */
//	for (conn_number = 0; conn_number < WizFi360_MAX_CONNECTIONS; conn_number++) {
//		if (
//			WizFi360->ActiveCommand == WizFi360_COMMAND_IDLE &&                                            /*!< Must be IDLE, if we are not because callbacks start command, stop execution */
//			WizFi360->Connection[conn_number].Active && WizFi360->Connection[conn_number].CallDataReceived /*!< We must call function for received data */
//		) {
//			/* Clear flag */
//			WizFi360->Connection[conn_number].CallDataReceived = 0;
//			
//			
//			/* Call user function according to connection type */
//			if (WizFi360->Connection[conn_number].Client) {
//				/* Client mode */
//				WizFi360_Callback_ClientConnectionDataReceived(WizFi360, &WizFi360->Connection[conn_number], WizFi360->Connection[conn_number].Data);
//			} else {
//				/* Server mode */
//				WizFi360_Callback_ServerConnectionDataReceived(WizFi360, &WizFi360->Connection[conn_number], WizFi360->Connection[conn_number].Data);
//			}
//			
//		}
//	}
}

static void ProcessSendData(WizFi360_t* WizFi360) {
	uint16_t found;
	WizFi360_Connection_t* Connection = WizFi360->SendDataConnection;
	/* Wrapper was found */
	WizFi360->Flags.F.WaitForWrapper = 0;
	
	/* Go to SENDDATA command as active */
	WizFi360->ActiveCommand = WizFi360_COMMAND_SENDDATA;
	
//	/* Get data from user */
//	if (Connection->Client) {
//		/* Get data as client */
//		found = WizFi360_Callback_ClientConnectionSendData(WizFi360, Connection, Connection->Data, 2046);
//	} else {
//		/* Get data as server */
//		found = WizFi360_Callback_ServerConnectionSendData(WizFi360, Connection, Connection->Data, 2046);
//	}
	found=strlen(Connection->Data);
	/* Check for input data */
	if (found > 2046) {
		found = 2046;
	}
	
	/* If data valid */
	if (found > 0) {
		/* Send data */
		WizFi360_LL_USARTSend((uint8_t *)Connection->Data, found);
		
		/* Increase number of bytes sent */
		WizFi360->TotalBytesSent += found;
	}
	/* Send zero at the end even if data are not valid = stop sending data to module */
	WizFi360_LL_USARTSend((uint8_t *)"\\0", 2);
}

/* Check if needle exists in haystack memory */
void* mem_mem(void* haystack, size_t haystacksize, void* needle, size_t needlesize) {
	unsigned char* hptr = (unsigned char *)haystack;
	unsigned char* nptr = (unsigned char *)needle;
	unsigned int i;

	/* Go through entire memory */
	for (i = 0; i < haystacksize; i++) {
		if (memcmp(&hptr[i], nptr, needlesize) == 0) {
			return &hptr[i];
		}
	}

	return 0;
}
