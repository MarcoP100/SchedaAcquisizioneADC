#include "mcp2515.h"

//static uint8_t emptyTXBuffer[3] = {true, true,true};
uint8_t idDataEmpty[4] = {0x0, 0x0, 0x0, 0x0};
uint8_t dataEmpty[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,0x0};
static uint8_t writeMessage[4];
volatile uint8_t transmissionComplete;

// Costruttore
uint8_t MCP2515_Init(MCP2515_HandleTypeDef* hdev, GPIO_TypeDef* csPort,  uint16_t csPin, SPI_HandleTypeDef* hspi, uint8_t baudrate,uint8_t intTxEnable ) {
    hdev->csPin = csPin;
    hdev->csPort = csPort;
    hdev->hspi = hspi;
    hdev->emptyTXBuffer[0] = true;
    hdev->emptyTXBuffer[1] = true;
    hdev->emptyTXBuffer[2] = true;
    
    uint8_t result = MCP2515_OK;
    result = MCP2515_deviceInit(hdev, baudrate, intTxEnable);
    
    return result;
       

}

uint8_t MCP2515_deviceInit(MCP2515_HandleTypeDef* hdev, uint8_t baudrate, uint8_t intTxEnable) {

    HAL_Delay(100);

    // Resetta il MCP2515
    uint8_t resultReset;
    //MCP2515_Reset(hdev);
    resultReset = MCP2515_Reset(hdev);
    if (resultReset != MCP2515_OK) {
        return resultReset;
    }

    // Imposta il baudrate
    uint8_t resultBaudrate;
    resultBaudrate = MCP2515_SetBaudrate(hdev, baudrate);
    if (resultBaudrate != MCP2515_OK) {
        return resultBaudrate;
    }
    
    // Imposta la modalità operativa
    uint8_t resultRun;
    resultRun = MCP2515_SetMode(hdev, MCP2515_NORMAL_MODE);
    if (resultRun != MCP2515_OK) {
        return MCP2515_SET_MODE_FAIL;
    }
    
    uint8_t resultResetInt;
    resultResetInt = MCP2515_ResetInt(hdev);
    if (resultResetInt != MCP2515_OK) {
        return MCP2515_FAIL;
    }
    // Imposta l'interrupt sui tx'
    if (intTxEnable){
        uint8_t resultIntTx;
        resultIntTx = MCP2515_SetIntTx(hdev);
        if (resultIntTx != MCP2515_OK) {
            return MCP2515_FAIL;
            printf("Errore nell'enable interrupt tx\n");
        }
    }


    return MCP2515_OK;
}

uint8_t MCP2515_Reset(MCP2515_HandleTypeDef* hdev) {
    uint8_t resetCommand = MCP2515_RESET;
    const uint32_t timeout = 10; // Timeout di 10 ms
    

    hdev->transmissionComplete = 0;  // Resetta lo stato
    
    HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_RESET);  // CS basso
    //HAL_Delay(2);
    HAL_SPI_Transmit_IT(hdev->hspi, &resetCommand, 1); // Invia il comando di reset
    uint32_t startTime = HAL_GetTick();
    // Attendi il completamento della trasmissione
    while (!hdev->transmissionComplete) {
        if ((HAL_GetTick() - startTime) > timeout) {
            HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS alto
            return MCP2515_RESET_TIMEOUT; // Esci dalla funzione, indicando un errore o fallimento
        }
    }
    hdev->transmissionComplete = 0;
    HAL_Delay(10);  // Attendi che il reset venga completato (10 ms è un valore standard)
    HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS alto

    

    uint8_t status_data;
    uint8_t resultRead;
    resultRead = MCP2515_ReadRegister(hdev,MCP2515_CANSTAT_MSG, &status_data);
    if (resultRead != MCP2515_OK)
        return resultRead;
    // Verifica se il modo è stato impostato correttamente
    uint8_t currentMode = (status_data >> 5) & 0x07;
    if (currentMode != MCP2515_CONFIGURATION_MODE) {
            return MCP2515_RESET_FAIL; }


    return MCP2515_OK;  // Restituisci un codice di successo
}

void MCP2515_SetTransmissionComplete(MCP2515_HandleTypeDef* hdev, uint8_t transmissionComplete) {
    hdev->transmissionComplete = transmissionComplete;
}


uint8_t MCP2515_SetBaudrate(MCP2515_HandleTypeDef* hdev, uint8_t baudrate) {
    const uint32_t timeout = 10; // Timeout di 10 ms
    uint8_t CNF1, CNF2, CNF3;

    // composizione messaggi
    switch (baudrate) {
        case CAN_BAUDRATE_125kbps:  // 125 kbps
            CNF1 = (MCP2515_SJW_DATA_125kbps << 6) | MCP2515_BRP_DATA_125kbps;
            CNF2 = (MCP2515_BTLMODE_DATA_125kbps << 7) | (MCP2515_SAM_DATA_125kbps << 6) | (MCP2515_PHSEG1_DATA_125kbps << 3) | MCP2515_PRSEG_DATA_125kbps;
            CNF3 = (MCP2515_SOF_DATA_125kbps << 7) | (MCP2515_WAKFIL_DATA_125kbps << 6) | MCP2515_PHSEG2_DATA_125kbps;
            break;

        case CAN_BAUDRATE_250kbps:  // 250 kbps
            CNF1 = (MCP2515_SJW_DATA_250kbps << 6) | MCP2515_BRP_DATA_250kbps;
            CNF2 = (MCP2515_BTLMODE_DATA_250kbps << 7) | (MCP2515_SAM_DATA_250kbps << 6) | (MCP2515_PHSEG1_DATA_250kbps << 3) | MCP2515_PRSEG_DATA_250kbps;
            CNF3 = (MCP2515_SOF_DATA_250kbps << 7) | (MCP2515_WAKFIL_DATA_250kbps << 6) | MCP2515_PHSEG2_DATA_250kbps;
            break;

        case CAN_BAUDRATE_500kbps:  // 500 kbps
            CNF1 = (MCP2515_SJW_DATA_500kbps << 6) | MCP2515_BRP_DATA_500kbps;
            CNF2 = (MCP2515_BTLMODE_DATA_500kbps << 7) | (MCP2515_SAM_DATA_500kbps << 6) | (MCP2515_PHSEG1_DATA_500kbps << 3) | MCP2515_PRSEG_DATA_500kbps;
            CNF3 = (MCP2515_SOF_DATA_500kbps << 7) | (MCP2515_WAKFIL_DATA_500kbps << 6) | MCP2515_PHSEG2_DATA_500kbps;
            break;

        case CAN_BAUDRATE_1Mbps:  // 1 Mbps
            CNF1 = (MCP2515_SJW_DATA_1Mbps << 6) | MCP2515_BRP_DATA_1Mbps;
            CNF2 = (MCP2515_BTLMODE_DATA_1Mbps << 7) | (MCP2515_SAM_DATA_1Mbps << 6) | (MCP2515_PHSEG1_DATA_1Mbps << 3) | MCP2515_PRSEG_DATA_1Mbps;
            CNF3 = (MCP2515_SOF_DATA_1Mbps << 7) | (MCP2515_WAKFIL_DATA_1Mbps << 6) | MCP2515_PHSEG2_DATA_1Mbps;
            break;

        default:
            return MCP2515_SET_BAUDRATE_FAIL;  // Baud rate non supportato
    }

    //printf("CNF1: 0x%02X, CNF2: 0x%02X, CNF3: 0x%02X\n", CNF1, CNF2, CNF3);


    // Scrivi i registri e attendi la conferma
    if (MCP2515_WriteRegisterWithTimeout(hdev,MCP2515_CNF1_MSG, CNF1, timeout) != MCP2515_OK)
        return MCP2515_SET_BAUDRATE_FAIL;

    if (MCP2515_WriteRegisterWithTimeout(hdev,MCP2515_CNF2_MSG, CNF2, timeout) != MCP2515_OK)
        return MCP2515_SET_BAUDRATE_FAIL;

    if (MCP2515_WriteRegisterWithTimeout(hdev,MCP2515_CNF3_MSG, CNF3, timeout) != MCP2515_OK)
        return MCP2515_SET_BAUDRATE_FAIL;

    // verifica scrittura
    uint8_t result_read_cnf1, result_read_cnf2, result_read_cnf3;
    uint8_t read_cnf1, read_cnf2, read_cnf3;
    // Leggi i registri CNF1, CNF2, CNF3
    result_read_cnf1 = MCP2515_ReadRegister(hdev,MCP2515_CNF1_MSG, &read_cnf1);
    result_read_cnf2 = MCP2515_ReadRegister(hdev,MCP2515_CNF2_MSG, &read_cnf2);
    result_read_cnf3 = MCP2515_ReadRegister(hdev,MCP2515_CNF3_MSG, &read_cnf3);

    if (result_read_cnf1 != MCP2515_OK ||
        result_read_cnf2 != MCP2515_OK ||
        result_read_cnf3 != MCP2515_OK) {
            return MCP2515_BAUDRATE_READ_FAIL;

        }

    // Confronta con i valori attesi
    if (read_cnf1 != CNF1 || 
        read_cnf2 != CNF2 || 
        read_cnf3 != CNF3) {
            return MCP2515_BAUDRATE_NOT_OK;
    }

    return MCP2515_OK;
}

uint8_t MCP2515_WriteRegisterWithTimeout(MCP2515_HandleTypeDef* hdev, uint8_t address, uint8_t value, uint32_t timeout){
    uint32_t startTime;
    
    uint8_t writeMessage[3];
    writeMessage[0] = MCP2515_WRITE;
    writeMessage[1] = address;
    writeMessage[2] = value;
    /*printf("value: 0x%02X\n", value);
    int arraySize = sizeof(writeMessage) / sizeof(writeMessage[0]);
    printByteArray(writeMessage, arraySize);*/

    
    HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_RESET);  // CS basso
    hdev->transmissionComplete = 0;
    HAL_SPI_Transmit_IT(hdev->hspi, writeMessage, 3);

    startTime = HAL_GetTick();
    while (!hdev->transmissionComplete) {
        if ((HAL_GetTick() - startTime) > timeout) {
            HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS alto
            return MCP2515_FAIL; // Timeout raggiunto
        }
    }
    hdev->transmissionComplete = 0;
    HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS alto
    
    return MCP2515_OK;
}

uint8_t MCP2515_WriteBitWithTimeout(MCP2515_HandleTypeDef* hdev, uint8_t address, uint8_t mask, uint8_t value, uint32_t timeout){
    uint32_t startTime;

    uint8_t writeMessage[3];
    writeMessage[0] = MCP2515_BIT_MODIFY;
    writeMessage[1] = address;
    writeMessage[2] = mask;
    writeMessage[3] = value;

    /*printf("value: 0x%02X\n", value);
    int arraySize = sizeof(writeMessage) / sizeof(writeMessage[0]);
    printByteArray(writeMessage, arraySize);*/


    HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_RESET);  // CS basso
    hdev->transmissionComplete = 0;
    HAL_SPI_Transmit_IT(hdev->hspi, writeMessage, 4);

    startTime = HAL_GetTick();
    while (!hdev->transmissionComplete) {
        if ((HAL_GetTick() - startTime) > timeout) {
            HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS alto
            return MCP2515_FAIL; // Timeout raggiunto
        }
    }
    hdev->transmissionComplete = 0;
    HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS alto

    return MCP2515_OK;
}


uint8_t MCP2515_ReadRegister(MCP2515_HandleTypeDef* hdev, uint8_t address, uint8_t* data){
    uint32_t startTime;
    const uint32_t timeout = 10; // Timeout di 10 ms
    uint8_t readMessage[2];
    readMessage[0] = MCP2515_READ;
    readMessage[1] = address;

    uint8_t dummyData = 0x00;  // Dato dummy per generare i clock necessari
    HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_RESET);  // CS basso
    hdev->transmissionComplete = 0;  // Resetta lo stato
    // Trasmetti il comando di lettura e l'indirizzo
    HAL_SPI_Transmit_IT(hdev->hspi, readMessage, 2);
    startTime = HAL_GetTick();
    // Attendi che la trasmissione sia completata
    while (!hdev->transmissionComplete) {
        if ((HAL_GetTick() - startTime) > timeout) {
            HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS alto
            printf("Timeout trasmission read");
            return MCP2515_READ_TIMEOUT_1; // Timeout raggiunto
        }
    }
    hdev->transmissionComplete = 0;  // Resetta lo stato
    // Ricevi il dato dal registro
    HAL_SPI_TransmitReceive_IT(hdev->hspi, &dummyData, data, 1);
    startTime = HAL_GetTick();
    // Attendi che la ricezione sia completata
    while (!hdev->transmissionComplete) {
        if ((HAL_GetTick() - startTime) > timeout) {
            HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS alto
            return MCP2515_READ_TIMEOUT_2; // Timeout raggiunto
        }
    }
    hdev->transmissionComplete = 0;
    HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS alto
    return MCP2515_OK;
}


uint8_t MCP2515_SetMode(MCP2515_HandleTypeDef* hdev, uint8_t mode) {
    const uint32_t timeout = 10; // Timeout di 10 ms
    uint8_t status_data;
    uint8_t canctrl_data;
    uint8_t valueToSend;

    // Leggi il valore corrente per preservare gli altri bit
    if (MCP2515_ReadRegister(hdev, MCP2515_CANCTRL_MSG, &canctrl_data) != MCP2515_OK)
        return MCP2515_FAIL;

    // Preserva gli altri bit e imposta quelli relativi alla trasmissione
    valueToSend = canctrl_data;
    valueToSend &= ~(0x7 << 5);
    valueToSend |= (mode << 5);

    if (MCP2515_WriteRegisterWithTimeout(hdev,MCP2515_CANCTRL_MSG, valueToSend, timeout) != MCP2515_OK)
        return MCP2515_FAIL;

    if (MCP2515_ReadRegister(hdev,MCP2515_CANSTAT_MSG, &status_data) != MCP2515_OK)
        return MCP2515_FAIL;
    // Verifica se il modo è stato impostato correttamente
    uint8_t currentMode = (status_data >> 5) & 0x07;
    if (currentMode == mode) {
        return MCP2515_OK;
    }else
        return MCP2515_FAIL; 
    
    return 
    MCP2515_FAIL;
}

uint8_t MCP2515_ResetInt(MCP2515_HandleTypeDef* hdev) {
	const uint32_t timeout = 5; // Timeout di 10 ms
	if (MCP2515_WriteRegisterWithTimeout(hdev,MCP2515_CANINTE_MSG, 0x0, timeout) != MCP2515_OK) // rivedere se si vogliono attivare altri enable
	   return MCP2515_FAIL;
	return MCP2515_OK;
}





uint8_t MCP2515_SetIntTx(MCP2515_HandleTypeDef* hdev) {
    const uint32_t timeout = 5; // Timeout di 10 ms
    uint8_t caninte_data;
    uint8_t intTx_value = 0x1C;
    
    // Leggi il valore corrente di CANINTE per preservare gli altri bit
    if (MCP2515_ReadRegister(hdev, MCP2515_CANINTE_MSG, &caninte_data) != MCP2515_OK)
        return MCP2515_FAIL;

    // Preserva gli altri bit e imposta quelli relativi alla trasmissione
    uint8_t valueToSend = caninte_data | intTx_value;

    if (MCP2515_WriteRegisterWithTimeout(hdev,MCP2515_CANINTE_MSG, valueToSend, timeout) != MCP2515_OK) // rivedere se si vogliono attivare altri enable
        return MCP2515_FAIL;

    if (MCP2515_ReadRegister(hdev,MCP2515_CANINTE_MSG, &caninte_data) != MCP2515_OK)
        return MCP2515_FAIL;

    // Controlla se i bit desiderati sono stati impostati
    if ((caninte_data & intTx_value) == intTx_value) {
        return MCP2515_OK;
    }else
        return MCP2515_FAIL; 

    return MCP2515_FAIL;
}

uint8_t MCP2515_LoadTXBuffer(MCP2515_HandleTypeDef* hdev, MCP2515_MessageBuffer* msgBuffer, uint8_t start) {

	static uint32_t start_time, timeWait, startTime;
	static uint8_t status_old;

	timeWait = HAL_GetTick() - start_time;
	switch (msgBuffer->status){

        case TRANSMISSION_IDLE:
            if (start == true){
                msgBuffer->status = TRANSMISSION_SET_VALUE;
                hdev->transmissionComplete = 0;
                hdev->emptyTXBuffer[msgBuffer->buffer] = false;
            }
            
            break;

        case TRANSMISSION_SET_VALUE:
            switch (msgBuffer->buffer){
                case MCP2515_TX_BUFFER_0:
                    msgBuffer->loadIDCmd = MCP2515_LOAD_TX_ID_0;
                    msgBuffer->loadDataCmd = MCP2515_LOAD_TX_DATA_0;
                    msgBuffer->txDLCAddress = MCP2515_TXB0DLC;
                    msgBuffer->txTXREQAddress = MCP2515_TXB0CTRL;
                    //printf("buffer0\n");
                    break;

                case MCP2515_TX_BUFFER_1:
                    msgBuffer->loadIDCmd = MCP2515_LOAD_TX_ID_1;
                    msgBuffer->loadDataCmd = MCP2515_LOAD_TX_DATA_1;
                    msgBuffer->txDLCAddress = MCP2515_TXB1DLC;
                    msgBuffer->txTXREQAddress = MCP2515_TXB1CTRL;
                    //printf("buffer1\n");
                    break;

                case MCP2515_TX_BUFFER_2:
                    msgBuffer->loadIDCmd = MCP2515_LOAD_TX_ID_2;
                    msgBuffer->loadDataCmd = MCP2515_LOAD_TX_DATA_2;
                    msgBuffer->txDLCAddress = MCP2515_TXB2DLC;
                    msgBuffer->txTXREQAddress = MCP2515_TXB2CTRL;
                    //printf("buffer2\n");
                    break;

                default:
                    break;
            }

            msgBuffer->status = TRANSMISSION_ID_CMD;
            hdev->transmissionComplete = 0;
            //break;

        case TRANSMISSION_ID_CMD:
            HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_RESET);  // CS basso
            hdev->transmissionComplete = 0;
            //printf("ID_ADDR: 0x%02X"\n, msgBuffer->loadIDCmd);
            HAL_SPI_Transmit_IT(hdev->hspi, &msgBuffer->loadIDCmd, 1);
            msgBuffer->status = TRANSMISSION_ID_VALUE;
            
            //break;

        case TRANSMISSION_ID_VALUE:

        	startTime = HAL_GetTick();
        	while (!hdev->transmissionComplete) {
        		if ((HAL_GetTick() - startTime)>=1){
					msgBuffer->status = TRANSMISSION_ERROR;
					return 100;
				}
        	}
        		if (hdev->transmissionComplete == 1) {
            	 // HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS basso
                hdev->transmissionComplete = 0;
                //HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_RESET);  // CS basso
                HAL_SPI_Transmit_IT(hdev->hspi, msgBuffer->idData, 4);
                msgBuffer->status = TRANSMISSION_DATA_CMD;
                if ((msgBuffer->idData[0] != 2) ||
                    				 ( msgBuffer->idData[1] != 0 &&
                    						 msgBuffer->idData[1] != 96 &&
                							 msgBuffer->idData[1] != 64 &&
                							 msgBuffer->idData[1] != 32)){
                    			  uint8_t aaa = 0;
                    		  }

            }
            /*if (timeWait>=1){
        		msgBuffer->status = TRANSMISSION_ERROR;
        		return 100;
        	}

            break;*/

        case TRANSMISSION_DATA_CMD:
        	startTime = HAL_GetTick();
        	while (!hdev->transmissionComplete) {
        		if ((HAL_GetTick() - startTime)>=1){
        						msgBuffer->status = TRANSMISSION_ERROR;
        						return 101;
        					}
        	        	}
             if (hdev->transmissionComplete == 1) {
            	HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS basso
                hdev->transmissionComplete = 0;
                HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_RESET);  // CS basso
                HAL_SPI_Transmit_IT(hdev->hspi, &msgBuffer->loadDataCmd, 1);
                msgBuffer->status = TRANSMISSION_DATA_VALUE;
            }
            /*if (timeWait>=1){
        	        		msgBuffer->status = TRANSMISSION_ERROR;
        	        		return 101;
        	        	}
            
            break;*/

        case TRANSMISSION_DATA_VALUE:
        	startTime = HAL_GetTick();
        	while (!hdev->transmissionComplete) {
        		if ((HAL_GetTick() - startTime)>=1){
        						msgBuffer->status = TRANSMISSION_ERROR;
        						return 102;
        					}
        	        	}

             if (hdev->transmissionComplete == 1) {
            	// HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS basso
                hdev->transmissionComplete = 0;
                /*uint8_t lenghtMsg = msgBuffer->length;
                if (lenghtMsg == 0){
                	lenghtMsg = 1;
                }*/

                HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_RESET);  // CS basso

                HAL_SPI_Transmit_IT(hdev->hspi, msgBuffer->data, msgBuffer->length);
                msgBuffer->status = TRANSMISSION_DLC;
            }
           /* if (timeWait>=1){
        	        		msgBuffer->status = TRANSMISSION_ERROR;
        	        		return 102;
        	        	}

            break;*/

        case TRANSMISSION_DLC:
        	startTime = HAL_GetTick();
        	while (!hdev->transmissionComplete) {
        		if ((HAL_GetTick() - startTime)>=1){
        						msgBuffer->status = TRANSMISSION_ERROR;
        						return 103;
        					}
        	        	}
             if (hdev->transmissionComplete == 1) {
                HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS basso
                writeMessage[0] = MCP2515_WRITE;
                writeMessage[1] = msgBuffer->txDLCAddress;
                if (msgBuffer->length <= 8)
                    writeMessage[2] = msgBuffer->length;
                else 
                    writeMessage[2] = 8;
                //printf("0: 0x%02X, 1: 0x%02X, 2: 0x%02X\n", writeMessage[0], writeMessage[1], writeMessage[2]);
                HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_RESET);  // CS basso
                hdev->transmissionComplete = 0;
                HAL_SPI_Transmit_IT(hdev->hspi, writeMessage, 3);
                msgBuffer->status = TRANSMISSION_TXREQ;
            }
           /* if (timeWait>=1){
        	        		msgBuffer->status = TRANSMISSION_ERROR;
        	        		return 103;
        	        	}

            break;*/

        case TRANSMISSION_TXREQ:
        	startTime = HAL_GetTick();
        	while (!hdev->transmissionComplete) {
        		if ((HAL_GetTick() - startTime)>=1){
        						msgBuffer->status = TRANSMISSION_ERROR;
        						return 104;
        					}
        	        	}

            if (hdev->transmissionComplete == 1) {
                HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS basso
                writeMessage[0] = MCP2515_BIT_MODIFY;
                writeMessage[1] = msgBuffer->txTXREQAddress;
                writeMessage[2] = MCP2515_TXREQ_MASK;
                writeMessage[3] = MCP2515_TXREQ_SET;

                HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_RESET);  // CS basso
                hdev->transmissionComplete = 0;
                HAL_SPI_Transmit_IT(hdev->hspi, writeMessage, 4);

                msgBuffer->status = TRANSMISSION_END;
                
            }
           /* if (timeWait>=1){
        	        		msgBuffer->status = TRANSMISSION_ERROR;
        	        		return 104;
        	        	}
            
            break;*/

        case TRANSMISSION_END:
        	startTime = HAL_GetTick();
        	while (!hdev->transmissionComplete) {
        		if ((HAL_GetTick() - startTime)>=1){
        						msgBuffer->status = TRANSMISSION_ERROR;
        						return 105;
        					}
        	        	}
        	 if (hdev->transmissionComplete == 1) {
                HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS alto
                hdev->transmissionComplete = 0;
                //uint8_t data;
                //printf("SendMsg\n");
                /*MCP2515_ReadRegister(hdev,0x30, &data);
                MCP2515_ReadRegister(hdev,0x31, &data);
                MCP2515_ReadRegister(hdev,0x32, &data);
                MCP2515_ReadRegister(hdev,0x33, &data);
                MCP2515_ReadRegister(hdev,0x34, &data);
                MCP2515_ReadRegister(hdev,0x35, &data);
                MCP2515_ReadRegister(hdev,0x36, &data);
                MCP2515_ReadRegister(hdev,0x37, &data);
                MCP2515_ReadRegister(hdev,0x38, &data);
                MCP2515_ReadRegister(hdev,0x39, &data);
                MCP2515_ReadRegister(hdev,0x3a, &data);
                MCP2515_ReadRegister(hdev,0x3b, &data);
                MCP2515_ReadRegister(hdev,0x3c, &data);
                MCP2515_ReadRegister(hdev,0x3d, &data);*/

                //MCP2515_ReadRegister(hdev,0x2A, &data);
                //MCP2515_ReadRegister(hdev,0x29, &data);
                //MCP2515_ReadRegister(hdev,0x28, &data);*/
                //HAL_Delay(100);
                //MCP2515_ReadRegister(hdev,0x2b, &data);
                //MCP2515_ReadRegister(hdev,0x2c, &data);
                //MCP2515_ReadRegister(hdev,MCP2515_TXB0CTRL, &data);
                //printf("Value: 0x%02X\n",data );*/

                msgBuffer->status = TRANSMISSION_RESET;


            }
        	/* if (timeWait>=1){
        	        		msgBuffer->status = TRANSMISSION_ERROR;
        	        		return 105;
        	        	}

            break;*/

        case TRANSMISSION_RESET:
        	msgBuffer->status = TRANSMISSION_IDLE;
        	break;

        case TRANSMISSION_ERROR:
            //printf("Errore trasmissione");
        	HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS alto
        	hdev->transmissionComplete = 0;
        	hdev->emptyTXBuffer[msgBuffer->buffer] = true;
            msgBuffer->status = TRANSMISSION_IDLE;
            
            break;

        default:
            msgBuffer->status = TRANSMISSION_IDLE;  
            break;
        
    }

	if (msgBuffer->status != status_old){
			status_old = msgBuffer->status;
			start_time = HAL_GetTick();
		}
    return 0;

}

uint8_t MCP2515_SendMessage(MCP2515_HandleTypeDef* hdev, MCP2515_MessageBuffer* msgBuffer, MCP2515_canMessage* canMessageTx){
	static uint8_t indexMsg = 0;
	uint8_t readyToSend = false;
	uint8_t result = 0;

    if (hdev->emptyTXBuffer[MCP2515_TX_BUFFER_0] ||
    		hdev->emptyTXBuffer[MCP2515_TX_BUFFER_1] ||
			hdev->emptyTXBuffer[MCP2515_TX_BUFFER_2]){
        readyToSend = true;
    }



    /*if (msgBuffer->status ==  TRANSMISSION_RESET ||
    		msgBuffer->status ==  TRANSMISSION_ERROR	){
    	canMessageTx[indexMsg].sending = false;
    	canMessageTx[indexMsg].newMsg = false;
    	indexMsg ++;
    	if (indexMsg >= BUFFER_TX_SPI){
    		indexMsg = 0;}
    }*/

    if (readyToSend && 
        hdev->hspi->State == HAL_SPI_STATE_READY &&
		msgBuffer->status ==  TRANSMISSION_IDLE &&
		canMessageTx[indexMsg].newMsg) {

    	if (hdev->emptyTXBuffer[MCP2515_TX_BUFFER_0]){
    		msgBuffer->buffer = MCP2515_TX_BUFFER_0;
		}
    	else if (hdev->emptyTXBuffer[MCP2515_TX_BUFFER_1]){
    		msgBuffer->buffer = MCP2515_TX_BUFFER_1;
		}
    	else if (hdev->emptyTXBuffer[MCP2515_TX_BUFFER_2]){
    		msgBuffer->buffer = MCP2515_TX_BUFFER_2;
		}
    	else{
    		msgBuffer->buffer = MCP2515_TX_BUFFER_NONE;
    	}

    	msgBuffer->idData = canMessageTx[indexMsg].msgID;
    	if ((msgBuffer->idData[0] != 2) ||
    				 ( msgBuffer->idData[1] != 0 &&
    						 msgBuffer->idData[1] != 96 &&
							 msgBuffer->idData[1] != 64 &&
							 msgBuffer->idData[1] != 32)){
    			  uint8_t aaa = 0;
    		  }
    	msgBuffer->data = canMessageTx[indexMsg].msgData;
    	msgBuffer->length = canMessageTx[indexMsg].dlc;
    	/*if (msgBuffer->length != 8){
    		if (canMessageTx[indexMsg].dlc != 8){
    			result = canMessageTx[indexMsg].dlc;}
    		else result=100;
    	}*/

		canMessageTx[indexMsg].sending = true;
		result = MCP2515_LoadTXBuffer(hdev, msgBuffer, true);
        readyToSend = false;
        canMessageTx[indexMsg].sending = false;
            	canMessageTx[indexMsg].newMsg = false;
            	indexMsg ++;
            	if (indexMsg >= BUFFER_TX_SPI){
            		indexMsg = 0;}
        //result = 0;
	}

    else{
        
    	result = MCP2515_LoadTXBuffer(hdev, msgBuffer, false);
        //printf("readyToSend: %d\n", readyToSend);
        //printf("SPI State: %d (HAL_SPI_STATE_READY = %d)\n", hdev->hspi->State, HAL_SPI_STATE_READY);
        //printf("Message Buffer Status: %d (TRANSMISSION_IDLE = %d)\n", msgBuffer->status, TRANSMISSION_IDLE);
    	/*if (canMessageTx[indexMsg].newMsg != 0)
    		result = 1;
    	else result = 2;*/
    }
    return result;
}


uint8_t MCP2515_InterruptHandler(MCP2515_HandleTypeDef* hdev, GPIO_PinState intFlag, MCP2515_MessageBuffer* msgBuffer){
	const uint8_t timeout = 10; // Timeout di 10 ms
    uint8_t result_read_TXBnCTRL[3] = {0x0,0x0,0x0};

    bool isIdle = (msgBuffer->status == TRANSMISSION_IDLE);
    bool isIntFlagSet = (intFlag==GPIO_PIN_RESET);
    bool isSpiReady = (hdev->hspi->State == HAL_SPI_STATE_READY);

    uint8_t TXxIF_val[3];

	uint8_t read_TXBnCTRL[3];
	uint8_t TXBnCTRL_addr[3] = {MCP2515_TXB0CTRL, MCP2515_TXB1CTRL, MCP2515_TXB2CTRL};
	uint8_t TXREQ_val[3];



    // Verifica che la trasmissione non sia in corso, che l'interrupt sia attivo, e che l'SPI sia pronto
    if (isIdle && isIntFlagSet && isSpiReady) {
        //printf("flag1: %d\n", *intFlag);
        // Resetta il flag dell'interrupt per evitare riattivazioni indesiderate

        //printf("flag2: %d\n", *intFlag);
        //printf("status: %d\n", msgBuffer->status);
        //printf("transmissionOn: %d\n", transmissionOn);
        // Legge il registro CANINTF per verificare quali interrupt sono attivi
        uint8_t result_read_canintf;
        uint8_t read_canintf;
        result_read_canintf = MCP2515_ReadRegister(hdev,MCP2515_CANINTF_MSG, &read_canintf);

        if (result_read_canintf != MCP2515_OK) {
            // Gestisci l'errore se la lettura fallisce
            //printf("Errore nella lettura del registro CANINTF\n");
            return 10;
        }

        //uint8_t canintf_val = read_canintf;


        for (uint8_t i=0; i < 3; i++){
            // Estrai il bit TXxIF corrispondente dal registro CANINTF
            TXxIF_val[i] = (read_canintf >> (i + 2)) & 0x01;
            //printf("TXxIF_val[%d]: %d\n", i, TXxIF_val[i]);

            if (TXxIF_val[i]) {

            	uint8_t mask = 1<<(2+i);

            	MCP2515_WriteBitWithTimeout(hdev, MCP2515_CANINTF_MSG, mask, 0x0, timeout);

                //canintf_val &= ~(1 << (2+i));
                // Se il bit TXxIF è impostato
                // Leggi il registro TXBnCTRL per verificare lo stato di TXREQ
                result_read_TXBnCTRL[i] = MCP2515_ReadRegister(hdev,TXBnCTRL_addr[i], &read_TXBnCTRL[i]);


                if (result_read_TXBnCTRL[i] != MCP2515_OK) {
                    // Gestisci l'errore se la lettura fallisce
                    //printf("Errore nella lettura del registro TXBnCTRL[%d]\n", i);
                    //emptyTXBuffer[i] = true;
                    //return result_read_TXBnCTRL[i];
                	/*HAL_Delay(2);
					result_read_TXBnCTRL[i] = MCP2515_ReadRegister(hdev,TXBnCTRL_addr[i], &read_TXBnCTRL[i]);
					TXREQ_val[i] = (read_canintf >> 3) & 0x01;
					//printf("TXREQ_val: %d\n", TXREQ_val[i]);

					if (!TXREQ_val[i]) { // Se TXREQ è 0, il buffer è vuoto
					   emptyTXBuffer[i] = true;

					}else
						emptyTXBuffer[i] = true;*/
                	hdev->emptyTXBuffer[i] = true;
                	return 11;
                }

                // Verifica lo stato di TXREQ
                TXREQ_val[i] = (read_TXBnCTRL[i] >> (3 + i)) & 0x01;
                //printf("TXREQ_val: %d\n", TXREQ_val[i]);

                if (!TXREQ_val[i]) { // Se TXREQ è 0, il buffer è vuoto
                	hdev->emptyTXBuffer[i] = true;

                }else{
                	//MCP2515_WriteRegisterWithTimeout(hdev,TXBnCTRL_addr[i], 0x0, timeout);
                	/*HAL_Delay(2);
                	result_read_TXBnCTRL[i] = MCP2515_ReadRegister(hdev,TXBnCTRL_addr[i], &read_TXBnCTRL[i]);
                	TXREQ_val[i] = (read_canintf >> 3) & 0x01;
                	                //printf("TXREQ_val: %d\n", TXREQ_val[i]);

                	                if (!TXREQ_val[i]) { // Se TXREQ è 0, il buffer è vuoto
                	                   emptyTXBuffer[i] = true;

                	                }else
                	                	emptyTXBuffer[i] = true;*/
                	hdev->emptyTXBuffer[i] = true;
                	return 12;
                }

            }
        }
        // reset dei flag interrupt
       // MCP2515_ReadRegister(hdev,MCP2515_CANINTF_MSG, &read_canintf);
        //canintf_val &= ~(0x7 << 2);
        //MCP2515_WriteRegisterWithTimeout(hdev,MCP2515_CANINTF_MSG, canintf_val, timeout);
    }/*else{

    	if (isIntFlagSet) {
			if (!isSpiReady && !isIdle) {
				return 13;
			} else if (!isSpiReady) {
				return 15;
			} else if (!isIdle) {
				return 16;
			} else {
				return 17;
			}
		} else {
			return 14;
		}
    }*/
    //*result = result_read_TXBnCTRL[1];
    return MCP2515_OK;

}

void initBuffer(MCP2515_MessageBuffer* msgBuffer){
	msgBuffer->buffer = MCP2515_TX_BUFFER_NONE;
	msgBuffer->idData = idDataEmpty;
    msgBuffer->data = dataEmpty;
    msgBuffer->length = 0;
    msgBuffer->loadIDCmd = 0x0;
    msgBuffer->loadDataCmd= 0x0;
    msgBuffer->txDLCAddress = 0x0;
    msgBuffer->txTXREQAddress = 0x0;
    msgBuffer->status = TRANSMISSION_IDLE;

}
