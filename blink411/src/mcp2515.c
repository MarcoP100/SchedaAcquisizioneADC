#include "mcp2515.h"

static uint8_t emptyTXBuffer[3] = {true, true,true};
uint8_t idDataEmpty[4] = {0x0, 0x0, 0x0, 0x0};
uint8_t dataEmpty[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,0x0};

// Costruttore
uint8_t MCP2515_Init(MCP2515_HandleTypeDef* hdev, GPIO_TypeDef* csPort,  uint16_t csPin, SPI_HandleTypeDef* hspi, uint8_t baudrate,uint8_t intTxEnable ) {
    hdev->csPin = csPin;
    hdev->csPort = csPort;
    hdev->hspi = hspi;
    
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


uint8_t MCP2515_SetIntTx(MCP2515_HandleTypeDef* hdev) {
    const uint32_t timeout = 10; // Timeout di 10 ms
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

void MCP2515_LoadTXBuffer(MCP2515_HandleTypeDef* hdev, MCP2515_MessageBuffer* msgBuffer, uint8_t start) {
    static uint8_t writeMessage[4];

    switch (msgBuffer->status){

        case TRANSMISSION_IDLE:
            if (start == 1){
                msgBuffer->status = TRANSMISSION_SET_VALUE;
                hdev->transmissionComplete = 0;
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
            break;

        case TRANSMISSION_ID_CMD:
            HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_RESET);  // CS basso
            hdev->transmissionComplete = 0;
            //printf("ID_ADDR: 0x%02X"\n, msgBuffer->loadIDCmd);
            HAL_SPI_Transmit_IT(hdev->hspi, &msgBuffer->loadIDCmd, 1);
            msgBuffer->status = TRANSMISSION_ID_VALUE;
            
            break;

        case TRANSMISSION_ID_VALUE:
            
             if (hdev->transmissionComplete == 1) {
                hdev->transmissionComplete = 0;
                HAL_SPI_Transmit_IT(hdev->hspi, msgBuffer->idData, 4);
                msgBuffer->status = TRANSMISSION_DATA_CMD;
            }
            
            break;

        case TRANSMISSION_DATA_CMD:
            
             if (hdev->transmissionComplete == 1) {
                hdev->transmissionComplete = 0;
                HAL_SPI_Transmit_IT(hdev->hspi, &msgBuffer->loadDataCmd, 1);
                msgBuffer->status = TRANSMISSION_DATA_VALUE;
            }
            
            break;

        case TRANSMISSION_DATA_VALUE:
            
             if (hdev->transmissionComplete == 1) {
                hdev->transmissionComplete = 0;
                HAL_SPI_Transmit_IT(hdev->hspi, msgBuffer->data, msgBuffer->length);
                msgBuffer->status = TRANSMISSION_DLC;
            }
            
            break;

        case TRANSMISSION_DLC:
            
            
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
            
            break;

        case TRANSMISSION_TXREQ:
            
            
            
            if (hdev->transmissionComplete == 1) {
                HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS basso
                writeMessage[0] = MCP2515_BIT_MODIFY;
                writeMessage[1] = msgBuffer->txTXREQAddress;
                writeMessage[2] = MCP2515_TXREQ_MASK;
                writeMessage[3] = MCP2515_TXREQ_SET;

                HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_RESET);  // CS basso
                hdev->transmissionComplete = 0;
                HAL_SPI_Transmit_IT(hdev->hspi, writeMessage, 4);
                emptyTXBuffer[msgBuffer->buffer] = false;
                msgBuffer->status = TRANSMISSION_END;
                
            }
            
            break;

        case TRANSMISSION_END:
            if (hdev->transmissionComplete == 1) {
                HAL_GPIO_WritePin(hdev->csPort, hdev->csPin, GPIO_PIN_SET);  // CS alto
                hdev->transmissionComplete = 0;
                //uint8_t data;
                
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
                MCP2515_ReadRegister(hdev,0x3d, &data);

                //MCP2515_ReadRegister(hdev,0x2A, &data);
                //MCP2515_ReadRegister(hdev,0x29, &data);
                //MCP2515_ReadRegister(hdev,0x28, &data);*/
                HAL_Delay(100);
                //MCP2515_ReadRegister(hdev,0x2b, &data);
                //MCP2515_ReadRegister(hdev,0x2c, &data);
                //MCP2515_ReadRegister(hdev,MCP2515_TXB0CTRL, &data);
                //printf("Value: 0x%02X\n",data );*/
                //printf("SendMsg\n");
                msgBuffer->status = TRANSMISSION_IDLE;
            }
            break;

        case TRANSMISSION_ERROR:
            printf("Errore trasmissione");
            msgBuffer->status = TRANSMISSION_IDLE;
            
            break;

        default:
            msgBuffer->status = TRANSMISSION_IDLE;  
            break;
        
    }

    return;

}

void MCP2515_SendMessage(MCP2515_HandleTypeDef* hdev, MCP2515_MessageBuffer* msgBuffer, uint8_t* data, uint8_t* msgID){

    uint8_t readyToSend = false;
    if (emptyTXBuffer[0]){
        readyToSend = true; 
    }
   
    if (readyToSend && 
        hdev->hspi->State == HAL_SPI_STATE_READY && 
        msgBuffer->status ==  TRANSMISSION_IDLE) {
        msgBuffer->buffer = MCP2515_TX_BUFFER_0;
        msgBuffer->data = data;
        msgBuffer->idData = msgID;
        msgBuffer->length = 8;
        MCP2515_LoadTXBuffer(hdev, msgBuffer, 1);
        readyToSend = 0;
    }
    else{
        
       MCP2515_LoadTXBuffer(hdev, msgBuffer, 0);
        //printf("readyToSend: %d\n", readyToSend);
        //printf("SPI State: %d (HAL_SPI_STATE_READY = %d)\n", hdev->hspi->State, HAL_SPI_STATE_READY);
        //printf("Message Buffer Status: %d (TRANSMISSION_IDLE = %d)\n", msgBuffer->status, TRANSMISSION_IDLE);

    }
}


void MCP2515_InterruptHandler(MCP2515_HandleTypeDef* hdev, uint8_t* intFlag, MCP2515_MessageBuffer* msgBuffer){
    uint8_t  transmissionOn = false;
    if (msgBuffer->status !=  TRANSMISSION_IDLE) {
        transmissionOn = true;
    }

    // Verifica che la trasmissione non sia in corso, che l'interrupt sia attivo, e che l'SPI sia pronto
    if (!transmissionOn && *intFlag && hdev->hspi->State == HAL_SPI_STATE_READY){
        //printf("flag1: %d\n", *intFlag);
        // Resetta il flag dell'interrupt per evitare riattivazioni indesiderate
        *intFlag = false;
        //printf("flag2: %d\n", *intFlag);
        // Legge il registro CANINTF per verificare quali interrupt sono attivi
        uint8_t result_read_canintf;
        uint8_t read_canintf;
        result_read_canintf = MCP2515_ReadRegister(hdev,MCP2515_CANINTF_MSG, &read_canintf);

        if (result_read_canintf != MCP2515_OK) {
            // Gestisci l'errore se la lettura fallisce
            printf("Errore nella lettura del registro CANINTF\n");
            return;
        }

        uint8_t canintf_val = read_canintf;
        uint8_t TXxIF_val[3];
        uint8_t result_read_TXBnCTRL[3];
        uint8_t read_TXBnCTRL[3];
        uint8_t TXBnCTRL_addr[3] = {MCP2515_TXB0CTRL, MCP2515_TXB1CTRL, MCP2515_TXB2CTRL};
        uint8_t TXREQ_val[3];

        for (uint8_t i=0; i < 3; i++){
            // Estrai il bit TXxIF corrispondente dal registro CANINTF
            TXxIF_val[i] = (read_canintf >> (i + 2)) & 0x01;
            //printf("TXxIF_val[%d]: %d\n", i, TXxIF_val[i]);

            if (TXxIF_val[i]) {
                canintf_val &= ~(1 << (2+i));
                // Se il bit TXxIF è impostato
                // Leggi il registro TXBnCTRL per verificare lo stato di TXREQ
                result_read_TXBnCTRL[i] = MCP2515_ReadRegister(hdev,TXBnCTRL_addr[i], &read_TXBnCTRL[i]);

                if (result_read_TXBnCTRL[i] != MCP2515_OK) {
                    // Gestisci l'errore se la lettura fallisce
                    printf("Errore nella lettura del registro TXBnCTRL[%d]\n", i);
                    return;
                }

                // Verifica lo stato di TXREQ
                TXREQ_val[i] = (read_canintf >> 3) & 0x01;
                //printf("TXREQ_val: %d\n", TXREQ_val[i]);

                if (!TXREQ_val[i]) { // Se TXREQ è 0, il buffer è vuoto
                   emptyTXBuffer[i] = true;        
                }
            }
        }
        
        // reset dei flag interrupt
        const uint8_t timeout = 10; // Timeout di 10 ms
        canintf_val &= ~(0x7 << 2);
        MCP2515_WriteRegisterWithTimeout(hdev,MCP2515_CANINTF_MSG, canintf_val, timeout);
    }

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