#ifndef MCP2515_H
#define MCP2515_H

#include "stm32f4xx_hal.h"
#include <stdio.h>


#define MCP2515_CNF1_MSG        0x2A
#define MCP2515_CNF2_MSG        0x29
#define MCP2515_CNF3_MSG        0x28
#define MCP2515_CANCTRL_MSG     0x0F
#define MCP2515_CANSTAT_MSG     0x0E


// impostazioni per clock
#define MCP2515_SJW_DATA_125kbps        0
#define MCP2515_BRP_DATA_125kbps        3
#define MCP2515_BTLMODE_DATA_125kbps    1
#define MCP2515_SAM_DATA_125kbps        1
#define MCP2515_PHSEG1_DATA_125kbps     6
#define MCP2515_PRSEG_DATA_125kbps      0
#define MCP2515_SOF_DATA_125kbps        1
#define MCP2515_WAKFIL_DATA_125kbps     0
#define MCP2515_PHSEG2_DATA_125kbps     6

#define MCP2515_SJW_DATA_250kbps        0
#define MCP2515_BRP_DATA_250kbps        1
#define MCP2515_BTLMODE_DATA_250kbps    1
#define MCP2515_SAM_DATA_250kbps        1
#define MCP2515_PHSEG1_DATA_250kbps     6
#define MCP2515_PRSEG_DATA_250kbps      0
#define MCP2515_SOF_DATA_250kbps        1
#define MCP2515_WAKFIL_DATA_250kbps     0
#define MCP2515_PHSEG2_DATA_250kbps     6

#define MCP2515_SJW_DATA_500kbps        0
#define MCP2515_BRP_DATA_500kbps        0
#define MCP2515_BTLMODE_DATA_500kbps    1
#define MCP2515_SAM_DATA_500kbps        1
#define MCP2515_PHSEG1_DATA_500kbps     6
#define MCP2515_PRSEG_DATA_500kbps      0
#define MCP2515_SOF_DATA_500kbps        1
#define MCP2515_WAKFIL_DATA_500kbps     0
#define MCP2515_PHSEG2_DATA_500kbps     6

#define MCP2515_SJW_DATA_1Mbps          0
#define MCP2515_BRP_DATA_1Mbps          0
#define MCP2515_BTLMODE_DATA_1Mbps      1
#define MCP2515_SAM_DATA_1Mbps          1
#define MCP2515_PHSEG1_DATA_1Mbps       3
#define MCP2515_PRSEG_DATA_1Mbps        0
#define MCP2515_SOF_DATA_1Mbps          1
#define MCP2515_WAKFIL_DATA_1Mbps       0
#define MCP2515_PHSEG2_DATA_1Mbps       0

// Definizione dei comandi SPI MCP2515
#define MCP2515_RESET       0xC0
#define MCP2515_READ        0x03
#define MCP2515_WRITE       0x02
#define MCP2515_BIT_MODIFY  0x05
#define MCP2515_READ_STATUS 0xA0
#define MCP2515_RX_STATUS   0xB0
#define MCP2515_RTS_TX0     0x81
#define MCP2515_RTS_TX1     0x82
#define MCP2515_RTS_TX2     0x84
#define MCP2515_LOAD_TX_ID_0     0x40
#define MCP2515_LOAD_TX_DATA_0   0x41
#define MCP2515_LOAD_TX_ID_1     0x42
#define MCP2515_LOAD_TX_DATA_1   0x43
#define MCP2515_LOAD_TX_ID_2     0x44
#define MCP2515_LOAD_TX_DATA_2   0x45

#define MCP2515_OK                  0
#define MCP2515_FAIL                1
#define MCP2515_RESET_FAIL          2
#define MCP2515_SET_BAUDRATE_FAIL   3
#define MCP2515_SET_MODE_FAIL       4
#define MCP2515_RESET_TIMEOUT       5
#define MCP2515_READ_TIMEOUT_1      6
#define MCP2515_READ_TIMEOUT_2      7
#define MCP2515_BAUDRATE_READ_FAIL  8
#define MCP2515_BAUDRATE_NOT_OK     9


#define MCP2515_NORMAL_MODE         0 
#define MCP2515_SLEEP_MODE          1 
#define MCP2515_LOOPBACK_MODE       2 
#define MCP2515_LISTENONLY_MODE     3
#define MCP2515_CONFIGURATION_MODE  4 

#define CAN_BAUDRATE_125kbps        0
#define CAN_BAUDRATE_250kbps        1
#define CAN_BAUDRATE_500kbps        2
#define CAN_BAUDRATE_1Mbps          3

#define MCP2515_TX_BUFFER_0         0
#define MCP2515_TX_BUFFER_1         1
#define MCP2515_TX_BUFFER_2         2

#define MCP2515_TXB0CTRL            0x30
#define MCP2515_TXB1CTRL            0x40
#define MCP2515_TXB2CTRL            0x50
#define MCP2515_TXB0DLC            0x35
#define MCP2515_TXB1DLC            0x45
#define MCP2515_TXB2DLC            0x55

#define MCP2515_TXREQ               3

enum TransmissionState {
    TRANSMISSION_IDLE,       // Valore 0
    TRANSMISSION_SET_VALUE,
    TRANSMISSION_ID_CMD, // Valore 1
    TRANSMISSION_ID_VALUE,     // Valore 2
    TRANSMISSION_DATA_CMD,        // Valore 3
    TRANSMISSION_DATA_VALUE,        // Valore 3
    TRANSMISSION_DLC,        // Valore 3
    TRANSMISSION_TXREQ,        // Valore 3
    TRANSMISSION_END,
    TRANSMISSION_ERROR        // Valore 3
};





// Definizione della struttura MCP2515_HandleTypeDef
typedef struct {
    GPIO_TypeDef* csPort;
    uint16_t csPin;
    SPI_HandleTypeDef* hspi;
    uint8_t transmissionComplete;
} MCP2515_HandleTypeDef;

typedef struct {
    uint8_t buffer;
    uint8_t* idData;
    uint8_t* data;
    uint8_t length;
    uint8_t loadIDCmd;
    uint8_t loadDataCmd;
    uint8_t txDLCAddress;
    uint8_t txTXREQAddress;

    enum TransmissionState status;
} MCP2515_MessageBuffer;

// Definizione delle funzioni
uint8_t MCP2515_Init(MCP2515_HandleTypeDef* hdev, GPIO_TypeDef* csPort, uint16_t csPin, SPI_HandleTypeDef* hspi, uint8_t baudrate);
uint8_t MCP2515_Reset(MCP2515_HandleTypeDef* hdev);
uint8_t MCP2515_deviceInit(MCP2515_HandleTypeDef* hdev, uint8_t baudrate);
uint8_t MCP2515_SetMode(MCP2515_HandleTypeDef* hdev, uint8_t mode);
uint8_t MCP2515_SetBaudrate(MCP2515_HandleTypeDef* hdev, uint8_t baudrate);
uint8_t MCP2515_WriteRegisterWithTimeout(MCP2515_HandleTypeDef* hdev, uint8_t address, uint8_t value, uint32_t timeout);
uint8_t MCP2515_ReadRegister(MCP2515_HandleTypeDef* hdev, uint8_t address, uint8_t* data);
void MCP2515_SetTransmissionComplete(MCP2515_HandleTypeDef* hdev, uint8_t transmissionComplete);
void MCP2515_LoadTXBuffer(MCP2515_HandleTypeDef* hdev, MCP2515_MessageBuffer* msgBuffer, uint8_t start);
void MCP2515_SendMessage(MCP2515_HandleTypeDef* hdev, MCP2515_MessageBuffer* msgBuffer, uint8_t readyToSend, uint8_t* data, uint8_t* msgID);

#endif