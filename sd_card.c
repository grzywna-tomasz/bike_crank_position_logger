/* Description of module (if needed) */
/* TODO remove not necessary call to HAL and move it to cfg */
/**********************************INCLUDE*************************************/
#include <string.h>
#include "sd_card_cfg.h"
/* TODO DET to be in separate module */
#include "main.h"
/* TODO */
#include "stm32f4xx_hal.h"
/**********************************DEFINES*************************************/
/* TODO remove SD_CARD_SPI_MAX_TIMEOUT */
#define SD_CARD_SPI_MAX_TIMEOUT             (HAL_MAX_DELAY - 1U)
#define SD_CARD_DUMMY_INIT_CLOCKS_LENGTH    (10U)
/* Just a random value */
#define SD_CARD_START_TOKEN_AWAITING_CYCLES (1000U)
/* Just a random value */
#define SD_CARD_BUSY_AWAITING_CYCLES        (1000U)

#define SD_CARD_COMMAND_INDEX_LENGTH        (1U)
#define SD_CARD_COMMAND_ARGUMENT_LENGTH     (4U)
#define SD_CARD_COMMAND_CRC_LENGTH          (1U)
#define SD_CARD_COMMAND_TX_LENGTH           (SD_CARD_COMMAND_INDEX_LENGTH + SD_CARD_COMMAND_ARGUMENT_LENGTH + SD_CARD_COMMAND_CRC_LENGTH)
/* Max number of frames SD card need to send response. Response byte not included */
#define SD_CARD_COMMAND_NCR_MAX_LENGTH      (8U)
#define SD_CARD_COMMAND_R1_RESP_LENGTH      (1U)
/* Macro used fir transmit and receive function. This will send command and await response */
#define SD_CARD_COMMAND_RESP_TOTAL_LENGTH   (SD_CARD_COMMAND_TX_LENGTH + SD_CARD_COMMAND_NCR_MAX_LENGTH + SD_CARD_COMMAND_R1_RESP_LENGTH)

#define SD_CARD_BUS_BUSY_VALUE          (0x00U)
#define SD_CARD_BUS_IDLE_VALUE          (0xFFU)
#define SD_CARD_START_TOKEN             (0xFEU)
#define SD_CARD_WRITE_RESPONSE_ACCEPTED (0b00000101)
#define SD_CARD_WRITE_RESPONSE_MASK     (0b00011111)


#define SD_CARD_IDLE_RESP       (0x00U)
#define SD_CARD_POWER_UP_BUSY   (0x01U)
#define SD_CARD_ERASE_RESET     (0x02U)
#define SD_CARD_ILLEGAL_COMMAND (0x04U)
#define SD_CARD_CRC_ERR         (0x08U)
#define SD_CARD_ERASE_ERROR     (0x10U)
#define SD_CARD_ADDRESS_ERROR   (0x20U)
#define SD_CARD_PARAMETER_ERROR (0x20U)

/* GO_IDLE_STATE */
#define SD_CARD_CMD0        (0U)
#define SD_CARD_CMD0_ARG    (0x00000000U)
/* CMD0 need CRC to be properly processed. I am using constant as I am not planning to use CRC in the project */
#define SD_CARD_CMD0_CRC    (0x95U)

/* SEND_IF_COND - verify SD Memory Card interface operating condition.*/
#define SD_CARD_CMD8        (8U)
#define SD_CARD_CMD8_ARG    (0x000001AAU)
/* CMD8 need CRC to be properly processed. I am using constant as I am not planning to use CRC in the project */
#define SD_CARD_CMD8_CRC    (0x87U)

/* Read CSD register */
#define SD_CARD_CMD9        (9U)
#define SD_CARD_CMD9_ARG    (0x00000000U)

#define SD_CARD_CSD_REG_LENGTH  (16U)

#define SD_CARD_CMD_NO_CRC  (0x00U)

/* SD_SEND_OP_COND - Initiate initialization process */
#define SD_CARD_CMD41       (41U)
#define SD_CARD_CMD41_ARG   (0x40000000U)

/* APP_CMD - Indicate the next command is APP_CMD */
#define SD_CARD_CMD55       (55U)
#define SD_CARD_CMD55_ARG   (0x00000000U)

/* Read Single block */
#define SD_CARD_CMD17       (17U)
/* Write Single block */
#define SD_CARD_CMD24       (24U)


#define SD_CARD_CSD_VERSION1    (0U)

/* TODO */
extern SPI_HandleTypeDef hspi1;
/* TODO move it to cfg.c  */
extern const uint8_t SDCard_TxDummyDataBuffer[1024U];

/**********************************TYPEDEFS************************************/

/**********************************PROTOTYPES**********************************/
static Std_ReturnType SDCard_SendCommand(uint8_t *TxDataPtr, uint16_t TxDataSize, uint8_t *RxDataPtr, uint16_t RxDataSize);
static void SDCard_CommandFillBuffer(uint8_t *TxBuffer, uint8_t Cmd, uint32_t Arg, uint8_t Crc);
/**********************************OBJECTS*************************************/
static uint8_t SDCard_MaxSpeed = 0;
static uint32_t SDCard_BlockLength = 0;
static uint32_t SDCard_BlockAddressMultiplier = 0;
static uint32_t SDCard_CSize = 0;
static uint32_t SDCard_CSizeMult = 0;
static uint32_t SDCard_SectorCount = 0;
static uint8_t SDCard_LastErrorResponse = 0;

/* TODO check what size is really needed */
static uint8_t SDCard_TxDataBuffer[1024U];
static uint8_t SDCard_RxDataBuffer[1024U];


/**********************************DEFINITIONS*********************************/
/* Initialize command data in buffer */
static void SDCard_CommandFillBuffer(uint8_t *TxBuffer, uint8_t Cmd, uint32_t Arg, uint8_t Crc)
{
    TxBuffer[0] = Cmd | 0x40;
    TxBuffer[1] = (uint8_t)(Arg >> 24);
    TxBuffer[2] = (uint8_t)(Arg >> 16);
    TxBuffer[3] = (uint8_t)(Arg >> 8);
    TxBuffer[4] = (uint8_t)(Arg);
    TxBuffer[5] = Crc;
}

static Std_ReturnType SDCard_SendCommand(uint8_t *TxDataPtr, uint16_t TxDataSize, uint8_t *RxDataPtr, uint16_t RxDataSize)
{
    uint16_t data_length = SD_CARD_COMMAND_RESP_TOTAL_LENGTH + TxDataSize + RxDataSize;
    
    /* Bytes used for reception should be cleaned (we transmit and receive at the same time) */
    memset(&TxDataPtr[SD_CARD_COMMAND_TX_LENGTH], SD_CARD_BUS_IDLE_VALUE, data_length - SD_CARD_COMMAND_TX_LENGTH);
    return SDCard_TransmitReceive(TxDataPtr, RxDataPtr, data_length);
}

static Std_ReturnType SDCard_ReadR1(uint8_t *RxDataPtr, uint16_t RxStartSearchIndex, uint16_t *R1RespIndex)
{
    Std_ReturnType ret_value = E_NOT_OK;
    /* Check response buffer for positive response. Skip bytes received at transmission, those will be always 0xFF.  */
    for (uint8_t index = RxStartSearchIndex; index < RxStartSearchIndex + SD_CARD_COMMAND_NCR_MAX_LENGTH; index++)
    {
        if (SD_CARD_BUS_IDLE_VALUE != RxDataPtr[index])
        {
            /* Response found */
            ret_value = E_OK;
            *R1RespIndex = index;
            break;
        }
    }
    return ret_value;
}

static Std_ReturnType SDCard_CheckR1PowerUpBusy(uint8_t *RxDataPtr, uint16_t RxStartSearchIndex, uint16_t *R1RespIndex)
{
    Std_ReturnType ret_val = E_OK;
    if (E_OK == SDCard_ReadR1(RxDataPtr, SD_CARD_COMMAND_TX_LENGTH, R1RespIndex))
    {
        if (SD_CARD_POWER_UP_BUSY != RxDataPtr[*R1RespIndex])
        {
            SDCard_LastErrorResponse = RxDataPtr[*R1RespIndex];
            DET_ErrorReception();
            ret_val = E_NOT_OK;
        }
    }
    return ret_val;
}

static Std_ReturnType SDCard_CheckR1PosResp(uint8_t *RxDataPtr, uint16_t RxStartSearchIndex, uint16_t *R1RespIndex)
{
    Std_ReturnType ret_val = E_OK;
    if (E_OK == SDCard_ReadR1(RxDataPtr, SD_CARD_COMMAND_TX_LENGTH, R1RespIndex))
    {
        if (SD_CARD_IDLE_RESP != RxDataPtr[*R1RespIndex])
        {
            SDCard_LastErrorResponse = RxDataPtr[*R1RespIndex];
            // DET_ErrorReception();
            ret_val = E_NOT_OK;
        }
    }
    return ret_val;
}

Std_ReturnType SDCard_InitializeCard(void)
{
    /* TODO use it everywhere. And fix function used */
    uint8_t temporary_buffer[SD_CARD_COMMAND_TX_LENGTH];
    uint16_t response_r1_index = 0;
    Std_ReturnType status = E_OK;
    /* According to information online we should wait 1ms before attempting anything */
    SD_CARD_DELAY_MS(1);

    /* Send at least 74 dummy clocks < 10 frames */
    memset(SDCard_TxDataBuffer, SD_CARD_BUS_IDLE_VALUE, SD_CARD_DUMMY_INIT_CLOCKS_LENGTH);
    status = SDCard_TransmitReceive(SDCard_TxDataBuffer, SDCard_RxDataBuffer, SD_CARD_DUMMY_INIT_CLOCKS_LENGTH);

    if (E_OK == status)
    {
        /* Send CMD0 - Software Reset */
        SDCard_CommandFillBuffer(SDCard_TxDataBuffer, SD_CARD_CMD0, SD_CARD_CMD0_ARG, SD_CARD_CMD0_CRC);
        status = SDCard_SendCommand(SDCard_TxDataBuffer, 0, SDCard_RxDataBuffer, 0);
    
        if (E_OK == status)
        {
            status = SDCard_CheckR1PowerUpBusy(SDCard_RxDataBuffer, SD_CARD_COMMAND_TX_LENGTH, &response_r1_index);
        }
    }

    if (E_OK == status)
    {
        /* Send CMD8 - Check voltage level supported. This is done to check card version */
        SDCard_CommandFillBuffer(SDCard_TxDataBuffer, SD_CARD_CMD8, SD_CARD_CMD8_ARG, SD_CARD_CMD8_CRC);
        status = SDCard_SendCommand(SDCard_TxDataBuffer, 0, SDCard_RxDataBuffer, 0);
        if (E_OK == status)
        {
            status = SDCard_CheckR1PowerUpBusy(SDCard_RxDataBuffer, SD_CARD_COMMAND_TX_LENGTH, &response_r1_index);
            if ((E_NOT_OK == status) && (SD_CARD_ILLEGAL_COMMAND == SDCard_RxDataBuffer[response_r1_index]))
            {
                /* SD Version No.1, add implementaion for it */
                DET_ErrorReception();
            }
        }
    }

    while (1)
    {
        if (E_OK == status)
        {
            /* Next command is APP_CMD */
            SDCard_CommandFillBuffer(SDCard_TxDataBuffer, SD_CARD_CMD55, SD_CARD_CMD55_ARG, SD_CARD_CMD_NO_CRC);
            status = SDCard_SendCommand(SDCard_TxDataBuffer, 0, SDCard_RxDataBuffer, 0);
            if (E_OK == status)
            {
                status = SDCard_CheckR1PowerUpBusy(SDCard_RxDataBuffer, SD_CARD_COMMAND_TX_LENGTH, &response_r1_index);
            }
        }

        if (E_OK == status)
        {
            /* Next command is APP_CMD */
            SDCard_CommandFillBuffer(SDCard_TxDataBuffer, SD_CARD_CMD41, SD_CARD_CMD41_ARG, SD_CARD_CMD_NO_CRC);
            status = SDCard_SendCommand(SDCard_TxDataBuffer, 0, SDCard_RxDataBuffer, 0);
            if (E_OK == status)
            {
                if (E_OK== SDCard_CheckR1PosResp(SDCard_RxDataBuffer, SD_CARD_COMMAND_TX_LENGTH, &response_r1_index))
                {
                    /* Positive response received, card ready to be used */
                    break;
                }
            }
        }
    }

    /* Check Data size, max speed */
    if (E_OK == status)
    {
        uint8_t csd_data[SD_CARD_CSD_REG_LENGTH];
        SDCard_CommandFillBuffer(temporary_buffer, SD_CARD_CMD9, SD_CARD_CMD9_ARG, SD_CARD_CMD_NO_CRC);
        SD_CARD_CS_LOW();
        HAL_SPI_Transmit(&hspi1, temporary_buffer, SD_CARD_COMMAND_TX_LENGTH, SD_CARD_SPI_MAX_TIMEOUT);

        /* TODO put it in function as ReadBlock use this */
        /* Wait for R1 response */
        for (uint8_t index = 0; index < SD_CARD_COMMAND_NCR_MAX_LENGTH; index++)
        {
            HAL_SPI_TransmitReceive(&hspi1, SDCard_TxDummyDataBuffer, temporary_buffer, 1, SD_CARD_SPI_MAX_TIMEOUT);
            if (SD_CARD_BUS_IDLE_VALUE != temporary_buffer[0])
            {
                /* Positive Response */
                break;
            }
        }

        /* TODO put it in function as ReadBlock use this */
        /* Wait for Start Token */
        for (uint8_t index = 0; index < SD_CARD_START_TOKEN_AWAITING_CYCLES; index++)
        {
            HAL_SPI_TransmitReceive(&hspi1, SDCard_TxDummyDataBuffer, temporary_buffer, 1, SD_CARD_SPI_MAX_TIMEOUT);
            if (SD_CARD_BUS_IDLE_VALUE != temporary_buffer[0])
            {
                /* Positive Response */
                break;
            }
        }

        /* TODO check if CRC is 2byte and received at all */
        HAL_SPI_TransmitReceive(&hspi1, SDCard_TxDummyDataBuffer, csd_data, SD_CARD_CSD_REG_LENGTH + 2U, SD_CARD_SPI_MAX_TIMEOUT);

        SD_CARD_CS_HIGH();

        /* Check version */
        if (SD_CARD_CSD_VERSION1 == (csd_data[0] >> 6))
        {
            SDCard_MaxSpeed = csd_data[3];
            SDCard_BlockLength = 1 << (csd_data[5] & 0x0F);
            SDCard_CSize = ((csd_data[6] & 0x03) << 10) | (csd_data[7] << 2) | ((csd_data[8] & 0xC0) >> 6);
            SDCard_CSizeMult = ((csd_data[9] & 0x03) << 1) | ((csd_data[10] & 0x80) >> 7);
            SDCard_SectorCount = (SDCard_CSize + 1) * (1 << (SDCard_CSizeMult + 2));
            /* Card is Version 1, therefore the adress passed during write command need to be multiplied */
            SDCard_BlockAddressMultiplier = SDCard_BlockLength;
        }
        else
        {
            SDCard_CSize = ((csd_data[7] & 0x3F) << 16) | (csd_data[8] << 8) | csd_data[9];
            SDCard_SectorCount = (SDCard_CSize + 1) * 1024;
            /* Card is Version 2, therefore the adress used during write is already block adress */
            SDCard_BlockAddressMultiplier = 1;
        }
    }

    /* TODO add Write size length change */

    return status;
}

uint8_t SDCard_ReadSingleBlock(uint32_t block_addr, uint8_t* buffer)
{
    /* Buffer used for transmitting command data and checking bus for response */
    uint8_t temporary_buffer[SD_CARD_COMMAND_TX_LENGTH];
    SDCard_CommandFillBuffer(temporary_buffer, SD_CARD_CMD17, block_addr * SDCard_BlockAddressMultiplier, SD_CARD_CMD_NO_CRC);
    SD_CARD_CS_LOW();
    HAL_SPI_Transmit(&hspi1, temporary_buffer, SD_CARD_COMMAND_TX_LENGTH, SD_CARD_SPI_MAX_TIMEOUT);

    /* Wait for R1 response */
    for (uint8_t index = 0; index < SD_CARD_COMMAND_NCR_MAX_LENGTH; index++)
    {
        HAL_SPI_TransmitReceive(&hspi1, SDCard_TxDummyDataBuffer, temporary_buffer, 1, SD_CARD_SPI_MAX_TIMEOUT);
        if (SD_CARD_BUS_IDLE_VALUE != temporary_buffer[0])
        {
            /* Positive Response */
            break;
        }
    }
    if (SD_CARD_IDLE_RESP != temporary_buffer[0])
    {
        /* TODO clean this up properly */
        return E_NOT_OK;
    }

    /* Wait for Start Token */
    for (uint8_t index = 0; index < SD_CARD_START_TOKEN_AWAITING_CYCLES; index++)
    {
        HAL_SPI_TransmitReceive(&hspi1, SDCard_TxDummyDataBuffer, temporary_buffer, 1, SD_CARD_SPI_MAX_TIMEOUT);
        if (SD_CARD_BUS_IDLE_VALUE != temporary_buffer[0])
        {
            /* Positive Response */
            break;
        }
    }
    if (SD_CARD_START_TOKEN != temporary_buffer[0])
    {
        /* TODO clean this up properly */
        return E_NOT_OK;
    }
    SDCard_Receive(buffer, 512);
    SD_CARD_CS_HIGH();
    return E_OK;
}

uint8_t SDCard_WriteSingleBlock(uint32_t block_addr, const uint8_t* buffer)
{
    /* Buffer used for transmitting command data and checking bus for response */
    uint8_t temporary_buffer[SD_CARD_COMMAND_TX_LENGTH];
    SDCard_CommandFillBuffer(temporary_buffer, SD_CARD_CMD24, block_addr * SDCard_BlockAddressMultiplier, SD_CARD_CMD_NO_CRC);
    SD_CARD_CS_LOW();
    HAL_SPI_Transmit(&hspi1, temporary_buffer, SD_CARD_COMMAND_TX_LENGTH, SD_CARD_SPI_MAX_TIMEOUT);

    /* Wait for R1 response */
    for (uint8_t index = 0; index < SD_CARD_COMMAND_NCR_MAX_LENGTH; index++)
    {
        HAL_SPI_TransmitReceive(&hspi1, SDCard_TxDummyDataBuffer, temporary_buffer, 1, SD_CARD_SPI_MAX_TIMEOUT);
        if (SD_CARD_BUS_IDLE_VALUE != temporary_buffer[0])
        {
            /* Positive Response */
            break;
        }
    }
    if (SD_CARD_IDLE_RESP != temporary_buffer[0])
    {
        /* TODO clean this up properly */
        return E_NOT_OK;
    }

    /* Send Start Token */
    temporary_buffer[0] = SD_CARD_START_TOKEN;
    HAL_SPI_Transmit(&hspi1, &temporary_buffer[0], 1, SD_CARD_SPI_MAX_TIMEOUT);
    /* Send data from input buffer */
    /* TODO do something about 512 length */
    SDCard_Transmit(buffer, 512);

    /* Wait for response */
    for (uint8_t index = 0; index < SD_CARD_COMMAND_NCR_MAX_LENGTH; index++)
    {
        SDCard_Receive(temporary_buffer, 1);
        if (SD_CARD_BUS_IDLE_VALUE != temporary_buffer[0])
        {
            /* Positive Response */
            break;
        }
    }

    if (SD_CARD_WRITE_RESPONSE_ACCEPTED != (temporary_buffer[0] & SD_CARD_WRITE_RESPONSE_MASK))
    {
        /* TODO clean this up properly */
        return E_NOT_OK;
    }

    /* Wait until SD card finnish */
    for (uint8_t index = 0; index < SD_CARD_BUSY_AWAITING_CYCLES; index++)
    {
        SDCard_Receive(temporary_buffer, 1);
        if (SD_CARD_BUS_BUSY_VALUE != temporary_buffer[0])
        {
            /* Positive Response */
            break;
        }
    }

    SD_CARD_CS_HIGH();
    return E_OK;
}

uint32_t SDCard_GetSectorCount(void)
{
    return SDCard_SectorCount;
}