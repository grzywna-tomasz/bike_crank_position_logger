#ifndef SD_CARD_CFG_H
#define SD_CARD_CFG_H

/* Internal configuration for SD card */
/**********************************INCLUDE*************************************/
#include "std_types.h"
#include "cmsis_os.h"

/**********************************DEFINES*************************************/
#define SD_CARD_CS_LOW()        HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET)
#define SD_CARD_CS_HIGH()       HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET)

#define SD_CARD_DELAY_MS(delay)             osDelay(delay)
#define SD_CARD_OS_WAIT_FOR_RECEIVE_END()   ulTaskNotifyTake(pdFALSE, 50U)

/**********************************TYPEDEFS************************************/

/**********************************PROTOTYPES**********************************/

/**********************************OBJECTS*************************************/

/**********************************DEFINITIONS*********************************/
Std_ReturnType SDCard_TransmitReceive(uint8_t *TxDataPtr, uint8_t *RxDataPtr, uint16_t data_length);
Std_ReturnType SDCard_Transmit(const uint8_t *TxDataPtr, uint16_t data_length);
Std_ReturnType SDCard_Receive(uint8_t *RxDataPtr, uint16_t data_length);
#endif /* SD_CARD_CFG_H */