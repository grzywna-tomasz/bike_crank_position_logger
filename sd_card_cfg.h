#ifndef SD_CARD_CFG_H
#define SD_CARD_CFG_H

/**********************************INCLUDE*************************************/
#include "std_types.h"
#include "cmsis_os.h"

/**********************************DEFINES*************************************/
#define SD_CARD_DELAY_MS(delay) osDelay(delay)
#define SD_CARD_CS_LOW()        HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET)
#define SD_CARD_CS_HIGH()       HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET)

/**********************************TYPEDEFS************************************/

/**********************************PROTOTYPES**********************************/

/**********************************OBJECTS*************************************/

/**********************************DEFINITIONS*********************************/
Std_ReturnType SDCard_TransmitReceive(uint8_t *TxDataPtr, uint8_t *RxDataPtr, uint16_t data_length);
#endif /* SD_CARD_CFG_H */