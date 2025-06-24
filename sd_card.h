#ifndef SD_CARD_H
#define SD_CARD_H

/**********************************INCLUDE*************************************/
#include "std_types.h"

/**********************************DEFINES*************************************/

/**********************************TYPEDEFS************************************/

/**********************************PROTOTYPES**********************************/
Std_ReturnType SDCard_InitializeCard(void);
uint8_t SDCard_ReadSingleBlock(uint32_t block_addr, uint8_t* buffer);
uint8_t SDCard_WriteSingleBlock(uint32_t block_addr, const uint8_t* buffer);
/**********************************OBJECTS*************************************/

/**********************************DEFINITIONS*********************************/

#endif /* SD_CARD_H */