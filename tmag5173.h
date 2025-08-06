#include "std_types.h"

typedef struct
{
    int16_t x_axis;             /* in LSB */
    int16_t y_axis;             /* in LSB */
    int16_t z_axis;             /* in LSB */
    int16_t angle;              /* in 0-360 deg */
    uint8_t conversion_status;  /* Register readout */
} TMAG5173_SensorDataType;

void TMAG5173_InitializeSensor(void);
void TMAG5173_ReadData(TMAG5173_SensorDataType *data);