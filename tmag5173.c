#include "tmag5173.h"
#include "cmsis_os.h"
#include "task.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "projdefs.h"

/* Config structure for I2C */
extern I2C_HandleTypeDef hi2c2;
extern osThreadId TMAG5173Handle;

#define TMAG5173_DEVICE_CONFIG2         (0x01U)
#define TMAG5173_SENSOR_CONFIG1         (0x02U)
#define TMAG5173_X_MSB_RESULT_REGISTER  (0x12U)

#define TMAG5173_DEVICE_CONFIG2_OPERATING_MODE_CONTINOUS        (0x02U)

#define TMAG5173_SENSOR_CONFIG1_MAGNETIC_CHANNEL_X_ENABLE       (0x10U)
#define TMAG5173_SENSOR_CONFIG1_MAGNETIC_CHANNEL_X_Y_ENABLE     (0x30U)
#define TMAG5173_SENSOR_CONFIG1_MAGNETIC_CHANNEL_X_Y_Z_ENABLE   (0x70U)

#define TMAG5173_SENSOR_CONFIG2_ANGLE_X_Y_ENABLE                (0x04U)
#define TMAG5173_SENSOR_CONFIG2_ANGLE_Y_Z_ENABLE                (0x08U)
#define TMAG5173_SENSOR_CONFIG2_ANGLE_X_Z_ENABLE                (0x0CU)

#define TMAG5173_DEVICE_8BIT_ADDRESS (0x6AU)
#define TMAG5173_I2C_MAX_TIMEOUT (HAL_MAX_DELAY - 1U)

#define TMAG5173_ANGLE_TO_INTEGER_SHIFT     (0x4U)

static uint8_t TMAG5173_DataBuffer[10U];

void TMAG5173_InitializeSensor(void)
{
    /* TODO rework init in nice structure */
    /* TODO Add initialization for all global variables */
    TMAG5173_DataBuffer[0U] = 0U | TMAG5173_DEVICE_CONFIG2_OPERATING_MODE_CONTINOUS;
    TMAG5173_DataBuffer[1U] = 0U | TMAG5173_SENSOR_CONFIG1_MAGNETIC_CHANNEL_X_Y_Z_ENABLE;
    TMAG5173_DataBuffer[2U] = 0U | TMAG5173_SENSOR_CONFIG2_ANGLE_Y_Z_ENABLE;
    HAL_I2C_Mem_Write(&hi2c2, TMAG5173_DEVICE_8BIT_ADDRESS, TMAG5173_DEVICE_CONFIG2, I2C_MEMADD_SIZE_8BIT, TMAG5173_DataBuffer, 3U, TMAG5173_I2C_MAX_TIMEOUT);
}

void TMAG5173_ReadData(TMAG5173_SensorDataType *data)
{
    // TODO add some kind of configuration for selecting only angle for conversion

    /* Non blocking read of all axis, conversion status and angle */
    (void)HAL_I2C_Mem_Read_IT(&hi2c2, TMAG5173_DEVICE_8BIT_ADDRESS, TMAG5173_X_MSB_RESULT_REGISTER, I2C_MEMADD_SIZE_8BIT, TMAG5173_DataBuffer, 9U);

    /* Non blocking waiting for I2C message reception */
    if (1U == ulTaskNotifyTake(pdFALSE, 1U))
    {
    ulTaskNotifyTake(pdFALSE, 4U);
        /* Message received properly */
        data->x_axis = (TMAG5173_DataBuffer[0U] << 8U) | TMAG5173_DataBuffer[1U];
        data->y_axis = (TMAG5173_DataBuffer[2U] << 8U) | TMAG5173_DataBuffer[3U];
        data->z_axis = (TMAG5173_DataBuffer[4U] << 8U) | TMAG5173_DataBuffer[5U];
        data->conversion_status = TMAG5173_DataBuffer[6U];
        data->angle = ((TMAG5173_DataBuffer[7U] << 8U) | TMAG5173_DataBuffer[8U]) >> TMAG5173_ANGLE_TO_INTEGER_SHIFT;
    }
}

/* Callback from I2C on compleate event
   TODO move this to I2C driver properly */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(TMAG5173Handle, &xHigherPriorityTaskWoken);
}