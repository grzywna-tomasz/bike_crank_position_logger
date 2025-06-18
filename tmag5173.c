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
//TODO check what is this, no timeout should ignore timeout at all
#define TMAG5173_I2C_NO_TIMEOUT (0U)
#define TMAG5173_I2C_MAX_TIMEOUT (HAL_MAX_DELAY - 1U)

#define TMAG5173_ANGLE_TO_INTEGER_SHIFT     (0x4U)

typedef void (*TMAG5173_FunctionPointerType)(void);
typedef enum
{
    TMAG5173_INIT,
    TMAG5173_READOUT,
    TMAG5173_STATES_NO,
} TMAG5173_DriverStateType;

typedef struct
{
    int16_t x_axis;             /* in LSB */
    int16_t y_axis;             /* in LSB */
    int16_t z_axis;             /* in LSB */
    int16_t angle;              /* in 0-360 deg */
    uint8_t conversion_status;  /* Register readout */
} TMAG5173_SensorDataType;

static void TMAG5173_InitializeSensor(void);
static void TMAG5173_ReadContinously(void);

static uint8_t TMAG5173_DataBuffer[10U];
static TMAG5173_SensorDataType TMAG5173_SensorData;

static TMAG5173_DriverStateType TMAG5173_MachineState = TMAG5173_INIT;
static TMAG5173_FunctionPointerType TMAG5173_StateArray[TMAG5173_STATES_NO] = 
{
    TMAG5173_InitializeSensor,
    TMAG5173_ReadContinously,
};

static void TMAG5173_InitializeSensor(void)
{
    /* TODO rework init in nice structure */
    /* TODO Add initialization for all global variables */
    TMAG5173_DataBuffer[0U] = 0U | TMAG5173_DEVICE_CONFIG2_OPERATING_MODE_CONTINOUS;
    TMAG5173_DataBuffer[1U] = 0U | TMAG5173_SENSOR_CONFIG1_MAGNETIC_CHANNEL_X_Y_Z_ENABLE;
    TMAG5173_DataBuffer[2U] = 0U | TMAG5173_SENSOR_CONFIG2_ANGLE_Y_Z_ENABLE;
    HAL_I2C_Mem_Write(&hi2c2, TMAG5173_DEVICE_8BIT_ADDRESS, TMAG5173_DEVICE_CONFIG2, I2C_MEMADD_SIZE_8BIT, TMAG5173_DataBuffer, 3U, TMAG5173_I2C_MAX_TIMEOUT);

    TMAG5173_MachineState = TMAG5173_READOUT;
}

static void TMAG5173_ReadContinously(void)
{
    // TODO add some kind of configuration for selecting only angle for conversion

    /* Non blocking read of all axis, conversion status and angle */
    (void)HAL_I2C_Mem_Read_IT(&hi2c2, TMAG5173_DEVICE_8BIT_ADDRESS, TMAG5173_X_MSB_RESULT_REGISTER, I2C_MEMADD_SIZE_8BIT, TMAG5173_DataBuffer, 9U);

    // TODO add nice handling for debug pin management
    HAL_GPIO_WritePin(GPIOA, Debug_pin1_Pin, GPIO_PIN_RESET);

    /* Non blocking waiting for I2C message reception */
    // if (1U == ulTaskNotifyTake(pdFALSE, 1U))
    // {
    // ulTaskNotifyTake(pdFALSE, 4U);
        // HAL_GPIO_WritePin(GPIOA, Debug_pin1_Pin, GPIO_PIN_SET);
        /* Message received properly */
        TMAG5173_SensorData.x_axis = (TMAG5173_DataBuffer[0U] << 8U) | TMAG5173_DataBuffer[1U];
        TMAG5173_SensorData.y_axis = (TMAG5173_DataBuffer[2U] << 8U) | TMAG5173_DataBuffer[3U];
        TMAG5173_SensorData.z_axis = (TMAG5173_DataBuffer[4U] << 8U) | TMAG5173_DataBuffer[5U];
        TMAG5173_SensorData.conversion_status = TMAG5173_DataBuffer[6U];
        TMAG5173_SensorData.angle = ((TMAG5173_DataBuffer[7U] << 8U) | TMAG5173_DataBuffer[8U]) >> TMAG5173_ANGLE_TO_INTEGER_SHIFT;
    // }
    // else
    // {
    //     // TODO add some kind of DET - development error tracer just to cleanup some things
    //     /* Add some error tracking */
    //     Error_Handler();
    // }

    // (void)HAL_I2C_Mem_Read(&hi2c2, TMAG5173_DEVICE_8BIT_ADDRESS, TMAG5173_X_MSB_RESULT_REGISTER, I2C_MEMADD_SIZE_8BIT, TMAG5173_DataBuffer, 9U, HAL_MAX_DELAY - 1U);


    //TODO add angle calculation
    //TODO add conversion to mT
    //TODO check if the magnet is not too strong for the sensor
    // (void)HAL_I2C_Master_Receive(&hi2c2, TMAG5173_DEVICE_8BIT_ADDRESS, Tamg5173_DataBuffer, 1U, HAL_MAX_DELAY - 1U);
}

void TMAG5173_MainFunction(void)
{
    if ((TMAG5173_STATES_NO > TMAG5173_MachineState) && ((void *)(0U) != TMAG5173_StateArray[TMAG5173_MachineState]))
    {
        TMAG5173_StateArray[TMAG5173_MachineState]();
    }
}

/* Callback from I2C on compleate event
   TODO move this to I2C driver properly */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(TMAG5173Handle, &xHigherPriorityTaskWoken);
}