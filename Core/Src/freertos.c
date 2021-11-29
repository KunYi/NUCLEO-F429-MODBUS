/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "semphr.h"
#include "Modbus.h"
#include "sysStatus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MODBUS_SLAVE_ID	       (1)
#define TOWER_ID               (12345)

#define SENSOR_EC              (1)
#define SENSOR_PH              (2)
#define SENSOR_TEMP1           (3)
#define SENSOR_TEMP2           (4)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

modbusHandler_t ModbusSlaveH;
modbusHandler_t ModbusMasterH;
uint8_t         ModbusSlaveCoilBuffer[10];
uint16_t        ModusSlaveDataBuffer[48];
uint16_t        SysStatus[48];
uint16_t        u16EC;
uint16_t        u16PH;
uint16_t        u16Temp1Array[4];
uint16_t        u16Temp2Array[4];
struct SystemStatus*  pSysStatus=(struct SystemStatus*)SysStatus;


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskMaster */
osThreadId_t myTaskMasterHandle;
const osThreadAttr_t myTaskMaster_attributes = {
  .name = "myTaskMaster",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTaskSlave */
osThreadId_t myTaskSlaveHandle;
const osThreadAttr_t myTaskSlave_attributes = {
  .name = "myTaskSlave",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void initModbusSlave(void);
static void initModbusMaster(void);
static void ModbusSlaveProc(void);
static void ModbusMasterProc(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTaskMaster(void *argument);
void StartTaskSlave(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  initModbusSlave();
  initModbusMaster();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTaskMaster */
  myTaskMasterHandle = osThreadNew(StartTaskMaster, NULL, &myTaskMaster_attributes);

  /* creation of myTaskSlave */
  myTaskSlaveHandle = osThreadNew(StartTaskSlave, NULL, &myTaskSlave_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskMaster */
/**
* @brief Function implementing the myTaskMaster thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMaster */
void StartTaskMaster(void *argument)
{
  /* USER CODE BEGIN StartTaskMaster */
  /* Infinite loop */
  for(;;)
  {
    ModbusMasterProc();
    osDelay(500);
  }
  /* USER CODE END StartTaskMaster */
}

/* USER CODE BEGIN Header_StartTaskSlave */
/**
* @brief Function implementing the myTaskSlave thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskSlave */
void StartTaskSlave(void *argument)
{
  /* USER CODE BEGIN StartTaskSlave */
  /* Infinite loop */
  for(;;)
  {
    xSemaphoreTake(ModbusSlaveH.ModBusSphrHandle , portMAX_DELAY);
    ModbusSlaveProc();
    xSemaphoreGive(ModbusSlaveH.ModBusSphrHandle);
    osDelay(100);
  }
  /* USER CODE END StartTaskSlave */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static void ModbusMasterProc(void) {
  modbus_t telegram;
  uint32_t u32NotificationValue;

  telegram.u8id = SENSOR_EC; // slave address
  telegram.u8fct = MB_FC_READ_REGISTERS; // function code
  telegram.u16RegAdd = 2; // start address in slave, 40003
  telegram.u16CoilsNo = 1; // number of elements (coils or registers) to read
  telegram.u16reg = &u16EC; // pointer to a memory array in the Arduino
  ModbusQuery(&ModbusMasterH, telegram);
  u32NotificationValue = ulTaskNotifyTake(pdTRUE, 500);
  if (u32NotificationValue != (uint32_t)ERR_OK_QUERY) {
    // error handler
    printf ("read EC failed\r\n");
  } else {
    printf ("read EC:0x%04X(%d)\r\n", u16EC, u16EC);
    pSysStatus->EC = u16EC;
  }

  telegram.u8id = SENSOR_PH; // slave address
  telegram.u8fct = MB_FC_READ_REGISTERS; // function code
  telegram.u16RegAdd = 6; // start address in slave, 40007
  telegram.u16CoilsNo = 1; // number of elements (coils or registers) to read
  telegram.u16reg = &u16PH; // pointer to a memory array in the Arduino
  ModbusQuery(&ModbusMasterH, telegram);
  u32NotificationValue = ulTaskNotifyTake(pdTRUE, 500);
  if (u32NotificationValue != (uint32_t)ERR_OK_QUERY) {
    // error handler
    printf ("read PH failed\r\n");
  } else {
    printf ("read PH:0x%04X(%d)\r\n", u16PH, u16PH);
    pSysStatus->PH = u16PH;
  }

  telegram.u8id = SENSOR_TEMP1;
  telegram.u8fct = MB_FC_READ_REGISTERS; // function code
  telegram.u16RegAdd = 40; // 0x28
  telegram.u16CoilsNo = 4;
  telegram.u16reg = u16Temp1Array;
  ModbusQuery(&ModbusMasterH, telegram);
  u32NotificationValue = ulTaskNotifyTake(pdTRUE, 500);
  if (u32NotificationValue != (uint32_t)ERR_OK_QUERY) {
    // error handler
    printf ("read temperature failed\r\n");
  } else {
    printf ("read Temp1 ch0:%d ch1:%d ch2:%d ch3:%d\r\n", 
    u16Temp1Array[0], u16Temp1Array[1], u16Temp1Array[2], u16Temp1Array[3]);
    pSysStatus->TempIn1 = u16Temp1Array[0];
    pSysStatus->TempOutput1 = u16Temp1Array[1];
    pSysStatus->TempIn2 = u16Temp1Array[2];
    pSysStatus->TempOutput2 = u16Temp1Array[2];
  }

  telegram.u8id = SENSOR_TEMP2;
  telegram.u8fct = MB_FC_READ_REGISTERS; // function code
  telegram.u16RegAdd = 40; // 0x28
  telegram.u16CoilsNo = 4;
  telegram.u16reg = u16Temp2Array;
  ModbusQuery(&ModbusMasterH, telegram);
  u32NotificationValue = ulTaskNotifyTake(pdTRUE, 500);
  if (u32NotificationValue != (uint32_t)ERR_OK_QUERY) {
    // error handler		
    printf (" read temperature failed\r\n");
  } else {
    printf ("read Temp2 ch0:%d ch1:%d ch2:%d ch3:%d\r\n", 
    u16Temp2Array[0], u16Temp2Array[1], u16Temp2Array[2], u16Temp2Array[3]);
    pSysStatus->TempIn3 = u16Temp2Array[0];
    pSysStatus->TempOutput3 = u16Temp2Array[1];
    pSysStatus->TempIn4 = u16Temp2Array[2];
    pSysStatus->TempOutput4 = u16Temp2Array[2];
  }
}

static void ModbusSlaveProc(void) {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, (GPIO_PinState)(ModbusSlaveH.u8coils[0] & (1<<0)));
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, (GPIO_PinState)(ModbusSlaveH.u8coils[0] & (1<<1)));
    //printf("reg 0:0x%04X, reg 1:0x%04X\r\n", ModbusSlaveH.u16regs[0], ModbusSlaveH.u16regs[1]);
    memcpy(ModusSlaveDataBuffer, pSysStatus, sizeof(SysStatus));
}

static void initModbusMaster(void) {
  u16EC = 0;
  u16PH = 0;
  for (int i = 0; i < 4; i++) {
    u16Temp1Array[i] = 0;
    u16Temp2Array[i] = 0;
  }

  ModbusMasterH.uModbusType = MB_MASTER;
  ModbusMasterH.xTypeHW = USART_HW;
  ModbusMasterH.port = &huart2;
  ModbusMasterH.u8id = 0; // for Mastter
  ModbusMasterH.u16timeOut = 1000;
  ModbusMasterH.EN_Port = NULL; // No RS485
  ModbusMasterH.u8coils = NULL;
  ModbusMasterH.u8coilsmask = NULL;
  ModbusMasterH.u16coilsize = 0;	
  ModbusMasterH.u16regs = &u16EC;
  ModbusMasterH.u16regsize = 1;
  ModbusMasterH.u8regsmask = NULL;
  //Initialize Modbus library
  ModbusInit(&ModbusMasterH);
  //Start capturing traffic on serial Port
  ModbusStart(&ModbusMasterH);
}

void initModbusSlave(void) {
  memset(SysStatus, 0, sizeof(SysStatus));
  pSysStatus->TowerNo = TOWER_ID;
  pSysStatus->TempIn1 = 189;           /* 18.9C */
  pSysStatus->TempOutput1 = 201;       /* 20.1C */
  pSysStatus->EC = 1234;               /* EC: 1234 us/cm */
  pSysStatus->PH = 720;                /* PH 7.2 */
  memcpy(ModusSlaveDataBuffer, pSysStatus, sizeof(SysStatus));
  /* Modbus Slave initialization */
  ModbusSlaveH.uModbusType = MB_SLAVE;
  ModbusSlaveH.xTypeHW = USART_HW;
  ModbusSlaveH.port =  &huart1; // This is the UART port connected to STLINK in the NUCLEO F429
  ModbusSlaveH.u8id = MODBUS_SLAVE_ID; //slave ID, always different than zero
  ModbusSlaveH.u16timeOut = 1000;
  ModbusSlaveH.EN_Port = NULL; // No RS485
  //ModbusH2.EN_Port = LD2_GPIO_Port; // RS485 Enable
  //ModbusH2.EN_Pin = LD2_Pin; // RS485 Enable
  // for coils
  ModbusSlaveH.u8coils = ModbusSlaveCoilBuffer;
  ModbusSlaveH.u8coilsmask = NULL;
  ModbusSlaveH.u16coilsize = sizeof(ModbusSlaveCoilBuffer);
  // for regs
  ModbusSlaveH.u16regs = ModusSlaveDataBuffer;
  ModbusSlaveH.u16regsize= sizeof(ModusSlaveDataBuffer)/sizeof(ModusSlaveDataBuffer[0]);
  ModbusSlaveH.u8regsmask = NULL;
  //Initialize Modbus library
  ModbusInit(&ModbusSlaveH);
  //Start capturing traffic on serial Port
  ModbusStart(&ModbusSlaveH);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
