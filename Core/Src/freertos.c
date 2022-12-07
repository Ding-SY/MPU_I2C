/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "i2c.h"
#include "mpu6xxx_reg.h"
#include "crc.h"
#include "arm_math.h"
#include <string.h>
#include <tim.h>
#include "RingBuffer.h"
#include "usbd_cdc_if.h"
#include "MadgwickAHRS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct imu_read
{
    uint16_t acclx;
    uint16_t accly;
    uint16_t acclz;
    uint16_t gyrox;
    uint16_t gyroy;
    uint16_t gyroz;
    uint16_t crc32h;
    uint16_t crc32l;
} imu_read_t;

typedef struct imu_real
{
    float32_t acclx; //m/ss
    float32_t accly;
    float32_t acclz;
    float32_t gyrox; //rad
    float32_t gyroy;
    float32_t gyroz;
} imu_real_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define lsm6ds3  //lsm3ds3  or  mpu6500
#define DATA_EVENT_START (0x01 << 0)
#define DATA_EVENT_END (0x01 << 0)
#define use_fifo 0
//three information can only choose one
#define print_raw_data 0
#define print_fir_data 0
#define print_raw_fir_data 0
#define print_rpy_data 1
#define print_quat_data 0
//gravity constant
#define gravity 9.7955f   // m/(s*s)
//fir
#define fir52 0
#define fir26 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
#if fir52
	#include "fdacoefs.h"
#endif
#if fir26
	#include "fdacoefs_26.h"
#endif
uint16_t led_delay = 1000;
uint8_t status_sum = 0;
imu_read_t imu_read_data;
imu_real_t imu_real_data;
uint16_t HLdata[7];//0:temperature; 1~3 acc; 4~6 gyro
imu_real_t imu_data_fifo[52];
imu_real_t imu_data_fir;
uint64_t imu_data_counts;

//usb serial
char usb_string[200];
//MadgwickAHRS
extern volatile float32_t q0, q1, q2, q3;
extern volatile float32_t roll,pitch,yaw;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for myTaskMPU */
osThreadId_t myTaskMPUHandle;
const osThreadAttr_t myTaskMPU_attributes = {
    .name = "myTaskMPU",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for myTaskMPU */
osThreadId_t myTaskFIRHandle;
const osThreadAttr_t myTaskFIR_attributes = {
    .name = "myTaskFIR",
    .stack_size = 128 * 16,
    .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for myEventData */
osEventFlagsId_t myEventDataStartHandle;
const osEventFlagsAttr_t myEventDataStart_attributes = {
  .name = "myEventDataStart"
};
/* Definitions for myEventData */
osEventFlagsId_t myEventDataEndHandle;
const osEventFlagsAttr_t myEventDataEnd_attributes = {
  .name = "myEventDataEnd"
};
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void LedStatus(HAL_StatusTypeDef stat);
unsigned long RealNoise(unsigned long limit);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTaskMPU(void *argument);
void StartTaskFIR(void *argument);
extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */

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

    /* Create the queue(s) */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* creation of myTaskMPU */
    myTaskMPUHandle = osThreadNew(StartTaskMPU, NULL, &myTaskMPU_attributes);

    /* creation of myTaskFIR */
    myTaskFIRHandle = osThreadNew(StartTaskFIR, NULL, &myTaskFIR_attributes);

		/* Create the event(s) */
		/* creation of myEventData */
		myEventDataStartHandle = osEventFlagsNew(&myEventDataStart_attributes);
		myEventDataEndHandle = osEventFlagsNew(&myEventDataEnd_attributes);
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
			//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			HAL_Delay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskMPU */
/**
* @brief Function implementing the myTaskMPU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMPU */
void StartTaskMPU(void *argument)
{
    /* USER CODE BEGIN StartTaskMPU */
    HAL_I2C_MspInit(&hi2c1);
    HAL_StatusTypeDef status;
    HAL_TIM_Base_MspInit(&htim11);
    uint16_t i2c_rw_delay = 100;
    uint16_t lsm_addr = 0x6B << 1;
    uint16_t lsm_reg[1] = {0x0F};
    uint8_t lsm_d[2] = {0, 0};

    //CTRL3_C
    lsm_d[0] = 0x12;
    lsm_d[1] = 0x01;
    status = HAL_I2C_Mem_Write(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1, 5);
    HAL_Delay(10);
    status_sum |= status;

    //LSM3DS3 WHO AM I
    status = HAL_I2C_Mem_Read(&hi2c1, lsm_addr, lsm_reg[0], 1, &lsm_d[1], 1, 5);
    HAL_Delay(10);
    if( status != HAL_OK)
        LedStatus(status);
    else if (lsm_d[1] != 0x69)
        LedStatus(HAL_ERROR);
    else
        LedStatus(HAL_OK);

    //FUNC_CFG_ACCESS
    lsm_d[0] = 0x01;
    lsm_d[1] = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1, 5);
    HAL_Delay(10);
    status_sum |= status;

    //FIFO_CTRL5
    lsm_d[0] = 0x0A;
    lsm_d[1] = 0x36; //FIFO ODR 416Hz, continus mode
    status = HAL_I2C_Mem_Write(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1, 5);
    HAL_Delay(10);
    status_sum |= status;

    //ORIENT_CFG_G
    lsm_d[0] = 0x0B;
    lsm_d[1] = 0x00; //angular sign (+ + +),orient (x y z)
    status = HAL_I2C_Mem_Write(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1, 5);
    HAL_Delay(10);
    status_sum |= status;

    //CTRL1_XL
    lsm_d[0] = 0x10;
    lsm_d[1] = 0x56; // 208Hz , 16g , Anti-aliasing filter bandwidth : 100Hz
    status = HAL_I2C_Mem_Write(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1, 5);
    HAL_Delay(10);
    status_sum |= status;

    //CTRL2_G
    lsm_d[0] = 0x11;
    lsm_d[1] = 0x5C; // 208Hz , 2000dps
    status = HAL_I2C_Mem_Write(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1, 5);
    HAL_Delay(10);
    status_sum |= status;


    //CTRL5_C
    lsm_d[0] = 0x14;
    lsm_d[1] = 0x60; // no self_test
    status = HAL_I2C_Mem_Write(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1, 5);
    HAL_Delay(10);
    status_sum |= status;

    //CTRL6_C
    lsm_d[0] = 0x15;
    lsm_d[1] = 0x10; //
    status = HAL_I2C_Mem_Write(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1, 5);
    HAL_Delay(10);
    status_sum |= status;

    //CTRL9_XL
    lsm_d[0] = 0x18;
    lsm_d[1] = 0x38; // acc xyz enable
    status = HAL_I2C_Mem_Write(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1, 5);
    HAL_Delay(10);
    status_sum |= status;


    //CTRL10_C
    lsm_d[0] = 0x19;
    lsm_d[1] = 0x38; // gyro xyz enable
    status = HAL_I2C_Mem_Write(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1, 5);
    HAL_Delay(10);
    status_sum |= status;

    if(status_sum)
    {
        LedStatus(HAL_ERROR);
        status_sum = 0;
    }
    else
    {
        LedStatus(HAL_OK);
    }

// imu data
    // accl -16g~16g  gyro  -2000dps~2000dps imu_data[x] x:1~3 accl.x.y.z, x:4~6 gyro.x.y.z
    float accl_lsb = 2047.9375f;
    float gyro_lsb = 16.3835f;
		imu_data_counts = 0;
//create imu data fifo
#if  use_fifo
    RingBuffer *imu_fifo = RingBuffer_Malloc(sizeof(imu_data_fifo));
#endif

		// calculate imu error
		imu_real_t imu_error;
		osEventFlagsSet(myEventDataEndHandle, DATA_EVENT_END);
    /* Infinite loop */
    for(;;)
    {
// wait for data process end signal
				uint32_t r_event = osEventFlagsWait(myEventDataEndHandle, DATA_EVENT_END, osFlagsWaitAll, osWaitForever);

//Temperature
//OUT_TEMP
        lsm_d[0] = 0x21;
        lsm_d[1] = 0x00; // temperature data HIGH
        status = HAL_I2C_Mem_Read_IT(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1);
        delay_us(i2c_rw_delay);
        status_sum |= status;
        HLdata[0]  = (uint16_t)lsm_d[1] << 8;

//OUT_TEMP_L
        lsm_d[0] = 0x20;
        lsm_d[1] = 0x00; // temperature data LOW
        status = HAL_I2C_Mem_Read_IT(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1);
        delay_us(i2c_rw_delay);
        status_sum |= status;
        HLdata[0]  |= lsm_d[1] ;

//acc
//OUTX_H_XL
        lsm_d[0] = 0x29;
        lsm_d[1] = 0x00; // acc_x data high byte
        status = HAL_I2C_Mem_Read_IT(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1);
        delay_us(i2c_rw_delay);
        status_sum |= status;
        HLdata[1] = (uint16_t)lsm_d[1] << 8;

//OUTX_L_XL
        lsm_d[0] = 0x28;
        lsm_d[1] = 0x00; // acc_x data low byte
        status = HAL_I2C_Mem_Read_IT(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1);
        delay_us(i2c_rw_delay);
        status_sum |= status;
        HLdata[1] |= lsm_d[1] ;

//OUTY_H_XL
        lsm_d[0] = 0x2B;
        lsm_d[1] = 0x00; // acc_y data high byte
        status = HAL_I2C_Mem_Read_IT(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1);
        delay_us(i2c_rw_delay);
        status_sum |= status;
        HLdata[2] = (uint16_t)lsm_d[1] << 8;

//OUTY_L_XL
        lsm_d[0] = 0x2A;
        lsm_d[1] = 0x00; // acc_y data low byte
        status = HAL_I2C_Mem_Read_IT(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1);
        delay_us(i2c_rw_delay);
        status_sum |= status;
        HLdata[2] |= lsm_d[1] ;

//OUTZ_H_XL
        lsm_d[0] = 0x2D;
        lsm_d[1] = 0x00; // acc_z data high byte
        status = HAL_I2C_Mem_Read_IT(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1);
        delay_us(i2c_rw_delay);
        status_sum |= status;
        HLdata[3] = (uint16_t)lsm_d[1] << 8;

//OUTZ_L_XL
        lsm_d[0] = 0x2C;
        lsm_d[1] = 0x00; // acc_z data low byte
        status = HAL_I2C_Mem_Read_IT(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1);
        delay_us(i2c_rw_delay);
        status_sum |= status;
        HLdata[3] |= lsm_d[1] ;

//GYRO
//OUTX_H_G
        lsm_d[0] = 0x23;
        lsm_d[1] = 0x00; // gyro_x data high byte
        status = HAL_I2C_Mem_Read_IT(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1);
        delay_us(i2c_rw_delay);
        status_sum |= status;
        HLdata[4] = (uint16_t)lsm_d[1] << 8;

//OUTX_L_G
        lsm_d[0] = 0x22;
        lsm_d[1] = 0x00; // gyro_x data low byte
        status = HAL_I2C_Mem_Read_IT(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1);
        delay_us(i2c_rw_delay);
        status_sum |= status;
        HLdata[4] |= lsm_d[1] ;

//OUTY_H_G
        lsm_d[0] = 0x25;
        lsm_d[1] = 0x00; // gyro_y data high byte
        status = HAL_I2C_Mem_Read_IT(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1);
        delay_us(i2c_rw_delay);
        status_sum |= status;
        HLdata[5] = (uint16_t)lsm_d[1] << 8;

//OUTY_L_G
        lsm_d[0] = 0x24;
        lsm_d[1] = 0x00; // gyro_y data low byte
        status = HAL_I2C_Mem_Read_IT(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1);
        delay_us(i2c_rw_delay);
        status_sum |= status;
        HLdata[5] |= lsm_d[1] ;

//OUTZ_H_G
        lsm_d[0] = 0x27;
        lsm_d[1] = 0x00; // gyro_z data high byte
        status = HAL_I2C_Mem_Read_IT(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1);
        delay_us(i2c_rw_delay);
        status_sum |= status;
        HLdata[6] = (uint16_t)lsm_d[1] << 8;

//OUTZ_L_G
        lsm_d[0] = 0x26;
        lsm_d[1] = 0x00; // gyro_z data low byte
        status = HAL_I2C_Mem_Read_IT(&hi2c1, lsm_addr, (uint16_t)lsm_d[0], 1, &lsm_d[1], 1);
        delay_us(i2c_rw_delay);
        status_sum |= status;
        HLdata[6] |= lsm_d[1] ;	

        if(status_sum)
        {
            LedStatus(HAL_ERROR);
            status_sum = 0;
        }
        else
        {
            LedStatus(HAL_OK);
        }
// get imu real data
        imu_real_data.acclx = -(int16_t)HLdata[1] / accl_lsb * gravity;
        imu_real_data.accly = -(int16_t)HLdata[2] / accl_lsb * gravity;
        imu_real_data.acclz = -(int16_t)HLdata[3] / accl_lsb * gravity;
        imu_real_data.gyrox = (int16_t)HLdata[4] / gyro_lsb * 0.0174532925f - 0.0492223203f; //rad
        imu_real_data.gyroy = (int16_t)HLdata[5] / gyro_lsb * 0.0174532925f + 0.0340829603f;
        imu_real_data.gyroz = (int16_t)HLdata[6] / gyro_lsb * 0.0174532925f + 0.0326923733f;
			
// refresh data by self
				if(imu_data_counts < num_taps)
					imu_data_fifo[imu_data_counts] = imu_real_data;
				else
				{
					for(uint16_t i=0;i<num_taps-1;i++)
					{
						imu_data_fifo[i] = imu_data_fifo[i+1]; 
					}
					imu_data_fifo[num_taps -1] = imu_real_data;
				}
//calculate error
				if(imu_data_counts <10000)
				{
					imu_error.acclx +=imu_real_data.acclx; 
					imu_error.accly +=imu_real_data.accly; 
					imu_error.acclz +=imu_real_data.acclz;
					imu_error.gyrox +=imu_real_data.gyrox;
					imu_error.gyroy +=imu_real_data.gyroy;
					imu_error.gyroz +=imu_real_data.gyroz;
				}
				else if (imu_data_counts ==10000)
				{
					imu_error.acclx /= 10000;
					imu_error.accly /= 10000;
					imu_error.acclz /= 10000;
					imu_error.gyrox /= 10000;
					imu_error.gyroy /= 10000;
					imu_error.gyroz /= 10000;
					HAL_Delay(1);
				}
				imu_data_counts ++;

//fifo
#if  use_fifo
        RingBuffer_In(imu_fifo, &imu_real_data, sizeof(imu_real_data));
#endif
				
// inform data process start 
				osEventFlagsSet(myEventDataStartHandle, DATA_EVENT_START);
    }
    /* USER CODE END StartTaskMPU */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartTaskFIR(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */

    /* Infinite loop */
		for(;;)
    {
// wait for data process start signal 
			uint32_t r_event = osEventFlagsWait(myEventDataStartHandle, DATA_EVENT_START, osFlagsWaitAll, osWaitForever);
			
// fir calculate by self
			for(uint16_t i=0;i<num_taps;i++)
			{
				imu_data_fir.acclx += imu_data_fifo[i].acclx * Coeffs[i];
				imu_data_fir.accly += imu_data_fifo[i].accly * Coeffs[i];
				imu_data_fir.acclz += imu_data_fifo[i].acclz * Coeffs[i];
				imu_data_fir.gyrox += imu_data_fifo[i].gyrox * Coeffs[i];
				imu_data_fir.gyroy += imu_data_fifo[i].gyroy * Coeffs[i];
				imu_data_fir.gyroz += imu_data_fifo[i].gyroz * Coeffs[i];
			}
			MadgwickAHRSupdateIMU(imu_data_fir.gyrox,imu_data_fir.gyroy,imu_data_fir.gyroz,-1.0f*imu_data_fir.acclx,-1.0f*imu_data_fir.accly,-1.0f*imu_data_fir.acclz);
#if print_raw_data 			
			sprintf(usb_string, "%9.5f,%9.5f,%9.5f,%9.5f,%9.5f,%9.5f,%d\r\n", 
							imu_real_data.acclx,imu_real_data.accly,imu_real_data.acclz,
							imu_real_data.gyrox,imu_real_data.gyroy,imu_real_data.gyroz,
							HAL_GetTick());
#endif
#if print_fir_data			
			sprintf(usb_string, "%9.5f,%9.5f,%9.5f,%9.5f,%9.5f,%9.5f,%d\r\n", 
							imu_data_fir.acclx,imu_data_fir.accly,imu_data_fir.acclz,
							imu_data_fir.gyrox,imu_data_fir.gyroy,imu_data_fir.gyroz,
							HAL_GetTick());
#endif			
#if print_raw_fir_data
			sprintf(usb_string, "%9.5f,%9.5f,%9.5f,%9.5f,%9.5f,%9.5f,%9.5f,%9.5f,%9.5f,%9.5f,%9.5f,%9.5f,%d\r\n", 
							imu_real_data.acclx,imu_real_data.accly,imu_real_data.acclz,
							imu_real_data.gyrox,imu_real_data.gyroy,imu_real_data.gyroz,
							imu_data_fir.acclx,imu_data_fir.accly,imu_data_fir.acclz,
							imu_data_fir.gyrox,imu_data_fir.gyroy,imu_data_fir.gyroz,
							HAL_GetTick());
#endif
#if print_rpy_data
			getRPY();
			sprintf(usb_string, "%9.5f,%9.5f,%9.5f,%d\r\n",roll,pitch,yaw,HAL_GetTick());
#endif
#if print_quat_data
			sprintf(usb_string, "%9.5f,%9.5f,%9.5f,%9.5f,%d\r\n",
							q0,q1,q2,q3,HAL_GetTick());
#endif
			CDC_Transmit_FS(usb_string, sizeof(usb_string));
			imu_data_fir.acclx = 0;
			imu_data_fir.accly = 0;
			imu_data_fir.acclz = 0;
			imu_data_fir.gyrox = 0;
			imu_data_fir.gyroy = 0;
			imu_data_fir.gyroz = 0;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
// inform data process end 
			osEventFlagsSet(myEventDataEndHandle, DATA_EVENT_END);
		}
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
void LedStatus(HAL_StatusTypeDef stat)
{
    switch(stat)
    {
    case HAL_OK:
        led_delay = 1000;
        break;
    case HAL_ERROR:
        led_delay = 500;
        break;
    case HAL_BUSY:
        led_delay = 200;
        break;
    case HAL_TIMEOUT:
        led_delay = 2000;
        break;
    default:
        led_delay = 1000;
        break;
    }
}

/* USER CODE END Application */

