/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "dma.h"
#include <time.h>
#include "ptpd.h"

typedef struct 
{
    uint16_t start_idx;
    uint16_t data_length;
}Uart_Rx_Control_t;

typedef struct 
{
    uint16_t stop_idx;
    uint16_t data_length;
}Uart_Rx_IdleLine_t;

typedef enum {
    RX_GPS_W8_SOF,
    RX_GPS_GNRMC,
    RX_GPS_TIME, /* hhmmss.sss */
    RX_GPS_VALID,
    RX_GPS_LAT,
    RX_GPS_NS,
    RX_GPS_LON,
    RX_GPS_EW,
    RX_GPS_SPEED,
    RX_GPS_COG,
    RX_GPS_DATE, /* ddmmyy */
    RX_GPS_MAG_VAR,
    RX_GPS_MAG_EW,
    RX_GPS_POS_MODE,
    RX_GPS_EOF,
    RX_GPS_CRC,
    RX_GPS_CR,
    RX_GPS_LF
} Rx_GPS_SM_t;

/* Variables */
static osMessageQDef(usart6_m_q, 8, Uart_Rx_Control_t*); // Declare a message queue
osMessageQId (usart6_m_q_id);           // Declare an ID for the message queue

static osMessageQDef(usart6_IL_m_q, 4, Uart_Rx_IdleLine_t*); // Declare a message queue
osMessageQId (usart6_IL_m_q_id);           // Declare an ID for the message queue

volatile static uint8_t uart_rx_buf[RX_BUF_SIZE];
volatile static Uart_Rx_Control_t uart_dma_isr_rx_stat;
volatile static Uart_Rx_Control_t uart_isr_rx_stat;
volatile static Uart_Rx_IdleLine_t uart_isr_rx_il_stat;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* Function prototypes */
static Rx_GPS_SM_t GPS_SM(uint8_t* time, uint8_t* date, uint8_t c);
static void GPS_CalculateTime( uint8_t* time_c, uint8_t* date_c, TimeInternal* time_stamp);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);

/* USER CODE END 0 */
void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
   // Error_Handler();
  }

}
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration    
    PC7     ------> USART6_RX
    PC6     ------> USART6_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_NORMAL;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      //Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart6_rx);

    /* USART6 interrupt Init */
    // RTOR

    // USART_CR1 -> RTOIE
    SET_BIT(uartHandle->Instance->CR1, USART_CR1_IDLEIE);
    /*
    SET_BIT(uartHandle->Instance->RTOR, 100u);
    SET_BIT(uartHandle->Instance->CR1, USART_CR1_RTOIE);
    */
    HAL_NVIC_SetPriority(USART6_IRQn, 0xF, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}


void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();
  
    /**USART6 GPIO Configuration    
    PC7     ------> USART6_RX
    PC6     ------> USART6_TX 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_7|GPIO_PIN_6);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */


 

void GPS_thread(void const * argument)
{
    Uart_Rx_Control_t* uart_rx_stats;
    static uint8_t time[11], date[7];
    uint8_t aaa[40];
    Rx_GPS_SM_t state = RX_GPS_W8_SOF;
    TimeInternal time_stamp;
    osEvent event;
    uint16_t end_idx = 0;
    usart6_m_q_id = osMessageCreate(osMessageQ(usart6_m_q), NULL);
    usart6_IL_m_q_id = osMessageCreate(osMessageQ(usart6_IL_m_q), NULL);
    /*init */
    time[10] = NULL;
    date[6] = NULL;
    
    MX_DMA_Init();
    MX_USART6_UART_Init();

    /* Reception start */
    HAL_UART_Receive_DMA(&huart6, (uint8_t*)(&uart_rx_buf[0]), RX_BUF_SLICE_SIZE);
    //UART_DMAError
     while(1)
    {
        event = osMessageGet(usart6_m_q_id, osWaitForever);
        if (event.status == osEventMessage)
        {
            uart_rx_stats = (Uart_Rx_Control_t*)event.value.p;
            end_idx = uart_rx_stats->start_idx + RX_BUF_SLICE_SIZE;
            if(end_idx > RX_BUF_SIZE - RX_BUF_SLICE_SIZE)
            {
                end_idx = 0;
            }
            HAL_UART_Receive_DMA(&huart6, (uint8_t*)(&uart_rx_buf[end_idx]), RX_BUF_SLICE_SIZE);
           
            end_idx = uart_rx_stats->start_idx + uart_rx_stats->data_length;
            for (uint16_t i = uart_rx_stats->start_idx; i <  end_idx; i++)
            {
                state = GPS_SM(time, date, uart_rx_buf[i]);
                uart_rx_buf[i] = 0xffu;
                if(state == RX_GPS_LF)
                {
                    printf(":time: %s\n",time);
                    printf(":date: %s\n",date);
                    GPS_CalculateTime(time, date, &time_stamp);
                    printf(":timestamp: %d.%d\n\n",time_stamp.seconds,time_stamp.nanoseconds);
                }
            }
        }
    }
    
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    uint32_t addr = (uint32_t)uart_rx_buf;
    osEvent event = osMessageGet(usart6_IL_m_q_id,0);
    if (event.status == osEventMessage)
    {
        Uart_Rx_IdleLine_t* uart_il = (Uart_Rx_IdleLine_t*)event.value.p;
        uart_dma_isr_rx_stat.data_length = RX_BUF_SLICE_SIZE - UartHandle->hdmarx->Instance->NDTR - uart_il->data_length;
        uart_dma_isr_rx_stat.start_idx = ((uint16_t)(UartHandle->hdmarx->Instance->M0AR - addr)) + uart_il->stop_idx;
    }
    else
    {
        uart_dma_isr_rx_stat.data_length = RX_BUF_SLICE_SIZE - UartHandle->hdmarx->Instance->NDTR;
        uart_dma_isr_rx_stat.start_idx = (uint16_t)(UartHandle->hdmarx->Instance->M0AR - addr);
    }
    osMessagePut(usart6_m_q_id,(uint32_t)&uart_dma_isr_rx_stat,0);
}


void USART6_ISR_IdleLine_Callback(UART_HandleTypeDef *UartHandle, uint16_t bytes_left)
{
    uint32_t addr = (uint32_t)uart_rx_buf;
    uart_isr_rx_il_stat.data_length = uart_isr_rx_stat.data_length = RX_BUF_SLICE_SIZE - bytes_left;
    /* M0AR - this register informs about DMA memory address */
    uart_isr_rx_stat.start_idx = (uint16_t)(UartHandle->hdmarx->Instance->M0AR - addr);
    osMessagePut(usart6_m_q_id,(uint32_t)&uart_isr_rx_stat,0);
    
    uart_isr_rx_il_stat.stop_idx = uart_isr_rx_stat.start_idx + uart_isr_rx_il_stat.data_length;
    osMessagePut(usart6_IL_m_q_id,(uint32_t)&uart_isr_rx_il_stat,0);
    
}

   
static Rx_GPS_SM_t GPS_SM(uint8_t* time, uint8_t* date, uint8_t c)
{
    static Rx_GPS_SM_t sm_state = RX_GPS_W8_SOF;
    static uint8_t msg[100]= {0};
    static uint8_t c_ctr = 0, t_ctr = 0, d_ctr = 0;
    msg[c_ctr] = c;
    switch (sm_state)
    {
        case RX_GPS_W8_SOF:
            if(msg[c_ctr] == '$')
            {
                c_ctr++;
                sm_state = RX_GPS_GNRMC;
            }
            break;
            
        case RX_GPS_GNRMC:
            if( msg[1] == 'G' &&
                msg[2] == 'N' &&
                msg[3] == 'R' &&
                msg[4] == 'M' &&
                msg[5] == 'C' &&
                msg[6] == ',' &&
                c_ctr  == 6
                )
            {
                c_ctr++;
                sm_state = RX_GPS_TIME;
            }
            else if (c_ctr < 6)
            {
                c_ctr++;
                sm_state = RX_GPS_GNRMC;
            }
            else
            {
                c_ctr = 0;
                sm_state = RX_GPS_W8_SOF;
                memset(msg,0,6);
            }
            break;
            
            
        case RX_GPS_TIME:
            if(msg[c_ctr] == ',' || t_ctr == 10)
            {
                sm_state = RX_GPS_VALID;
                t_ctr = 0;
            }
            else
            {
                time[t_ctr] = msg[c_ctr];
                t_ctr++;
                sm_state = RX_GPS_TIME;
            }
            c_ctr++;
            break;

        case RX_GPS_VALID:
        case RX_GPS_LAT:
        case RX_GPS_NS:
        case RX_GPS_LON:
        case RX_GPS_EW:
        case RX_GPS_SPEED:
        case RX_GPS_COG:
            if(msg[c_ctr] == ',')
            {               
                sm_state++;
            }
            c_ctr++;
            break;

        case RX_GPS_DATE:
            if(msg[c_ctr] == ',' || d_ctr == 6)
            {
                sm_state = RX_GPS_MAG_VAR;
                d_ctr = 0;
            }
            else
            {
                date[d_ctr] = msg[c_ctr];
                d_ctr++;
                sm_state = RX_GPS_DATE;
            }
            c_ctr++;
            break;
            
        case RX_GPS_MAG_VAR:
        case RX_GPS_MAG_EW:
        case RX_GPS_POS_MODE:
        case RX_GPS_EOF:
            if (msg[c_ctr] == '*')
            {
                sm_state = RX_GPS_LF;
            }
            c_ctr++;
            break;

        case RX_GPS_CRC:
        case RX_GPS_CR:
        case RX_GPS_LF:
            c_ctr = 0;
            memset(msg,0,10);
            sm_state = RX_GPS_W8_SOF;
            break;
        
        default:
            sm_state = RX_GPS_W8_SOF;
            break;
    }
    return sm_state;
}

static void GPS_CalculateTime( uint8_t* time_c, uint8_t* date_c, TimeInternal* time_stamp)
{
    struct tm t;
    time_t t_of_day;
    uint16_t d,m,y;
    uint32_t ns;

    d = (date_c[0] - '0') * 10 + (date_c[1] - '0');
    m = (date_c[2] - '0') * 10 + (date_c[3] - '0');
    y = (date_c[4] - '0') * 10 + (date_c[5] - '0');
    if( y >= 80 )
    {
        y += 1000;
    }
    else
    {
        y += 2000;
    }
    
    t.tm_year = y-1900;  // Year - 1900
    t.tm_mon = m;           // Month, where 0 = jan
    t.tm_mday = d;          // Day of the month
    t.tm_hour = ((time_c[0] - '0')*10) + (time_c[1] - '0');
    t.tm_min = ((time_c[2] - '0')*10) + (time_c[3] - '0');
    t.tm_sec = ((time_c[4] - '0')*10) + (time_c[5] - '0');
    t.tm_isdst = 1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
    t_of_day = mktime(&t);
    
    
    ns = (time_c[7] - '0') * 100000000U;
    ns += (time_c[8] - '0') * 10000000U;
    ns += (time_c[9] - '0') * 1000000U;
    
    time_stamp->seconds = (int32_t) t_of_day;
    time_stamp->nanoseconds = (int32_t) ns;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
