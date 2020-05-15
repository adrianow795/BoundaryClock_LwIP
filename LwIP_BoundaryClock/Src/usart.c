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
#include "cmsis_os.h"
#include "dma.h"
#include <time.h>
#include "ptpd.h"

osMessageQDef(usart6_m_q, 400, uint8_t); // Declare a message queue
osMessageQId (usart6_m_q_id);           // Declare an ID for the message queue

void USART6_Rx_ISR(struct __UART_HandleTypeDef *huart);
/* USER CODE END 0 */

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_tx;

/* USART6 init function */

void MX_USART6_UART_Init(void)
{
    uint32_t temp = 0;
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
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
    //Error_Handler();
  }
  CLEAR_BIT(huart6.Instance->CR1, (USART_CR1_RE | USART_CR1_UE));
  SET_BIT(huart6.Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_CMIE));
  temp = ((uint32_t)('*')) << 24;
  SET_BIT(huart6.Instance->CR2, (temp | USART_CR2_ADDM7));
  //SET_BIT(huart6.Instance->CR3, USART_CR3_OVRDIS);
  SET_BIT(huart6.Instance->CR1, (USART_CR1_RE | USART_CR1_UE));
  

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
    /* USART6_TX Init */
    hdma_usart6_tx.Instance = DMA2_Stream6;
    hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_tx.Init.Mode = DMA_NORMAL;
    hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK)
    {
      //Error_Handler();
    }

    //__HAL_LINKDMA(uartHandle,hdmatx,hdma_usart6_tx);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
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

typedef enum {
    RX_GPS_W8_SOF,
    RX_GPS_SOF,
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
    
Rx_GPS_SM_t GPS_SM(uint8_t* time, uint8_t* date, uint8_t c)
{
    static Rx_GPS_SM_t sm_state = RX_GPS_W8_SOF;
    static uint8_t msg[100]= {0};
    static uint8_t c_ctr = 0, t_ctr = 0, d_ctr = 0;
    
    switch (sm_state)
    {
        case RX_GPS_W8_SOF:
            if(msg[c_ctr] == '$')
            {
                c_ctr++;
                sm_state = RX_GPS_SOF;
            }
            break;
            
        case RX_GPS_GNRMC:
            if( msg[1] == 'G' &&
                msg[2] == 'N' &&
                msg[3] == 'R' &&
                msg[4] == 'M' &&
                msg[5] == 'C' &&
                c_ctr  == 5
                )
            {
                c_ctr++;
                sm_state = RX_GPS_TIME;
            }
            else if (c_ctr < 5)
            {
                c_ctr++;
                sm_state = RX_GPS_GNRMC;
            }
            else
            {
                c_ctr = 0;
                sm_state = RX_GPS_W8_SOF;
            }
            break;
            
            
        case RX_GPS_TIME:
            if(msg[c_ctr] == ',' || t_ctr == 10)
            {
                sm_state = RX_GPS_VALID;
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
                c_ctr++;
                sm_state++;
            }
            break;

        case RX_GPS_DATE:
            if(msg[c_ctr] == ',' || d_ctr == 6)
            {
                sm_state = RX_GPS_MAG_VAR;
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
            if(msg[c_ctr] == ',')
            {                      
                sm_state++;
            }
            else if (msg[c_ctr] == '*')
            {
                sm_state = RX_GPS_CRC;
            }
            c_ctr++;
            break;

        case RX_GPS_CRC:
        case RX_GPS_CR:
        case RX_GPS_LF:
            c_ctr = 0;
            sm_state = RX_GPS_W8_SOF;
            break;
        
        default:
            break;
    }
    return sm_state;
}

void GPS_CalculateTime( uint8_t* time_c, uint8_t* date_c, TimeInternal* time_stamp)
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
    ns += (time_c[7] - '0') * 10000000U;
    ns += (time_c[7] - '0') * 1000000U;
    
    time_stamp->seconds = (int32_t) t_of_day;
    time_stamp->nanoseconds = (int32_t) ns;
}

void GPS_thread(void const * argument)
{
    
    static uint8_t time [10], date[6];
    uint8_t aaa[40];
    Rx_GPS_SM_t state = RX_GPS_W8_SOF;
    TimeInternal time_stamp;
    osEvent event;
    
    usart6_m_q_id = osMessageCreate(osMessageQ(usart6_m_q), NULL);
    //MX_DMA_Init();
    
    MX_USART6_UART_Init();
    huart6.RxISR = USART6_Rx_ISR;
    while(1)
    {
       // HAL_UART_Receive_IT(&huart6,aaa,40);
        event = osMessageGet(usart6_m_q_id, osWaitForever);
        if (event.status == osEventMessage)
        {
            state = GPS_SM(time, date, *((uint8_t*)(event.value.p)));
            if(state == RX_GPS_LF)
            {
                GPS_CalculateTime(time, date, &time_stamp);
            }
        }
    }
    
}


void USART6_Rx_ISR(struct __UART_HandleTypeDef *huart)
{

   
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t c = 0;
    
    HAL_StatusTypeDef uart_status = HAL_ERROR;
    uart_status = HAL_UART_Receive_IT(huart, &c, 1);
    if (HAL_OK == uart_status )
    {
       osMessagePut(usart6_m_q_id, c, 0);
    }
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
