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

#include "cmsis_os.h"
#include "dma.h"
#include <time.h>
#include "ptpd.h"
#include "stm32f769i_discovery.h"
#include "ethernetif.h"

#define RX_BUF_SLICE_SIZE               ((uint16_t)(16))
#define RX_BUF_SIZE                     ((uint16_t)(400))
#define GPS_PMTK_NORMAL_MODE            "$PMTK225,0*2B\r\n"
#define GPS_PMTK_1S_PERIOD              "$PMTK220,1000*1F\r\n"
#define GPS_PMTK_PPS_ON                 "$PMTK255,1*2D\r\n"
#define GPS_PMTK_PPS_1MS_PULSE          "$PMTK285,4,100*38\r\n"

typedef struct 
{
    uint16_t start_idx;
    uint16_t data_length;
}Uart_Rx_Control_t;

typedef enum 
{
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
static osMessageQDef(usart6_m_q, 8, Uart_Rx_Control_t); // Declare a message queue
static osMessageQId (usart6_m_q_id);           // Declare an ID for the message queue

static osMessageQDef(gps_time_q, 4, ptptime_t); // Declare a message queue
static osMessageQId (gps_time_q_id);           // Declare an ID for the message queue

volatile static uint8_t uart_rx_buf[RX_BUF_SIZE];
volatile static Uart_Rx_Control_t uart_isr_rx_stat;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* Function prototypes */
static Rx_GPS_SM_t GPS_SM(uint8_t* time, uint8_t* date, uint8_t c);
static void GPS_CalculateTime( uint8_t* time_c, uint8_t* date_c, ptptime_t* time_stamp);
static void MX_USART6_UART_Init(void);
static void GPS_PPS_Init(void);
static void GPS_Init(void);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

volatile static uint16_t led_ctr = 0;
volatile static uint16_t uar_ctr = 0;
volatile static ptptime_t u_tim;
volatile static ptptime_t pps_tim;

void GPS_thread(void const * argument)
{
    static uint8_t time[11];
    static uint8_t date[7];
    static uint8_t pps_initilized_flag = 0;
    uint16_t end_idx = 0;
    Uart_Rx_Control_t uart_rx_stats;
    ptptime_t gps_time_stats;
    Rx_GPS_SM_t state = RX_GPS_W8_SOF;
    osEvent event;
    
    usart6_m_q_id = osMessageCreate(osMessageQ(usart6_m_q), NULL);
    gps_time_q_id = osMessageCreate(osMessageQ(gps_time_q), NULL);
    time[10] = NULL;
    date[6] = NULL;
    
    GPS_Init();
    HAL_UART_Receive_DMA(&huart6, (uint8_t *)&uart_rx_buf[0], RX_BUF_SLICE_SIZE);     
     while(1)
     {
        event = osMessageGet(usart6_m_q_id, osWaitForever);
        if (event.status == osEventMessage)
        {
            uart_rx_stats = *((Uart_Rx_Control_t*)event.value.p);
            end_idx = uart_rx_stats.start_idx + RX_BUF_SLICE_SIZE;
            if(end_idx > RX_BUF_SIZE - RX_BUF_SLICE_SIZE)
            {
                end_idx = 0;
            }
            HAL_UART_Receive_DMA(&huart6, (uint8_t *)&uart_rx_buf[end_idx], RX_BUF_SLICE_SIZE);
           
            end_idx = uart_rx_stats.start_idx + uart_rx_stats.data_length;
            for (uint16_t i = uart_rx_stats.start_idx; i <  end_idx; i++)
            {
                state = GPS_SM(time, date, uart_rx_buf[i]);
                uart_rx_buf[i] = 0xffu;
                if(state == RX_GPS_LF)
                {
                    ETH_PTPTime_GetTime( (ptptime_t*)&u_tim);
                    if(pps_initilized_flag == 0)
                    {
                        pps_initilized_flag = 1;
                        GPS_PPS_Init();  
                    }
                    //printf(":time: %s\n",time);
                    //printf(":date: %s\n",date);
                    GPS_CalculateTime(time, date, &gps_time_stats);
                    osMessagePut(gps_time_q_id,(uint32_t)&gps_time_stats,0);
                    uar_ctr++;
                    BSP_LED_Toggle(LED_GREEN);
                    //printf(":timestamp: %d.%d\n\n",time_stamp.seconds,time_stamp.nanoseconds);
                }
            }
        }
    }   
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    uint32_t addr = (uint32_t)uart_rx_buf;
    uart_isr_rx_stat.data_length = RX_BUF_SLICE_SIZE - UartHandle->hdmarx->Instance->NDTR;
    uart_isr_rx_stat.start_idx = (uint16_t)(UartHandle->hdmarx->Instance->M0AR - addr);
    osMessagePut(usart6_m_q_id,(uint32_t)&uart_isr_rx_stat,0);
}

volatile static osEvent pps_isr_event;
volatile static ptptime_t pps_isr_time_stats;
/* In this configuration EXTI IRQ is after UART frame  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    pps_isr_event = osMessageGet(gps_time_q_id, 0);
    if (pps_isr_event.status == osEventMessage)
    {
        pps_isr_time_stats = *((ptptime_t*)pps_isr_event.value.p);
    }
    ETH_PTPTime_GetTime( (ptptime_t*)&pps_tim);
    led_ctr++;
    BSP_LED_Toggle(LED_RED);
}


static void GPS_Init(void)
{
    BSP_LED_Init(LED_RED);
    BSP_LED_Init(LED_GREEN);
    BSP_LED_On(LED_RED);
    MX_DMA_Init();
    MX_USART6_UART_Init();
    BSP_LED_On(LED_GREEN);

    HAL_UART_Transmit(&huart6,(uint8_t*)GPS_PMTK_NORMAL_MODE, sizeof(GPS_PMTK_NORMAL_MODE)-1,100); //normal mode
    HAL_UART_Transmit(&huart6,(uint8_t*)GPS_PMTK_1S_PERIOD, sizeof(GPS_PMTK_1S_PERIOD)-1,100); //1s
    HAL_UART_Transmit(&huart6,(uint8_t*)GPS_PMTK_PPS_ON, sizeof(GPS_PMTK_PPS_ON)-1,100); // pps on
    HAL_UART_Transmit(&huart6,(uint8_t*)GPS_PMTK_PPS_1MS_PULSE, sizeof(GPS_PMTK_PPS_1MS_PULSE)-1,100); // pps 100ms pulse width
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


static void GPS_CalculateTime( uint8_t* time_c, uint8_t* date_c, ptptime_t* time_stamp)
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
    
    time_stamp->tv_sec = (int32_t) t_of_day;
    time_stamp->tv_nsec = (int32_t) ns;
}


static void MX_USART6_UART_Init(void)
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
    HAL_NVIC_SetPriority(USART6_IRQn, 0xF, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  }
}


static void GPS_PPS_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOJ_CLK_ENABLE();
    
    /*Configure GPIO pin : PJ1 */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);
    
    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI1_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
