#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__


#include "lwip/err.h"
#include "lwip/netif.h"
#include "cmsis_os.h"

/* Exported types ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
err_t ethernetif_init(struct netif *netif);

/*********************************************
*$#>Added compilation purpose *
**********************************************/

typedef struct sptptime {
  int32_t tv_sec;
  int32_t tv_nsec;
}ptptime_t;

#if LWIP_PTP
void ETH_PTPTime_SetTime(ptptime_t * timestamp);
void ETH_PTPTime_GetTime(ptptime_t * timestamp);
void ETH_PTPTime_UpdateOffset(ptptime_t * timeoffset);
void ETH_PTPTime_AdjFreq(int32_t Adj);

/* Examples of subsecond increment and addend values using SysClk = 144 MHz
 
 Addend * Increment = 2^63 / SysClk

 ptp_tick = Increment * 10^9 / 2^31

 +-----------+-----------+------------+
 | ptp tick  | Increment | Addend     |
 +-----------+-----------+------------+
 |  119 ns   |   255     | 0x0EF8B863 |
 |  100 ns   |   215     | 0x11C1C8D5 |
 |   50 ns   |   107     | 0x23AE0D90 |
 |   20 ns   |    43     | 0x58C8EC2B |
 |   14 ns   |    30     | 0x7F421F4F |
 +-----------+-----------+------------+
*/

/* Examples of subsecond increment and addend values using SysClk = 168 MHz
 
 Addend * Increment = 2^63 / SysClk

 ptp_tick = Increment * 10^9 / 2^31

 +-----------+-----------+------------+
 | ptp tick  | Increment | Addend     |
 +-----------+-----------+------------+
 |  119 ns   |   255     | 0x0CD53055 |
 |  100 ns   |   215     | 0x0F386300 |
 |   50 ns   |   107     | 0x1E953032 |
 |   20 ns   |    43     | 0x4C19EF00 |
 |   14 ns   |    30     | 0x6D141AD6 |
 +-----------+-----------+------------+ 
*/

/*
HCLK = 200MHz
Sytem time update logic requires 50MHz to achieve 20ns accuracy
200MHz/50Mhz = 4
2^32 / 4 = 0x40000000
*/
                                    
#define ADJ_FREQ_BASE_ADDEND      0x3FECD300 // 0x3FECD300 //0x40000000 
//0x58C8EC2B
#define ADJ_FREQ_BASE_INCREMENT   43
/*********************************************
*$#> <end>
**********************************************/
#endif /* LWIP_PTP */

#endif
