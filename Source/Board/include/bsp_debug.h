/*******************************************************************************
* 文 件 名: bsp_debug.h
* 创 建 者: Keda Huang
* 版    本: V1.0
* 创建日期: 2016-09-07
* 文件说明: 串口调试模块
*******************************************************************************/

#ifndef __BSP_DEBUG_H
#define __BSP_DEBUG_H

/* 头文件 --------------------------------------------------------------------*/
#include "cpu.h"

/* 参数配置 ------------------------------------------------------------------*/
#define BSP_DEBUG_UART_BAUDRATE     ( 115200 )  /* 串口波特率                 */
#define BSP_DEBUG_TX_BUF_SIZE       ( 1024 )    /* 串口发送缓存大小           */
#define BSP_DEBUG_RX_BUF_SIZE       ( 256 )     /* 串口接收缓存大小           */

/* 接口函数 ------------------------------------------------------------------*/
void bsp_DebugInit(void);
void bsp_DebugStartReceive(void);
bool bsp_DebugIsReceiveComplete(void);
uint8_t *bsp_DebugGetReceiveBuffer(void);
uint16_t bsp_DebugGetReceiveLen(void);

#endif  /* __BSP_DEBUG_H */
