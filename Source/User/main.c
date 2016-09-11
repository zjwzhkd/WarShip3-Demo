/*******************************************************************************
* 文 件 名: main.c
* 创 建 者: Keda Huang
* 版    本: V1.0
* 创建日期: 2016-08-23
* 文件说明: 主函数
*******************************************************************************/

#include "cpu.h"
#include "sched.h"
#include "bsp.h"

void main(void)
{
char *recvbuf;

    sched_Init();
    cpu_Init();
    bsp_Init();

    recvbuf = (char *)bsp_DebugGetReceiveBuffer();
    bsp_DebugStartReceive();
    while (1)
    {
        if ( bsp_DebugIsReceiveComplete() )
        {
            debug_printf(recvbuf);
            debug_printf("\r\n");
            cpu_DelayMs(10);
            bsp_DebugStartReceive();
        }
    }
}
