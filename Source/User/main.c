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
#include "usmart.h"

void main(void)
{
    sched_Init();
    cpu_Init();
    bsp_Init();
    usmart_dev.init(CPU_FREQ_HZ/1000000);

    while (1)
    {
    }
}
