/*******************************************************************************
* 文 件 名: bsp_debug.c
* 创 建 者: Keda Huang
* 版    本: V1.0
* 创建日期: 2016-09-07
* 文件说明: 串口调试模块
*******************************************************************************/

#include "bsp_debug.h"
#include "cpulib_fifo.h"
/*******************************************************************************

                                    全局变量

*******************************************************************************/
struct _debug_dev
{
    FIFO_t     *tx_fifo;    /*发送FIFO    */
    FIFO_t     *rx_fifo;    /*接收FIFO    */
    uint8_t     rx_flag;    /*接收完成标志*/
};
static struct _debug_dev debug_dev;
static STRUCT_FIFO(uint8_t, BSP_DEBUG_TX_BUF_SIZE) uartTxFifo;
static STRUCT_FIFO(uint8_t, BSP_DEBUG_RX_BUF_SIZE) uartRxFifo;

static void prvUartInit(void);
static void prvUartStartSend(void);
/*******************************************************************************

                                    接口函数

*******************************************************************************/
/**
 * 初始化串口调试
 */
void bsp_Debug_Init(void)
{
    INIT_FIFO(&uartTxFifo);
    INIT_FIFO(&uartRxFifo);
    debug_dev.tx_fifo = &uartTxFifo.fifo;
    debug_dev.rx_fifo = &uartRxFifo.fifo;
    debug_dev.rx_flag = 0;

    prvUartInit();
}

/**
 * 开始新行的接收
 *
 * @note: 当接收到CR,LF,或者接收缓存满时, 完成一行内容的接收, 接收完成标志置位
 */
void bsp_Debug_ReceiveNew(void)
{
cpu_t cpu_sr;

    cpu_sr = CPU_EnterCritical();
    {
        fifo_Reset(debug_dev.rx_fifo);
        debug_dev.rx_flag = 0;
    }
    CPU_ExitCritical(cpu_sr);
}

/**
 * 判断接收是否完成
 *
 * @return: 布尔值, 完成接收返回true, 反之返回false
 */
bool bsp_Debug_IsReceiveComplete(void)
{
    if ( debug_dev.rx_flag )
    {
        return (true);
    }
    else
    {
        return (false);
    }
}

/**
 * 获取接收缓存地址
 *
 * @return: 返回接收缓存地址
 */
uint8_t *bsp_Debug_GetReceiveBuffer(void)
{
    return (uartRxFifo.buf);
}

/**
 * 获取已接收内容的长度
 *
 * @return: 返回已接收内容的长度
 */
uint16_t bsp_Debug_GetReceiveLen(void)
{
    return (fifo_GetCount(debug_dev.rx_fifo));
}

/*文件流重定向*/
int fputc(int ch, FILE *f)
{
cpu_t cpu_sr;
char c = ch;

    cpu_sr = CPU_EnterCritical();
    {
        fifo_In(debug_dev.tx_fifo, &c, 1);
    }
    CPU_ExitCritical(cpu_sr);
    prvUartStartSend();

    return (ch);
}

/*******************************************************************************

                                    Uart外设

*******************************************************************************/
/* 端口定义 ------------------------------------------------------------------*/
#define UART_x                      USART1
#define GPIO_PORT_TX                GPIOA
#define GPIO_PORT_RX                GPIOA
#define GPIO_PIN_TX                 GPIO_Pin_9
#define GPIO_PIN_RX                 GPIO_Pin_10

#define UART_x_IRQn                 USART1_IRQn
#define UART_x_IRQHandler           USART1_IRQHandler
#define UART_x_PreemptPriority      ( 15 )

#define RCC_UART_CLK_ENABLE()       RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE)
#define RCC_GPIO_CLK_ENABLE()       RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE)

/* 外设驱动 ------------------------------------------------------------------*/
/*初始化UART*/
static void prvUartInit(void)
{
GPIO_InitTypeDef GPIO_InitStruct;
USART_InitTypeDef USART_InitStruct;

    /*外设时钟使能*/
    RCC_UART_CLK_ENABLE();
    RCC_GPIO_CLK_ENABLE();

    /*GPIO初始化*/
    GPIO_InitStruct.GPIO_Pin   = GPIO_PIN_TX;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIO_PORT_TX, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin   = GPIO_PIN_RX;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIO_PORT_RX, &GPIO_InitStruct);

    /*初始化UART*/
    USART_InitStruct.USART_BaudRate   = BSP_DEBUG_UART_BAUDRATE;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits   = USART_StopBits_1;
    USART_InitStruct.USART_Parity     = USART_Parity_No;
    USART_InitStruct.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART_x, &USART_InitStruct);

    /*设置中断*/
    cpu_NVIC_SetPriority(UART_x_IRQn, UART_x_PreemptPriority, 0);
    cpu_NVIC_EnableIRQ(UART_x_IRQn);
    USART_ITConfig(UART_x, USART_IT_RXNE, ENABLE);

    /*使能外设*/
    USART_Cmd(UART_x, ENABLE);
}

/*启动UART发送*/
static void prvUartStartSend(void)
{
    USART_ITConfig(UART_x, USART_IT_TXE, ENABLE);
}

/* 中断函数 ------------------------------------------------------------------*/
void UART_x_IRQHandler(void)
{
    /*UART接收缓存非空*/
    if ( RESET != USART_GetITStatus(UART_x, USART_IT_RXNE) )
    {
    uint8_t recvData;

        recvData = UART_x->DR;
        if ( 0 == debug_dev.rx_flag )
        {
            if ( ('\r' == recvData) || ('\n' == recvData) || (1 == fifo_GetAvail(debug_dev.rx_fifo)) )
            {
                recvData = 0;
                fifo_In(debug_dev.rx_fifo, &recvData, 1);
                debug_dev.rx_flag = 1;
            }
            else if ( (recvData>=0x20) && (recvData<0x7f) )
            {
                fifo_In(debug_dev.rx_fifo, &recvData, 1);
            }
        }
    }

    /*UART发送缓存空*/
    if ( RESET != USART_GetITStatus(UART_x, USART_IT_TXE) )
    {
    uint8_t sendData;
    uint8_t len;
    cpu_t cpu_sr;

        cpu_sr = CPU_EnterCriticalFromISR();
        {
            len = fifo_Out(debug_dev.tx_fifo, &sendData, 1);
        }
        CPU_ExitCriticalFromISR(cpu_sr);

        if ( 1 == len )
        {
            UART_x->DR = sendData;
        }
        else
        {
            USART_ITConfig(UART_x, USART_IT_TXE, DISABLE);
        }
    }
}
