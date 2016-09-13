/*******************************************************************************
* 文 件 名: bsp_sdio_sd.c
* 创 建 者: Keda Huang
* 版    本: V1.0
* 创建日期: 2016-09-12
* 文件说明: SD卡SDIO接口的驱动函数库, 移植自STM32F10E_EVAL例程
*******************************************************************************/

#include "bsp_sdio_sd.h"
#include <string.h>
/*******************************************************************************

                                    参数定义

*******************************************************************************/
/*SDIO数据块大小*/
#define DATA_BLOCK_SIZE                 ((uint32_t)SDIO_DataBlockSize_512b)

/* SDIO Static flags, TimeOut, FIFO Address */
#define SDIO_STATIC_FLAGS               ((uint32_t)(SDIO_FLAG_CCRCFAIL | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_CTIMEOUT |\
                                                    SDIO_FLAG_DTIMEOUT | SDIO_FLAG_TXUNDERR | SDIO_FLAG_RXOVERR  |\
                                                    SDIO_FLAG_CMDREND  | SDIO_FLAG_CMDSENT  | SDIO_FLAG_DATAEND  |\
                                                    SDIO_FLAG_DBCKEND))
#define SDIO_CMD0TIMEOUT                ((uint32_t)0x00010000)
#define SDIO_FIFO_ADDRESS               ((uint32_t)0x40018080)

/*
    SDIO时钟计算公式:SDIO_CK时钟=SDIOCLK/[clkdiv+2];其中,SDIOCLK一般为72Mhz,
    SDIO初始化频率,最大400KHz,
    SDIO传输频率,最大24MHz
*/
#define SDIO_INIT_CLK_DIV               ((uint8_t)0xB2)
#define SDIO_TRANSFER_CLK_DIV           ((uint8_t)0x01)

/*******************************************************************************

                                   内部宏定义

*******************************************************************************/
/* Mask for errors Card Status R1 (OCR Register) */
#define SD_OCR_ADDR_OUT_OF_RANGE        ((uint32_t)0x80000000)
#define SD_OCR_ADDR_MISALIGNED          ((uint32_t)0x40000000)
#define SD_OCR_BLOCK_LEN_ERR            ((uint32_t)0x20000000)
#define SD_OCR_ERASE_SEQ_ERR            ((uint32_t)0x10000000)
#define SD_OCR_BAD_ERASE_PARAM          ((uint32_t)0x08000000)
#define SD_OCR_WRITE_PROT_VIOLATION     ((uint32_t)0x04000000)
#define SD_OCR_LOCK_UNLOCK_FAILED       ((uint32_t)0x01000000)
#define SD_OCR_COM_CRC_FAILED           ((uint32_t)0x00800000)
#define SD_OCR_ILLEGAL_CMD              ((uint32_t)0x00400000)
#define SD_OCR_CARD_ECC_FAILED          ((uint32_t)0x00200000)
#define SD_OCR_CC_ERROR                 ((uint32_t)0x00100000)
#define SD_OCR_GENERAL_UNKNOWN_ERROR    ((uint32_t)0x00080000)
#define SD_OCR_STREAM_READ_UNDERRUN     ((uint32_t)0x00040000)
#define SD_OCR_STREAM_WRITE_OVERRUN     ((uint32_t)0x00020000)
#define SD_OCR_CID_CSD_OVERWRIETE       ((uint32_t)0x00010000)
#define SD_OCR_WP_ERASE_SKIP            ((uint32_t)0x00008000)
#define SD_OCR_CARD_ECC_DISABLED        ((uint32_t)0x00004000)
#define SD_OCR_ERASE_RESET              ((uint32_t)0x00002000)
#define SD_OCR_AKE_SEQ_ERROR            ((uint32_t)0x00000008)
#define SD_OCR_ERRORBITS                ((uint32_t)0xFDFFE008)

/* Masks for R6 Response */
#define SD_R6_GENERAL_UNKNOWN_ERROR     ((uint32_t)0x00002000)
#define SD_R6_ILLEGAL_CMD               ((uint32_t)0x00004000)
#define SD_R6_COM_CRC_FAILED            ((uint32_t)0x00008000)

#define SD_VOLTAGE_WINDOW_SD            ((uint32_t)0x80100000)
#define SD_HIGH_CAPACITY                ((uint32_t)0x40000000)
#define SD_STD_CAPACITY                 ((uint32_t)0x00000000)
#define SD_CHECK_PATTERN                ((uint32_t)0x000001AA)

#define SD_MAX_VOLT_TRIAL               ((uint32_t)0x0000FFFF)
#define SD_ALLZERO                      ((uint32_t)0x00000000)

#define SD_WIDE_BUS_SUPPORT             ((uint32_t)0x00040000)
#define SD_SINGLE_BUS_SUPPORT           ((uint32_t)0x00010000)
#define SD_CARD_LOCKED                  ((uint32_t)0x02000000)

#define SD_DATATIMEOUT                  ((uint32_t)0xFFFFFFFF)
#define SD_0TO7BITS                     ((uint32_t)0x000000FF)
#define SD_8TO15BITS                    ((uint32_t)0x0000FF00)
#define SD_16TO23BITS                   ((uint32_t)0x00FF0000)
#define SD_24TO31BITS                   ((uint32_t)0xFF000000)
#define SD_MAX_DATA_LENGTH              ((uint32_t)0x01FFFFFF)

#define SD_HALFFIFO                     ((uint32_t)0x00000008)
#define SD_HALFFIFOBYTES                ((uint32_t)0x00000020)

/* Command Class Supported */
#define SD_CCCC_LOCK_UNLOCK             ((uint32_t)0x00000080)
#define SD_CCCC_WRITE_PROT              ((uint32_t)0x00000040)
#define SD_CCCC_ERASE                   ((uint32_t)0x00000020)

/*CMD8: 获取SD卡版本*/
#define SDIO_SEND_IF_COND               ((uint32_t)SD_CMD_HS_SEND_EXT_CSD)

/*******************************************************************************

                                    全局变量

*******************************************************************************/
/*SD卡类型*/
static uint32_t CardType = SDIO_STD_CAPACITY_SD_CARD_V1_1;
/*SD卡CSD,CID以及RCA数据*/
static uint32_t CSD_Tab[4], CID_Tab[4], RCA = 0;
/*SD卡状态数据*/
static uint8_t SDSTATUS_Tab[16];
/*需要发送停止传输命令的标志(用于DMA传输)*/
static __IO uint32_t StopCondition = 0;
/*传输错误码(用于DMA传输)*/
static __IO SD_Error TransferError = SD_OK;
/*SD/DMA传输结束标志(用于DMA传输)*/
static __IO uint32_t TransferEnd = 0, DMAEndOfTransfer = 0;
/*SD卡信息*/
SD_CardInfo SDCardInfo;

SDIO_InitTypeDef SDIO_InitStructure;
SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
SDIO_DataInitTypeDef SDIO_DataInitStructure;

static SD_Error SD_PowerON(void);
static SD_Error SD_PowerOFF(void);
static SD_Error SD_InitializeCards(void);
static SD_Error SD_SelectDeselect(uint32_t addr);
static SD_Error SD_SendStatus(uint32_t *pcardstatus);
static SD_Error CmdError(void);
static SD_Error CmdResp1Error(uint8_t cmd);
static SD_Error CmdResp7Error(void);
static SD_Error CmdResp3Error(void);
static SD_Error CmdResp2Error(void);
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t *prca);
static SD_Error SDEnWideBus(FunctionalState NewState);
static SD_Error IsCardProgramming(uint8_t *pstatus);
static SD_Error FindSCR(uint16_t rca, uint32_t *pscr);
static void SD_LowLevel_DeInit(void);
static void SD_LowLevel_Init(void);
static void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize);
static void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize);

/*******************************************************************************

                                    操作函数

*******************************************************************************/

/*******************************************************************************

                                    私有函数

*******************************************************************************/

/*******************************************************************************

                                    底层接口

*******************************************************************************/
/* 接口定义 ------------------------------------------------------------------*/
#define SD_SDIO_DMA                     DMA2
#define SD_SDIO_DMA_CLK                 RCC_AHBPeriph_DMA2
#define SD_SDIO_DMA_CHANNEL             DMA2_Channel4
#define SD_SDIO_DMA_FLAG_TC             DMA2_FLAG_TC4
#define SD_SDIO_DMA_FLAG_TE             DMA2_FLAG_TE4
#define SD_SDIO_DMA_FLAG_HT             DMA2_FLAG_HT4
#define SD_SDIO_DMA_FLAG_GL             DMA2_FLAG_GL4
#define SD_SDIO_DMA_IRQn                DMA2_Channel4_5_IRQn
#define SD_SDIO_DMA_IRQHANDLER          DMA2_Channel4_5_IRQHandler
#define SDIO_PreemptPriority            ( 0 )
#define SDIO_DMA_PreemptPriority        ( 1 )

/* 外设驱动 ------------------------------------------------------------------*/
/*SDIO接口解除初始化*/
void SD_LowLevel_DeInit(void)
{
GPIO_InitTypeDef  GPIO_InitStructure;

    /*!< Disable SDIO Clock */
    SDIO_ClockCmd(DISABLE);

    /*!< Set Power State to OFF */
    SDIO_SetPowerState(SDIO_PowerState_OFF);

    /*!< DeInitializes the SDIO peripheral */
    SDIO_DeInit();

    /*!< Disable the SDIO AHB Clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SDIO, DISABLE);

    /*!< Configure PC.08, PC.09, PC.10, PC.11, PC.12 pin: D0, D1, D2, D3, CLK pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*!< Configure PD.02 CMD line */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*SDIO接口底层初始化*/
void SD_LowLevel_Init(void)
{
GPIO_InitTypeDef  GPIO_InitStructure;

    /*!< GPIOC and GPIOD Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    /*!< Configure PC.08, PC.09, PC.10, PC.11, PC.12 pin: D0, D1, D2, D3, CLK pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*!< Configure PD.02 CMD line */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /*!< Enable the SDIO AHB Clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SDIO, ENABLE);

    /*!< Enable the DMA2 Clock */
    RCC_AHBPeriphClockCmd(SD_SDIO_DMA_CLK, ENABLE);

    /*设置NVIC*/
    cpu_NVIC_SetPriority(SDIO_IRQn, SDIO_PreemptPriority, 0);
    cpu_NVIC_SetPriority(SD_SDIO_DMA_IRQn, SDIO_DMA_PreemptPriority, 0);
    cpu_NVIC_EnableIRQ(SDIO_IRQn);
    cpu_NVIC_EnableIRQ(SD_SDIO_DMA_IRQn);
}

/**
 * 配置DMA通道实现SDIO发送
 *
 * @param BufferSRC: 源缓存地址
 *
 * @param BufferSize: 缓存大小(字节)
 */
void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize)
{
DMA_InitTypeDef DMA_InitStructure;

    DMA_ClearFlag(SD_SDIO_DMA_FLAG_TC | SD_SDIO_DMA_FLAG_TE | SD_SDIO_DMA_FLAG_HT | SD_SDIO_DMA_FLAG_GL);

    /*!< DMA2 Channel4 disable */
    DMA_Cmd(SD_SDIO_DMA_CHANNEL, DISABLE);

    /*!< SDIO DMA CHANNEL Config */
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferSRC;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = BufferSize / 4;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(SD_SDIO_DMA_CHANNEL, &DMA_InitStructure);

    DMA_ITConfig(SD_SDIO_DMA_CHANNEL, DMA_IT_TC, ENABLE);

    /*!< SDIO DMA CHANNEL enable */
    DMA_Cmd(SD_SDIO_DMA_CHANNEL, ENABLE);
}

/**
 * 配置DMA通道实现SDIO接收
 *
 * @param BufferDST: 目标缓存地址
 *
 * @param BufferSize: 缓存大小(字节)
 */
void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize)
{
DMA_InitTypeDef DMA_InitStructure;

    DMA_ClearFlag(SD_SDIO_DMA_FLAG_TC | SD_SDIO_DMA_FLAG_TE | SD_SDIO_DMA_FLAG_HT | SD_SDIO_DMA_FLAG_GL);

    /*!< SDIO DMA CHANNEL disable */
    DMA_Cmd(SD_SDIO_DMA_CHANNEL, DISABLE);

    /*!< SDIO DMA CHANNEL Config */
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferDST;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = BufferSize / 4;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(SD_SDIO_DMA_CHANNEL, &DMA_InitStructure);

    DMA_ITConfig(SD_SDIO_DMA_CHANNEL, DMA_IT_TC, ENABLE);

    /*!< SDIO DMA CHANNEL enable */
    DMA_Cmd(SD_SDIO_DMA_CHANNEL, ENABLE);
}

/* 中断函数 ------------------------------------------------------------------*/
