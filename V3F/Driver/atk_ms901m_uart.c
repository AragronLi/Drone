/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : atk_ms901m_uart.c
 * 作者                : WCH
 * 版本                : V1.0.0
 * 日期                : 2025/05/28
 * 描述                : ATK-MS901M 十轴 IMU 串口接口层实现
 *                      USART3: PB10(TX) + PB11(RX), 中断驱动接收 FIFO
 *********************************************************************************
 * Copyright (c) 2025 南京沁恒微电子股份有限公司
 *******************************************************************************/

#include "atk_ms901m_uart.h"

/* ---- 接收环形 FIFO ---- */
static struct {
    uint8_t  buf[ATK_MS901M_UART_RX_FIFO_SIZE];  /* 缓冲区 */
    uint16_t size;                                /* 缓冲区大小 */
    uint16_t reader;                              /* 读指针 */
    uint16_t writer;                              /* 写指针 */
} g_uart_rx_fifo;

/*********************************************************************
 * @fn      atk_ms901m_uart_rx_fifo_write
 * @brief   向接收 FIFO 写入数据（中断服务函数中调用）
 * @param   dat : 数据缓冲区
 *          len : 数据长度
 * @retval  0: 写入成功（无溢出保护，覆盖旧数据）
 */
uint8_t atk_ms901m_uart_rx_fifo_write(uint8_t *dat, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        g_uart_rx_fifo.buf[g_uart_rx_fifo.writer] = dat[i];
        g_uart_rx_fifo.writer = (g_uart_rx_fifo.writer + 1) % g_uart_rx_fifo.size;
    }
    return 0;
}

/*********************************************************************
 * @fn      atk_ms901m_uart_rx_fifo_read
 * @brief   从接收 FIFO 读取数据
 * @param   dat : 输出缓冲区
 *          len : 期望读取的长度
 * @retval  实际读取的字节数
 */
uint16_t atk_ms901m_uart_rx_fifo_read(uint8_t *dat, uint16_t len)
{
    uint16_t fifo_usage;
    uint16_t i;

    /* 计算 FIFO 中已有的数据量 */
    if (g_uart_rx_fifo.writer >= g_uart_rx_fifo.reader) {
        fifo_usage = g_uart_rx_fifo.writer - g_uart_rx_fifo.reader;
    } else {
        fifo_usage = g_uart_rx_fifo.size - g_uart_rx_fifo.reader + g_uart_rx_fifo.writer;
    }

    /* FIFO 数据不足时仅返回现有数据 */
    if (len > fifo_usage) {
        len = fifo_usage;
    }

    for (i = 0; i < len; i++) {
        dat[i] = g_uart_rx_fifo.buf[g_uart_rx_fifo.reader];
        g_uart_rx_fifo.reader = (g_uart_rx_fifo.reader + 1) % g_uart_rx_fifo.size;
    }

    return len;
}

/*********************************************************************
 * @fn      atk_ms901m_uart_rx_fifo_flush
 * @brief   清空接收 FIFO
 */
void atk_ms901m_uart_rx_fifo_flush(void)
{
    g_uart_rx_fifo.writer = g_uart_rx_fifo.reader;
}

/*********************************************************************
 * @fn      atk_ms901m_uart_send
 * @brief   通过 USART3 发送数据（阻塞模式）
 * @param   dat : 数据缓冲区
 *          len : 数据长度
 */
void atk_ms901m_uart_send(uint8_t *dat, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++) {
        while (USART_GetFlagStatus(ATK_MS901M_UART, USART_FLAG_TXE) == RESET);
        USART_SendData(ATK_MS901M_UART, dat[i]);
    }
}

/*********************************************************************
 * @fn      atk_ms901m_uart_init
 * @brief   初始化 USART3 用于 ATK-MS901M 通信
 * @param   baudrate: 串口波特率（如 115200 / 460800）
 */
void atk_ms901m_uart_init(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure  = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    /* 时钟使能: USART3 挂载于 HB1, GPIOB 挂载于 HB2 */
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART3, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOB, ENABLE);

    /* USART3_TX = PB10, AF7, 复用推挽输出 */
    GPIO_PinAFConfig(ATK_MS901M_UART_TX_PORT, ATK_MS901M_UART_TX_PIN_SRC, ATK_MS901M_UART_GPIO_AF);
    GPIO_InitStructure.GPIO_Pin   = ATK_MS901M_UART_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(ATK_MS901M_UART_TX_PORT, &GPIO_InitStructure);

    /* USART3_RX = PB11, AF7, 浮空输入 */
    GPIO_PinAFConfig(ATK_MS901M_UART_RX_PORT, ATK_MS901M_UART_RX_PIN_SRC, ATK_MS901M_UART_GPIO_AF);
    GPIO_InitStructure.GPIO_Pin  = ATK_MS901M_UART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ATK_MS901M_UART_RX_PORT, &GPIO_InitStructure);

    /* USART3 参数配置 */
    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(ATK_MS901M_UART, &USART_InitStructure);
    USART_Cmd(ATK_MS901M_UART, ENABLE);

    /* 使能接收中断 RXNE */
    USART_ITConfig(ATK_MS901M_UART, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(ATK_MS901M_UART_IRQn);

    /* 初始化接收 FIFO */
    g_uart_rx_fifo.size   = ATK_MS901M_UART_RX_FIFO_SIZE;
    g_uart_rx_fifo.reader = 0;
    g_uart_rx_fifo.writer = 0;
}

/*********************************************************************
 * @fn      ATK_MS901M_UART_IRQHandler
 * @brief   USART3 中断处理函数，由 ch32h417_it.c 中的 USART3_IRQHandler 调用
 *          清除溢出标志，将接收到的字节写入 RX FIFO
 */
void ATK_MS901M_UART_IRQHandler(void)
{
    uint8_t tmp;

    /* 溢出错误 ORE — 先读 STATR 再读 DATAR 清除标志 */
    if (USART_GetFlagStatus(ATK_MS901M_UART, USART_FLAG_ORE) != RESET) {
        (void)ATK_MS901M_UART->STATR;
        (void)ATK_MS901M_UART->DATAR;
    }

    /* 接收缓冲区非空 RXNE — 读取字节存入 FIFO */
    if (USART_GetFlagStatus(ATK_MS901M_UART, USART_FLAG_RXNE) != RESET) {
        tmp = (uint8_t)USART_ReceiveData(ATK_MS901M_UART);
        atk_ms901m_uart_rx_fifo_write(&tmp, 1);
    }
}
