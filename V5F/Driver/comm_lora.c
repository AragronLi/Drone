/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : comm_lora.c
 * 描述                : ATK LoRa 数传模块 USART8 驱动实现
 *                      USART8: PE8(TX) + PE7(RX), 中断驱动接收 FIFO
 *******************************************************************************/

#include "comm_lora.h"

/* ---- 接收环形 FIFO ---- */
static struct {
    uint8_t  buf[COMM_LORA_UART_RX_FIFO_SIZE];
    uint16_t size;
    uint16_t reader;
    uint16_t writer;
} g_lora_rx_fifo;

/*********************************************************************
 * @fn      comm_lora_init
 * @brief   初始化 USART8 用于 ATK LoRa 数传通信
 * @param   baudrate: 串口波特率 (默认 115200)
 */
void comm_lora_init(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure  = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    /* 时钟使能: USART8 挂载于 HB1, GPIOE 挂载于 HB2 */
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART8, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOE, ENABLE);

    /* USART8_TX = PE8, AF7, 复用推挽输出 */
    GPIO_PinAFConfig(COMM_LORA_UART_TX_PORT, COMM_LORA_UART_TX_PIN_SRC, COMM_LORA_UART_GPIO_AF);
    GPIO_InitStructure.GPIO_Pin   = COMM_LORA_UART_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(COMM_LORA_UART_TX_PORT, &GPIO_InitStructure);

    /* USART8_RX = PE7, AF7, 浮空输入 */
    GPIO_PinAFConfig(COMM_LORA_UART_RX_PORT, COMM_LORA_UART_RX_PIN_SRC, COMM_LORA_UART_GPIO_AF);
    GPIO_InitStructure.GPIO_Pin  = COMM_LORA_UART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(COMM_LORA_UART_RX_PORT, &GPIO_InitStructure);

    /* USART8 参数配置: 8N1 */
    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(COMM_LORA_UART, &USART_InitStructure);
    USART_Cmd(COMM_LORA_UART, ENABLE);

    /* 使能接收中断 RXNE */
    USART_ITConfig(COMM_LORA_UART, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(COMM_LORA_UART_IRQn);

    /* 初始化接收 FIFO */
    g_lora_rx_fifo.size   = COMM_LORA_UART_RX_FIFO_SIZE;
    g_lora_rx_fifo.reader = 0;
    g_lora_rx_fifo.writer = 0;
}

/*********************************************************************
 * @fn      comm_lora_send_bytes
 * @brief   通过 USART8 阻塞发送字节数组
 * @param   dat : 数据缓冲区
 *          len : 数据长度
 */
void comm_lora_send_bytes(const uint8_t *dat, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        while (USART_GetFlagStatus(COMM_LORA_UART, USART_FLAG_TXE) == RESET);
        USART_SendData(COMM_LORA_UART, dat[i]);
    }
}

/*********************************************************************
 * @fn      comm_lora_available
 * @brief   返回接收 FIFO 中可读字节数
 * @retval  可读字节数
 */
uint16_t comm_lora_available(void)
{
    if (g_lora_rx_fifo.writer >= g_lora_rx_fifo.reader) {
        return g_lora_rx_fifo.writer - g_lora_rx_fifo.reader;
    } else {
        return g_lora_rx_fifo.size - g_lora_rx_fifo.reader + g_lora_rx_fifo.writer;
    }
}

/*********************************************************************
 * @fn      comm_lora_read_byte
 * @brief   从接收 FIFO 读取一个字节 (调用前需确保 available > 0)
 * @retval  读取的字节
 */
uint8_t comm_lora_read_byte(void)
{
    uint8_t dat = g_lora_rx_fifo.buf[g_lora_rx_fifo.reader];
    g_lora_rx_fifo.reader = (g_lora_rx_fifo.reader + 1) % g_lora_rx_fifo.size;
    return dat;
}

/*********************************************************************
 * @fn      comm_lora_flush
 * @brief   清空接收 FIFO
 */
void comm_lora_flush(void)
{
    g_lora_rx_fifo.reader = g_lora_rx_fifo.writer;
}

/*********************************************************************
 * @fn      COMM_LORA_UART_IRQHandler
 * @brief   USART8 中断处理函数, 由 ch32h417_it.c 中的 USART8_IRQHandler 调用
 *          将接收到的字节写入 RX FIFO
 */
void COMM_LORA_UART_IRQHandler(void)
{
    uint8_t tmp;

    /* 溢出错误 ORE — 先读 STATR 再读 DATAR 清除标志 */
    if (USART_GetFlagStatus(COMM_LORA_UART, USART_FLAG_ORE) != RESET) {
        (void)COMM_LORA_UART->STATR;
        (void)COMM_LORA_UART->DATAR;
    }

    /* 接收缓冲区非空 RXNE — 读取字节存入 FIFO */
    if (USART_GetFlagStatus(COMM_LORA_UART, USART_FLAG_RXNE) != RESET) {
        tmp = (uint8_t)USART_ReceiveData(COMM_LORA_UART);
        g_lora_rx_fifo.buf[g_lora_rx_fifo.writer] = tmp;
        g_lora_rx_fifo.writer = (g_lora_rx_fifo.writer + 1) % g_lora_rx_fifo.size;
    }
}
