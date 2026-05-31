/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : bsp_tof_tfmini.c
 * 描述                : TFmini Plus / VI530x 激光测距模块驱动
 *                       USART6 RX: PC11, AF8, HB1, 115200-8N1
 *                       中断接收 + 状态机解析
 *******************************************************************************/
#include "bsp_tof_tfmini.h"

#define FRAME_HEADER  0xBB
#define FRAME_LEN      6

static tfmini_data_t g_tfmini_data;
volatile uint32_t g_tof_irq_count;
volatile uint32_t g_tof_frame_count;
volatile uint32_t g_tof_crc_error_count;
volatile uint32_t g_tof_len_error_count;
volatile uint8_t  g_tof_last_rx_byte;
volatile uint8_t  g_tof_last_frame[FRAME_LEN];
volatile uint32_t g_tof_poll_count;
volatile uint8_t  g_tof_poll_last_byte;

/* 接收状态机 */
static uint8_t  rx_buf[FRAME_LEN];
static uint8_t  rx_idx;
static uint8_t  rx_sync;    /* 0=找帧头, 1=收数据 */

/*********************************************************************
 * @fn      bsp_tof_tfmini_init
 * @brief   初始化 USART6 RX 中断接收 (PC11, AF8)
 */
void bsp_tof_tfmini_init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure = {0};
    USART_InitTypeDef       USART_InitStructure = {0};

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART6, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOC | RCC_HB2Periph_AFIO, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF8);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART6, &USART_InitStructure);

    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART6_IRQn);

    USART_Cmd(USART6, ENABLE);
}

/*********************************************************************
 * @fn      bsp_tof_tfmini_poll
 * @brief   读取最新解析好的 TOF 数据, 读完清 updated
 */
void bsp_tof_tfmini_poll(tfmini_data_t *data)
{
    if (g_tfmini_data.updated) {
        data->distance_mm = g_tfmini_data.distance_mm;
        data->confidence  = g_tfmini_data.confidence;
        data->updated     = 1;
        g_tfmini_data.updated = 0;
    } else {
        data->updated = 0;
    }
}

/*********************************************************************
 * @fn      USART6_IRQHandler
 * @brief   USART6 RX 中断: 接收 6 字节帧, CRC 校验后更新全局数据
 *
 * 状态机:
 *   非帧头 → 跳过等待 0xBB
 *   帧头   → 从 0 开始收 6 字节
 *   收满 6 字节 → CRC 校验 → 解析 → 更新 g_tfmini_data
 */
void USART6_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART6_IRQHandler(void)
{
    if (USART_GetITStatus(USART6, USART_IT_RXNE) != SET)
        return;

    g_tof_irq_count++;
    uint8_t ch = USART_ReceiveData(USART6);
    g_tof_last_rx_byte = ch;

    if (rx_sync == 0) {
        if (ch == FRAME_HEADER) {
            rx_buf[0] = ch;
            rx_idx    = 1;
            rx_sync   = 1;
        }
    } else {
        rx_buf[rx_idx++] = ch;
        if (rx_idx >= FRAME_LEN) {
            rx_sync = 0;

            for (uint8_t i = 0; i < FRAME_LEN; i++) {
                g_tof_last_frame[i] = rx_buf[i];
            }

            if (rx_buf[1] != 0x03) {
                g_tof_len_error_count++;
                return;
            }

            /* CRC: 前 5 字节累加和 */
            uint8_t sum = 0;
            for (uint8_t i = 0; i < FRAME_LEN - 1; i++)
                sum += rx_buf[i];

            if (sum == rx_buf[FRAME_LEN - 1]) {
                g_tfmini_data.distance_mm = rx_buf[2] | ((uint16_t)rx_buf[3] << 8);
                g_tfmini_data.confidence  = rx_buf[4];
                g_tfmini_data.updated     = 1;
                g_tof_frame_count++;
            } else {
                g_tof_crc_error_count++;
            }
        }
    }
}
