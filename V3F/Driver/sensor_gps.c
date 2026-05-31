/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : sensor_gps.c
 * 描述                : SR2525M10D GPS NMEA0183 驱动实现
 *                       USART4: PC6/PC7, 中断接收 + 主循环解析
 *******************************************************************************/
#include "sensor_gps.h"
#include <string.h>

/* ---- 接收环形 FIFO ---- */
static struct {
    uint8_t  buf[SENSOR_GPS_RX_FIFO_SIZE];
    uint16_t reader;
    uint16_t writer;
} g_gps_rx_fifo;

static gps_data_t g_gps_data;
static char g_nmea_line[SENSOR_GPS_NMEA_LINE_MAX];
static uint16_t g_nmea_idx;
static uint8_t g_nmea_collecting;

static uint16_t fifo_available(void)
{
    if (g_gps_rx_fifo.writer >= g_gps_rx_fifo.reader) {
        return g_gps_rx_fifo.writer - g_gps_rx_fifo.reader;
    }
    return SENSOR_GPS_RX_FIFO_SIZE - g_gps_rx_fifo.reader + g_gps_rx_fifo.writer;
}

static uint8_t fifo_read(void)
{
    uint8_t dat = g_gps_rx_fifo.buf[g_gps_rx_fifo.reader];
    g_gps_rx_fifo.reader = (g_gps_rx_fifo.reader + 1) % SENSOR_GPS_RX_FIFO_SIZE;
    return dat;
}

static void copy_field(char *dst, uint8_t dst_len, const char *src, uint8_t len)
{
    uint8_t n = (len < (dst_len - 1)) ? len : (dst_len - 1);
    memcpy(dst, src, n);
    dst[n] = '\0';
}

static int32_t parse_int(const char *s)
{
    int32_t val = 0;
    int8_t neg = 0;

    if (*s == '-') {
        neg = 1;
        s++;
    }

    while (*s >= '0' && *s <= '9') {
        val = val * 10 + (*s - '0');
        s++;
    }

    return neg ? -val : val;
}

/* 解析十进制字段, 返回 value * scale */
static int32_t parse_decimal_scaled(const char *s, int32_t scale)
{
    int8_t neg = 0;
    int32_t ip = 0;
    int32_t fp = 0;
    int32_t div = 1;

    if (*s == '-') {
        neg = 1;
        s++;
    }

    while (*s >= '0' && *s <= '9') {
        ip = ip * 10 + (*s - '0');
        s++;
    }

    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9' && div < scale) {
            fp = fp * 10 + (*s - '0');
            div *= 10;
            s++;
        }
    }

    while (div < scale) {
        fp *= 10;
        div *= 10;
    }

    ip = ip * scale + fp;
    return neg ? -ip : ip;
}

/* NMEA ddmm.mmmmm / dddmm.mmmmm -> degE7 */
static int32_t parse_latlon_deg_e7(const char *field, char hemi)
{
    int32_t raw = parse_decimal_scaled(field, 100000); /* ddmm.mmmmm * 1e5 */
    int32_t deg = raw / (100 * 100000);
    int32_t min = raw - deg * 100 * 100000;
    int32_t deg_e7 = deg * 10000000 + (int32_t)(((int64_t)min * 10000000) / (60 * 100000));

    if (hemi == 'S' || hemi == 'W') {
        deg_e7 = -deg_e7;
    }

    return deg_e7;
}

static uint32_t parse_utc_ms(const char *field)
{
    uint32_t raw = (uint32_t)parse_decimal_scaled(field, 1000); /* hhmmss.sss */
    uint32_t hh = raw / (10000 * 1000);
    uint32_t mm = (raw / (100 * 1000)) % 100;
    uint32_t ss = (raw / 1000) % 100;
    uint32_t ms = raw % 1000;
    return ((hh * 60 + mm) * 60 + ss) * 1000 + ms;
}

static uint8_t hex_val(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0xFF;
}

static uint8_t nmea_checksum_ok(const char *line)
{
    uint8_t sum = 0;
    const char *p = line;

    if (*p != '$') return 0;
    p++;

    while (*p && *p != '*') {
        sum ^= (uint8_t)*p++;
    }

    if (*p != '*') return 0;
    if (hex_val(p[1]) > 0x0F || hex_val(p[2]) > 0x0F) return 0;

    return sum == (uint8_t)((hex_val(p[1]) << 4) | hex_val(p[2]));
}

static uint8_t split_fields(char *line, char *fields[], uint8_t max_fields)
{
    uint8_t count = 0;
    char *p = line;

    if (*p == '$') p++;

    fields[count++] = p;
    while (*p && *p != '*' && count < max_fields) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
        p++;
    }

    if (*p == '*') {
        *p = '\0';
    }

    return count;
}

static uint8_t talker_supported(const char *id)
{
    if ((id[0] == 'G' && (id[1] == 'P' || id[1] == 'N' || id[1] == 'L' || id[1] == 'A' || id[1] == 'Q')) ||
        (id[0] == 'B' && id[1] == 'D') ||
        (id[0] == 'G' && id[1] == 'B')) {
        return 1;
    }
    return 0;
}

static void parse_gga(char *fields[], uint8_t count)
{
    if (count < 10) return;

    if (fields[1][0]) g_gps_data.utc_time_ms = parse_utc_ms(fields[1]);
    if (fields[2][0] && fields[3][0] && fields[4][0] && fields[5][0]) {
        g_gps_data.latitude_deg_e7  = parse_latlon_deg_e7(fields[2], fields[3][0]);
        g_gps_data.longitude_deg_e7 = parse_latlon_deg_e7(fields[4], fields[5][0]);
    }
    g_gps_data.fix_quality = (uint8_t)parse_int(fields[6]);
    g_gps_data.fix_valid = (g_gps_data.fix_quality > 0) ? 1 : g_gps_data.fix_valid;
    g_gps_data.satellites_used = (uint8_t)parse_int(fields[7]);
    g_gps_data.hdop_x100 = (uint16_t)parse_decimal_scaled(fields[8], 100);
    g_gps_data.altitude_mm = parse_decimal_scaled(fields[9], 1000);
    if (count > 11 && fields[11][0]) {
        g_gps_data.geoid_separation_mm = parse_decimal_scaled(fields[11], 1000);
    }
    g_gps_data.updated = 1;
}

static void parse_rmc(char *fields[], uint8_t count)
{
    if (count < 10) return;

    if (fields[1][0]) g_gps_data.utc_time_ms = parse_utc_ms(fields[1]);
    g_gps_data.fix_valid = (fields[2][0] == 'A') ? 1 : 0;
    if (fields[3][0] && fields[4][0] && fields[5][0] && fields[6][0]) {
        g_gps_data.latitude_deg_e7  = parse_latlon_deg_e7(fields[3], fields[4][0]);
        g_gps_data.longitude_deg_e7 = parse_latlon_deg_e7(fields[5], fields[6][0]);
    }
    if (fields[7][0]) {
        uint32_t knots_x1000 = (uint32_t)parse_decimal_scaled(fields[7], 1000);
        g_gps_data.speed_cms = (uint32_t)(((uint64_t)knots_x1000 * 51444) / 1000000);
    }
    if (fields[8][0]) {
        g_gps_data.course_deg_x100 = (uint16_t)parse_decimal_scaled(fields[8], 100);
        g_gps_data.course_valid = 1;
    }
    if (fields[9][0]) g_gps_data.date_ddmmyy = (uint32_t)parse_int(fields[9]);
    g_gps_data.updated = 1;
}

static void parse_vtg(char *fields[], uint8_t count)
{
    if (count < 9) return;

    if (fields[1][0]) {
        g_gps_data.course_deg_x100 = (uint16_t)parse_decimal_scaled(fields[1], 100);
        g_gps_data.course_valid = 1;
    }
    if (fields[7][0]) {
        uint32_t kmh_x1000 = (uint32_t)parse_decimal_scaled(fields[7], 1000);
        g_gps_data.speed_cms = (uint32_t)(((uint64_t)kmh_x1000 * 100000) / 3600000);
    }
    g_gps_data.updated = 1;
}

static void parse_gsa(char *fields[], uint8_t count)
{
    if (count < 18) return;

    g_gps_data.fix_type = (uint8_t)parse_int(fields[2]);
    g_gps_data.pdop_x100 = (uint16_t)parse_decimal_scaled(fields[15], 100);
    g_gps_data.hdop_x100 = (uint16_t)parse_decimal_scaled(fields[16], 100);
    g_gps_data.vdop_x100 = (uint16_t)parse_decimal_scaled(fields[17], 100);
    g_gps_data.updated = 1;
}

static void parse_gsv(char *fields[], uint8_t count)
{
    if (count < 4) return;

    g_gps_data.satellites_visible = (uint8_t)parse_int(fields[3]);
    g_gps_data.updated = 1;
}

static void parse_gll(char *fields[], uint8_t count)
{
    if (count < 7) return;

    if (fields[1][0] && fields[2][0] && fields[3][0] && fields[4][0]) {
        g_gps_data.latitude_deg_e7  = parse_latlon_deg_e7(fields[1], fields[2][0]);
        g_gps_data.longitude_deg_e7 = parse_latlon_deg_e7(fields[3], fields[4][0]);
    }
    if (fields[5][0]) g_gps_data.utc_time_ms = parse_utc_ms(fields[5]);
    g_gps_data.fix_valid = (fields[6][0] == 'A') ? 1 : 0;
    g_gps_data.updated = 1;
}

static void parse_nmea_line(char *line)
{
    char *fields[32];
    uint8_t count;
    char sentence[6];

    g_gps_data.sentence_count++;
    if (!nmea_checksum_ok(line)) {
        g_gps_data.checksum_error_count++;
        return;
    }
    g_gps_data.checksum_ok_count++;

    count = split_fields(line, fields, 32);
    if (count == 0 || strlen(fields[0]) < 5) return;
    if (!talker_supported(fields[0])) return;

    copy_field(sentence, sizeof(sentence), fields[0] + 2, 3);
    copy_field(g_gps_data.last_sentence, sizeof(g_gps_data.last_sentence), sentence, 3);

    if (strcmp(sentence, "GGA") == 0) {
        parse_gga(fields, count);
    } else if (strcmp(sentence, "RMC") == 0) {
        parse_rmc(fields, count);
    } else if (strcmp(sentence, "VTG") == 0) {
        parse_vtg(fields, count);
    } else if (strcmp(sentence, "GSA") == 0) {
        parse_gsa(fields, count);
    } else if (strcmp(sentence, "GSV") == 0) {
        parse_gsv(fields, count);
    } else if (strcmp(sentence, "GLL") == 0) {
        parse_gll(fields, count);
    }
}

void sensor_gps_init(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure  = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART4, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOC | RCC_HB2Periph_AFIO, ENABLE);

    GPIO_PinAFConfig(SENSOR_GPS_UART_TX_PORT, SENSOR_GPS_UART_TX_PIN_SRC, SENSOR_GPS_UART_GPIO_AF);
    GPIO_InitStructure.GPIO_Pin   = SENSOR_GPS_UART_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(SENSOR_GPS_UART_TX_PORT, &GPIO_InitStructure);

    GPIO_PinAFConfig(SENSOR_GPS_UART_RX_PORT, SENSOR_GPS_UART_RX_PIN_SRC, SENSOR_GPS_UART_GPIO_AF);
    GPIO_InitStructure.GPIO_Pin  = SENSOR_GPS_UART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(SENSOR_GPS_UART_RX_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(SENSOR_GPS_UART, &USART_InitStructure);
    USART_Cmd(SENSOR_GPS_UART, ENABLE);

    memset(&g_gps_rx_fifo, 0, sizeof(g_gps_rx_fifo));
    memset(&g_gps_data, 0, sizeof(g_gps_data));

    USART_ITConfig(SENSOR_GPS_UART, USART_IT_RXNE, ENABLE);
    NVIC_SetPriority(SENSOR_GPS_UART_IRQn, 1);
    NVIC_EnableIRQ(SENSOR_GPS_UART_IRQn);

    g_gps_data.rxne_flag = (USART_GetFlagStatus(SENSOR_GPS_UART, USART_FLAG_RXNE) != RESET);
    g_gps_data.ore_flag  = (USART_GetFlagStatus(SENSOR_GPS_UART, USART_FLAG_ORE)  != RESET);
    g_gps_data.fe_flag   = (USART_GetFlagStatus(SENSOR_GPS_UART, USART_FLAG_FE)   != RESET);
    g_gps_data.ne_flag   = (USART_GetFlagStatus(SENSOR_GPS_UART, USART_FLAG_NE)   != RESET);
    g_nmea_idx = 0;
    g_nmea_collecting = 0;
}

void sensor_gps_poll(void)
{
    g_gps_data.rxne_flag = (USART_GetFlagStatus(SENSOR_GPS_UART, USART_FLAG_RXNE) != RESET);
    g_gps_data.ore_flag  = (USART_GetFlagStatus(SENSOR_GPS_UART, USART_FLAG_ORE)  != RESET);
    g_gps_data.fe_flag   = (USART_GetFlagStatus(SENSOR_GPS_UART, USART_FLAG_FE)   != RESET);
    g_gps_data.ne_flag   = (USART_GetFlagStatus(SENSOR_GPS_UART, USART_FLAG_NE)   != RESET);

    /* 调试兜底: 若中断未进但 RXNE 置位，主循环直接读出，便于区分“引脚/AF问题”和“NVIC问题” */
    while (USART_GetFlagStatus(SENSOR_GPS_UART, USART_FLAG_RXNE) != RESET) {
        uint8_t tmp = (uint8_t)USART_ReceiveData(SENSOR_GPS_UART);
        uint16_t next = (g_gps_rx_fifo.writer + 1) % SENSOR_GPS_RX_FIFO_SIZE;
        g_gps_data.poll_count++;
        g_gps_data.last_rx_byte = tmp;
        if (next != g_gps_rx_fifo.reader) {
            g_gps_rx_fifo.buf[g_gps_rx_fifo.writer] = tmp;
            g_gps_rx_fifo.writer = next;
        }
    }

    g_gps_data.fifo_level = fifo_available();

    while (fifo_available() > 0) {
        char ch = (char)fifo_read();

        if (ch == '$') {
            g_nmea_collecting = 1;
            g_nmea_idx = 0;
            g_nmea_line[g_nmea_idx++] = ch;
            continue;
        }

        if (!g_nmea_collecting) continue;

        if (ch == '\r') continue;

        if (ch == '\n') {
            g_nmea_line[g_nmea_idx] = '\0';
            g_nmea_collecting = 0;
            parse_nmea_line(g_nmea_line);
            continue;
        }

        if (g_nmea_idx < (SENSOR_GPS_NMEA_LINE_MAX - 1)) {
            g_nmea_line[g_nmea_idx++] = ch;
        } else {
            g_nmea_collecting = 0;
            g_nmea_idx = 0;
        }
    }
}

uint8_t sensor_gps_get_data(gps_data_t *data)
{
    uint8_t updated = g_gps_data.updated;
    *data = g_gps_data;
    g_gps_data.updated = 0;
    return updated;
}

void SENSOR_GPS_UART_IRQHandler(void)
{
    uint8_t tmp;
    uint16_t next;

    if (USART_GetFlagStatus(SENSOR_GPS_UART, USART_FLAG_ORE) != RESET) {
        g_gps_data.ore_flag = 1;
        (void)SENSOR_GPS_UART->STATR;
        (void)SENSOR_GPS_UART->DATAR;
    }

    if (USART_GetFlagStatus(SENSOR_GPS_UART, USART_FLAG_FE) != RESET) {
        g_gps_data.fe_flag = 1;
    }
    if (USART_GetFlagStatus(SENSOR_GPS_UART, USART_FLAG_NE) != RESET) {
        g_gps_data.ne_flag = 1;
    }

    if (USART_GetFlagStatus(SENSOR_GPS_UART, USART_FLAG_RXNE) != RESET) {
        tmp = (uint8_t)USART_ReceiveData(SENSOR_GPS_UART);
        next = (g_gps_rx_fifo.writer + 1) % SENSOR_GPS_RX_FIFO_SIZE;
        g_gps_data.irq_count++;
        g_gps_data.last_rx_byte = tmp;
        if (next != g_gps_rx_fifo.reader) {
            g_gps_rx_fifo.buf[g_gps_rx_fifo.writer] = tmp;
            g_gps_rx_fifo.writer = next;
        }
        g_gps_data.fifo_level = fifo_available();
    }
}
