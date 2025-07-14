/**
 * @file uart_transport_parser.c
 * @brief Реализация парсера UART-пакетов
 * @details Содержит реализацию функций для разбора и сборки пакетов,
 *          включая расчет контрольной суммы (CRC).
 *
 * @author Evgenii Shkliaev (zon07)
 * @date 12-06-2025
 */

#include "uart_transport_parser.h"
#include <string.h>

#if UART_TRANSPORT_MAX_PAYLOAD < 1 || UART_TRANSPORT_MAX_PAYLOAD > 255
#error "UART_TRANSPORT_MAX_PAYLOAD must be between 1 and 255"
#endif

static UartParserContext_t parser_ctx;


#if UART_TRANSPORT_CRC_ENABLED
static inline uint8_t Calculate_crc(uint8_t current_crc, uint8_t new_byte)
{
    uint8_t crc = current_crc ^ new_byte;
    for(uint8_t i = 0; i < 8; i++)
    {
        crc = (crc << 1) ^ ((crc & 0x80) ? 0x07 : 0);
    }
    return crc;
}
#endif


void UartTransport_ParserInit(void)
{
    memset(&parser_ctx, 0, sizeof(parser_ctx));
    parser_ctx.state = STATE_WAIT_AA;
}


bool UartParser_ProcessByte(uint8_t byte, uint8_t* buffer, uint16_t* out_len)
{
    if (buffer == NULL || out_len == NULL)
        return false;

    TickType_t now = xTaskGetTickCount();
    *out_len = 0;

    // Таймаут (200 мс без данных → сброс парсера)
    if (now - parser_ctx.last_byte_time > pdMS_TO_TICKS(200))
        UartTransport_ParserInit();

    parser_ctx.last_byte_time = now;

    switch (parser_ctx.state)
    {
        case STATE_WAIT_AA:
            if (byte == 'A')
                parser_ctx.state = STATE_WAIT_SECOND_A;
            break;

        case STATE_WAIT_SECOND_A:
            if (byte == 'A')
                parser_ctx.state = STATE_WAIT_LENGTH;
            else
                UartTransport_ParserInit();
            break;

        case STATE_WAIT_LENGTH:
            parser_ctx.expected_length = byte;
            parser_ctx.data_index = 0;

#if UART_TRANSPORT_CRC_ENABLED
            parser_ctx.calculated_crc = Calculate_crc(0, byte);
#endif

            if (parser_ctx.expected_length > UART_TRANSPORT_MAX_PAYLOAD)
            {
                UartTransport_ParserInit();
            }
            else if (parser_ctx.expected_length > 0)
            {
                parser_ctx.state = STATE_WAIT_DATA;
            }
            else
            {
                // Пакет нулевой длины
#if UART_TRANSPORT_CRC_ENABLED
                parser_ctx.state = STATE_WAIT_CRC;
#else
                *out_len = 0;
                UartTransport_ParserInit();
                return true;
#endif
            }
            break;

        case STATE_WAIT_DATA:
            buffer[parser_ctx.data_index++] = byte;
#if UART_TRANSPORT_CRC_ENABLED
            parser_ctx.calculated_crc = Calculate_crc(parser_ctx.calculated_crc, byte);
#endif
            if (parser_ctx.data_index >= parser_ctx.expected_length)
            {
#if UART_TRANSPORT_CRC_ENABLED
                parser_ctx.state = STATE_WAIT_CRC;
#else
                *out_len = parser_ctx.expected_length;
                UartTransport_ParserInit();
                return true;
#endif
            }
            break;

#if UART_TRANSPORT_CRC_ENABLED
        case STATE_WAIT_CRC:
            bool crc_valid = (parser_ctx.calculated_crc == byte);
            *out_len = crc_valid ? parser_ctx.expected_length : 0;
            UartTransport_ParserInit();
            return crc_valid;
#endif
    }

    return false;
}

bool UartTransport_BuildPacket(const uint8_t* payload, uint16_t payload_len,
                             uint8_t* output_buffer, uint16_t* out_packet_len)
{
    if(!payload || !output_buffer || !out_packet_len)
    	return false;
    if(payload_len > UART_TRANSPORT_MAX_PAYLOAD)
    	return false;

    // Заголовок
    output_buffer[0] = 'A';
    output_buffer[1] = 'A';
    output_buffer[2] = (uint8_t)payload_len;

    // Полезная нагрузка
    if(payload_len > 0)
    {
        memcpy(&output_buffer[3], payload, payload_len);
    }

#if UART_TRANSPORT_CRC_ENABLED
    // Расчёт CRC по длине и данным
    uint8_t crc = 0;
    crc = Calculate_crc(crc, output_buffer[2]); // Длина
    for(uint16_t i = 0; i < payload_len; i++)
    {
        crc = Calculate_crc(crc, payload[i]);
    }
    output_buffer[3 + payload_len] = crc;
    *out_packet_len = 3 + payload_len + 1;
#else
    *out_packet_len = 3 + payload_len;
#endif

    return true;
}

