/**
 * @file uart_transport_parser.h
 * @brief Парсер и сборщик UART-пакетов формата [A][A][LEN][DATA][CRC]
 * @details Модуль предоставляет функции для разбора входящих и формирования исходящих пакетов с опциональной проверкой CRC.
 *
 * Формат пакета:
 * - 2 байта заголовка (0xAA 0xAA)
 * - 1 байт длины данных (0-64)
 * - N байт данных
 * - 1 байт CRC (опционально)
 *
 * @author Evgenii Shkliaev
 * @date 12-06-2025
 */

#ifndef UART_TRANSPORT_UART_TRANSPORT_PARSER_H_
#define UART_TRANSPORT_UART_TRANSPORT_PARSER_H_

#include <stddef.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

/**
 * @def UART_TRANSPORT_CRC_ENABLED
 * @brief Флаг включения проверки CRC (1 - включено, 0 - выключено)
 * @note Может быть переопределен перед включением заголовочного файла
 */
#ifndef UART_TRANSPORT_CRC_ENABLED
#define UART_TRANSPORT_CRC_ENABLED 1
#endif

/**
 * @def UART_TRANSPORT_MAX_PAYLOAD
 * @brief Максимальный размер полезной нагрузки в байтах
 * @note Может быть переопределен перед включением заголовочного файла.
 *       Минимальное значение - 1, максимальное - 255.
 */
#ifndef UART_TRANSPORT_MAX_PAYLOAD
#define UART_TRANSPORT_MAX_PAYLOAD 255U
#endif

/**
 * @def UART_TRANSPORT_HEADER_SIZE
 * @brief Размер заголовка пакета (AA + AA + LEN)
 */
#define UART_TRANSPORT_HEADER_SIZE 3

/**
 * @def UART_TRANSPORT_PACKET_OVERHEAD
 * @brief Дополнительные байты поверх данных (заголовок + CRC)
 */
#if UART_TRANSPORT_CRC_ENABLED
#define UART_TRANSPORT_PACKET_OVERHEAD (UART_TRANSPORT_HEADER_SIZE + 1)
#else
#define UART_TRANSPORT_PACKET_OVERHEAD UART_TRANSPORT_HEADER_SIZE
#endif

/**
 * @def UART_TRANSPORT_MAX_PACKET_SIZE
 * @brief Максимальный полный размер пакета
 */
#define UART_TRANSPORT_MAX_PACKET_SIZE \
    (UART_TRANSPORT_HEADER_SIZE + UART_TRANSPORT_MAX_PAYLOAD + \
    (UART_TRANSPORT_CRC_ENABLED ? 1 : 0))


/**
 * @enum UartParserState_t
 * @brief Состояния конечного автомата парсера
 */
typedef enum {
    STATE_WAIT_AA,       ///< Ожидание первого байта заголовка (0xAA)
    STATE_WAIT_SECOND_A, ///< Ожидание второго байта заголовка (0xAA)
    STATE_WAIT_LENGTH,   ///< Ожидание байта длины данных
    STATE_WAIT_DATA,     ///< Ожидание данных
#if UART_TRANSPORT_CRC_ENABLED
    STATE_WAIT_CRC       ///< Ожидание контрольной суммы (только если CRC включен)
#endif
} UartParserState_t;

/**
 * @struct UartParserContext_t
 * @brief Контекст парсера для хранения состояния между вызовами
 */
typedef struct {
    UartParserState_t state;      ///< Текущее состояние парсера
    uint8_t expected_length;      ///< Ожидаемая длина данных
#if UART_TRANSPORT_CRC_ENABLED
    uint8_t calculated_crc;       ///< Накопленное значение CRC
#endif
    uint16_t data_index;          ///< Индекс текущего байта данных
    TickType_t last_byte_time;    ///< Время получения последнего байта
} UartParserContext_t;

/**
 * @brief Инициализация парсера
 * @details Сбрасывает внутреннее состояние парсера
 */
void UartTransport_ParserInit(void);

/**
 * @brief Обработка входящего байта
 * @param[in] byte Входящий байт данных
 * @param[out] buffer Буфер для записи данных (размер >= UART_TRANSPORT_MAX_PAYLOAD)
 * @param[out] out_len Указатель для возврата длины данных (0 если пакет не собран)
 * @return true - пакет собран и проверен, false - пакет в процессе сборки или ошибка
 * @note При отключенной проверке CRC возвращает true для любого корректного пакета
 */
bool UartParser_ProcessByte(uint8_t byte, uint8_t* buffer, uint16_t* out_len);

/**
 * @brief Формирование UART-пакета для отправки
 * @param[in] payload Указатель на данные для отправки
 * @param[in] payload_len Длина данных (0..UART_TRANSPORT_MAX_PAYLOAD)
 * @param[out] output_buffer Буфер для готового пакета
 * @param[out] out_packet_len Указатель для возврата длины сформированного пакета
 * @return true - пакет сформирован, false - ошибка параметров
 * @note Размер output_buffer должен быть не менее UART_TRANSPORT_MAX_PAYLOAD + 4
 */
bool UartTransport_BuildPacket(const uint8_t* payload, uint16_t payload_len,
                             uint8_t* output_buffer, uint16_t* out_packet_len);

#endif /* UART_TRANSPORT_UART_TRANSPORT_PARSER_H_ */
