/**
  * @file BspUartTransport_If.h
  * @brief Интерфейс UART транспорта с фиксированным размером буфера
  * @author Evgenii Shkliaev (zon07)
  * @date 10-06-2025
 */

#ifndef BSP_UART_TRANSPORT_H_
#define BSP_UART_TRANSPORT_H_

#include <stdint.h>
#include <stdbool.h>

#include "uart_transport_parser.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#ifndef UART_TRANSPORT_MAX_PAYLOAD
/**
 * @brief Максимальный размер полезной нагрузки
 */
#define UART_TRANSPORT_MAX_PAYLOAD 255U
#endif


/**
 * @brief Структура UART сообщения
 * @details Содержит статический буфер и длину полезных данных
 */
typedef struct
{
    uint8_t payload[UART_TRANSPORT_MAX_PAYLOAD]; 	///< Буфер данных
    uint16_t payload_len;                    		///< Фактическая длина данных
} BSP_UartTransportMessage_t;

/**
 * @brief Инициализация UART транспорта
 * @param baudrate Скорость передачи (бод)
 */
bool Bsp_UartTransport_Init_If(uint32_t baudrate);

/**
 * @brief Отправка сообщения (неблокирующая)
 * @param msg Указатель на сообщение
 * @return true - успешно, false - ошибка
 */
bool Bsp_UartTransportTransmit_If(const BSP_UartTransportMessage_t *msg);


/**
 * @brief Приём сообщения
 * @param msg Указатель для сохранения сообщения
 * @return true - данные получены, false - нет данных
 */
bool Bsp_UartTransportReceive_If(BSP_UartTransportMessage_t* msg);



/**
 * @brief Обработчик прерываний UART
 */
void Bsp_UartTransport_Uart_IRQHandler_If(BaseType_t *pxHigherPriorityTaskWoken);

/**
 * @brief Обработчик прерываний DMA
 */
void Bsp_UartTransport_DMA_IRQHandler_If(void);


#endif /* BSP_UART_TRANSPORT_H_ */
