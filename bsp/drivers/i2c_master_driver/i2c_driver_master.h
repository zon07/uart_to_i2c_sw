/**
 * @file i2c_driver_master.h
 * @brief Драйвер I2C в режиме мастера
 * @details Предоставляет интерфейс для работы с I2C в режиме мастера с использованием FreeRTOS
 * @date 26-06-2025
 * @author Evgneii Shkliaev (zon07)
 *
 * @copyright Copyright (c) 2025
 */

#ifndef SENSORS_CONTROLLER_I2C_DRIVER_I2C_DRIVER_MASTER_H_
#define SENSORS_CONTROLLER_I2C_DRIVER_I2C_DRIVER_MASTER_H_

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/**
 * @brief Режимы работы I2C
 */
typedef enum
{
    I2C_OP_WRITE_THEN_READ,  // Запись команды + чтение данных (с RESTART)
    I2C_OP_WRITE_ONLY,       // Только запись (адрес + команда + данные)
    I2C_OP_READ_ONLY         // Только чтение (редко)
} I2C_Master_OpMode_t;


/**
 * @brief Структура транзакции I2C
 * @note Максимальная длина команды - 16 байт
 */
typedef struct {
    uint8_t devAddr;
    I2C_Master_OpMode_t opMode;
    uint8_t *command;
    uint16_t commandLen;   // Длина команды
    uint8_t *data;
    uint16_t dataLen;
    SemaphoreHandle_t completionSem; // Семафор для уведомления о завершении
    bool result;
} I2C_Master_Transaction_t;


/**
 * @brief Инициализация драйвера I2C
 * @details Создает очередь и выполняет настройку аппаратного модуля I2C
 */
void Drv_I2C_Master_Init(void);

/**
 * @brief Отправка транзакции в очередь на выполнение
 * @param trans Указатель на структуру транзакции
 * @param timeout Время ожидания в тиках FreeRTOS
 * @return true - транзакция успешно добавлена в очередь
 * @return false - ошибка параметров или очередь переполнена
 * @note Проверяет что длина команды не превышает 16 байт
 */
bool Drv_I2C_Master_SendTransaction(I2C_Master_Transaction_t *trans, TickType_t timeout);

/**
 * @brief Обработчик прерываний I2C
 * @param pxHigherPriorityTaskWoken Указатель для возврата флага необходимости переключения контекста
 * @details Должен вызываться из соответствующего IRQHandler
 */
void Drv_I2C_Master_IRQ_Handler(BaseType_t *pxHigherPriorityTaskWoken);



#endif /* SENSORS_CONTROLLER_I2C_DRIVER_I2C_DRIVER_MASTER_H_ */
