/**
 * @file bspUartTransport.c
 * @brief RTOS-based UART транспортный протокол
 * @details Реализация транспортного уровня с использованием DMA и FreeRTOS
 *
 * @author Evgenii Shkliaev (zon07)
 * @date 10-06-2025
 */

#include "BspUartTransport_If.h"
#include "mik32_memory_map.h"
#include "power_manager.h"
#include "uart.h"
#include "epic.h"
#include "gpio.h"
#include "pad_config.h"
#include "dma_config.h"
#include <string.h>


#define BSP_UART_T_M_CLOCK_FREQ        (32000000U)  ///< Тактовая частота модуля
#define BSP_UART_T_BAUDRATE_9600       (9600U)     ///< Скорость передачи по умолчанию
#define BSP_UART_T_RX_BYTE_QUEUE_LENGH (16U)       ///< Длина очереди принятых байтов


/**
 * @enum BspUartTransportTxDma_state_t
 * @brief Состояния DMA для передачи
 */
typedef enum {
    DMA_STATE_READY,  ///< DMA готов к передаче
    DMA_STATE_BUSY    ///< DMA занят передачей
} BspUartTransportTxDma_state_t;


static const uint8_t DMA_Channel = 0;  ///< Используемый канал DMA

static volatile BspUartTransportTxDma_state_t dmaState = DMA_STATE_READY;  ///< Текущее состояние DMA
static uint8_t dma_tx_buffer[UART_TRANSPORT_MAX_PACKET_SIZE];  ///< Буфер для передачи через DMA
static uint16_t dma_tx_packet_len = 0;  ///< Длина пакета для передачи

static QueueHandle_t xFromBspUartQueue = NULL;  ///< Очередь для отправляемых сообщений
static QueueHandle_t xToBspUartQueue = NULL;  ///< Очередь для принятых сообщений
static QueueHandle_t xUartRxByteQueue = NULL; ///< Очередь для принятых байтов
static TaskHandle_t xUartTransportTask = NULL; ///< Обработчик задачи UART

static bool StartDmaTransfer(const uint8_t* data, uint16_t length);
static void USART_Init(uint32_t baudrate);
static void DMA_Init(void);


/**
 * @brief Задача обработки UART-трафика
 * @param[in] pvParams Параметры задачи (не используются)
 */
static void vBsp_UartTransportTask(void *pvParams)
{
    const UBaseType_t uxMaxBytesPerCycle = 32; // Макс. байт за 1 цикл (половина буфера)
    uint8_t rxByte = 0;
    BSP_UartTransportMessage_t rxMsgBuff;
    BSP_UartTransportMessage_t txMsgBuff;
    bool messagePending = false; // Флаг ожидания отправки
    UBaseType_t uxProcessedCount = 0;

    while(1)
    {
        uxProcessedCount = 0;

        // Обрабатываем байты с ограничением
        while((uxProcessedCount++ < uxMaxBytesPerCycle) &&
              (xQueueReceive(xUartRxByteQueue, &rxByte, 0) == pdPASS))
        {
            if(UartParser_ProcessByte(rxByte, rxMsgBuff.payload, &rxMsgBuff.payload_len))
            {
                xQueueSend(xToBspUartQueue, &rxMsgBuff, 0);
            }
        }

        // Обработка исходящих сообщений
        if (dmaState == DMA_STATE_READY)
        {
            if (!messagePending)
            {
                // Пытаемся получить новое сообщение только если нет ожидающего
                if(xQueueReceive(xFromBspUartQueue, &txMsgBuff, 0) == pdPASS)
                {
                    messagePending = true;
                }
            }

            if (messagePending)
            {
                if(UartTransport_BuildPacket(txMsgBuff.payload, txMsgBuff.payload_len,
                                          dma_tx_buffer, &dma_tx_packet_len))
                {
                    if(StartDmaTransfer(dma_tx_buffer, dma_tx_packet_len))
                    {
                        messagePending = false; // Сброс флага при успешной отправке
                    }
                    // Если StartDmaTransfer вернул false, флаг останется установленным
                    // и на следующей итерации будет повторная попытка
                }
            }
        }

        // Фиксированная задержка 5 мс
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/**
 * @brief Инициализация UART транспорта
 * @param[in] baudrate Скорость передачи
 * @return true - инициализация успешна, false - ошибка
 */
bool Bsp_UartTransport_Init_If(uint32_t baudrate)
{
	xFromBspUartQueue = xQueueCreate(5, sizeof(BSP_UartTransportMessage_t));
    xToBspUartQueue = xQueueCreate(5, sizeof(BSP_UartTransportMessage_t));
    xUartRxByteQueue = xQueueCreate(BSP_UART_T_RX_BYTE_QUEUE_LENGH, sizeof(uint8_t));

    BaseType_t taskStatus = xTaskCreate(vBsp_UartTransportTask, "UartT_Task",
                                      configMINIMAL_STACK_SIZE*2, NULL,
                                      tskIDLE_PRIORITY + 2, &xUartTransportTask);

    if (taskStatus == pdFAIL || xToBspUartQueue == NULL || xFromBspUartQueue == NULL)
    {
        return false;
    }

    USART_Init(baudrate);
    DMA_Init();
    return true;
}

/**
 * @brief Передача сообщения через UART
 * @param[in] msg Указатель на сообщение для передачи
 * @return true - передача начата, false - ошибка или DMA занят
 */
bool Bsp_UartTransportTransmit_If(const BSP_UartTransportMessage_t* msg)
{
    if(msg == NULL || msg->payload_len > UART_TRANSPORT_MAX_PAYLOAD)
    {
        return false;
    }
    return (xQueueSend(xFromBspUartQueue, msg, 0) == pdPASS);
}

/**
 * @brief Запуск передачи через DMA
 * @param[in] data Данные для передачи
 * @param[in] length Длина данных
 * @return true - передача начата, false - ошибка
 */
static bool StartDmaTransfer(const uint8_t* data, uint16_t length)
{
    if ((DMA_CONFIG->CONFIG_STATUS & DMA_STATUS_READY(DMA_Channel)) == 0)
    {
        return false;  // Канал занят
    }

    DMA_CHANNEL_TypeDef *dmaChannelInst = &DMA_CONFIG->CHANNELS[DMA_Channel];

    dmaChannelInst->CFG &= ~DMA_CH_CFG_ENABLE_M;
    // Настройка DMA канала
    dmaChannelInst->SRC = (uint32_t)data;
    dmaChannelInst->DST = (uint32_t)&UART_0->TXDATA;
    dmaChannelInst->LEN = length - 1;

    // Конфигурация канала
    dmaChannelInst->CFG =
        DMA_CH_CFG_PRIOR_HIGH_M |                 		// Высокий приоритет
        DMA_CH_CFG_READ_MODE_MEMORY_M |           		// Чтение из памяти
        DMA_CH_CFG_WRITE_MODE_PERIPHERY_M |       		// Запись в периферию
        DMA_CH_CFG_READ_INCREMENT_M |             		// Инкремент адреса источника
        DMA_CH_CFG_WRITE_NO_INCREMENT_M |         		// Фиксированный адрес приемника
        DMA_CH_CFG_READ_SIZE_BYTE_M |            		// Размер чтения - байт
        DMA_CH_CFG_WRITE_SIZE_BYTE_M |            		// Размер записи - байт
        DMA_CH_CFG_READ_REQUEST(DMA_UART_0_INDEX) | 	// Запрос DMA для UART0 (чтение)
        DMA_CH_CFG_WRITE_REQUEST(DMA_UART_0_INDEX) | 	// Запрос DMA для UART0 (запись)
        DMA_CH_CFG_IRQ_EN_M;                      		// Разрешить прерывание канала

    DMA_CONFIG->CONFIG_STATUS =
        DMA_CONFIG_GLOBAL_IRQ_ENA_M |             		// Разрешить глобальные прерывания DMA
        DMA_CONFIG_CURRENT_VALUE_M;                		// Разрешить чтение текущих значений

    // Устанавливаем состояние "занято"
    dmaState = DMA_STATE_BUSY;

    dmaChannelInst->CFG |= DMA_CH_CFG_ENABLE_M;
    return true;
}


void Bsp_UartTransport_Uart_IRQHandler_If(BaseType_t *pxHigherPriorityTaskWoken)
{
	if(UART_0->FLAGS & (UART_FLAGS_ORE_M | UART_FLAGS_FE_M | UART_FLAGS_NF_M))
	{
		UART_0->FLAGS = UART_FLAGS_ORE_M | UART_FLAGS_FE_M | UART_FLAGS_NF_M;
		UartTransport_ParserInit(); //Сброс парсера
	}

	if (UART_0->FLAGS & UART_FLAGS_RXNE_M)
	{
		uint8_t rxByte = UART_0->RXDATA;
		xQueueSendFromISR(xUartRxByteQueue, &rxByte, pxHigherPriorityTaskWoken);
	}
}


void Bsp_UartTransport_DMA_IRQHandler_If(void)
{
    if(DMA_CONFIG->CONFIG_STATUS & (1 << (DMA_STATUS_CHANNEL_IRQ_S + DMA_Channel)))
    {
        DMA_CONFIG->CONFIG_STATUS |= (1 << DMA_Channel);

        dmaState = DMA_STATE_READY;
    }
}


bool Bsp_UartTransportReceive_If(BSP_UartTransportMessage_t* msg)
{
    if(msg == NULL)
        return false;

    return (xQueueReceive(xToBspUartQueue, msg, 0) == pdPASS);
}


/**
 * @brief Инициализация UART
 * @param[in] baudrate Скорость передачи
 */
static void USART_Init(uint32_t baudrate)
{
    /* Включаем тактирование модуля */
    PM->CLK_APB_P_SET |= PM_CLOCK_APB_P_UART_0_M;
    EPIC->MASK_LEVEL_SET = EPIC_LINE_M(EPIC_LINE_UART_0_S);

    // 1. Отключить UART перед настройкой
    UART_0->CONTROL1 &= ~UART_CONTROL1_UE_M;
    UART_0->FLAGS = 0xFFFFFFFF;

    UART_0->CONTROL1 &= ~(UART_CONTROL1_M_M | UART_CONTROL1_PCE_M);
    UART_0->CONTROL1 |= UART_CONTROL1_M_8BIT_M;
    UART_0->CONTROL1 |= UART_CONTROL1_TE_M | UART_CONTROL1_RE_M;
    UART_0->CONTROL1 |= UART_CONTROL1_UE_M | UART_CONTROL1_RXNEIE_M;
    UART_0->CONTROL1 |= UART_CONTROL1_TE_M | UART_CONTROL1_RE_M;

    UART_0->CONTROL2 = UART_CONTROL2_STOP_1_M;
    UART_0->CONTROL3 = 0 | UART_CONTROL3_EIE_M | UART_CONTROL3_DMAT_M;

    // Настройка скорости передачи
    uint32_t brr_value = BSP_UART_T_M_CLOCK_FREQ / baudrate;
    UART_0->DIVIDER = (brr_value << UART_DIVIDER_BRR_S) & UART_DIVIDER_BRR_M;

    UART_0->CONTROL1 |= UART_CONTROL1_UE_M;

    PM->CLK_APB_P_SET |= PM_CLOCK_APB_P_GPIO_0_M;
    /* Настройка портов на режим UART */
    PAD_CONFIG->PORT_0_CFG &= ~(PAD_CONFIG_PIN_M(5) | PAD_CONFIG_PIN_M(6));
    PAD_CONFIG->PORT_0_CFG |= PAD_CONFIG_PIN(5, 0b01); // RX
    PAD_CONFIG->PORT_0_CFG |= PAD_CONFIG_PIN(6, 0b01); // TX
}

/**
 * @brief Инициализация DMA
 */
static void DMA_Init(void)
{
    PM->CLK_AHB_SET |= PM_CLOCK_AHB_DMA_M;  // Включить тактирование DMA
    EPIC->MASK_LEVEL_SET |= EPIC_LINE_M(EPIC_DMA_INDEX);
}
