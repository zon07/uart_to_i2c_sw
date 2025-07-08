#include <leds_controller/leds.h>
#include "BspLedsController_If.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"


// Команда для очереди
typedef struct
{
    BSP_LedcLeds led;
    BSP_LedcModes newMode;
} LED_Command_t;

typedef struct
{
    BSP_Leds led;
    BSP_LedcModes mode;
    TimerHandle_t blinkTimer;
    TickType_t blinkInterval;
} LED_State_t;

// Прототип функции обратного вызова таймера
static void BlinkTimerCallback(TimerHandle_t xTimer);

static void LedFsmProcess(LED_State_t* ledState, BSP_LedcModes newMode);

static void SetupBlinkTimer(LED_State_t* ledState, TickType_t interval);

static QueueHandle_t xLedQueue = NULL;
static TaskHandle_t xControllerTask = NULL;

static void vBsp_LedControllerTask(void *pvParams);


static LED_State_t LedState_VD1 = {
    .led = BSP_LED_VD1,
    .mode = BSP_LEDC_MODE_OFF,
    .blinkTimer = NULL,
    .blinkInterval = 0
};

static LED_State_t LedState_VD2 = {
    .led = BSP_LED_VD2,
    .mode = BSP_LEDC_MODE_OFF,
    .blinkTimer = NULL,
    .blinkInterval = 0
};

// Обработчик таймера мигания
static void BlinkTimerCallback(TimerHandle_t xTimer)
{
    LED_State_t* ledState = (LED_State_t*)pvTimerGetTimerID(xTimer);
    if(ledState != NULL)
    {
        BSP_LED_Toggle(ledState->led);
    }
}

// Создание/настройка таймера мигания
static void SetupBlinkTimer(LED_State_t* ledState, TickType_t interval)
{
    if(ledState->blinkTimer == NULL)
    {
        ledState->blinkTimer = xTimerCreate(
            "LedBlinkTimer",
            interval,
            pdTRUE, // Автоповтор
            (void*)ledState,
            BlinkTimerCallback
        );
    }
    else
    {
        xTimerChangePeriod(ledState->blinkTimer, interval, 0);
    }

    ledState->blinkInterval = interval;
    xTimerStart(ledState->blinkTimer, 0);
}

static void LedFsmProcess(LED_State_t* ledState, BSP_LedcModes newMode)
{
    if(ledState == NULL) return;

    // Если режим не изменился - ничего не делаем
    if(ledState->mode == newMode) return;

    // Останавливаем предыдущий таймер, если был
    if(ledState->blinkTimer != NULL)
    {
        xTimerStop(ledState->blinkTimer, 0);
    }

    ledState->mode = newMode;

    switch (ledState->mode)
    {
        case BSP_LEDC_MODE_OFF:
            BSP_LED_Set(ledState->led, BSP_LED_STATE_OFF);
            break;

        case BSP_LEDC_MODE_ON:
            BSP_LED_Set(ledState->led, BSP_LED_STATE_ON);
            break;

        case BSP_LEDC_MODE_1: // 200ms blink
            BSP_LED_Set(ledState->led, BSP_LED_STATE_ON);
            SetupBlinkTimer(ledState, pdMS_TO_TICKS(100));
            break;

        case BSP_LEDC_MODE_2: // 1000ms blink
            BSP_LED_Set(ledState->led, BSP_LED_STATE_ON);
            SetupBlinkTimer(ledState, pdMS_TO_TICKS(500));
            break;

        default:
            break;
    }
}

// Задача-контроллер
static void vBsp_LedControllerTask(void *pvParams)
{
    LED_Command_t cmd;

    while (1)
    {
        if (xQueueReceive(xLedQueue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            if(cmd.led == BSP_LEDC_LED_VD1)
            {
                LedFsmProcess(&LedState_VD1, cmd.newMode);
            }
            else if(cmd.led == BSP_LEDC_LED_VD2)
            {
                LedFsmProcess(&LedState_VD2, cmd.newMode);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Инициализация
bool Bsp_LedC_Init_If(void)
{
    xLedQueue = xQueueCreate(5, sizeof(LED_Command_t));
    if (xLedQueue == NULL) {
        return false;
    }

    BaseType_t taskStatus = xTaskCreate(vBsp_LedControllerTask, "LED_CTRL", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &xControllerTask);
    if (taskStatus == pdFAIL)
    {
        return false;
    }
    BSP_LED_Init();
    return true;
}

bool Bsp_LedC_SetLedMode_If(BSP_LedcLeds led, BSP_LedcModes mode)
{
    LED_Command_t cmd = { .led = led, .newMode = mode };
    return (xQueueSend(xLedQueue, &cmd, 0) == pdPASS);
}
