#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "lcd.h"      // Driver for the LCD display
#include "gpio.h"     // GPIO control for LEDs and buttons

// Task handles
TaskHandle_t SwitchTaskHandle, LEDTaskHandle, LCDTaskHandle;
SemaphoreHandle_t xMutex;

// Shared variables
volatile uint8_t switchState = 0;  // Bit 0 = Switch 1, Bit 1 = Switch 2
volatile uint8_t ledState = 0;     // 0 = No LEDs, 1 = Red, 2 = Green, 3 = Both

// Function prototypes
void SwitchTask(void *pvParameters);
void LEDTask(void *pvParameters);
void LCDTask(void *pvParameters);

// GPIO Pin definitions
#define SWITCH_1_PIN    GPIO_PIN_0
#define SWITCH_2_PIN    GPIO_PIN_1
#define LED_RED_PIN     GPIO_PIN_2
#define LED_GREEN_PIN   GPIO_PIN_3

// Task: Read switch inputs
void SwitchTask(void *pvParameters) {
    while(1) {
        // Read the switch states
        uint8_t s1 = GPIO_Read(SWITCH_1_PIN);
        uint8_t s2 = GPIO_Read(SWITCH_2_PIN);
        
        // Acquire mutex and update the shared switch state
        xSemaphoreTake(xMutex, portMAX_DELAY);
        switchState = (s1 << 0) | (s2 << 1);
        xSemaphoreGive(xMutex);

        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms delay
    }
}

// Task: Control LEDs based on switch inputs
void LEDTask(void *pvParameters) {
    while(1) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        uint8_t localSwitchState = switchState;
        xSemaphoreGive(xMutex);

        if (localSwitchState == 0x01) {
            // Switch 1 pressed, turn on Red LED
            GPIO_Write(LED_RED_PIN, 1);
            GPIO_Write(LED_GREEN_PIN, 0);
            ledState = 1;
        } else if (localSwitchState == 0x02) {
            // Switch 2 pressed, turn on Green LED
            GPIO_Write(LED_RED_PIN, 0);
            GPIO_Write(LED_GREEN_PIN, 1);
            ledState = 2;
        } else if (localSwitchState == 0x03) {
            // Both switches pressed, maintain current state
            // Leave LEDs as they are
            ledState = 3;
        } else {
            // No buttons pressed, alternate LEDs
            static uint8_t toggle = 0;
            if (toggle) {
                GPIO_Write(LED_RED_PIN, 1);
                GPIO_Write(LED_GREEN_PIN, 0);
                ledState = 1;
            } else {
                GPIO_Write(LED_RED_PIN, 0);
                GPIO_Write(LED_GREEN_PIN, 1);
                ledState = 2;
            }
            toggle = !toggle;
        }

        vTaskDelay(pdMS_TO_TICKS(500));  // 500ms delay for alternating LEDs
    }
}

// Task: Update LCD with switch and LED status
void LCDTask(void *pvParameters) {
    char lcdBuffer[16];

    while(1) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        uint8_t localSwitchState = switchState;
        uint8_t localLedState = ledState;
        xSemaphoreGive(xMutex);

        snprintf(lcdBuffer, 16, "S1:%d S2:%d LED:%d", 
                 (localSwitchState & 0x01) != 0, 
                 (localSwitchState & 0x02) != 0, 
                 localLedState);

        LCD_SetCursor(0, 0);
        LCD_Print(lcdBuffer);

        vTaskDelay(pdMS_TO_TICKS(200));  // Refresh every 200ms
    }
}

// Main function
int main(void) {
    // Initialize hardware components (GPIO, LCD, etc.)
    GPIO_Init();
    LCD_Init();

    // Create the mutex
    xMutex = xSemaphoreCreateMutex();

    // Create the tasks
    xTaskCreate(SwitchTask, "SwitchTask", 128, NULL, 1, &SwitchTaskHandle);
    xTaskCreate(LEDTask, "LEDTask", 128, NULL, 1, &LEDTaskHandle);
    xTaskCreate(LCDTask, "LCDTask", 128, NULL, 1, &LCDTaskHandle);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach here
    while(1);
}
