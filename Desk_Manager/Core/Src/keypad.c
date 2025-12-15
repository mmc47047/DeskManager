#include "keypad.h"
#include "main.h"
#include "string.h"
#include "stdio.h"

#define ROW_NUM 4
#define COL_NUM 4

// main.c? GPIO ??? ?? ??
const uint16_t rowPins[ROW_NUM] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15}; // ?? ?
const uint16_t colPins[COL_NUM] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_10};    // ?? ?

GPIO_TypeDef* rowPorts[ROW_NUM] = {GPIOB, GPIOB, GPIOB, GPIOB}; // ?? ?? GPIOB
GPIO_TypeDef* colPorts[COL_NUM] = {GPIOB, GPIOB, GPIOB, GPIOB}; // ?? ?? GPIOB

char keys[ROW_NUM][COL_NUM] = {
  {'1','2','3','A'},  // Row 1 (PB12): S4? ??? 'A'
  {'4','5','6','B'},  // Row 2 (PB13)
  {'7','8','9','C'},  // Row 3 (PB14)
  {'*','0','#','D'}   // Row 4 (PB15)
};

void Keypad_Init(void)
{
  // main.c?? ?? GPIO ???? ???????
  // ?? ???? ?????, ? ??? HIGH? ??
  for (int i = 0; i < ROW_NUM; i++) {
    HAL_GPIO_WritePin(rowPorts[i], rowPins[i], GPIO_PIN_SET);
  }
  
  // ??? ???
  extern UART_HandleTypeDef huart2;
  HAL_UART_Transmit(&huart2, (uint8_t*)"Keypad Init - GPIO B pins\r\n", 28, 1000);
}

char Keypad_Read(void)
{
  extern UART_HandleTypeDef huart2;
  
  for (int row = 0; row < ROW_NUM; row++) {
    // ?? ?? LOW? ??
    HAL_GPIO_WritePin(rowPorts[row], rowPins[row], GPIO_PIN_RESET);
    HAL_Delay(1); // ?? ???
    
    for (int col = 0; col < COL_NUM; col++) {
      if (HAL_GPIO_ReadPin(colPorts[col], colPins[col]) == GPIO_PIN_RESET) {
        // ?? ?? ??
        char pressed_key = keys[row][col];
        
        // ??? ???
        char debug_msg[50];
        sprintf(debug_msg, "Key pressed: %c (R%d,C%d)\r\n", pressed_key, row+1, col+1);
        HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), 1000);
        
        // ????
        HAL_Delay(20);
        while (HAL_GPIO_ReadPin(colPorts[col], colPins[col]) == GPIO_PIN_RESET) {
          // ?? ??? ??? ??
        }
        
        // ? ?? ?? HIGH? ??
        HAL_GPIO_WritePin(rowPorts[row], rowPins[row], GPIO_PIN_SET);
        
        return pressed_key;
      }
    }
    
    // ?? ?? ?? HIGH? ??
    HAL_GPIO_WritePin(rowPorts[row], rowPins[row], GPIO_PIN_SET);
  }
  
  return 0; // ?? ??? ??
}