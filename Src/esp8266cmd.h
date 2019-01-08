#include "stm32f7xx_hal.h"
void sendCmd(UART_HandleTypeDef *huart3,char * cmd);
void resetWifi(UART_HandleTypeDef *huart3);
void linkWifi(UART_HandleTypeDef *huart3);
void openTCP(UART_HandleTypeDef *huart3);
void closeTCP(UART_HandleTypeDef *huart3);
void noEcho(UART_HandleTypeDef *huart3);


