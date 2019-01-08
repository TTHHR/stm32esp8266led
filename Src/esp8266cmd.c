#include "esp8266cmd.h"
#include "string.h"
char NO_ECHO[]= "ATE0\r\n";
char RST_WIFI[]= "AT+RST\r\n";
char LINK_WIFI[]="AT+CWJAP=\"TEST\",\"testtest\"\r\n";
char OPEN_TCP[]="AT+CIPSTART=\"TCP\",\"wjy.qingyuyu.cn\",2333\r\n";
char CLOSE_TCP[]="AT+CIPCLOSE\r\n";

void sendCmd(UART_HandleTypeDef *huart3,char * cmd)
{
	HAL_UART_Transmit(huart3,(uint8_t*)cmd,strlen(cmd),0xffff);
}

void noEcho(UART_HandleTypeDef *huart3)
{
	sendCmd(huart3,NO_ECHO);
}
void linkWifi(UART_HandleTypeDef *huart3)
{
	sendCmd(huart3,LINK_WIFI);
}
void openTCP(UART_HandleTypeDef *huart3)
{
	sendCmd(huart3,OPEN_TCP);
}

void resetWifi(UART_HandleTypeDef *huart3)
{
	sendCmd(huart3,RST_WIFI);
}
void closeTCP(UART_HandleTypeDef *huart3)
{
	sendCmd(huart3,CLOSE_TCP);
}
