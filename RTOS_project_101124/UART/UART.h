/*
 * UART.h
 *
 *  Created on: Nov 24, 2024
 *      Author: karti
 */

#ifndef UART_H_
#define UART_H_

extern UART_HandleTypeDef huart2;
extern char buffer[256];

int bufsize(char *buf){
	int i=0;
	while(*buf++!='\0'){
	i++;}
	return i;
}
void buffclear(char*string){
	uint8_t len=strlen(string);
	for(int i=0;i<len;i++){
		string[i]='\0';
	}
}

void send_uart(char*string){
	uint8_t len=strlen(string);
	HAL_UART_Transmit(&huart2, (uint8_t *)string, len, 2000);
	buffclear(string);
}

#endif /* UART_H_ */
