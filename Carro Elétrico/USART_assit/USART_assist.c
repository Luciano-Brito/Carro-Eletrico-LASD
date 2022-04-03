/*
 * USART_assist.c
 *
 * Created: 03/04/2022 12:30:32
 *  Author: Luciano Júnior Santos Brito - 118111399
 */ 

#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

void USART_init(const int baud){
	#define BAUD baud
	#define MYUBRR (F_CPU/16/BAUD-1)
	UBRR0H = (unsigned char)(MYUBRR>>8); //Ajusta a taxa de transmissão
	UBRR0L = (unsigned char)MYUBRR;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //Habilita o transmissor e o receptor
	UCSR0C = (3<<UCSZ00); //Ajusta o formato do frame: 8 bits de dados e 1 de parada
}

void USART_send_byte (uint8_t byte){
	char Data [3] = {0,0,0};
	sprintf(Data,"%u",byte);	//converte uint8_t para char
	
	if(byte == 0)				//se o recebido for 0 escreve "0"
	USART_send_char("0");

	for(int i = 0; byte; i++){		//escreve até ultimo caracter válido
		USART_send_char(Data[i]);
		byte /= 10;
	}
}

void USART_send_char (char Caracter){
	while(!(UCSR0A & (1<<UDRE0)));	//espera a limpeza do registrador de transmissão
	UDR0 = Caracter;				//envia o caracter
}

void USART_send_string (const char *Texto){
	while(*Texto){
		USART_send_char(*Texto++);
	}
}

void USART_send_stringln (const char *Texto){
	while(*Texto){
		USART_send_char(*Texto++);
	}
	USART_send_char('\n');
}