/*
 * USART_assit.h
 *
 * Created: 03/04/2022 12:28:45
 *  Author: Luciano Júnior Santos Brito - 118111399
 */ 


#ifndef USART_ASSIT_H_
#define USART_ASSIT_H_

void USART_init(const int baud);				//inicializa a USART
void USART_send_byte (uint8_t data);			//envia um byte em decimal pela serial
void USART_send_char (char Caracter);			//envia um caracter pela serial
void USART_send_string (const char *Texto);		//envia uma string pela serial
void USART_send_stringln (const char *Texto);	//envia uma string pela serial e pula a linha

#endif /* USART_ASSIT_H_ */