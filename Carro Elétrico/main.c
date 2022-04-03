/*
* main.c
*
* Created: 3/7/2022 10:01:42 AM
*  Author: Luciano J�nior Santos Brito - 118111399
*/

/* 
Lista de mudan�as que n�o tive tepo de mostrar no v�deo:

---> Encapsulei e melhorei as fun��es relacionadas ao registrador de deslocamneto e escrevi fun��es para:
		Escrever um bit
		Escrever um inteiro sem sinal de 16 bits

---> Escrevi e encapsulei fun��es relacionadas o monitor serial para:
		Inicializar a USART
		Escrever Strings (com e sem quebra de linha)
		Escrever Caracteres
		Escrever byte (sem sinal)
		
---> Adicionei a biblioteca do display oled:
		Bitmap para o alerta de tens�o baixa
		Bitmap para o alerta de temperatura alta
		Bitmap para o alerta de cinto (com e sem seta)
		(esses bitmaps me deram bastante trabalho)
		
---> Coloquei o Timer 2 para gerar uma interrup��o a cada 1ms para marcar o tempo de execu��o
    (que acabou gerando uma interrup��o a cada 0,5ms por algum bug do simulador)
	
---> Escrevi a fun��o map() para facilitar os calculos de conver�o de escala
*/

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/eeprom.h>
#include <stdio.h>

#include "SSD1306\SSD1306.h"
#include "SSD1306\Font5x8.h"
#include "SSD1306\Tensao_Baixa.h"
#include "SSD1306\Temperatura_Alta.h"
#include "SSD1306\Cinto.h"
#include "Shift_reg\Shift_reg.h"
#include "USART_assit\USART_assit.h"

#define bt1 (1<<3)	//Bot�o 1 no pino 2 da porta
#define bt2 (1<<4)	//Bot�o 2 no pino 3 da porta
#define I1	(1<<2)	//Entrada da onda quadrada
#define POT (1<<0)	//entrada do acelerador
#define TEMP (1<<1)	//entrada do termistor
#define BAT (1<<2)	//entrada da bateria
#define HB_O (1<<5)	//sa�da pwm
#define Dri_Rea (1<<6)	//entrada Drive/Rear
#define Park	(1<<7)	//entrada Park
#define Cinto_	(1<<3)	//Entrada do cinto
#define Sonar	(1<<0)	//Entada do Sonar
#define Power_rele		(1<<4)	//Sa�da para o Rele da bateria
#define Buzzer			(1<<5)	//Sa�da para o buzzer
#define endereco_eeprom_dist 0
#define endereco_eeprom_diam 2
#define endereco_eeprom_tempmax 4
#define Temp_bat_max 70		//Temperatura m�xima (em �C)
#define V_bat_min 614		//V_bat_min = (1023/5) * 3V

// Vari�veis globais
uint8_t Diametro = 0;
uint8_t Diametro_a = 0;
uint8_t Nivel_Bat = 0;
uint8_t Temp_Bat = 0;
uint16_t Velocidade = 0;
uint16_t Distancia_KM = 0;
uint16_t Distancia_sonar = 0;
uint16_t RPM = 0;
uint16_t RPM_a = 0;
uint16_t ADC_Acelerador = 0;
uint16_t ADC_Bat = 0;
uint16_t ADC_Temp = 0;
uint16_t ADC_FSG = 0;
uint16_t ADC1_a = 0;
uint16_t ADC2_a = 0;
uint16_t tempo_ms = 0;
uint32_t Distancia_cm = 0;

bool Drive_flag = 0;
bool Drive_flag_a = 0;
bool Park_flag = 0;
bool Park_flag_a = 0;
bool Flag_Tensao = 0;
bool Flag_Temp = 0;
bool Flag_cinto = 0;

//Prot�tipos
double map(double x, double en_min, double en_max, double sa_min, double sa_max); //mapeia uma entrada de en_min a en_max para uma saida de sa_min a sa_max
void refresh_oled ();						//atualiza as informa��es no display

int main(void)
{
	//I/O
	DDRB &= !(Sonar|Cinto_);				//definindo o pinos pinos do Sonar e Cinto_ como entrada
	DDRB |= (Buzzer|Power_rele);			//definindo o pinos pinos do Buzzer e Power_rele como sa�da
	PORTB |= (Cinto_|Power_rele);			//habilitando resistor de pull-up no pino do cinto e habilita o Power_rele
	DDRD &= !(bt1|bt2|I1|Dri_Rea|Park);		//definindo os pinos do bt1, bt2, I1, Dri_Rea e Park como entrada
	DDRD |= (HB_O);							//definindo a o pino de controle da ponte h como sa�da
	PORTD |= (bt1|bt2);						//habilitando resistor de pull-up nos pinos dos bot�es
	DDRC &= !(POT|TEMP|BAT);				//definindo o pinos pinos do POT, TEMP e BAT como entrada

	//Configura��o do ADC
	ADMUX = 0b01000000;		//tens�o Vcc de refer�ncia (5.0 V), canal 0
	ADCSRA = 0b11101111;	//habilita o AD, desabilita interrup��o, modo de convers�o cont�nua, prescaler de 128
	ADCSRB = 0x00;			//modo de convers�o cont�nua
	DIDR0 = 0b00111000;		//habilita o PC0, PC1 e PC2 como entrada para o ADC

	//Configura��o dos Timers
	//Timer 2
	TCCR2A = 0b00000010;	//Habilita modo CTC do timer 2
	TCCR2B = 0b00000011;	//Liga o timer 2 com prescaler de 64
	OCR2A = 249;			//Ajusta o valor de compara��o do timer 2 para 249
	TIMSK2 = 0b00000010;	//Habilita a interrup��o na igualdade de compara��o com OCR2A e OCR2A. Ocorre a interrup��o a cada 1 ms = (64*(249+1))/16MHz
	//Timer 0
	TCCR0A = 0b00100011;	//habilita Timer 0 no modo PWM r�pido, OC0B no modo n�o invertido
	TCCR0B = 0b00000011;	//liga TC0 com prescaler de 64, fPWM = 16000000/(256*64) = 976.56 Hz
	//Timer 1
	TCCR1B = (1<<ICES1)|(1<<CS12);	//habilita modo de captura na borda de subida, TC1 com prescaler de 256; uma contagem a cada 16 us

	//Interrup��es
	TIMSK1 = 1<<ICIE1; //habilita interrp��o por captura do TC1
	EICRA = 0b00001010;		//interrup��es IN0 e INT1 na borda de descida
	EIMSK = 0b00000011;		//habilita interrup��o INT0 e INT1
	PCICR = 0b00000100;		//habilita interrup��o por mudan�a de estado na porta D
	PCMSK2 = 0b11010000;	//habilita interrup��o por mudan�a de estado no pino PD4, PD6 e PD7
	sei();					//habilita interrup��es globais

	Park_flag = PIND&Park;
	Drive_flag = PIND&Dri_Rea;
	
	Distancia_KM = eeprom_read_word(endereco_eeprom_dist);	//lendo valor guardado na eeprom
	Diametro = eeprom_read_word(endereco_eeprom_diam);		//lendo valor guardado na eeprom

	Distancia_cm = Distancia_KM*100000;	//recalculando distancia em cm com base no valor guardado na eeprom
	
	Shift_reg_init();
	USART_init(9600);
	GLCD_Setup();									//inicializa��o do display
	GLCD_SetFont(Font5x8, 5, 8, GLCD_Merge);	//definindo a fonte a ser usada no display
	
	while(ADMUX != 0b01000011){}	//espera as leituras do ADC
	if(ADC_Temp > 114)
	Flag_Temp = 1;
	if(ADC_Bat < V_bat_min)
	Flag_Tensao = 1;

	while (1)
	{
		static uint16_t tempo_ms_temp = 0, tempo_ms_Vbat = 0;
		
		if(eeprom_read_word(endereco_eeprom_dist) != Distancia_KM)	//quando a distancia for mudada escreve-se o novo valor na eeprom
		eeprom_write_word(endereco_eeprom_dist,Distancia_KM);
		
		if(Temp_Bat > eeprom_read_byte(endereco_eeprom_tempmax))		//se a temperatura atual for maior que a guardada na eeprom
		eeprom_write_byte(endereco_eeprom_tempmax,Temp_Bat);			//grava a nova m�xima na eeprom
		
		if(ADC_FSG>562 && PINB&Cinto_)
		Flag_cinto = 1;
		else
		Flag_cinto = 0;
		
		if(Temp_Bat > Temp_bat_max){
			if((tempo_ms - tempo_ms_temp) > 1000)		//se a teperatura permanecer acima do m�ximo por 1 seg
			Flag_Temp = 1;
		}
		else
		tempo_ms_temp = tempo_ms;
		
		if(ADC_Bat < V_bat_min){
			if((tempo_ms - tempo_ms_Vbat) > 1000)	//se a tens�o permanecer abaixo do m�ximo por 1 seg
			Flag_Tensao = 1;
		}
		else
		tempo_ms_Vbat = tempo_ms;
		
		if((Flag_Tensao == 0) && (Flag_Temp == 0)){		//s� faz o calculo do curso do acelerador se nenhuma das flags relacionadas a bateria for 1
			PORTB |= Power_rele;
			if(Velocidade > 20 && Distancia_sonar < 300)	//se a velocidade for superior a 20km e a distancia inferior a 3m o duty cicle � limitado a 10%
			OCR0B = map(ADC_Acelerador,0,1023,0,255)/10;
			else											//se n�o, n�o h� limita��o
			OCR0B = map(ADC_Acelerador,0,1023,0,255);
		}
		else{
			OCR0B = 0;
			PORTB &= !Power_rele;
		}
		
		if(ADC_Temp != ADC1_a || ADC_Bat != ADC2_a){	//se algum valor do ADC mudar refaz os calculos relacionados ao ADC
			ADC1_a = ADC_Temp;
			ADC2_a = ADC_Bat;
			
			if(map(ADC_Temp,92.9907,152.99988,0,200) < 0)		//Corrige overflow caso o numero seja negativo
			Temp_Bat = 0;
			else
			Temp_Bat = map(ADC_Temp,93,153,0,197);	//mapeia o valor de ADC_Temp de 93 a 153 para 0 a 200 e salva em Temp_Bat
			
			if(map(ADC_Bat,613.8,859.32,0,100)<0)				//Corrige overflow caso o numero seja negativo
			Nivel_Bat = 0;
			else
			Nivel_Bat = map(ADC_Bat,613.8,859.32,0,100);			//converte uma varia��o de 3 V a 4.2 V para uma varia��o de 0% a 100%
		}

		if(RPM != RPM_a ||Diametro != Diametro_a){	//s� ocorre uma atualiza��o dos valores quando algo relacionado muda
			RPM_a = RPM;
			Diametro_a = Diametro;
			Velocidade = (Diametro*3.141592*RPM)*6/10000;	//diametro*pi*RPM = cm/min ==> cm/min * 60/100000 = km/h
			Shift_reg(Velocidade);
		}
		if(tempo_ms%100 == 0)	//atualiza o display oled a cada 100 ms
		refresh_oled ();
	}
}

//Implementa��o das fun��es
void refresh_oled (){
	bool flag_tempo = 0;
	if(tempo_ms%100 == 0)
	flag_tempo = 1;
	if((Flag_Tensao == 0) && (Flag_Temp == 0)){	//Se as flags da bateria n�o forem true
		uint8_t y = 1; //posi��o vertical do cursor
		GLCD_Clear();
		GLCD_GotoXY(1,y);
		GLCD_PrintString("LASD Car");
		GLCD_GotoXY(107,y+3);
		GLCD_PrintInteger(Temp_Bat);
		GLCD_PrintString("C");
		GLCD_GotoXY(1,y+=12);
		GLCD_PrintInteger(RPM);
		GLCD_PrintString(" rpm");
		GLCD_GotoXY(107,y+3);
		GLCD_PrintInteger(Nivel_Bat);
		GLCD_PrintString("%");
		GLCD_GotoXY(1,y+=12);
		GLCD_PrintString("Sonar: ");
		GLCD_PrintInteger(Distancia_sonar);
		GLCD_GotoXY(1,y+=12);
		GLCD_PrintString("D. Pneu: ");
		GLCD_PrintInteger(Diametro);
		GLCD_PrintString(" cm");
		GLCD_GotoXY(18,y+=14);
		GLCD_PrintInteger(Distancia_KM);
		GLCD_PrintString("Km");
		GLCD_GotoXY(115,y);

		switch((Park_flag<<1)+Drive_flag){ //testando estado das flags
			case 0b10:
			GLCD_PrintString("D");
			break;
			case 0b11:
			GLCD_PrintString("R");
			break;
			default:
			GLCD_PrintString("P");
			break;
		}
		GLCD_DrawLine(1,10,48,10,GLCD_Black);
		GLCD_DrawRectangle(10,y-3,60,y+10,GLCD_Black);
		GLCD_DrawRectangle(110,y-3,125,y+10,GLCD_Black);
		GLCD_DrawRectangle(104,1,126,25,GLCD_Black);
		if(Flag_cinto){
			static bool Switch = 0;
			if(flag_tempo){	//Tentei Switch = !Switch e n�o funcionou :(
				if(Switch)
				Switch = 0;
				else
				Switch = 1;
			}
			GLCD_GotoXY(77,1);
			if(Switch)
			GLCD_DrawBitmap(Cinto,23,29,GLCD_Merge);
			else
			GLCD_DrawBitmap(Cinto_Seta,23,29,GLCD_Merge);
		}
		
		GLCD_DrawRoundRectangle(104,27,124,35,1,GLCD_Black);	//icone est�tico da bateria
		GLCD_FillRectangle(124,30,126,32,GLCD_Black);			//icone est�tico da bateria
		GLCD_FillRectangle(104,28,map(Nivel_Bat,0,100,105,124)/*((Nivel_Bat*((float)19/100))+105)*/,34,GLCD_Black); //anima��o carga da bateria
		GLCD_Render();
	}
	if(Flag_Tensao){
		GLCD_Clear();
		GLCD_GotoXY(0,0);
		GLCD_DrawBitmap (Tensao_Baixa,128,64,GLCD_Overwrite);
		_delay_ms(500);
		GLCD_InvertScreen();
		_delay_ms(500);
		GLCD_Render();
	}
	if(Flag_Temp){
		GLCD_Clear();
		GLCD_GotoXY(0,0);
		GLCD_DrawBitmap (Temperatura_Alta,128,64,GLCD_Overwrite);
		_delay_ms(500);
		GLCD_InvertScreen();
		_delay_ms(500);
		GLCD_Render();
	}
}

double map(double x, double en_min, double en_max, double sa_min, double sa_max) {
	return (x - en_min) * (sa_max - sa_min) / (en_max - en_min) + sa_min;
}

//interrup��es

//ADC
ISR(ADC_vect){
	
	static uint8_t cont_ADC = 0;
	
	switch(cont_ADC){
		case 0:
		ADC_Acelerador = ADC;	//l� o valor do canal 0 do adc
		ADMUX = 0b01000001;		//muda a proxima leitura para o canal 1
		break;
		
		case 1:
		ADC_Temp = ADC;			//l� o valor do canal 1 do adc
		ADMUX = 0b01000010;		//muda a proxima leitura para o canal 2
		break;
		
		case 2:
		ADC_Bat = ADC;			//l� o valor do canal 2 do adc
		ADMUX = 0b01000011;		//muda a proxima leitura para o canal 0
		break;
		
		case 3:
		ADC_FSG = ADC;			//l� o valor do canal 3 do adc
		ADMUX = 0b01000000;		//muda a proxima leitura para o canal 2
		cont_ADC = -1;			//reseta o contador
		break;
	}
	cont_ADC++;
}

//Captura do Timer 1
ISR(TIMER1_CAPT_vect){
	
	static uint16_t t_subida = 0;
	static uint16_t t_sinal = 0;
	
	if(TCCR1B & (1<<ICES1))	//l� o avalor de contagem do TC1 na borda de subida
	t_subida = ICR1;		//salva contagem na borda de subida
	
	else					//l� o avalor de contagem do TC1 na borda de descida
	t_sinal = (ICR1-t_subida)*16;	//1 contagem = 16 us
	
	TCCR1B ^= (1<<ICES1);	//inverte a borda de captura
	
	Distancia_sonar = t_sinal / 58;	//calcula a distancia em cm lida pelo sonar
}

//Compara��o Timer 2
ISR(TIMER2_COMPA_vect){
	static uint32_t tempo_ms_bug = 0;	//corrige bug do simulador
	tempo_ms_bug++;						//corrige bug do simulador
	tempo_ms = tempo_ms_bug/2;
}

//Mudan�a de estado INT0
ISR(INT0_vect){
	static uint8_t cont_borda = 0;
	static uint16_t t_descida = 0;
	static uint16_t periodo_16us = 0;
	
	if(cont_borda==3){	//l� o avalor de contagem do TC1 a cada 3 bordas
		cont_borda = 0;
		periodo_16us = (TCNT1-t_descida);	//tempo de 3 bordas
		t_descida = TCNT1;
		RPM = 3750000/((float)periodo_16us/3);	//calcula RPM com a m�dia de 3 leituras
		Distancia_cm += (3.141592*Diametro*3);	//calcula distancia a cada 3 leituras
		Distancia_KM = Distancia_cm/100000;
	}
	cont_borda++;
}

//Mudan�a de estado INT1
ISR(INT1_vect){
	Diametro++;
	eeprom_write_word(endereco_eeprom_diam,Diametro);		//quando o diametro for mudado escreve-se o novo valor na eeprom
}

//Mudan�a de estado na porta D
ISR(PCINT2_vect){
	if(!(PIND&bt2)){
		if(Diametro>0){
			Diametro--;
			eeprom_write_word(endereco_eeprom_diam,Diametro);		//quando o diametro for mudado escreve-se o novo valor na eeprom
		}
		
	}
	Park_flag = PIND&Park;		//atualiza a flag com o estado do pino
	Drive_flag = PIND&Dri_Rea;
}

//Mensagem recebia via USART
ISR(USART_RX_vect){
	char Recebido;
	
	Recebido = UDR0;	//guarda o valor recebido
	if(Recebido == 'd'){												// se o valor recebido for o caracter "d"
		USART_send_string("Temperatura m�xima: ");
		USART_send_byte(eeprom_read_byte(endereco_eeprom_tempmax));		//retorna o valor da temp. max guardado na eeprom
		USART_send_string("�C");
	}
	
	if(Recebido == 'l'){								// se o valor recebido for o caracter "l"
		USART_send_stringln("A temperatura m�xima registrada foi zerada!");
		eeprom_write_byte(endereco_eeprom_tempmax,0);	//atribui 0 a o valor da temp. max guardado na eeprom
	}
}