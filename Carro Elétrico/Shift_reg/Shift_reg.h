/*
 * Shift_reg.h
 *
 * Created: 03/04/2022 12:15:16
 *  Author: Luciano Júnior Santos Brito - 118111399
 */ 


#ifndef SHIFT_REG_H_
#define SHIFT_REG_H_

#include <avr/io.h>

#define PORTA_reg	PORTB
#define DDR_reg		DDRB

#define Shift_reg_clk	(1<<1)	//clock do shift reg
#define Shift_reg_en	(1<<2)	//entrada do shift reg
#define t 100			//tempo em us entre mudança de estado do shift_reg

void Shift_reg_init();							//inicia a porta do registrador
void Shift_reg(uint16_t velocidade_veiculo);	//carrega a uint16_t no registrador
void Shift_reg_bit(uint8_t bit);				//carrega um bit no registrador

#endif /* SHIFT_REG_H_ */