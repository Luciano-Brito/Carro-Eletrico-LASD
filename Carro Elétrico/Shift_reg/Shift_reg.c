/*
 * Shift_reg.c
 *
 * Created: 03/04/2022 12:18:17
 *  Author: Luciano Júnior Santos Brito - 118111399
 */ 

#include <util/delay.h>
#include <avr/io.h>
#include "Shift_reg.h"

void Shift_reg_init(){
	DDR_reg |= (Shift_reg_clk|Shift_reg_en);
}

void Shift_reg(uint16_t velocidade_veiculo){
	PORTA_reg |= (Shift_reg_clk);	//aplica um pulso de clock
	_delay_us(t);
	PORTA_reg &= !(Shift_reg_clk);
	_delay_us(t);
	Shift_reg_bit(1);		//carrega o segundo bit 1
	for(uint8_t i=0;i<4;i++){	//gravando unidade no registrador
		if(((velocidade_veiculo%10)>>i)&(0b1))
		Shift_reg_bit(1);
		else
		Shift_reg_bit(0);
	}
	
	for(uint8_t i=0;i<4;i++){	//gravando dezena no registrador
		if((((velocidade_veiculo/10)%10)>>i)&(0b1))
		Shift_reg_bit(1);
		else
		Shift_reg_bit(0);
	}
	for(uint8_t i=0;i<4;i++){	//gravando centena no registrador
		if(((velocidade_veiculo/100)>>i)&(0b1))
		Shift_reg_bit(1);
		else
		Shift_reg_bit(0);
	}
}

void Shift_reg_bit(uint8_t bit){
	PORTA_reg &= !(Shift_reg_clk|Shift_reg_en );	//zerando saída da porta B
	_delay_us(t);
	if(bit){
		PORTA_reg |= (Shift_reg_en);
		_delay_us(t);
		PORTA_reg |= (Shift_reg_clk)|(Shift_reg_en);
		_delay_us(t);
	}
	else{
		PORTA_reg |= (Shift_reg_clk);
		_delay_us(t);
	}
	PORTA_reg &= !(Shift_reg_clk|Shift_reg_en );
	_delay_us(t);
}