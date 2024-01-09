/*
 * lcd1602A.c
 *
 *  Created on: 2022. 9. 19.
 *      Author: PECSCHO
 */
#include "device_registers.h"
#include "clocks_and_modes.h"
#include "lcd1602A.h"

int flag_counter=0;

#define BASE 	 9
#define EN		 4
#define RW		 5
#define RS		 6
#define BF		 3


void delay_100ns(uint32_t ns){
	   uint32_t timeout;
	  PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);     //Clock Src = 6 (SPLL2_DIV2_CLK)
	  PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK;  //Enable clk to LPIT0 regs

	  LPIT0->MCR |= LPIT_MCR_M_CEN_MASK;

	  timeout=ns * 4;

	  LPIT0->TMR[0].TVAL = timeout;
	  LPIT0->TMR[0].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK;

	   while (0 == (LPIT0->MSR & LPIT_MSR_TIF0_MASK)) {} /* Wait for LPIT0 CH0 Flag */
	   flag_counter++;         /* Increment LPIT0 timeout counter */
	               LPIT0->MSR |= LPIT_MSR_TIF0_MASK; /* Clear LPIT0 timer flag 0 */

}




void lcdEN(void){
	PTD->PSOR |= 1<<(BASE+EN);
}

void lcdNEN(void){
	PTD->PCOR |= 1<<(BASE+EN);
}




void lcdinit(void){


	lcdinput(0x28);	//lcd function set #1
	delay_100ns(70000);

	lcdinput(0x28);	//lcd function set #2
	delay_100ns(70000);

	lcdinput(0x28);	//lcd function set #3
	delay_100ns(70000);


	lcdinput(0x08);	//lcd off
	delay_100ns(700);

	lcdinput(0x01);	//lcd clear
	delay_100ns(20000);

	lcdinput(0x06);	//lcd modeset
	delay_100ns(700);


	lcdinput(0x0F);	//lcd display,cursor
	delay_100ns(20000);

	lcdinput(0x80);	//dd ram address
	delay_100ns(700);


	lcdinput(0x02);	//return home
	delay_100ns(20000);

}

void lcdinput(uint16_t data){
	uint16_t data1 = ((uint16_t)data&0xF0)<<(BASE-4);	//D7 ~ 4
	uint16_t data2 = (((uint16_t)data&0x0F)<<BASE);		//D3 ~ 0
	PTD->PCOR |= 1<<(BASE+RW);

	//data1
	lcdEN();
	PTD->PCOR |= 0xF<<BASE;
	PTD->PSOR |= data1;
	delay_100ns(5);

	lcdNEN();

	//clear before 2nd transmission
	delay_100ns(2);
	PTD->PCOR |= 0xF<<BASE;
	delay_100ns(30);

	//data2
	lcdEN();
	PTD->PCOR |= 0xF<<BASE;
	PTD->PSOR |= data2;
	delay_100ns(5);

	lcdNEN();

	//clear before next order
	delay_100ns(2);
	PTD->PCOR |= 0xF<<BASE;
	PTD->PCOR |= ((1<<(BASE+RS))+(0xF<<BASE));
	delay_100ns(30);

}

void lcdcharinput(char data){
	uint16_t data1 = ((uint16_t)data&0xF0)<<(BASE-4); //D7 ~ 4
	uint16_t data2 = (((uint16_t)data&0x0F)<<BASE); //D3 ~ 0

	PTD->PSOR |= 1<<(BASE+RS);
	//data1
	lcdEN();
	PTD->PCOR |= (0xF<<BASE);
	PTD->PSOR |= data1;
	delay_100ns(3);

	lcdNEN();

	//clear before 2nd transmission
	delay_100ns(2);
	PTD->PCOR |= 0xF<<BASE;
	delay_100ns(30);


	//data2
	lcdEN();
	PTD->PCOR |= 0xF<<BASE;
	PTD->PSOR |= data2;
	delay_100ns(30);

	lcdNEN();

	//clear before next order
	delay_100ns(2);
	PTD->PCOR |= 0xF<<BASE;
	PTD->PCOR |= ((1<<(BASE+RS))+(0xF<<BASE));
	delay_100ns(30);

}
