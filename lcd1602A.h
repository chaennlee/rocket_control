/*
 * lcd1602A.h
 *
 *  Created on: 2022. 9. 19.
 *      Author: PECSCHO
 */

#ifndef LCD1602A_H_
#define LCD1602A_H_

void lcdinit(void);
void lcdinput(uint16_t data);
void lcdcharinput(char data);
void busycheck(void);
#endif /* LCD1602A_H_ */
