#ifndef __KEYBOARD_H
#define __KEYBOARD_H	 
#include "stm32f4xx.h" 



void keyboard_init(void);
uint8_t get_key_board(void);
void key_state(void);


#endif
