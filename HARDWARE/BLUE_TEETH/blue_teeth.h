#ifndef __BLUE_TEETH_H
#define __BLUE_TEETH_H	 
#include "stm32f4xx.h" 
#include "sys.h"
#include "includes.h"

extern uint32_t i;

extern void usart3_init(uint32_t baud);
extern void usart3_send_str(const char *str);


#endif


