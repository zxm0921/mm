#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 


extern void usart_init(uint32_t baud);
extern void usart2_init(uint32_t baud);
extern void usart2_send_str(const char *str);
extern void usart1_send_str(const char *str);

#endif


