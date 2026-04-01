/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                             (c) Copyright 2013; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                           MASTER INCLUDES
*
*                                       IAR Development Kits
*                                              on the
*
*                                    STM32F429II-SK KICKSTART KIT
*
* Filename      : includes.h
* Version       : V1.00
* Programmer(s) : FT
*********************************************************************************************************
*/

#ifndef  INCLUDES_MODULES_PRESENT
#define  INCLUDES_MODULES_PRESENT


/*
*********************************************************************************************************
*                                         STANDARD LIBRARIES
*********************************************************************************************************
*/


#include  <stdio.h>
#include  <string.h>
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>
#include  <math.h>


/*
*********************************************************************************************************
*                                                 OS
*********************************************************************************************************
*/

#include  <os.h>


/*
*********************************************************************************************************
*                                              LIBRARIES
*********************************************************************************************************
*/

#include  <cpu.h>
#include  <lib_def.h>
#include  <lib_ascii.h>
#include  <lib_math.h>
#include  <lib_mem.h>
#include  <lib_str.h>

/*
*********************************************************************************************************
*                                              APP / BSP
*********************************************************************************************************
*/

#include  <app_cfg.h>
#include  <bsp.h>



//事件标志组：硬件使用到的标志位
#define FLAG_GRP_RTC_WAKEUP					0x01
#define FLAG_GRP_SR04_WAKEUP				0x02
#define FLAG_GRP_BLUE_SEND					0x04
#define FLAG_GRP_IR_WAKEUP					0x08
#define FLAG_GRP_BEEP_CLOSE					0x10
#define FLAG_GRP_GSM_WAKEUP				    0x20
#define FLAG_GRP_NUM_WAKEUP				    0x40

//内核对象
extern OS_FLAG_GRP			g_flag_grp;					//事件标志组的对象

extern OS_MUTEX				g_mutex_printf;				//互斥锁的对象
extern OS_MUTEX				g_mutex_oled;				//互斥锁的对象

extern OS_Q					g_queue_led;				//消息队列的对象
extern OS_Q	 				g_queue_usart1;				//消息队列的对象
extern OS_Q	 				g_queue_usart2;				//消息队列的对象
extern OS_Q					g_queue_ir;					//消息队列的对象
extern OS_Q					g_queue_beep;				//消息队列的对象
extern OS_Q					g_queue_blueteeth;			//消息队列的对象
extern OS_Q					g_queue_rfid;				//消息队列的对象
extern OS_Q					g_queue_smoke;				//消息队列的对象
extern OS_Q					g_queue_gsm;				//消息队列的对象
extern OS_Q					g_queue_keyboard;			//消息队列的对象
extern OS_Q					g_queue_dht11;				//消息队列的对象
extern OS_Q					g_queue_send;				//消息队列的对象


extern OS_SEM				g_sem_led;					//信号量的对象
extern OS_SEM				g_sem_beep;					//信号量的对象

//任务控制块
extern OS_TCB app_task_tcb_led;
extern OS_TCB app_task_tcb_rtc;

extern OS_TCB app_task_tcb_dht11;
extern OS_TCB app_task_tcb_ir;
extern OS_TCB app_task_tcb_key;
extern OS_TCB app_task_tcb_sta;

extern char g_ir;
extern int ir_state;

#endif
