#include "sys.h"
#include <stdarg.h>
#include <string.h>
#include "includes.h"
#include "delay.h"
#include "usart.h"
#include "dht11.h"
#include "oled.h"
#include "ir.h"
#include "rtc.h"
#include "bmp.h"
#include "beep.h"
#include "iwdg.h"
#include "blue_teeth.h"
#include "sr04.h"
#include "fire.h"
#include "smoke.h"
#include "RFID.h"
#include "MFRC522.h"
#include "keyboard.h"


uint32_t er = 0;
uint32_t er2 = 0;
static volatile char gsm_buf[100] = {0};


//任务init控制块
OS_TCB app_task_tcb_init;
void app_task_init(void *parg);
CPU_STK app_task_stk_init[512];			//任务堆栈，大小为512字，也就是2048字节



//任务beep控制块
OS_TCB app_task_tcb_beep;
void app_task_beep(void *parg);
CPU_STK app_task_stk_beep[512];			//任务堆栈，大小为512字，也就是2048字节

//任务rtc控制块
OS_TCB app_task_tcb_rtc;
void app_task_rtc(void *parg);
CPU_STK app_task_stk_rtc[512];			//任务堆栈，大小为512字，也就是2048字节



//任务dht11控制块
OS_TCB app_task_tcb_dht11;
void app_task_dht11(void *parg);
CPU_STK app_task_stk_dht11[512];		//任务堆栈，大小为512字，也就是2048字节


//任务ir控制块
OS_TCB app_task_tcb_ir;
void app_task_ir(void *parg);
CPU_STK app_task_stk_ir[512];			//任务堆栈，大小为512字，也就是2048字节



//任务usart1控制块
OS_TCB app_task_tcb_usart1;
void app_task_usart1(void *parg);
CPU_STK app_task_stk_usart1[512];		//任务堆栈，大小为512字，也就是2048字节

//任务usart2控制块
OS_TCB app_task_tcb_usart2;
void app_task_usart2(void *parg);
CPU_STK app_task_stk_usart2[512];		//任务堆栈，大小为512字，也就是2048字节


//任务blueteeth控制块
OS_TCB app_task_tcb_blueteeth;
void app_task_blueteeth(void *parg);
CPU_STK app_task_stk_blueteeth[512];		//任务堆栈，大小为512字，也就是2048字节


////任务sr04控制块
//OS_TCB app_task_tcb_sr04;
//void app_task_sr04(void *parg);
//CPU_STK app_task_stk_sr04[512];		//任务堆栈，大小为512字，也就是2048字节


//任务gsm控制块
OS_TCB app_task_tcb_gsm;
void app_task_gsm(void *parg);
CPU_STK app_task_stk_gsm[512];		//任务堆栈，大小为512字，也就是2048字节


//任务fire控制块
OS_TCB app_task_tcb_fire;
void app_task_fire(void *parg);
CPU_STK app_task_stk_fire[512];		//任务堆栈，大小为512字，也就是2048字节


//任务smoke控制块
OS_TCB app_task_tcb_smoke;
void app_task_smoke(void *parg);
CPU_STK app_task_stk_smoke[512];		//任务堆栈，大小为512字，也就是2048字节


//任务rfid控制块
OS_TCB app_task_tcb_rfid;
void app_task_rfid(void *parg);
CPU_STK app_task_stk_rfid[512];		//任务堆栈，大小为512字，也就是2048字节

//任务keyboard控制块
OS_TCB app_task_tcb_keyboard;
void app_task_keyboard(void *parg);
CPU_STK app_task_stk_keyboard[512];		//任务堆栈，大小为512字，也就是2048字节


//任务发送数据给蓝牙控制块
OS_TCB app_task_tcb_send;
void app_task_send(void *parg);
CPU_STK app_task_stk_send[512];		//任务堆栈，大小为512字，也就是2048字节


//任务计算错误输入次数
OS_TCB app_task_tcb_num;
void app_task_num(void *parg);
CPU_STK app_task_stk_num[512];		//任务堆栈，大小为512字，也就是2048字节


OS_FLAG_GRP				g_flag_grp;			//事件标志组的对象

OS_MUTEX				g_mutex_printf;		//互斥锁的对象
OS_MUTEX				g_mutex_oled;		//互斥锁的对象


OS_Q					g_queue_usart1;		//串口1消息队列的对象
OS_Q					g_queue_usart2;		//串口2消息队列的对象
OS_Q					g_queue_ir;			//红外线消息队列的对象
OS_Q					g_queue_beep;		//蜂鸣器消息队列的对象
OS_Q					g_queue_blueteeth;	//蓝牙消息队列的对象
OS_Q					g_queue_rfid;		//RFID消息队列的对象
OS_Q					g_queue_smoke;		//烟雾传感器消息队列的对象
OS_Q					g_queue_gsm;		//GSM模块消息队列的对象
OS_Q					g_queue_keyboard;	//矩阵键盘消息队列的对象
OS_Q					g_queue_dht11;		//温湿度消息队列的对象
OS_Q					g_queue_send;		//发送给蓝牙消息队列的对象


OS_SEM					g_sem_beep;			//信号量的对象

OS_TMR 					g_tmr_wdt;			//软件定时器的对象	



//安全输出串口信息函数
#define DEBUG_PRINTF_EN	1
void dgb_printf_safe(const char *format, ...)
{
#if DEBUG_PRINTF_EN	
	OS_ERR err;
	
	va_list args;
	va_start(args, format);
	
	OSMutexPend(&g_mutex_printf,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
	vprintf(format, args);
	OSMutexPost(&g_mutex_printf,OS_OPT_POST_NONE,&err);
	
	va_end(args);
#else
	(void)0;
#endif
}

void  timer_wdt_callback(OS_TMR *p_tmr, void *p_arg);



//主函数
int main(void)
{
	OS_ERR err;

	systick_init();  											//时钟初始化
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//中断分组配置
	
	usart_init(115200);  				 						//串口1初始化
	usart2_init(115200);  				 						//串口2初始化
	usart3_init(9600);  				 						//串口3初始化
	
	        											


	//OS初始化，它是第一个运行的函数,初始化各种的全局变量，例如中断嵌套计数器、优先级、存储器
	OSInit(&err);
	
	
	//创建任务
	OSTaskCreate(	&app_task_tcb_init,							//任务控制块，等同于线程id
					"app_task_init",							//任务的名字，名字可以自定义的
					app_task_init,								//任务函数，等同于线程函数
					0,											//传递参数，等同于线程的传递参数
					7,											//任务的优先级7		
					app_task_stk_init,							//任务堆栈基地址
					512/10,										//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					512,										//任务堆栈大小			
					0,											//禁止任务消息队列
					0,											//默认是抢占式内核															
					0,											//不需要补充用户存储区
					OS_OPT_TASK_NONE,							//创建任务无额外操作
					&err										//返回的错误码
				);	
				

	//启动OS，进行任务调度
	OSStart(&err);
				
	
					
	while(1);
}


void app_task_init(void *parg)
{
	OS_ERR err;	
	
	//蜂鸣器初始化
	BEEP_Init();
	
	//温湿度传感器的初始化
	dht11_init();

	//火焰传感器adc初始化
	adc_init();
	fire_init();
	
	//烟雾传感器初始化
	smoke_init();
	smoke_adc_init();
	
	//矩阵键盘初始化
	keyboard_init();
	
	//初始化OLED
	OLED_Init();
	OLED_Clear();
	
	//初始化RFID
	MFRC522_Initializtion();
	
	//显示名字
	OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_name);	

	//持续2秒
	delay_ms(2000);
	OLED_Clear();	
	
	
	//清屏
	OLED_Clear();
	
	
	//rtc初始化
	rtc_init();	
	
	
	
	//红外的初始化
	ir_init();
	
	
	
	
	
	
	//创建事件标志组，所有标志位初值为0
	OSFlagCreate(&g_flag_grp,"g_flag_grp",0,&err);
	
	
	//创建消息队列，用于控制蜂鸣器
	OSQCreate(&g_queue_beep,"g_queue_beep",16,&err);
	
	//创建消息队列，用于处理ir
	OSQCreate(&g_queue_ir,"g_queue_ir",16,&err);	
	
	//创建消息队列，用于处理串口1数据
	OSQCreate(&g_queue_usart1,"g_queue_usart1",16,&err);

	//创建消息队列，用于处理串口2数据
	OSQCreate(&g_queue_usart2,"g_queue_usart2",16,&err);
	
	//创建消息队列，用于处理rfid数据
	OSQCreate(&g_queue_rfid,"g_queue_rfid",16,&err);
	
	//创建消息队列，用于处理gsm数据
	OSQCreate(&g_queue_gsm,"g_queue_gsm",16,&err);
	
	//创建消息队列，用于处理keyboard数据
	OSQCreate(&g_queue_keyboard,"g_queue_keyboard",16,&err);
	
	//创建消息队列，用于处理blueteeth数据
	OSQCreate(&g_queue_blueteeth,"g_queue_blueteeth",16,&err);
	
	
	//创建消息队列，用于处理发送数据给蓝牙数据
	OSQCreate(&g_queue_send,"g_queue_send",16,&err);
	
	
	//创建消息队列，用于处理dht11数据
	OSQCreate(&g_queue_dht11,"g_queue_dht11",16,&err);
	
	
	//创建互斥锁
	OSMutexCreate(&g_mutex_printf,	"g_mutex_printf",&err);	
	OSMutexCreate(&g_mutex_oled,	"g_mutex_oled",&err);	


	
	
	//创建信号量，初值为0，有一个资源
	OSSemCreate(&g_sem_beep,"g_sem_beep",0,&err);	
		
	
	
	//创建蜂鸣器管理任务
	OSTaskCreate(	&app_task_tcb_beep,							//任务控制块，等同于线程id
					"app_task_beep",							//任务的名字，名字可以自定义的
					app_task_beep,								//任务函数，等同于线程函数
					0,											//传递参数，等同于线程的传递参数
					7,											//任务的优先级7		
					app_task_stk_beep,							//任务堆栈基地址
					512/10,										//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					512,										//任务堆栈大小			
					0,											//禁止任务消息队列
					0,											//默认是抢占式内核															
					0,											//不需要补充用户存储区
					OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,	//开启堆栈检测与清空任务堆栈
					&err										//返回的错误码
				);		
	
	//创建时间管理任务
	OSTaskCreate(	&app_task_tcb_rtc,							//任务控制块
					"app_task_rtc",								//任务的名字
					app_task_rtc,								//任务函数
					0,											//传递参数
					7,											//任务的优先级7		
					app_task_stk_rtc,							//任务堆栈基地址
					512/10,										//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					512,										//任务堆栈大小			
					0,											//禁止任务消息队列
					0,											//默认是抢占式内核																
					0,											//不需要补充用户存储区
					OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,	//开启堆栈检测与清空任务堆栈
					&err										//返回的错误码
				);
				
				
	//创建温湿度管理任务
	OSTaskCreate(	&app_task_tcb_dht11,						//任务控制块
					"app_task_dht11",							//任务的名字
					app_task_dht11,								//任务函数
					0,											//传递参数
					7,											//任务的优先级7		
					app_task_stk_dht11,							//任务堆栈基地址
					512/10,										//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					512,										//任务堆栈大小			
					0,											//禁止任务消息队列
					0,											//默认是抢占式内核																
					0,											//不需要补充用户存储区
					OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,	//开启堆栈检测与清空任务堆栈
					&err										//返回的错误码
				);
				
	//创建红外线管理任务
	OSTaskCreate(	&app_task_tcb_ir,							//任务控制块
					"app_task_ir",								//任务的名字
					app_task_ir,								//任务函数
					0,											//传递参数
					7,											//任务的优先级7		
					app_task_stk_ir,							//任务堆栈基地址
					512/10,										//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					512,										//任务堆栈大小			
					0,											//禁止任务消息队列
					0,											//默认是抢占式内核																
					0,											//不需要补充用户存储区
					OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,	//开启堆栈检测与清空任务堆栈
					&err										//返回的错误码
				);			


	
				
	//创建串口1管理任务
	OSTaskCreate(	&app_task_tcb_usart1,						//任务控制块
					"app_task_usart1",							//任务的名字
					app_task_usart1,							//任务函数
					0,											//传递参数
					7,											//任务的优先级7		
					app_task_stk_usart1,						//任务堆栈基地址
					512/10,										//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					512,										//任务堆栈大小			
					0,											//禁止任务消息队列
					0,											//默认是抢占式内核																
					0,											//不需要补充用户存储区
					OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,	//开启堆栈检测与清空任务堆栈
					&err										//返回的错误码
				);

	//创建串口2管理任务
	OSTaskCreate(	&app_task_tcb_usart2,						//任务控制块
					"app_task_usart2",							//任务的名字
					app_task_usart2,							//任务函数
					0,											//传递参数
					7,											//任务的优先级7		
					app_task_stk_usart2,						//任务堆栈基地址
					512/10,										//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					512,										//任务堆栈大小			
					0,											//禁止任务消息队列
					0,											//默认是抢占式内核																
					0,											//不需要补充用户存储区
					OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,	//开启堆栈检测与清空任务堆栈
					&err										//返回的错误码
				);


	//创建蓝牙接收数据管理任务
	OSTaskCreate(	&app_task_tcb_blueteeth,						//任务控制块
					"app_task_blueteeth",							//任务的名字
					app_task_blueteeth,							//任务函数
					0,											//传递参数
					7,											//任务的优先级7		
					app_task_stk_blueteeth,						//任务堆栈基地址
					512/10,										//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					512,										//任务堆栈大小			
					0,											//禁止任务消息队列
					0,											//默认是抢占式内核																
					0,											//不需要补充用户存储区
					OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,	//开启堆栈检测与清空任务堆栈
					&err										//返回的错误码
				);
				
				
//	//创建超声波管理任务
//	OSTaskCreate(	&app_task_tcb_sr04,						//任务控制块
//					"app_task_sr04",							//任务的名字
//					app_task_sr04,							//任务函数
//					0,											//传递参数
//					7,											//任务的优先级7		
//					app_task_stk_sr04,						//任务堆栈基地址
//					512/10,										//任务堆栈深度限位，用到这个位置，任务不能再继续使用
//					512,										//任务堆栈大小			
//					0,											//禁止任务消息队列
//					0,											//默认是抢占式内核																
//					0,											//不需要补充用户存储区
//					OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,	//开启堆栈检测与清空任务堆栈
//					&err										//返回的错误码
//				);
				
				
	//创建发信息管理任务
	OSTaskCreate(	&app_task_tcb_gsm,						//任务控制块
					"app_task_gsm",							//任务的名字
					app_task_gsm,							//任务函数
					0,											//传递参数
					7,											//任务的优先级7		
					app_task_stk_gsm,						//任务堆栈基地址
					512/10,										//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					512,										//任务堆栈大小			
					0,											//禁止任务消息队列
					0,											//默认是抢占式内核																
					0,											//不需要补充用户存储区
					OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,	//开启堆栈检测与清空任务堆栈
					&err										//返回的错误码
				);
				
				
	//创建火光管理任务
	OSTaskCreate(	&app_task_tcb_fire,						//任务控制块
					"app_task_fire",							//任务的名字
					app_task_fire,							//任务函数
					0,											//传递参数
					7,											//任务的优先级7		
					app_task_stk_fire,						//任务堆栈基地址
					512/10,										//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					512,										//任务堆栈大小			
					0,											//禁止任务消息队列
					0,											//默认是抢占式内核																
					0,											//不需要补充用户存储区
					OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,	//开启堆栈检测与清空任务堆栈
					&err										//返回的错误码
				);


	//创建烟雾管理任务
	OSTaskCreate(	&app_task_tcb_smoke,						//任务控制块
					"app_task_smoke",							//任务的名字
					app_task_smoke,							//任务函数
					0,											//传递参数
					7,											//任务的优先级7		
					app_task_stk_smoke,						//任务堆栈基地址
					512/10,										//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					512,										//任务堆栈大小			
					0,											//禁止任务消息队列
					0,											//默认是抢占式内核																
					0,											//不需要补充用户存储区
					OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,	//开启堆栈检测与清空任务堆栈
					&err										//返回的错误码
				);
				
				
	//创建RFID管理任务
	OSTaskCreate(	&app_task_tcb_rfid,						//任务控制块
					"app_task_rfid",							//任务的名字
					app_task_rfid,							//任务函数
					0,											//传递参数
					7,											//任务的优先级7		
					app_task_stk_rfid,						//任务堆栈基地址
					512/10,										//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					512,										//任务堆栈大小			
					0,											//禁止任务消息队列
					0,											//默认是抢占式内核																
					0,											//不需要补充用户存储区
					OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,	//开启堆栈检测与清空任务堆栈
					&err										//返回的错误码
				);
				
				
	//创建键盘管理任务
	OSTaskCreate(	&app_task_tcb_keyboard,						//任务控制块
					"app_task_keyboard",							//任务的名字
					app_task_keyboard,							//任务函数
					0,											//传递参数
					7,											//任务的优先级7		
					app_task_stk_keyboard,						//任务堆栈基地址
					512/10,										//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					512,										//任务堆栈大小			
					0,											//禁止任务消息队列
					0,											//默认是抢占式内核																
					0,											//不需要补充用户存储区
					OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,	//开启堆栈检测与清空任务堆栈
					&err										//返回的错误码
				);
				
				
	//创建发送蓝牙数据管理任务
	OSTaskCreate(	&app_task_tcb_send,						//任务控制块
					"app_task_send",						//任务的名字
					app_task_send,							//任务函数
					0,											//传递参数
					7,											//任务的优先级7		
					app_task_stk_send,						//任务堆栈基地址
					512/10,										//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					512,										//任务堆栈大小			
					0,											//禁止任务消息队列
					0,											//默认是抢占式内核																
					0,											//不需要补充用户存储区
					OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,	//开启堆栈检测与清空任务堆栈
					&err										//返回的错误码
				);
				
	//创建发送蓝牙数据管理任务
	OSTaskCreate(	&app_task_tcb_num,						//任务控制块
					"app_task_num",						//任务的名字
					app_task_num,							//任务函数
					0,											//传递参数
					7,											//任务的优先级7		
					app_task_stk_num,						//任务堆栈基地址
					512/10,										//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					512,										//任务堆栈大小			
					0,											//禁止任务消息队列
					0,											//默认是抢占式内核																
					0,											//不需要补充用户存储区
					OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,	//开启堆栈检测与清空任务堆栈
					&err										//返回的错误码
				);
						

	//独立看门狗初始化，超时时间为1秒
	//iwdg_init();
	
	//软件定时器
	//10，延迟10*10ms=100ms才真正运行
	//10,10*10ms=100ms的超时时间
	//OS_OPT_TMR_PERIODIC，软件定时器周期执行	
	OSTmrCreate(&g_tmr_wdt,"g_tmr_wdt",10,50,OS_OPT_TMR_PERIODIC,(OS_TMR_CALLBACK_PTR)timer_wdt_callback,NULL,&err); 
	
	
	//启动软件定时器
	OSTmrStart(&g_tmr_wdt,&err);


	//删除自身任务，进入休眠态
	OSTaskDel(NULL,&err);
}




void app_task_beep(void *parg)//蜂鸣器管理任务
{
	OS_ERR err;

	OS_MSG_SIZE msg_size;
	
	uint8_t *p=NULL;	
	uint8_t beep_sta=0;

	dgb_printf_safe("app_task_beep is create ok\r\n");
	
	while(1)
	{
		//等待消息队列
		p=OSQPend(&g_queue_beep,0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
		
		if(err != OS_ERR_NONE)
		{
			dgb_printf_safe("[app_task_beep][OSQPend]Error Code = %d\r\n",err);

			continue;
		}		
		
		//控制对应的蜂鸣器
		if(p && msg_size)
		{
			beep_sta = *p;
			
			if(beep_sta)
				BEEP(1);
			else
				BEEP(0);

		}	

		//释放信号量，告诉对方，当前蜂鸣器控制任务已经完成
		OSSemPost(&g_sem_beep,OS_OPT_POST_1,&err);	
		
		if(err != OS_ERR_NONE)
		{
			dgb_printf_safe("[app_task_beep][OSSemPost]Error Code = %d\r\n",err);
		}		
	}
}

void app_task_rtc(void *parg)//时钟管理任务
{
	char buf[16]={0};
	
	RTC_TimeTypeDef  	RTC_TimeStructure;

	
	OS_ERR err;
	
	OS_FLAGS flags=0;

	
	dgb_printf_safe("app_task_rtc is create ok\r\n");
	
	
#if 0	
	//显示字符串，保持两秒显示
	OLED_ShowString(0,0,"Teacher.Wen",16);
	delay_ms(2000);
	
	//行清空
	memset(buf,' ',16);
	
	OLED_ShowString(0,0,buf,16);
	
#endif
	
	while(1)
	{
		//一直阻塞等待事件标志组，等待成功后，将对应bit清0
		flags=OSFlagPend(&g_flag_grp,FLAG_GRP_RTC_WAKEUP,0,OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME+OS_OPT_PEND_BLOCKING,NULL,&err);	
			
		if(err != OS_ERR_NONE)
		{
			dgb_printf_safe("[app_task_rtc][OSFlagPend]Error Code = %d\r\n",err);
			
			continue;
		}
			
		
		//rtc唤醒中断
		if(flags & FLAG_GRP_RTC_WAKEUP)
		{
			//RTC_GetTime，获取时间
			RTC_GetTime(RTC_Format_BCD, &RTC_TimeStructure); 
				
			//格式化字符串
			sprintf(buf,"%02x:%02x:%02x",RTC_TimeStructure.RTC_Hours,RTC_TimeStructure.RTC_Minutes,RTC_TimeStructure.RTC_Seconds);
			
			//dgb_printf_safe("%s\r\n",buf);
			
			//阻塞等待互斥锁
			OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);			
			
			//oled显示时间
			OLED_ShowString(0,0,(uint8_t *)buf,16);
					
			//立即释放互斥锁
			OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);			
		}
	}
}


void app_task_dht11(void *parg)//温湿度管理任务
{
	OS_ERR err;
	
	int32_t rt; 	
	uint8_t dht11_data[5]={0};
	uint8_t buf[16] = {0};
	uint8_t oled_buf[32] = {0};
	
	int32_t d = 0;
	int32_t d2 = 0;	
	
	dgb_printf_safe("app_task_dht11 is create ok\r\n");

	while(1)
	{

		//获取温湿度
		rt = dht11_read(dht11_data);

		if(rt < 0)//获取失败
		{
//			dgb_printf_safe("dht11 read fail,error code = %d\r\n",rt);

//			delay_ms(1000);

			continue;
		}
		else//获取成功 
		{

			sprintf((char *)buf,"Tn:%02d.%d,Hn:%02d.%d",dht11_data[2],dht11_data[3],dht11_data[0],dht11_data[1]);
			
			//OSQPost(&g_queue_send,(char *)buf,strlen((char *)buf),OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);

			//阻塞等待互斥锁
			OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);	

			OLED_ShowString(0,2,buf,16);		

			//释放互斥锁
			OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
			
			d = *(volatile uint32_t *)(0x08040000+4*1);
			d2 = *(volatile uint32_t *)(0x08040000+4*2);
			
			sprintf((char *)oled_buf,"Tset:%d,Hset:%d",d,d2);
			//阻塞等待互斥锁
			OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);	

			OLED_ShowString(0,4,oled_buf,16);		

			//释放互斥锁
			OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
			
		}
		delay_ms(6000);
	}
}

void app_task_ir(void *parg)//红外线管理任务
{
	OS_ERR err;
	//OS_FLAGS flags=0;
	uint8_t *pmsg=NULL;
	uint8_t sta=0;
	
	OS_MSG_SIZE msg_size;
		
	
	uint8_t ir_data[4]={0};
	int32_t rt=0;
	
	uint8_t buf[16];	

	//CPU_SR_ALLOC();			//申请保护CPU的状态
	

	dgb_printf_safe("app_task_ir is create ok\r\n");

	while(1)
	{
		
		//等待消息队列
		pmsg=OSQPend(&g_queue_ir,0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
		
		if(err != OS_ERR_NONE)
		{
			dgb_printf_safe("[app_task_ir][OSQPend]Error Code = %d\r\n",err);
			
			continue;
		}			
	
		//禁止EXTI8触发中断
		NVIC_DisableIRQ(EXTI9_5_IRQn);

		
		memcpy(ir_data,pmsg,msg_size);
		
		
		memset(pmsg,0,msg_size);


		
		//允许EXTI8触发中断
		NVIC_EnableIRQ(EXTI9_5_IRQn);	
			
		//清空EXTI8中断标志位
		EXTI_ClearITPendingBit(EXTI_Line8);	
		
		if(rt < 0)//获取失败
		{
			dgb_printf_safe("ir read fail,error code = %d\r\n",rt);
			dgb_printf_safe("%02X %02X %02X %02X\r\n",ir_data[0],ir_data[1],ir_data[2],ir_data[3]);
			continue;
		}	
		else//获取成功
		{
			sprintf((char *)buf,"ir:%02X",ir_data[2]);	
			dgb_printf_safe("%s\r\n",buf);
			
			if(strstr((char *)buf,"43"))//关闭蜂鸣器
			{
				sta = 0;
				//控制蜂鸣器
				OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
				
				if(err != OS_ERR_NONE)
				{
					dgb_printf_safe("[sr04_beep][OSQPost]Error Code %d\r\n",err);
					
					continue;
				}
				
				
				OSFlagPost(&g_flag_grp,FLAG_GRP_BEEP_CLOSE,OS_OPT_POST_FLAG_SET,&err);
				
			}
			
//			if(strstr((char *)buf,"44"))//开启SR04
//			{
//				OSTaskResume(&app_task_tcb_sr04,&err);
//				OSFlagPost(&g_flag_grp,FLAG_GRP_SR04_WAKEUP,OS_OPT_POST_FLAG_SET,&err);
//				
//			}
//			
//			if(strstr((char *)buf,"40"))//关闭SR04
//			{
//				OSTaskSuspend(&app_task_tcb_sr04,&err);
//			}
			

		}			
		
		
		
		//delay_ms(1000);
		
	}
}

	


void  app_task_usart1(void *parg)  //串口1管理任务
{  
	OS_ERR err;  
 	
	uint8_t *pmsg=NULL;
	char *p=NULL;
	
	OS_MSG_SIZE msg_size;
	
	
	RTC_TimeTypeDef  	RTC_TimeStructure;

	uint32_t i=0;
	
	//申请CPU状态保存
	CPU_SR_ALLOC();	
	
	dgb_printf_safe("app_task_usart1 is create ok\r\n");
	
	while(1)
	{
		//等待消息队列
		pmsg=OSQPend(&g_queue_usart1,0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
		
		if(err != OS_ERR_NONE)
		{
			dgb_printf_safe("[app_task_usart1][OSQPend]Error Code = %d\r\n",err);
			
			continue;
		}		
		//dgb_printf_safe("pmsg = %s\r\n",pmsg);
		//正确接收到消息
		if(pmsg && msg_size)
		{
			//关闭串口1中断
			NVIC_DisableIRQ(USART1_IRQn);
			
			//设置时间
			if(strstr((char *)pmsg,"TIME SET"))
			{
				//以等号分割字符串
				strtok((char *)pmsg,"-");
				
				//获取时
				p=strtok(NULL,"-");
				i = atoi(p);
				
				
				//进入临界区，原因RTC获取时间用到相同的结构体和寄存器
				OS_CRITICAL_ENTER();
				
				
				//通过时，判断是AM还是PM
				if(i<12)
					RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
				else
					RTC_TimeStructure.RTC_H12     = RTC_H12_PM;
					
				//转换为BCD编码
				i= (i/10)*16+i%10;
				RTC_TimeStructure.RTC_Hours   = i;
				
				//获取分
				p=strtok(NULL,"-");
				i = atoi(p);	
				
				//转换为BCD编码
				i= (i/10)*16+i%10;	
				RTC_TimeStructure.RTC_Minutes = i;
				
				//获取秒
				p=strtok(NULL,"-");
				i = atoi(p);	
				
				//转换为BCD编码
				i= (i/10)*16+i%10;					
				RTC_TimeStructure.RTC_Seconds = i; 					
				
				//设置RTC时间
				RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);
				
				//退出临界区
				OS_CRITICAL_EXIT();
				
				dgb_printf_safe("rtc set time ok\r\n");				
			}			
			else
			{
				usart2_send_str((char *)pmsg);	
			
			}
			
			
			
			memset(p,0,msg_size);
			
			//使能串口1中断
			NVIC_EnableIRQ(USART1_IRQn);	
		}
	}
}



void  app_task_usart2(void *parg)//串口2管理任务
{
	
	
	
	
	
	OS_ERR err;
	
	//OS_MSG_SIZE msg_size;
	
	//申请CPU状态保存
	//CPU_SR_ALLOC();	
	
	
	
	dgb_printf_safe("app_task_usart2 is create ok\r\n");
	
//	while(1)
//	{
		//等待消息队列
//		pmsg=OSQPend(&g_queue_usart2,0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
//		
//		if(err != OS_ERR_NONE)
//		{
//			dgb_printf_safe("[app_task_usart2][OSQPend]Error Code = %d\r\n",err);
//			
//			continue;
//		}
		
		//关闭串口2中断
		//NVIC_DisableIRQ(USART2_IRQn);
		
		//正确接收到消息
//		if(pmsg && msg_size)
//		{
			memset((void *)gsm_buf,0,sizeof(gsm_buf));
			sprintf((char *)gsm_buf,"%s\r\n","AT+CMGF=1");
			usart2_send_str((char *)gsm_buf);
			
			
			delay_ms(500);
			delay_ms(500);
			delay_ms(500);
			delay_ms(500);
			
			memset((void *)gsm_buf,0,sizeof(gsm_buf));
			sprintf((char *)gsm_buf,"%s\r\n","AT+CSMP=17,167,2,25");
			usart2_send_str((char *)gsm_buf);
			
			delay_ms(500);
			delay_ms(500);
			delay_ms(500);
			delay_ms(500);
			
			memset((void *)gsm_buf,0,sizeof(gsm_buf));
			sprintf((char *)gsm_buf,"%s\r\n","AT+CSCS=\"UCS2\"");
			usart2_send_str((char *)gsm_buf);
			
			
			delay_ms(500);
			delay_ms(500);
			delay_ms(500);
			delay_ms(500);
			
//			memset((void *)gsm_buf,0,sizeof(gsm_buf));
//			sprintf(gsm_buf,"%s\r\n","AT+CMGS=\"00310033003000320035003800350032003100320032\"");//指定手机号
//			usart2_send_str(gsm_buf);
//			
//			delay_ms(500);
//			delay_ms(500);
//			delay_ms(500);
//			delay_ms(500);
//		}
		dgb_printf_safe("gsm set ok\r\n");
//	}
		OSTaskDel(NULL,&err);
}




void app_task_blueteeth(void *parg)//蓝牙管理任务
{
	
	uint8_t *pmsg=NULL;
	char *p=NULL;
	char data[10] = {0};
	
	uint32_t old_password = 0;
	uint32_t old_wendu = 0;
	uint32_t old_shidu = 0;
	uint32_t num = 0;
	int32_t rt; 	
	uint8_t dht11_data[5]={0};
	uint8_t buf[16] = {0};
	
	OS_MSG_SIZE msg_size=0;//保存接收到消息的大小
	OS_ERR err;
	
	dgb_printf_safe("app_task_tcb_blueteeth is create ok\r\n");
	
	delay_ms(1000);
	
	//配置蓝牙模块的名字
	usart3_send_str("AT+NAMELJQ\r\n");
	
	while(1)
	{
		//等待消息队列
		pmsg=OSQPend(&g_queue_blueteeth,0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
		
		if(err != OS_ERR_NONE)
		{
			dgb_printf_safe("[app_task_blueteeth][OSQPend]Error Code = %d\r\n",err);
			
			continue;
		}
		
		
		//正确接收到消息
		if(pmsg && msg_size)
		{
			
			//关闭串口3中断
			NVIC_DisableIRQ(USART3_IRQn);
			
			if(strncmp((char *)pmsg,"th",2) == 0)//查询温湿度
			{
				rt = dht11_read(dht11_data);

				if(rt < 0)//获取失败
				{
					dgb_printf_safe("dht11 read fail,error code = %d\r\n",rt);
					
				}
				else//获取成功 
				{
					sprintf((char *)buf,"Tn:%02d.%d,Hn:%02d.%d",dht11_data[2],dht11_data[3],dht11_data[0],dht11_data[1]);
					OSQPost(&g_queue_send,(char *)buf,strlen((char *)buf),OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					OSFlagPost(&g_flag_grp,FLAG_GRP_BLUE_SEND,OS_OPT_POST_FLAG_SET,&err);
				}

			
				
				dgb_printf_safe("查询温湿度\r\n");
				
				
			}
			
			
			
			
			if(strncmp((char *)pmsg,"t:",2) == 0)//设置温度上限
			{
				p = strtok((char *)pmsg,":");
				
				num = atoi(p+2);
				
				if(num >= 100 && num < 1000)
				{
					num = num /10;
				}
				else if(num >= 1000 && num < 10000)
				{
					num = num /100;
				}
				else if(num >= 10000)
				{
					num = num /1000;
				}
				
				//dgb_printf_safe("num = %d\r\n",num);
				
				//先读取扇区中原来的数据
				old_password = *(volatile uint32_t *)(0x08040000+0*4);
				//old_wendu = *(volatile uint32_t *)0x08040000+1*4;
				old_shidu = *(volatile uint32_t *)(0x08040000+2*4);
				
				//解除写保护
				FLASH_Unlock();
				
				//清除上一次出现的错误码
				FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
					 FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
				
				//扇区擦除,使用电压等级3（2.7V~3.6V的供电电压），擦除数据宽度为32位
				if(FLASH_COMPLETE!=FLASH_EraseSector(FLASH_Sector_6,VoltageRange_3))
				{
					dgb_printf_safe("FLASH_EraseSector fail\r\n");
					
					while(1);
				}
				
				//写入数据
				
				if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*0, old_password))
				{
					dgb_printf_safe("FLASH_ProgramWord old_password fail\r\n");					
					while(1);	
				
				}
				
				if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*1, num))
				{
					dgb_printf_safe("FLASH_ProgramWord num fail\r\n");					
					while(1);	
				
				}
				
				if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*2, old_shidu))
				{
					dgb_printf_safe("FLASH_ProgramWord old_shidu fail\r\n");					
					while(1);	
				
				}
				
				
				
				//加上写保护
				FLASH_Lock();
				
				dgb_printf_safe("修改温度成功\r\n");
			}
			
			if(strncmp((char *)pmsg,"h:",2) == 0)//设置湿度上限
			{
				
				p = strtok((char *)pmsg,":");
				num = atoi(p+2);
				
				if(num >= 100 && num < 1000)
				{
					num = num /10;
				}
				else if(num >= 1000 && num < 10000 )
				{
					num = num /100;
				}
				else if(num >= 10000)
				{
					num = num /1000;
				}
				//dgb_printf_safe("num = %d\r\n",num);
				
				//先读取扇区中原来的数据
				old_password = *(volatile uint32_t *)(0x08040000+0*4);
				old_wendu = *(volatile uint32_t *)(0x08040000+1*4);
				//old_shidu = *(volatile uint32_t *)0x08040000+2*4;
				
				//解除写保护
				FLASH_Unlock();
				
				//清除上一次出现的错误码
				FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
					 FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
				
				//扇区擦除,使用电压等级3（2.7V~3.6V的供电电压），擦除数据宽度为32位
				if(FLASH_COMPLETE!=FLASH_EraseSector(FLASH_Sector_6,VoltageRange_3))
				{
					dgb_printf_safe("FLASH_EraseSector2 fail\r\n");
					
					while(1);
				}
				
				//写入数据
				if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*0, old_password))
				{
					dgb_printf_safe("FLASH_ProgramWord old_password fail\r\n");					
					while(1);	
				
				}
				
				if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*1, old_wendu))
				{
					dgb_printf_safe("FLASH_ProgramWord num fail\r\n");					
					while(1);	
				
				}
				
				if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*2, num))
				{
					dgb_printf_safe("FLASH_ProgramWord num fail\r\n");					
					while(1);	
				
				}
				
				
				//加上写保护
				FLASH_Lock();
				dgb_printf_safe("修改湿度成功\r\n");
			}
			
			if(strncmp((char *)pmsg,"p:",2) == 0)
			{
				p = strtok((char *)pmsg,":");
				num = atoi(p+2) ;
				
				//先读取扇区中原来的数据
				//old_password = *(volatile uint32_t *)0x08040000+0*4;
				old_wendu = *(volatile uint32_t *)(0x08040000+1*4);
				old_shidu = *(volatile uint32_t *)(0x08040000+2*4);
				
				if(num >= 1000)
				{
					num = num /10;
				}
				
				//解除写保护
				FLASH_Unlock();
				
				//清除上一次出现的错误码
				FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
					 FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
				
				//扇区擦除,使用电压等级3（2.7V~3.6V的供电电压），擦除数据宽度为32位
				if(FLASH_COMPLETE!=FLASH_EraseSector(FLASH_Sector_6,VoltageRange_3))
				{
					dgb_printf_safe("FLASH_EraseSector0 fail\r\n");
					
					while(1);
				}
				
				//写入数据
				
				if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*0, num))
				{
					dgb_printf_safe("FLASH_ProgramWord password fail\r\n");					
					while(1);	
				
				}
				
				if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*1, old_wendu))
				{
					dgb_printf_safe("FLASH_ProgramWord old_wendu fail\r\n");					
					while(1);	
				
				}
				
				if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*2, old_shidu))
				{
					dgb_printf_safe("FLASH_ProgramWord old_shidu fail\r\n");					
					while(1);	
				
				}
				
				
				//加上写保护
				FLASH_Lock();
				dgb_printf_safe("修改密码成功\r\n");
			}
			
			
			
			//使能串口3中断
			NVIC_EnableIRQ(USART3_IRQn);
			
		}
			
	}

}

	
//void app_task_sr04(void *parg)//超声波管理任务
//{
//	int32_t distance=0;
//	OS_ERR err;
//	OS_FLAGS flags=0;
//	uint8_t sta=0;
//	
//	uint8_t buf[20] = {"find invasion"};
//	uint8_t buf2[20] = {"safety"};
//	CPU_STK_SIZE free,used;  
//	dgb_printf_safe("app_task_sr04 is create ok\r\n");
//	
//	while(1)
//	{
//		//一直阻塞等待事件标志组，等待成功后，将对应bit清0
//		flags=OSFlagPend(&g_flag_grp,FLAG_GRP_SR04_WAKEUP,0,OS_OPT_PEND_FLAG_SET_ANY +OS_OPT_PEND_BLOCKING,NULL,&err);//+OS_OPT_PEND_FLAG_CONSUME
//		
//		if(err != OS_ERR_NONE)
//		{
//			dgb_printf_safe("[app_task_sr04][OSFlagPend]Error Code = %d\r\n",err);
//			continue;
//		}
//		
//		if(flags & FLAG_GRP_SR04_WAKEUP)
//		{
//			//dgb_printf_safe("123\r\n");
//			distance = sr04_get_distance();
//			//dgb_printf_safe("456\r\n");
//			//delay_ms(500);
//			
//			//测距异常
//			if(distance<20 || distance>4000)
//			{
//				continue;
//			}
//			
//			dgb_printf_safe("distance = %dmm\r\n",distance);
//			
//			if(distance > 0 && distance < 100)
//			{
//				dgb_printf_safe("find people\r\n");
//				
//				sta = 1;
//				//控制蜂鸣器
//				OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
//				
//				if(err != OS_ERR_NONE)
//				{
//					dgb_printf_safe("[sr04_beep][OSQPost]Error Code %d\r\n",err);
//					
//					continue;
//				}
//				
//				//阻塞等待互斥锁
//				OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//				
//				OLED_Clear();
//				OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_near);
//				
//				//一直阻塞等待事件标志组，等待成功后，将对应bit清0
//				flags=OSFlagPend(&g_flag_grp,FLAG_GRP_BEEP_CLOSE,0,OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME+OS_OPT_PEND_BLOCKING,NULL,&err);
//				
//				if(err != OS_ERR_NONE)
//				{
//					dgb_printf_safe("[sr04_beep][OSFlagPend]Error Code = %d\r\n",err);
//					continue;
//				}
//				
//				if(flags & FLAG_GRP_BEEP_CLOSE)
//				{
//					OLED_Clear();
//					//释放互斥锁
//					OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
//				}
//				
//				
//				
//			}
//			else if(distance >= 1500)
//			{
//				//阻塞等待互斥锁
//				OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//				
//				//oled屏幕显示没人
//				OLED_ShowString(0,4,buf2,16);
//				
//				//释放互斥锁
//				OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
//			}
//		}
//		
//		dgb_printf_safe("sr04 is work\r\n");
//		delay_ms(1000);



//		OSTaskStkChk (&app_task_tcb_keyboard,&free,&used,&err); 
//		dgb_printf_safe("app_task_tcb_keyboard    stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free)); 
//	
//		OSTaskStkChk (&app_task_tcb_rfid,&free,&used,&err); 
//		dgb_printf_safe("app_task_tcb_rfid    stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free)); 
//	
//		
//		delay_ms(3000);

//	}
//	
//}


void  app_task_gsm(void *parg)//发送信息管理任务
{
	OS_ERR err;
	uint8_t *p=NULL;	
	//char buf[100] = {0};
	OS_MSG_SIZE msg_size;

	
	dgb_printf_safe("app_task_gsm is create ok\r\n");
		
	while(1)
	{
		//等待消息队列
		p=OSQPend(&g_queue_gsm,0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
		
		if(err != OS_ERR_NONE)
		{
			dgb_printf_safe("[app_task_gsm][OSQPend]Error Code = %d\r\n",err);

			continue;
		}
		
		if(p && msg_size)
		{
			
			
			if(strstr((char *)p,"fire"))
			{
				memset((void *)gsm_buf,0,sizeof(gsm_buf));
				sprintf((char *)gsm_buf,"%s\r\n","AT+CMGS=\"00310033003000320035003800350032003100320032\"");//指定手机号
				usart2_send_str((char *)gsm_buf);
				
				delay_ms(500);
				delay_ms(500);
				delay_ms(500);
				delay_ms(500);
				
				memset((void *)gsm_buf,0,sizeof(gsm_buf));
				sprintf((char *)gsm_buf,"5C4B518551FA73B0660E706B\r\n");//屋内出现明火 
				usart2_send_str((char *)gsm_buf);
				
				delay_ms(500);
				delay_ms(500);
				delay_ms(500);
				delay_ms(500);
				
				USART_SendData(USART2,0x1A);
			}
			
			if(strstr((char *)p,"smoke"))
			{
				memset((void *)gsm_buf,0,sizeof(gsm_buf));
				sprintf((char *)gsm_buf,"%s\r\n","AT+CMGS=\"00310033003000320035003800350032003100320032\"");//指定手机号
				usart2_send_str((char *)gsm_buf);
				
				delay_ms(500);
				delay_ms(500);
				delay_ms(500);
				delay_ms(500);
				
				memset((void *)gsm_buf,0,sizeof(gsm_buf));
				sprintf((char *)gsm_buf,"5C4B518551FA73B070DF96FE62168005670953EF71C36C144F536CC49732\r\n");//屋内出现烟雾或者有可燃气体泄露 
				usart2_send_str((char *)gsm_buf);
				
				//delay_ms(2000);
				delay_ms(500);
				delay_ms(500);
				delay_ms(500);
				delay_ms(500);
				
				USART_SendData(USART2,0x1A);
			}
			
			
			if(strstr((char *)p,"num"))
			{
				memset((void *)gsm_buf,0,sizeof(gsm_buf));
				sprintf((char *)gsm_buf,"%s\r\n","AT+CMGS=\"00310033003000320035003800350032003100320032\"");//指定手机号
				usart2_send_str((char *)gsm_buf);
				
				delay_ms(500);
				delay_ms(500);
				delay_ms(500);
				delay_ms(500);
				
				memset((void *)gsm_buf,0,sizeof(gsm_buf));
				sprintf((char *)gsm_buf,"95E879815BC678016076610F8F935165\r\n");//门禁密码恶意输入
				usart2_send_str((char *)gsm_buf);
				
				//delay_ms(2000);
				delay_ms(500);
				delay_ms(500);
				delay_ms(500);
				delay_ms(500);
				
				USART_SendData(USART2,0x1A);
			}
			
			if(strstr((char *)p,"unknowcard"))
			{
				memset((void *)gsm_buf,0,sizeof(gsm_buf));
				sprintf((char *)gsm_buf,"%s\r\n","AT+CMGS=\"00310033003000320035003800350032003100320032\"");//指定手机号
				usart2_send_str((char *)gsm_buf);
				
				delay_ms(500);
				delay_ms(500);
				delay_ms(500);
				delay_ms(500);
				
				
				memset((void *)gsm_buf,0,sizeof(gsm_buf));
				sprintf((char *)gsm_buf,"6709672A77E595E8798153615C1D8BD55F009501\r\n");//有未知门禁卡尝试开锁
				usart2_send_str((char *)gsm_buf);
				
				//delay_ms(2000);
				delay_ms(500);
				delay_ms(500);
				delay_ms(500);
				delay_ms(500);
				
				USART_SendData(USART2,0x1A);
			}
			
			
			
			
			
			//delay_ms(1000);
			
		}
	}

}


void app_task_fire(void *parg)//火光管理任务
{
	uint32_t adc_val=0;
	uint32_t adc_vol=0;
	char fire_buf[64]={0};
	OS_ERR err;
	OS_FLAGS flags=0;
	uint8_t sta=0;
	
	dgb_printf_safe("app_task_fire is create ok\r\n");
	
	//启动ADC1工作
	ADC_SoftwareStartConv(ADC1);
	
	while(1)
	{
		
		
		//等待转换完毕
		while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
		
		
		ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
		
		
		
		//获取转换结果值
		adc_val=ADC_GetConversionValue(ADC1);
		//dgb_printf_safe("adc_val=%d\r\n",adc_val);
		
		adc_vol = adc_val *5000 /4095;
		//dgb_printf_safe("fire adc_vol=%d\r\n",adc_vol);
		
		if(adc_vol >= 1500)
		{
			if(PCin(9) == 0)
			{
				sta = 1;
				memcpy(fire_buf,"find fire\r\n",13);
				
				//控制蜂鸣器
				OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);		
				if(err != OS_ERR_NONE)
				{
					dgb_printf_safe("[app_task_fire][OSQPost]Error Code %d\r\n",err);
					
					continue;
				}
				
				//阻塞等待互斥锁
				OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
				
				OLED_Clear();
				OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_fire);
				
				
				dgb_printf_safe("find fire\r\n");
				OSQPost(&g_queue_gsm,&fire_buf,sizeof(fire_buf),OS_OPT_POST_FIFO,&err);
				OSQPost(&g_queue_send,&fire_buf,sizeof(fire_buf),OS_OPT_POST_FIFO,&err);
				
				//一直阻塞等待事件标志组，等待成功后，将对应bit清0
				flags=OSFlagPend(&g_flag_grp,FLAG_GRP_BEEP_CLOSE,0,OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME+OS_OPT_PEND_BLOCKING,NULL,&err);
				
				if(err != OS_ERR_NONE)
				{
					dgb_printf_safe("[fire_beep][OSFlagPend]Error Code = %d\r\n",err);
					continue;
				}
				
				if(flags & FLAG_GRP_BEEP_CLOSE)
				{
					OLED_Clear();
					//释放互斥锁
					OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
					
				}
			}
			
			
		}
		
			
		//dgb_printf_safe("fire is work\r\n");
		delay_ms(3000);
		
	}
	
}


void app_task_smoke(void *parg)//烟雾管理任务
{
	uint32_t adc_val=0;
	uint32_t adc_vol=0;
	char smoke_buf[64]={0};
	OS_ERR err;
	OS_FLAGS flags=0;
	uint8_t sta=0;
	
	dgb_printf_safe("app_task_smoke is create ok\r\n");
	
	ADC_SoftwareStartConv(ADC1);
	
	while(1)
	{	
		
		//等待转换完毕
		while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
		ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
		
		
		//获取转换结果值
		adc_val=ADC_GetConversionValue(ADC1);
		//dgb_printf_safe("adc_val=%d\r\n",adc_val);
		
		adc_vol = adc_val *5000 /4095;
		//dgb_printf_safe("smoke adc_vol=%d\r\n",adc_vol);
		
	
		if(adc_vol >= 2500)
		{
			if(PCin(7) == 0)
			{
				sta = 1;
				//控制蜂鸣器
				OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
				
				if(err != OS_ERR_NONE)
				{
					dgb_printf_safe("[app_task_fire][OSQPost]Error Code %d\r\n",err);
					
					continue;
				}
				
				//阻塞等待互斥锁
				OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
				
				OLED_Clear();
				OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_smoke);
				
				memcpy(smoke_buf,"find smoke\r\n",14);
				dgb_printf_safe("find smoke\r\n");
				OSQPost(&g_queue_gsm,&smoke_buf,sizeof(smoke_buf),OS_OPT_POST_FIFO,&err);
				OSQPost(&g_queue_send,&smoke_buf,sizeof(smoke_buf),OS_OPT_POST_FIFO,&err);
				
				//一直阻塞等待事件标志组，等待成功后，将对应bit清0
				flags=OSFlagPend(&g_flag_grp,FLAG_GRP_BEEP_CLOSE,0,OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME+OS_OPT_PEND_BLOCKING,NULL,&err);
				
				if(err != OS_ERR_NONE)
				{
					dgb_printf_safe("[smoke_beep][OSFlagPend]Error Code = %d\r\n",err);
					continue;
				}
				
				if(flags & FLAG_GRP_BEEP_CLOSE)
				{
					OLED_Clear();
					//释放互斥锁
					OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
				}
			}
			
		}

			
		//dgb_printf_safe("smoke is work\r\n");
		delay_ms(3000);

	}	
		
}


void app_task_rfid(void *parg)//RFID管理任务
{
	
	OS_MSG_SIZE msg_size;
	
	uint8_t *p=NULL;
	uint32_t i = 0;
	uint32_t k = 0;
	//uint32_t z = 0;
	uint8_t sta=0;
	
	int16_t d = 0;
	uint8_t  card_buf[30] = {0};
	
	OS_ERR err;
	
	dgb_printf_safe("app_task_rfid is create ok\r\n");
	
	while(1)
	{
		MFRC522_Initializtion();
		d = MFRC522Test();
		if(d == -1)
		{
			delay_ms(1000);
			continue;
		}
		OSTaskSuspend(&app_task_tcb_keyboard,&err);
		while(1)
		{
			
			
			
			//等待消息队列
			p=OSQPend(&g_queue_rfid,0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
			
			if(err != OS_ERR_NONE)
			{
				dgb_printf_safe("[app_task_rfid][OSQPend]Error Code = %d\r\n",err);

				continue;
			}
			
			break;
		}

		for(i = 0;*((volatile uint32_t *)(0x08020000+(i*4))) != 0xFFFFFFFF;i++)
		{
			if(*((volatile uint32_t *)(0x08020000+(i*4))) != *p)
			{
				
				continue;
			}
			k++;
			break;
		}
		
		if(k == 0)
		{
			sta = 1;
			//控制蜂鸣器
			OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
			
			if(err != OS_ERR_NONE)
			{
				dgb_printf_safe("[app_task_rfid2][OSQPost]Error Code %d\r\n",err);
				
				continue;
			}
			
			delay_ms(200);
			
			sta = 0;
			//控制蜂鸣器
			OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
			
			if(err != OS_ERR_NONE)
			{
				dgb_printf_safe("[app_task_rfid3][OSQPost]Error Code %d\r\n",err);
				
				continue;
			}
			
			delay_ms(100);
			
			sta = 1;
			//控制蜂鸣器
			OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
			
			if(err != OS_ERR_NONE)
			{
				dgb_printf_safe("[app_task_rfid4][OSQPost]Error Code %d\r\n",err);
				
				continue;
			}
			
			delay_ms(500);
			
			sta = 0;
			//控制蜂鸣器
			OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
			
			if(err != OS_ERR_NONE)
			{
				dgb_printf_safe("[app_task_rfid5][OSQPost]Error Code %d\r\n",err);
				
				continue;
			}
			
			delay_ms(100);
			
			sta = 1;
			//控制蜂鸣器
			OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
			
			if(err != OS_ERR_NONE)
			{
				dgb_printf_safe("[app_task_rfid6][OSQPost]Error Code %d\r\n",err);
				
				continue;
			}
			
			delay_ms(500);
			
			sta = 0;
			//控制蜂鸣器
			OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
			
			if(err != OS_ERR_NONE)
			{
				dgb_printf_safe("[app_task_rfid7][OSQPost]Error Code %d\r\n",err);
				
				continue;
			}
			
			dgb_printf_safe("卡号不存在1\r\n");
			memset(card_buf,0,sizeof(card_buf));
			
			k = 0;
			
			if(er2 == 0)
			{
				OSFlagPost(&g_flag_grp,FLAG_GRP_NUM_WAKEUP,OS_OPT_POST_FLAG_SET,&err);
			}
			er2++;
			if(er2 >= 3)
			{
				OSQPost(&g_queue_gsm,"unknowcard",10,OS_OPT_POST_FIFO,&err);
				dgb_printf_safe("恶意试卡\r\n");
			}
			
			//阻塞等待互斥锁
			OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			
			OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_fail);
			delay_ms(1000);
			OLED_Clear();
			
			//释放互斥锁
			OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
		}
		
		if(k > 0)
		{
			sta = 1;
			//控制蜂鸣器
			OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
			
			if(err != OS_ERR_NONE)
			{
				dgb_printf_safe("[app_task_rfid8][OSQPost]Error Code %d\r\n",err);
				
				continue;
			}
			
			delay_ms(300);
			
			sta = 0;
			//控制蜂鸣器
			OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
			
			if(err != OS_ERR_NONE)
			{
				dgb_printf_safe("[app_task_rfid9][OSQPost]Error Code %d\r\n",err);
				
				continue;
			}
			
			dgb_printf_safe("进入成功\r\n");
			k = 0;
			
			memset(card_buf,0,sizeof(card_buf));
			
			//阻塞等待互斥锁
			OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			
			OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_success);
			delay_ms(1000);
			OLED_Clear();
			
			//释放互斥锁
			OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
		}
		

		
		OSTaskResume(&app_task_tcb_keyboard,&err);
		
			
	}

}

void app_task_keyboard(void *parg)//矩阵键盘管理任务
{
	OS_MSG_SIZE msg_size;
	
	uint8_t *p=NULL;
	OS_ERR err;
	
	uint32_t i = 0;
	uint32_t k = 0;
	uint32_t x = 0;
	int num = 0;
	int32_t d = 0;
	
	uint8_t sta=0;
	
	uint32_t old_password = 0;
	uint32_t old_wendu = 0;
	uint32_t old_shidu = 0;
	uint8_t  card_buf[50] = {0};
	
	uint32_t old_card[50] = {0};
	char key_value[10] = {0};
	char buf[20] = {0};
	
	char *data = NULL;
	
	dgb_printf_safe("app_task_keyboard is create ok\r\n");
	
	while(1)
	{
		key_state();
		
		//等待消息队列
		p=OSQPend(&g_queue_keyboard,0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
		
		if(err != OS_ERR_NONE)
		{
			dgb_printf_safe("[app_task_keyboard][OSQPend]Error Code = %d\r\n",err);
			continue;
		}
		
		if(p && msg_size)
		{
			sta = 1;
			//控制蜂鸣器
			OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
			
			if(err != OS_ERR_NONE)
			{
				dgb_printf_safe("[app_task_keyboard2][OSQPost]Error Code %d\r\n",err);
				
				continue;
			}
			
			delay_ms(80);
			
			sta = 0;
			//控制蜂鸣器
			OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
			
			if(err != OS_ERR_NONE)
			{
				dgb_printf_safe("[app_task_keyboard3][OSQPost]Error Code %d\r\n",err);
				
				continue;
			}

//			BEEP(1);
//			delay_ms(80);
//			BEEP(0);
			
			if(strcmp((char *)p,"*") == 0)
			{
				memset(key_value,0,sizeof(key_value));
				memset(old_card,0,sizeof(old_card));
				memset(buf,0,sizeof(buf));
				dgb_printf_safe("clean\r\n");
			}
			dgb_printf_safe("key_value = %s\r\n",p);
			if(strcmp((char *)p,"#") != 0)//用于辨别确认输入完成
			{
				strcat(key_value,(char *)p);
				continue;
			}
			
			//strcat(key_value,(char *)p);
			
			if(strncmp(key_value,"A",1) == 0)//改密码操作,3位密码
			{
				memset(key_value,0,sizeof(key_value));
				dgb_printf_safe("请输入旧密码\r\n");
				
				while(1)
				{
					key_state();
			
					//等待消息队列
					p=OSQPend(&g_queue_keyboard,0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
					
					if(err != OS_ERR_NONE)
					{
						dgb_printf_safe("[app_task_keyboard][OSQPend]Error Code = %d\r\n",err);
						continue;
					}
					
					
					sta = 1;
					//控制蜂鸣器
					OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
					
					if(err != OS_ERR_NONE)
					{
						dgb_printf_safe("[app_task_keyboard2][OSQPost]Error Code %d\r\n",err);
						
						continue;
					}
					
					delay_ms(80);
					
					sta = 0;
					//控制蜂鸣器
					OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
					
					if(err != OS_ERR_NONE)
					{
						dgb_printf_safe("[app_task_keyboard3][OSQPost]Error Code %d\r\n",err);
						
						continue;
					}
					
					
					if(strcmp((char *)p,"*") == 0)
					{
						memset(key_value,0,sizeof(key_value));
						memset(old_card,0,sizeof(old_card));
						memset(buf,0,sizeof(buf));
						dgb_printf_safe("clean\r\n");
					}
					
					if(strcmp((char *)p,"#") != 0)//用于辨别确认输入完成
					{
						strcat(key_value,(char *)p);
						continue;
					}
					else
					{
						break;
					}
					
					
				}
				
				//data = strtok(key_value,"A");
				
				num = *(volatile uint32_t *)(0x08040000+0*4);
				sprintf(buf,"%d",num);
				
				if(strcmp(key_value,buf) == 0)
				{
					BEEP(1);
					delay_ms(300);
					BEEP(0);
					
					dgb_printf_safe("密码正确\r\n");
					memset(key_value,0,sizeof(key_value));
					memset(old_card,0,sizeof(old_card));
					memset(buf,0,sizeof(buf));
					
					dgb_printf_safe("请输入新密码\r\n");
					
					while(1)
					{
						key_state();
				
						//等待消息队列
						p=OSQPend(&g_queue_keyboard,0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
						
						if(err != OS_ERR_NONE)
						{
							dgb_printf_safe("[app_task_keyboard][OSQPend]Error Code = %d\r\n",err);
							continue;
						}
						
						
						sta = 1;
						//控制蜂鸣器
						OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
						
						if(err != OS_ERR_NONE)
						{
							dgb_printf_safe("[app_task_keyboard2][OSQPost]Error Code %d\r\n",err);
							
							continue;
						}
						
						delay_ms(80);
						
						sta = 0;
						//控制蜂鸣器
						OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
						
						if(err != OS_ERR_NONE)
						{
							dgb_printf_safe("[app_task_keyboard3][OSQPost]Error Code %d\r\n",err);
							
							continue;
						}
						
						
						
						if(strcmp((char *)p,"*") == 0)
						{
							memset(key_value,0,sizeof(key_value));
							memset(old_card,0,sizeof(old_card));
							memset(buf,0,sizeof(buf));
							dgb_printf_safe("clean\r\n");
						}
						
						if(strcmp((char *)p,"#") != 0)//用于辨别确认输入完成
						{
							strcat(key_value,(char *)p);
							continue;
						}
						
						
						dgb_printf_safe("key value2 = %s\r\n",key_value);
						//data = strtok(key_value,"A");
						num = atoi(key_value);
						
						//先读取扇区中原来的数据
						//old_password = *(volatile uint32_t *)0x08040000+0*4;
						old_wendu = *(volatile uint32_t *)(0x08040000+1*4);
						old_shidu = *(volatile uint32_t *)(0x08040000+2*4);
						
						
						
						//解除写保护
						FLASH_Unlock();
						
						//清除上一次出现的错误码
						FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
							 FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
						
						//扇区擦除,使用电压等级3（2.7V~3.6V的供电电压），擦除数据宽度为32位
						if(FLASH_COMPLETE!=FLASH_EraseSector(FLASH_Sector_6,VoltageRange_3))
						{
							dgb_printf_safe("FLASH_EraseSector0 fail\r\n");
							
							while(1);
						}
						
						//写入数据
						
						if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*0, num))
						{
							dgb_printf_safe("FLASH_ProgramWord password fail\r\n");					
							while(1);	
						
						}
						
						if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*1, old_wendu))
						{
							dgb_printf_safe("FLASH_ProgramWord old_wendu fail\r\n");					
							while(1);	
						
						}
						
						if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*2, old_shidu))
						{
							dgb_printf_safe("FLASH_ProgramWord old_shidu fail\r\n");					
							while(1);	
						
						}
						
						
						//加上写保护
						FLASH_Lock();
						memset(key_value,0,sizeof(key_value));
						memset(old_card,0,sizeof(old_card));
						memset(buf,0,sizeof(buf));
						dgb_printf_safe("修改成功\r\n");
						//阻塞等待互斥锁
						OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
						
						OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_success);
						delay_ms(1000);
						OLED_Clear();
						
						//释放互斥锁
						OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
						break;
					}
					
					
				}
				else
				{
					BEEP(1);
					delay_ms(200);
					BEEP(0);
					
					delay_ms(100);
					
					BEEP(1);
					delay_ms(500);
					BEEP(0);
					
					delay_ms(100);
					
					BEEP(1);
					delay_ms(500);
					BEEP(0);
					
					dgb_printf_safe("旧密码错误\r\n");
					memset(key_value,0,sizeof(key_value));
					memset(old_card,0,sizeof(old_card));
					memset(buf,0,sizeof(buf));
					
					//阻塞等待互斥锁
					OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
					
					OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_fail);
					delay_ms(1000);
					OLED_Clear();
					
					//释放互斥锁
					OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
				}
				
				
				
				
				
				
			}
			
			else if(strncmp(key_value,"B",1) == 0)//设置温度上限操作,2位数
			{
				//dgb_printf_safe("请输入温度上限\r\n");
				dgb_printf_safe("key value = %s\r\n",key_value);
				data = strtok(key_value,"B");
				dgb_printf_safe("data = %s\r\n",data);
				num = atoi(data);
				dgb_printf_safe("num = %d\r\n",num);
				
				//先读取扇区中原来的数据
				old_password = *(volatile uint32_t *)(0x08040000+0*4);
				//old_wendu = *(volatile uint32_t *)0x08040000+1*4;
				old_shidu = *(volatile uint32_t *)(0x08040000+2*4);
				
				dgb_printf_safe("old_shidu = %d\r\n",old_shidu);
				
				//解除写保护
				FLASH_Unlock();
				
				//清除上一次出现的错误码
				FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
					 FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
				
				//扇区擦除,使用电压等级3（2.7V~3.6V的供电电压），擦除数据宽度为32位
				if(FLASH_COMPLETE!=FLASH_EraseSector(FLASH_Sector_6,VoltageRange_3))
				{
					dgb_printf_safe("FLASH_EraseSector fail\r\n");
					
					while(1);
				}
				
				//写入数据
				
				if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*0, old_password))
				{
					dgb_printf_safe("FLASH_ProgramWord old_password fail\r\n");					
					while(1);	
				
				}
				
				if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*1, num))
				{
					dgb_printf_safe("FLASH_ProgramWord num fail\r\n");					
					while(1);	
				
				}
				
				if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*2, old_shidu))
				{
					dgb_printf_safe("FLASH_ProgramWord old_shidu fail\r\n");					
					while(1);	
				
				}
				
				
				
				//加上写保护
				FLASH_Lock();
				memset(key_value,0,sizeof(key_value));
				memset(old_card,0,sizeof(old_card));
				memset(buf,0,sizeof(buf));
				dgb_printf_safe("修改成功\r\n");
				d = *(volatile uint32_t *)(0x08040000+4*1);
				dgb_printf_safe("wendu = %d\r\n",d);
				
//				//阻塞等待互斥锁
//				OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//				
//				OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_success);
//				delay_ms(1000);
//				OLED_Clear();
//				
//				//释放互斥锁
//				OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
			}
			
			else if(strncmp(key_value,"C",1) == 0)//设置湿度上限操作,2位数
			{
				dgb_printf_safe("key value = %s\r\n",key_value);
				data = strtok(key_value,"C");
				num = atoi(data);
				
				//先读取扇区中原来的数据
				old_password = *(volatile uint32_t *)(0x08040000+0*4);
				old_wendu = *(volatile uint32_t *)(0x08040000+1*4);
				//old_shidu = *(volatile uint32_t *)0x08040000+2*4;
				
				dgb_printf_safe("old_wendu = %d\r\n",old_wendu);
				
				//解除写保护
				FLASH_Unlock();
				
				//清除上一次出现的错误码
				FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
					 FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
				
				//扇区擦除,使用电压等级3（2.7V~3.6V的供电电压），擦除数据宽度为32位
				if(FLASH_COMPLETE!=FLASH_EraseSector(FLASH_Sector_6,VoltageRange_3))
				{
					dgb_printf_safe("FLASH_EraseSector2 fail\r\n");
					
					while(1);
				}
				
				//写入数据
				if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*0, old_password))
				{
					dgb_printf_safe("FLASH_ProgramWord old_password fail\r\n");					
					while(1);	
				
				}
				
				if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*1, old_wendu))
				{
					dgb_printf_safe("FLASH_ProgramWord num fail\r\n");					
					while(1);	
				
				}
				
				if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08040000+4*2, num))
				{
					dgb_printf_safe("FLASH_ProgramWord num fail\r\n");					
					while(1);	
				
				}
				
				
				//加上写保护
				FLASH_Lock();
				memset(key_value,0,sizeof(key_value));
				memset(old_card,0,sizeof(old_card));
				memset(buf,0,sizeof(buf));
				dgb_printf_safe("修改成功\r\n");
				d = *(volatile uint32_t *)(0x08040000+4*2);
				dgb_printf_safe("shidu = %d\r\n",d);
				
//				//阻塞等待互斥锁
//				OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//				
//				OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_success);
//				delay_ms(1000);
//				OLED_Clear();
//				
//				//释放互斥锁
//				OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
			}
			
			else if(strncmp(key_value,"D",1) == 0)//录入卡ID号操作,5位数
			{
				OSTaskSuspend(&app_task_tcb_rfid,&err);
				
				dgb_printf_safe("请将卡靠近感应区域\r\n");
				
				while(1)
				{
					MFRC522_Initializtion();
					d = MFRC522Test();
					if(d == -1)
					{
						delay_ms(1000);
						continue;
					}
					
					//等待消息队列
					p=OSQPend(&g_queue_rfid,0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
					
					if(err != OS_ERR_NONE)
					{
						dgb_printf_safe("[app_task_rfid][OSQPend]Error Code = %d\r\n",err);

						continue;
					}
					
//					if(strcmp((char *)p,"over") != 0)
//					{
//						card_buf[m] = *p;
//						m++;
//						continue;
//					}
					break;
					
					
				}
				
				
				if(p && msg_size)
				{
//					card_buf[0] = atoi(strtok((char *)p,"+"));
//					card_buf[1] = atoi(strtok(NULL,"+"));
//					card_buf[2] = atoi(strtok(NULL,"+"));
//					card_buf[3] = atoi(strtok(NULL,"+"));
//					card_buf[4] = atoi(strtok(NULL,"+"));
					
					//dgb_printf_safe("p = %d\r\n",*p);
					
					for(i = 0;*(volatile uint32_t *)(0x08020000+i*4) != 0xFFFFFFFF;i++) 
					{
						
					}//循环出来后加上i的地址就是空白无数据的地址
					
					
					//解除写保护
					FLASH_Unlock();
					
					//清除上一次出现的错误码
					FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
						 FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
					
					//由于不是覆盖原来的数据，所以不需要擦除
					
					//写入数据
//					for(z = 0;z < 5;z++)
//					{
//						if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08020000+(4*z+i*20), card_buf[z]))
//						{
//							dgb_printf_safe("FLASH_ProgramWord num fail\r\n");					
//							while(1);	
//						
//						}
//						
//					}
					
					if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08020000+(i*4), *p))
					{
						dgb_printf_safe("FLASH_ProgramWord num fail\r\n");					
						while(1);	
					
					}
					
					//加上写保护
					FLASH_Lock();
					
				}
				
				//dgb_printf_safe("buf = %d\r\n",*(volatile uint32_t *)(0x08020000+i*4));
				
				memset(key_value,0,sizeof(key_value));
				memset(old_card,0,sizeof(old_card));
				memset(buf,0,sizeof(buf));
				memset(card_buf,0,sizeof(card_buf));
				
				
				
				
				//阻塞等待互斥锁
				OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
				
				OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_success);
				delay_ms(1000);
				OLED_Clear();
				
				//释放互斥锁
				OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
				OSTaskResume(&app_task_tcb_rfid,&err);
				delay_ms(2000);
				dgb_printf_safe("成功\r\n");
			}
			
			else if(strncmp(key_value,"000",3) == 0)//进入删除卡模式
			{
				OSTaskSuspend(&app_task_tcb_rfid,&err);
				
				dgb_printf_safe("请将要删除的卡靠近感应区域\r\n");
				
				while(1)
				{
					MFRC522_Initializtion();
					d = MFRC522Test();
					if(d == -1)
					{
						delay_ms(1000);
						continue;
					}
					
					//等待消息队列
					p=OSQPend(&g_queue_rfid,0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
					
					if(err != OS_ERR_NONE)
					{
						dgb_printf_safe("[app_task_rfid][OSQPend]Error Code = %d\r\n",err);

						continue;
					}
					
//					if(strcmp((char *)p,"over") != 0)
//					{
//						card_buf[m] = *p;
//						m++;
//						continue;
//					}
					break;
				}
				
				
				if(p && msg_size)
				{
//					card_buf[0] = atoi(strtok((char *)p,"+"));
//					card_buf[1] = atoi(strtok(NULL,"+"));
//					card_buf[2] = atoi(strtok(NULL,"+"));
//					card_buf[3] = atoi(strtok(NULL,"+"));
//					card_buf[4] = atoi(strtok(NULL,"+"));
					
//					for(i = 0;*(volatile uint32_t *)(0x08020000+i*20) != 0xFFFFFFFF;i++)
//					{
//						for(z = 0;z < 5;z++)
//						{
//							
//							if(*(volatile uint32_t *)(0x08020000+(i*20+z*4)) == card_buf[z])
//							{
//								k++;
//								continue;
//							}
//							else
//							{
//								break;
//							}
//						}
//						
//					}
					
					for(i = 0;*((volatile uint32_t *)(0x08020000+(i*4))) != 0xFFFFFFFF;i++)
					{
						if(*((volatile uint32_t *)(0x08020000+(i*4))) != *p)
						{
							
							continue;
						}
						k++;
						break;
					}
					
					if(k == 0)
					{
						dgb_printf_safe("卡号不存在2\r\n");
						//阻塞等待互斥锁
						OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
						
						OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_fail);
						delay_ms(1000);
						OLED_Clear();
						
						//释放互斥锁
						OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
					}
					
					if(k > 0)
					{
						//读取旧数据，不读取将要删除的卡
						for(i = 0;*(volatile uint32_t *)(0x08020000+i*4) != 0xFFFFFFFF;i++)
						{
//							for(z = 0;z < 5;z++)
//							{
//								//dgb_printf_safe("z = %d,i = %d\r\n",z,i);
//								if(*(volatile uint32_t *)(0x08020000+(i*20+4*z)) != card_buf[z])
//								{
//									old_card[x] = *(volatile uint32_t *)(0x08020000+(i*20+4*z));
//									x++;
//								}
//								
//							}
							if(*(volatile uint32_t *)(0x08020000+i*4) != *p)
							{
								old_card[x] = *(volatile uint32_t *)(0x08020000+i*4);
								x++;
							}
							
						}
						
						
						//解除写保护
						FLASH_Unlock();
						
						//清除上一次出现的错误码
						FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
							 FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
						
						
						//扇区擦除,使用电压等级3（2.7V~3.6V的供电电压），擦除数据宽度为32位
						if(FLASH_COMPLETE!=FLASH_EraseSector(FLASH_Sector_5,VoltageRange_3))
						{
							dgb_printf_safe("FLASH_EraseSector5 fail\r\n");
							
							while(1);
						}
						
						
						//写入数据
						for(i = 0;i < x;i++)
						{
							if(FLASH_COMPLETE!=FLASH_ProgramWord(0x08020000+4*i, old_card[i]))
							{
								dgb_printf_safe("FLASH_ProgramWord num fail\r\n");					
								while(1);	
							
							}
						}
						
						
						
						//加上写保护
						FLASH_Lock();
						
						
						
						//阻塞等待互斥锁
						OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
						
						OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_success);
						delay_ms(2000);
						OLED_Clear();
						
						//释放互斥锁
						OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
						dgb_printf_safe("删除成功\r\n");
					}
					memset(key_value,0,sizeof(key_value));
					memset(old_card,0,sizeof(old_card));
					memset(buf,0,sizeof(buf));
					k = 0;
					
					x = 0;
					delay_ms(2000);
					OSTaskResume(&app_task_tcb_rfid,&err);
					
				}
			}
			else
			{
				num = *(volatile uint32_t *)(0x08040000+0*4);
				sprintf(buf,"%d",num);
				
				dgb_printf_safe("key_value5 = %s\r\n",key_value);
				
				if(strcmp(key_value,buf) == 0)
				{
					sta = 1;
					//控制蜂鸣器
					OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
					
					if(err != OS_ERR_NONE)
					{
						dgb_printf_safe("[app_task_keyboard2][OSQPost]Error Code %d\r\n",err);
						
						continue;
					}
					
					delay_ms(300);
					
					sta = 0;
					//控制蜂鸣器
					OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
					
					if(err != OS_ERR_NONE)
					{
						dgb_printf_safe("[app_task_keyboard3][OSQPost]Error Code %d\r\n",err);
						
						continue;
					}
					
					
					
					dgb_printf_safe("密码正确\r\n");
					memset(key_value,0,sizeof(key_value));
					memset(old_card,0,sizeof(old_card));
					memset(buf,0,sizeof(buf));
					
					//阻塞等待互斥锁
					OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
					
					OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_success);
					delay_ms(1000);
					OLED_Clear();
					
					//释放互斥锁
					OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
				}
				else
				{
					sta = 1;
					//控制蜂鸣器
					OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
					
					if(err != OS_ERR_NONE)
					{
						dgb_printf_safe("[app_task_keyboard2][OSQPost]Error Code %d\r\n",err);
						
						continue;
					}
					
					delay_ms(200);
					
					sta = 0;
					//控制蜂鸣器
					OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
					
					if(err != OS_ERR_NONE)
					{
						dgb_printf_safe("[app_task_keyboard3][OSQPost]Error Code %d\r\n",err);
						
						continue;
					}
					
					delay_ms(100);
					
					sta = 1;
					//控制蜂鸣器
					OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
					
					if(err != OS_ERR_NONE)
					{
						dgb_printf_safe("[app_task_keyboard2][OSQPost]Error Code %d\r\n",err);
						
						continue;
					}
					
					delay_ms(500);
					
					sta = 0;
					//控制蜂鸣器
					OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
					
					if(err != OS_ERR_NONE)
					{
						dgb_printf_safe("[app_task_keyboard3][OSQPost]Error Code %d\r\n",err);
						
						continue;
					}
					
					delay_ms(100);
					
					sta = 1;
					//控制蜂鸣器
					OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
					
					if(err != OS_ERR_NONE)
					{
						dgb_printf_safe("[app_task_keyboard2][OSQPost]Error Code %d\r\n",err);
						
						continue;
					}
					
					delay_ms(500);
					
					sta = 0;
					//控制蜂鸣器
					OSQPost(&g_queue_beep,&sta,1,OS_OPT_POST_FIFO,&err);
					
					if(err != OS_ERR_NONE)
					{
						dgb_printf_safe("[app_task_keyboard3][OSQPost]Error Code %d\r\n",err);
						
						continue;
					}
					
					dgb_printf_safe("密码错误\r\n");
					memset(key_value,0,sizeof(key_value));
					memset(old_card,0,sizeof(old_card));
					memset(buf,0,sizeof(buf));
					
					if(er == 0)
					{
						OSFlagPost(&g_flag_grp,FLAG_GRP_NUM_WAKEUP,OS_OPT_POST_FLAG_SET,&err);
					}
					
					
					
					//阻塞等待互斥锁
					OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
					
					OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_fail);
					delay_ms(1000);
					OLED_Clear();
					
					//释放互斥锁
					OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
					
					er++;
					if(er >= 3)
					{
						dgb_printf_safe("恶意输入\r\n");
						OSQPost(&g_queue_gsm,"num",3,OS_OPT_POST_FIFO,&err);
					}
				}
			}
			
			
		}
	}

}


void app_task_send(void *parg)//发送数据给蓝牙
{
	OS_ERR err;
	OS_MSG_SIZE msg_size;
	
	
	uint8_t *pmsg=NULL;
	uint8_t data[32]={0};
	
	dgb_printf_safe("app_task_send is create ok\r\n");

	while(1)
	{
		
			
			
		//等待消息队列
		pmsg=OSQPend(&g_queue_send,0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
		sprintf((char *)data,"%s",pmsg);
		if(err != OS_ERR_NONE)
		{
			dgb_printf_safe("[app_task_send][OSQPend]Error Code = %d\r\n",err);
			
			continue;
		}
		
		if(pmsg && msg_size)
		{
			usart3_send_str((char *)data);
			
		}
			
		
	}



}



void app_task_num(void *parg)//计算错误输入次数
{
	OS_ERR err;
	OS_FLAGS flags=0;
	
	dgb_printf_safe("app_task_num is create ok\r\n");
	
	while(1)
	{
		//一直阻塞等待事件标志组，等待成功后，将对应bit清0
		flags=OSFlagPend(&g_flag_grp,FLAG_GRP_NUM_WAKEUP,0,OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME+OS_OPT_PEND_BLOCKING,NULL,&err);
		
		if(err != OS_ERR_NONE)
		{
			dgb_printf_safe("[app_task_num][OSFlagPend]Error Code = %d\r\n",err);
			continue;
		}
		
		if(flags & FLAG_GRP_NUM_WAKEUP)
		{
			if(er == 1)
			{
				delay_ms(20000);
				er = 0;
			}
			
			if(er2 == 1)
			{
				delay_ms(20000);
				er2 = 0;
			}
		}
		
		
	}
	
}


void  timer_wdt_callback(OS_TMR *p_tmr, void *p_arg)
{
	//申请CPU状态保存
	CPU_SR_ALLOC();	
	
	//进入临界区，关中断
	CPU_CRITICAL_ENTER();
	
	//喂狗，刷新自身计数值
	IWDG_ReloadCounter();
	
	//dgb_printf_safe("iwdg feed\r\n");
	
	//退出临界区，开中断
	CPU_CRITICAL_EXIT();
}


