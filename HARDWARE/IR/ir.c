#include "sys.h"
#include "delay.h"
#include "ir.h"
#include "includes.h"

static GPIO_InitTypeDef	GPIO_InitStructure;
static EXTI_InitTypeDef	EXTI_InitStructure;
static NVIC_InitTypeDef	NVIC_InitStructure;

static volatile uint8_t 	g_ir_buf[5]={0};


void ir_init(void)
{
	//使能端口A的硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	

	//使能系统配置硬件时钟，说白了就是对系统配置的硬件供电
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);	
	
	
	//配置PA8为输入模式
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;			//第8个引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;		//输入模式
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;	//引脚高速工作，收到指令立即工作；缺点：功耗高
	//GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;	//增加输出电流的能力
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;	//不需要上下拉电阻
	GPIO_Init(GPIOA,&GPIO_InitStructure);	
	
	
	//将PA8引脚连接到EXTI8
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource8);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;				//EXTI8
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//中断触发
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿，按键按下，就立即触发中断请求，通知CPU立即处理 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				//使能EXTI0
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;		//外部中断5~8的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;//抢占优先级 0x2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;	//响应优先级 0x2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//NVIC使能EXTI8中断请求通道
	NVIC_Init(&NVIC_InitStructure);	

}

int32_t ir_read(uint8_t *pir_data)
{
	uint32_t t=0;
	
	uint8_t d;
	
	int32_t i=0;
	int32_t j=0;
	

	if(PAin(8))
	{
		return -1;
	}
	
	//检测引导码的低电平时间9ms
	t=0;
	while(PAin(8)==0)
	{
	
		t++;
		
		delay_us(10);
		
		//超时时间为10ms
		if(t >= 1000)
			return -2;

			
	}
	
	
	//检测引导码的低电平时间4.5ms
	t=0;
	while(PAin(8))
	{
	
		t++;
	
		delay_us(10);
		
		//超时时间为6ms
		if(t >= 600)
			return -3;
	}
	
	//连续接收4个字节
	for(j=0; j<4; j++)
	{
		d = 0;
		//完成8个bit数据的接收，低位优先
		for(i=0; i<8; i++)
		{
			//等待560us低电平持续完毕
			t=0;
			while(PAin(8)==0)
			{
			
				t++;
				
				delay_us(10);
				
				//超时时间为1ms
				if(t >= 100)
					return -4;
			}	
			
			//延时600us，用于区分数据0或数据1
			delay_us(600);
			
			if(PAin(8))
			{
				d|=1<<i;
				
				//等待数据1的高电平时间持续完毕
				t=0;
				while(PAin(8))
				{
				
					t++;
					
					delay_us(10);
					
					if(t >= 1000)
						return -5;
				}			
			
			}
		}	
	
		pir_data[j] = d;
	}
	
	
	//通信的结束
	delay_us(600);
	
	//校验数据
	if((pir_data[0]+pir_data[1])==255)
		if((pir_data[2]+pir_data[3])==255)
			return 0;
	
	return -6;
}

void EXTI9_5_IRQHandler(void)
{
	OS_ERR err;
	
	int32_t rt=0;
	
	//进入中断
	OSIntEnter(); 
	
	//检测标志位
	if(EXTI_GetITStatus(EXTI_Line8) == SET)
	{
		
		rt=ir_read((uint8_t *)g_ir_buf);
		
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
	
	if(rt == 0)
	{
		OSQPost(&g_queue_ir,(void *)g_ir_buf,sizeof g_ir_buf,OS_OPT_POST_FIFO,&err);
		
		if(err != OS_ERR_NONE)
		{
			printf("[EXTI9_5_IRQHandler]OSQPost error code %d\r\n",err);
		}
		
		
	}	

	//退出中断
	OSIntExit();
	
	
	
}
