#include "blue_teeth.h"

uint32_t i= 0;
char buf[64] = {0};

void usart3_init(uint32_t baud)
{
	GPIO_InitTypeDef 	GPIO_InitStruct;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	USART_InitTypeDef 	USART_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	//使能端口B硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;//复用功能模式，PB10和PB11引脚交给内部的USART3自动管理
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;//推挽输出模式,默认的
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11;//指定第10 11根引脚
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;//高速，但是功耗是最高
	GPIO_Init(GPIOB,&GPIO_InitStruct);	
	
	//将引脚复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);
	
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3,&USART_InitStructure);
	
	//使能串口3的接收中断——触发中断的方式
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	//配置串口3的抢占优先级和响应优先级，打开它的中断请求通道给到NVIC
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	

	//使能USART3工作
	USART_Cmd(USART3, ENABLE);
}

void usart3_send_str(const char *str)
{
	const char *p = str;
	
	
	while(p && *p!='\0')
	{
	
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);	
		USART_SendData(USART3,*p);

		p++;
	}


}

void USART3_IRQHandler(void)
{
	uint8_t d=0;
	
	
	OS_ERR err;
	
	//检测是否接收到数据
	if(SET == USART_GetITStatus(USART3,USART_IT_RXNE))
	{
		//读取接收到的数据
		d=USART_ReceiveData(USART3);
		
		if(d == '#')
		{
			OSQPost(&g_queue_blueteeth,(void *)buf,strlen(buf),OS_OPT_POST_ALL,&err);
			
			i = 0;
			
		}
		else
		{
			memcpy(&buf[i],&d,1);
			
			i++;
			
		}
		
		
		
		//将读取到的数据发送给PC
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);	
		USART_SendData(USART1,d);		
		

		//将读取到的数据发送给蓝牙模块
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);	
		USART_SendData(USART3,d);		
		
		//清空标志位
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
	}
	
	
}


