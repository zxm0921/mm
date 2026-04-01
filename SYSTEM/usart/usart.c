#include "sys.h"
#include "usart.h"	
 
#include "includes.h"					//ucos 使用	  


static volatile uint8_t  g_usart1_buf[128]={0};
static volatile uint32_t g_usart1_cnt=0;

//static volatile uint8_t  g_usart2_buf[128]={0};
//static volatile uint32_t g_usart2_cnt=0;

//char  g_usart2_buf2[128]={0};

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  

#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;   
	
	//清空发送完成标志位
	USART1->SR = (uint16_t)~USART_FLAG_TC;
	return ch;
}

 
//初始化IO 串口1 
//baud:波特率
void usart_init(u32 baud)
{
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = baud;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

	
}


void usart2_init(uint32_t baud)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	//使能端口A硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;//复用功能模式，PB10和PB11引脚交给内部的USART3自动管理
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;//推挽输出模式,默认的
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;//指定第2 3根引脚
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;//高速，但是功耗是最高
	GPIO_Init(GPIOA,&GPIO_InitStruct);	
	
	//将引脚复用为USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
	
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2,&USART_InitStructure);
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	
	//使能串口2的接收中断——触发中断的方式
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	//配置串口2的抢占优先级和响应优先级，打开它的中断请求通道给到NVIC
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	

	//使能USART2工作
	USART_Cmd(USART2, ENABLE);
}


void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	uint8_t d=0;
	
	OS_ERR err;
	
	uint32_t usart1_packet_complete=0;

	//进入中断
	OSIntEnter();    

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		//接收串口数据
		d=USART_ReceiveData(USART1);	
		
		g_usart1_buf[g_usart1_cnt] = d;
		
		g_usart1_cnt++;

		if(d == '#'|| g_usart1_cnt>= sizeof(g_usart1_buf))//
		{
			usart1_packet_complete=1;
			
		}		
		
		
		//清空串口接收中断标志位
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	} 
	
	//发送消息队列
	if(usart1_packet_complete)
	{
		OSQPost(&g_queue_usart1,(void *)g_usart1_buf,g_usart1_cnt,OS_OPT_POST_FIFO,&err);
		//memset(g_usart1_buf,0,sizeof(g_usart1_buf));
		g_usart1_cnt=0;
		
		if(err != OS_ERR_NONE)
		{
			printf("[USART1_IRQHandler]OSQPost error code %d\r\n",err);
		}
	
	}	
	
	//退出中断
	OSIntExit();    
} 


void USART2_IRQHandler(void)
{
	uint8_t d=0;
	OS_ERR err;
	uint32_t usart2_packet_complete=0;
	
	//进入中断
	OSIntEnter();
	
	//检测是否接收到数据
	if(SET == USART_GetITStatus(USART2,USART_IT_RXNE))
	{
		//读取接收到的数据
		d=USART_ReceiveData(USART2);
		
		//将读取到的数据发送给PC
		while(USART_GetFlagStatus(USART2,USART_FLAG_RXNE)==SET);	
		USART_SendData(USART1,d);		
		
//		g_usart2_buf[g_usart2_cnt] = d;
//		
//		g_usart2_cnt++;

//		if(d == '\n'|| g_usart2_cnt>= sizeof(g_usart2_buf))
//		{
//			usart2_packet_complete=1;
//		}	
		
		//清空标志位
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	}
	
	//发送消息队列
//	if(usart2_packet_complete)
//	{
//		memcpy(g_usart2_buf2,(void *)g_usart2_buf,sizeof(g_usart2_buf));
//		
//		if(strncmp((char *)g_usart2_buf2,"AT+",3) != 0)
//		{
//			OSQPost(&g_queue_usart2,(void *)g_usart2_buf,g_usart2_cnt,OS_OPT_POST_FIFO,&err);
//			
//			g_usart2_cnt=0;
//			
//			if(err != OS_ERR_NONE)
//			{
//				printf("[USART2_IRQHandler]OSQPost error code %d\r\n",err);
//			}
//			
//		}
//		
//	
//	}
	
	//退出中断
	OSIntExit();
}


void usart1_send_str(const char *str)
{
	const char *p = str;
	
	
	while(p && *p!='\0')
	{
	
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);	
		USART_SendData(USART1,*p);

		p++;
	}


}


void usart2_send_str(const char *str)
{
	const char *p = str;
	
	
	while(p && *p!='\0')
	{
	
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);	
		USART_SendData(USART2,*p);

		p++;
	}


} 



