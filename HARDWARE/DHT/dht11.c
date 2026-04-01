#include "sys.h"
#include "delay.h"
#include "includes.h"

static GPIO_InitTypeDef		GPIO_InitStructure;


void dht11_init(void)
{
	//使能端口G的硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);	

	
	//配置PG9为输出模式
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;			//第9个引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;		//输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;		//引脚高速工作，收到指令立即工作；缺点：功耗高
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;		//增加输出电流的能力
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;		//不需要上下拉电阻
	GPIO_Init(GPIOG,&GPIO_InitStructure);	


	//PG9初始电平为高电平
	PGout(9)=1;
}


static void dht11_pin_mode(GPIOMode_TypeDef gpio_mode)
{
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;			//第9个引脚
	GPIO_InitStructure.GPIO_Mode=gpio_mode;			//输出/输入模式
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;		//引脚高速工作，收到指令立即工作；缺点：功耗高
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;		//增加输出电流的能力
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;		//不需要上下拉电阻
	GPIO_Init(GPIOG,&GPIO_InitStructure);	

}



int32_t dht11_read(uint8_t *pdht_data)
{
	uint32_t t=0;
	
	uint8_t d;
	
	int32_t i=0;
	int32_t j=0;
	uint32_t check_sum=0;
	
	//保证引脚为输出模式
	dht11_pin_mode(GPIO_Mode_OUT);	
	
	PGout(9)=0;
	
	delay_ms(20);
	
	PGout(9)=1;	
	
	delay_us(30);
	
	//保证引脚为输入模式
	dht11_pin_mode(GPIO_Mode_IN);

	
	//等待DHT11响应，等待低电平出现
	t=0;
	while(PGin(9))
	{
	
		t++;
		
		delay_us(1);
		
		if(t >= 4000)
			return -1;
	}
	
	//若低电平超出100us
	t=0;
	while(PGin(9)==0)
	{
	
		t++;
		
		delay_us(1);
		
		if(t >= 100)
			return -2;
	}
	
	
	//若高电平超出100us
	t=0;
	while(PGin(9))
	{
	
		t++;
		
		delay_us(1);
		
		if(t >= 100)
			return -3;
	}
	
	//连续接收5个字节
	for(j=0; j<5; j++)
	{
		d = 0;
		//完成8个bit数据的接收，高位优先
		for(i=7; i>=0; i--)
		{
			//等待低电平持续完毕
			t=0;
			while(PGin(9)==0)
			{
			
				t++;
				
				delay_us(1);
				
				if(t >= 100)
					return -4;
			}	
			
			
			delay_us(40);
			
			if(PGin(9))
			{
				d|=1<<i;
				
				//等待数据1的高电平时间持续完毕
				t=0;
				while(PGin(9))
				{
				
					t++;
					
					delay_us(1);
					
					if(t >= 100)
						return -5;
				}			
			
			}
		}	
	
		pdht_data[j] = d;
	}
	
	
	//通信的结束
	delay_us(100);
	
	//计算校验和
	check_sum=pdht_data[0]+pdht_data[1]+pdht_data[2]+pdht_data[3];
	
	
	
	check_sum = check_sum & 0xFF;
	
	if(check_sum != pdht_data[4])
		return -6;
	
	return 0;
}
