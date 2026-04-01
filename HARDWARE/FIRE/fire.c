#include "fire.h"




void adc_init(void)
{
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	GPIO_InitTypeDef 	  GPIO_InitStruct;
	
	//使能端口A的硬件时钟，就是对端口A供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	
	//使能ADC1的硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	

	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AN;//模拟信号模式
	//GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;//推挽输出模式,默认的
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6;//指定第6根引脚
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;//高速，但是功耗是最高
	GPIO_Init(GPIOA,&GPIO_InitStruct);



	//配置ADC1相关的参数
	
	/* ADC常规初始化（ADC1 ~ ADC3 都使用该配置）*/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//只需要一个adc工作
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;//adc硬件时钟=84MHz/2=42MHz
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;//关闭直接存储访问
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样点的间隔是5个ADC时钟的时间是多少=5*(1/42MHz)
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位的精度（分辨率）
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//单个通道扫描；如果实现多个通道的扫描，配置为ENABLE
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//连续工作；否则为只工作一次就停止
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//不需要定时器的脉冲计数来触发
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//存储格式为右对齐
	ADC_InitStructure.ADC_NbrOfConversion = 1;//告诉ADC1有多少个通道进行扫描，当前只有一个通道
	ADC_Init(ADC1, &ADC_InitStructure);
	
	
	//用于指定ADC1的扫描通道，规则序号为1，采样时间为3个ADC时钟=3*(1/42MHz)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_3Cycles);
	
	//使能ADC1工作
	ADC_Cmd(ADC1, ENABLE);

}

void fire_init(void)
{
	GPIO_InitTypeDef 	  GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN;        //输入模式
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9; //指定第9根引脚
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;     //高速，但是功耗是最高
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;  //无需上下拉（亦可使能下拉电阻）
	GPIO_Init(GPIOC,&GPIO_InitStruct);              //D口

}
