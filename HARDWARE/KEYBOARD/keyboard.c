#include "keyboard.h"
#include "includes.h"
#include "sys.h"
#include "delay.h"


void keyboard_init(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;        //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;       //推挽输出模式,默认的
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14|GPIO_Pin_0; //指定第14 0根引脚
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;     //高速，但是功耗是最高
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;  //无需上下拉（亦可使能下拉电阻）
	GPIO_Init(GPIOD,&GPIO_InitStructure);              //D口
    
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7|GPIO_Pin_9; //指定第7 9根引脚
    GPIO_Init(GPIOE,&GPIO_InitStructure);              //E口
	
	
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;	   //上拉
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;         //输入模式
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;            //指定第6根引脚
    GPIO_Init(GPIOD,&GPIO_InitStructure);              //D口
    
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15 | GPIO_Pin_11 | GPIO_Pin_13;            //指定引脚
    GPIO_Init(GPIOE,&GPIO_InitStructure);              //E口
}


uint8_t get_key_board(void)
{
    
    PDout(14) = 0;
    PDout(0) = 1;
    PEout(7) = 1;
    PEout(9) = 1;
    delay_ms(2);
	
    
    if( PEin(15) == 0 ) return 'B';
    else if( PEin(11) == 0 ) return 'D';
    else if( PEin(13) == 0 ) return 'C';
    else if( PDin(9) == 0 ) return 'A';
    
    PDout(14) = 1;
    PDout(0) = 0;
    PEout(7) = 1;
    PEout(9) = 1;
    delay_ms(2);

    
    if( PEin(15) == 0 ) return '6';
    else if( PEin(11) == 0 ) return '#';
    else if( PEin(13) == 0 ) return '9';
    else if( PDin(9) == 0 ) return '3';
    
    PDout(14) = 1;
    PDout(0) = 1;
    PEout(7) = 0;
    PEout(9) = 1;
    delay_ms(2);
   
   if( PEin(15) == 0 ) return '5';
    else if( PEin(11) == 0 ) return '0';
    else if( PEin(13) == 0 ) return '8';
    else if( PDin(9) == 0 ) return '2';
    
    PDout(14) = 1;
    PDout(0) = 1;
    PEout(7) = 1;
    PEout(9) = 0;
    delay_ms(2);
   
    if( PEin(15) == 0 ) return '4';
    else if( PEin(11) == 0 ) return '*';
    else if( PEin(13) == 0 ) return '7';
    else if( PDin(9) == 0 ) return '1';
    
    return 'N';
}


void key_state(void)//获取按键状态，用于按键消抖
{
	uint32_t key_sta=0;
	char key_old=0;
	char key_cur=0;
	OS_ERR err;
	//char key_ret=0;
	
	while(1)
	{
		/* 使用状态机思想得到按键的状态 */
		switch(key_sta)
		{
			case 0://获取按下的按键
			{
				
				key_cur = get_key_board();	

				if(key_cur != 'N')
				{
					key_old = key_cur;
					key_sta=1;
				}
					
			
			}break;
			
			
			case 1://确认按下的按键
			{
				
				key_cur = get_key_board();	
					
				if((key_cur != 'N') && (key_cur == key_old))
				{
					
					key_sta=2;
				}
								
			
			}break;
		
			case 2://获取释放的按键
			{
				
				key_cur = get_key_board();	
					
				if(key_cur == 'N')
				{	
					
					if(key_old == '1')
					{
						OSQPost(&g_queue_keyboard,"1",1,OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					}
					
					if(key_old == '2')
					{
						OSQPost(&g_queue_keyboard,"2",1,OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					}
					
					if(key_old == '3')
					{
						OSQPost(&g_queue_keyboard,"3",1,OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					}
					
					if(key_old == '4')
					{
						OSQPost(&g_queue_keyboard,"4",1,OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					}
					
					if(key_old == '5')
					{
						OSQPost(&g_queue_keyboard,"5",1,OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					}
					
					if(key_old == '6')
					{
						OSQPost(&g_queue_keyboard,"6",1,OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					}
					
					if(key_old == '7')
					{
						OSQPost(&g_queue_keyboard,"7",1,OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					}
					
					if(key_old == '8')
					{
						OSQPost(&g_queue_keyboard,"8",1,OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					}
					
					if(key_old == '9')
					{
						OSQPost(&g_queue_keyboard,"9",1,OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					}
					
					if(key_old == '0')
					{
						OSQPost(&g_queue_keyboard,"0",1,OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					}
					
					if(key_old == '#')
					{
						OSQPost(&g_queue_keyboard,"#",1,OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					}
					
					if(key_old == '*')
					{
						OSQPost(&g_queue_keyboard,"*",1,OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					}
					
					if(key_old == 'A')
					{
						OSQPost(&g_queue_keyboard,"A",1,OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					}
					
					if(key_old == 'B')
					{
						OSQPost(&g_queue_keyboard,"B",1,OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					}
					
					if(key_old == 'C')
					{
						OSQPost(&g_queue_keyboard,"C",1,OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					}
					
					if(key_old == 'D')
					{
						OSQPost(&g_queue_keyboard,"D",1,OS_OPT_POST_FIFO+OS_OPT_POST_ALL,&err);
					}
					
					
					
					key_sta=0;			
					key_old =  'N';
					return;
				}
								
			
			}break;

			default:
			{
				
				break;
			}	
			
		}

	}
}
