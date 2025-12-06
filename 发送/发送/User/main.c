#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "NRF24L01.h"
#include "AD.h"

uint16_t AD0, AD1, AD2, AD3;
int i=0;
uint8_t TX_data[32]={0};
uint8_t AD_send(int i);
uint8_t TX_Status_0,TX_Status_1;

int main(void)
{
	
//	OLED_Init();


	NRF24L01_Init();
	AD_Init();
	
	while (1)
	{
		AD0 = AD_GetValue(ADC_Channel_0);
		AD1 = AD_GetValue(ADC_Channel_1);
		AD2 = AD_GetValue(ADC_Channel_2);
		AD3 = AD_GetValue(ADC_Channel_3);
		
		if(AD0 > 4000 || AD0 < 100)
		{
			TX_Status_1 = AD_send(1);
		}
		else
		{
			TX_Status_0 = AD_send(0);
		}
//		Delay_us(30);
////		Delay_ms(2);
//		TX_Status_1 = AD_send(1);
//		Delay_us(30);
////		Delay_ms(2);
	}
}

uint8_t AD_send(int i)
{
	TX_data[0] = AD0 >> 8;//取高八位
	TX_data[1] = AD0;	  //取低八位
	TX_data[2] = AD1 >> 8;//取高八位
	TX_data[3] = AD1;	  //取低八位
	TX_data[4] = AD2 >> 8;//取高八位
	TX_data[5] = AD2;	  //取低八位
	TX_data[6] = AD3 >> 8;//取高八位
	TX_data[7] = AD3;	  //取低八位
	
	return Send(TX_data,i);
}
