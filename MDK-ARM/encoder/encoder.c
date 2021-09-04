#include "encoder.h"

/* ¶ÁÈ¡±àÂëÆ÷µÄÖµ */
int Read_Encoder(u8 TIMX)
{
	int Encoder_TIM;    
	switch(TIMX) {
		case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
		case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;	
		case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
		default: Encoder_TIM=0;
	}
	return Encoder_TIM;
}
