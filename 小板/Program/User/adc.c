#include "stm32f4xx.h"
#include "adc.h"
int16_t VBAT_buff[10]={0};
void Bsp_ADC_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	ADC_InitTypeDef adc;
	ADC_CommonInitTypeDef adc_common;
	adc_common.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;
	adc_common.ADC_Mode=ADC_Mode_Independent;
	adc_common.ADC_Prescaler=ADC_Prescaler_Div2;
	adc_common.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&adc_common);

	adc.ADC_ContinuousConvMode=ENABLE;
	adc.ADC_DataAlign=ADC_DataAlign_Right;
	adc.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T1_CC1;
	adc.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;
	adc.ADC_NbrOfConversion=1;
	adc.ADC_Resolution=ADC_Resolution_12b;
	adc.ADC_ScanConvMode=DISABLE;
	ADC_Init(ADC1,&adc);
	
	
	DMA2_Stream0->M0AR =(uint32_t)VBAT_buff;
	DMA2_Stream0->NDTR =sizeof(VBAT_buff)/sizeof(int16_t);
	DMA_Cmd(DMA2_Stream0,ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_18,1,ADC_SampleTime_15Cycles);
	
	ADC_VBATCmd(ENABLE);
	
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	
	ADC_Cmd(ADC1,ENABLE);
}
