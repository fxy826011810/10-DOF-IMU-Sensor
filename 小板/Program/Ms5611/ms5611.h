#ifndef __MS5611_H__
#define __MS5611_H__

#define MS5611_ADDR          	 0x77
#define RESET          				 0x1E
#define ADC_READ							 0x00

#define Convert_D1_256           0x40
#define Convert_D1_512           0x42
#define Convert_D1_1024          0x44
#define Convert_D1_2048					 0x46
#define Convert_D1_4096					 0x48

#define Convert_D2_256           0x50
#define Convert_D2_512           0x52
#define Convert_D2_1024          0x54
#define Convert_D2_2048					 0x56
#define Convert_D2_4096					 0x58

#define PROM_READ							 0xA0//to 0xae

typedef struct 
{
	uint32_t temp;
	uint32_t pressure;
}Ms5611DataDef;

typedef enum
{
	prepareADC,
	readADC,
}Ms5611Status;



void Ms5611_Init(void);
//uint8_t Ms5611_ReadBytes(SimIIC_Typedef *simiic,uint8_t *pbuffer,uint8_t len);
//void Ms5611_Read(uint8_t reg,  uint8_t *pbuffer, uint8_t len);
//uint8_t Ms5611_WriteByte(SimIIC_Typedef *simiic,uint8_t reg,uint8_t flag);
uint8_t Ms5611_ReadD(uint8_t reg,uint32_t *trans);
//uint8_t Ms5611_Reset(void);
//uint8_t Ms5611_PromRead(SimIIC_Typedef *simiic,uint8_t reg,uint8_t Data);
#endif