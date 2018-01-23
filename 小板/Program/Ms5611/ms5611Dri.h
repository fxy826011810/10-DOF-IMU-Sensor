#ifndef __MS5611DRI_H__
#define __MS5611DRI_H__

uint8_t Ms5611_WriteByte(uint8_t reg, uint8_t pbuffer);
uint8_t Ms5611_Read(uint8_t reg,  uint8_t *pbuffer, uint8_t len);
uint8_t Ms5611_ReadByte(uint8_t reg, uint8_t *pbuffer);



#endif