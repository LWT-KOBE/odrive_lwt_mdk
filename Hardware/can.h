#ifndef __CAN_H
#define __CAN_H

#include "MyProject.h"
/*CAN发送结构体*/
typedef struct {
	uint8_t cmd;
	uint8_t data[8];
}CANSendStruct_t;

void CAN1_Init(void);
void CAN1_SendData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint8_t len,CANSendStruct_t* CanSendData);
#endif