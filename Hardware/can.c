#include "can.h"

// CAN1初始化函数
void CAN1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;           // 定义GPIO初始化结构体
    CAN_InitTypeDef CAN_InitStructure;             // 定义CAN初始化结构体
    CAN_FilterInitTypeDef CAN_FilterInitStructure; // 定义CAN滤波器初始化结构体

    // 1. 使能GPIOB和CAN1的时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);    // 使能GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);     // 使能CAN1时钟

    // 2. 配置PB8为CAN1_RX，PB9为CAN1_TX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);  // PB8复用为CAN1
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);  // PB9复用为CAN1

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;   // 选择PB8和PB9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;             // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;       // 高速
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;           // 推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;         // 无上下拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);                   // 初始化GPIOB

    // 3. CAN配置
    CAN_DeInit(CAN1);                                        // 复位CAN1
    CAN_StructInit(&CAN_InitStructure);                      // 初始化CAN结构体为默认值
    CAN_InitStructure.CAN_TTCM = DISABLE;                    // 禁用时间触发通信模式
    CAN_InitStructure.CAN_ABOM = DISABLE;                    // 禁用自动离线管理
    CAN_InitStructure.CAN_AWUM = DISABLE;                    // 禁用自动唤醒
    CAN_InitStructure.CAN_NART = ENABLE;                     // 禁用自动重传
    CAN_InitStructure.CAN_RFLM = DISABLE;                    // 禁用接收FIFO锁定模式
    CAN_InitStructure.CAN_TXFP = DISABLE;                    // 禁用发送FIFO优先级
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;            // 设置为正常模式
    // 1Mbps: CAN_CLK = 42MHz, Prescaler = 3, BS1 = 11, BS2 = 4, SJW = 1
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;                 // 同步跳转宽度1
    CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;                // 时间段1为11
    CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;                 // 时间段2为4
    CAN_InitStructure.CAN_Prescaler = 3;                     // 预分频为3
    CAN_Init(CAN1, &CAN_InitStructure);                      // 初始化CAN1

    // 4. CAN滤波器配置（接收所有报文）
    CAN_FilterInitStructure.CAN_FilterNumber = 0;            // 滤波器编号0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; // 标识符屏蔽模式
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; // 32位宽度
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;       // 标识符高16位
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;        // 标识符低16位
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;   // 屏蔽高16位
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;    // 屏蔽低16位
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; // 分配到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;   // 使能滤波器
    CAN_FilterInit(&CAN_FilterInitStructure);                // 初始化CAN滤波器
}

// CAN1数据发送函数
// 参数：CANx - CAN控制器实例（如CAN1/CAN2）
//       ID_CAN - 报文标识符（标准ID）
//       len - 数据长度（0-8字节）
//       CanSendData - 包含发送数据的结构体指针
void CAN1_SendData(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len, CANSendStruct_t* CanSendData)
{
    CanTxMsg *txMessage;    // 定义CAN发送报文结构体指针
    uint8_t mbox;           // 用于存储发送邮箱号（0-2）
    uint8_t count;          // 数据拷贝循环计数器
    uint16_t i = 0;         // 发送状态检查超时计数器

    // 动态分配CAN报文内存（8字节对齐）
    txMessage = (CanTxMsg*)aqCalloc(8,sizeof(CanTxMsg));
    
    // 设置CAN报文头信息
    txMessage->StdId = ID_CAN;      // 设置标准标识符
    txMessage->IDE = CAN_Id_Standard; // 使用标准帧格式（非扩展帧）
    txMessage->RTR = CAN_RTR_Data;  // 设置为数据帧（非远程帧）
    txMessage->DLC = len;           // 设置数据长度（0-8）

    // 拷贝用户数据到CAN报文
    for (count = 0; count < len; count++) {
        txMessage->Data[count] = (uint8_t)CanSendData->data[count]; // 逐字节拷贝数据
    }

    // 启动CAN发送并获取使用的邮箱号
    mbox = CAN_Transmit(CANx, txMessage);

    // 等待发送完成（带超时保护）
    while (CAN_TransmitStatus(CANx,mbox) == 0x00) { // 0x00表示发送未完成
        i++;
        if (i >= 0xFFF) break; // 超过4095次等待则超时退出
    }

    // 释放动态分配的内存
    aqFree(txMessage,8,sizeof(CanTxMsg));
}

CanRxMsg can1_rx_msg;
u32 rxbuf3;
void CAN1_RX0_IRQHandler(void){
	//CanRxMsg can1_rx_msg;
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET){
		// 清除中断标志和标志位
		CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
		
		// 从接收 FIFO 中读取消息		
		CAN_Receive(CAN1, CAN_FIFO0, &can1_rx_msg);
		
		// 存储接收到的标准 ID
		rxbuf3=can1_rx_msg.StdId;

		/*********以下是自定义部分**********/
		


	}
}