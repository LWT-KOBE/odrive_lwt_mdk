#include "MyProject.h"
#include "timer.h"
/*****************************************************************************/
void TIM1_PWM_Init(void)
{
	NVIC_InitTypeDef          NVIC_InitStructure;
	GPIO_InitTypeDef          GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef         TIM_OCInitStructure;
	TIM_BDTRInitTypeDef       TIM_BDTRInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_TIM1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_DeInit(TIM1);
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;
	TIM_TimeBaseInitStructure.TIM_Period = 3500;     //168M/7000=24KHz
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 2;      //重复计数器，每(2+1)次
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;    //使能输出
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;  //使能互补输出
	TIM_OCInitStructure.TIM_Pulse = 1750;                            //值越小，占空比越大
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;        //输出极性为高
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;      //输出互补极性为高
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;     //空闲状态为低
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;   //空闲状态为低
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime = 100;    // 100/168M=595ns
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	
  	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
  	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);	  //更新作为触发输出
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);    //更新中断使能
	//TIM_CtrlPWMOutputs(TIM1, ENABLE);  //PWM不输出，发送指令A开始输出
	//TIM_Cmd(TIM1, ENABLE);             //先不使能，等ADC配置完毕后再使能
}
/*****************************************************************************/
//TIM2更新中断触发ADC1的规则转换,1KHz频率
void TIM2_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period = 1000-1;      //1ms
	TIM_TimeBaseInitStructure.TIM_Prescaler=84-1;       //84分频=1MHz
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);	  //更新作为触发输出
	TIM_Cmd(TIM2,ENABLE);
}
/******************************************************************************/
//TIM3用于AB编码器计数
void TIM3_Encoder_Init(void)
{
	GPIO_InitTypeDef         GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef        TIM_ICInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;  //M0_ENC_A、M0_ENC_B
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;       //有外部上拉3.3K   
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);
  	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3);
	
  	TIM_TimeBaseInitStructure.TIM_Period = 0xffff;
	TIM_TimeBaseInitStructure.TIM_Prescaler=0;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; //时钟分割
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//计数模式3
	
	TIM_ICStructInit(&TIM_ICInitStructure); 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 4;  //滤波器值
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
  	TIM_SetCounter(TIM3,0); //TIM3->CNT=0
  	TIM_Cmd(TIM3, ENABLE); 
}
/*****************************************************************************/

void TIM3_InputCapture_Config(void){
	TIM_ICInitTypeDef TIM_ICInitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	// 1. 使能 GPIO 和 TIM3 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    
    // 2. 配置 PB4 (TIM3_CH1), PB5 (TIM3_CH2), PC9 (TIM3_CH4) 为复用功能
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // PB4 (TIM3_CH1)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;        // 复用模式
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      // 推挽输出
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);  // PB4 复用为 TIM3_CH1

    // PB5 (TIM3_CH2)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);  // PB5 复用为 TIM3_CH2

    // PC9 (TIM3_CH4)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);  // PC9 复用为 TIM3_CH4

	// 3. 配置 TIM3 时基（基本定时器设置）
    TIM_TimeBaseStruct.TIM_Prescaler = 0;       // 不分频
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_Period = 0xFFFF;          // 最大计数值
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

    // 4. 配置输入捕获（以 TIM3_CH1 为例）
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;    // PB4 (TIM3_CH1)
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;  // 上升沿捕获
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;  // 直接输入
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;  // 无分频
    TIM_ICInitStruct.TIM_ICFilter = 0x0;             // 无滤波
    TIM_ICInit(TIM3, &TIM_ICInitStruct);

    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;    // PB5 (TIM3_CH2)
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;  // 上升沿捕获
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;  // 直接输入
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;  // 无分频
    TIM_ICInitStruct.TIM_ICFilter = 0x0;             // 无滤波
    TIM_ICInit(TIM3, &TIM_ICInitStruct);

	TIM_ICInitStruct.TIM_Channel = TIM_Channel_4;    // PC9 (TIM3_CH4)
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;  // 上升沿捕获
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;  // 直接输入
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;  // 无分频
    TIM_ICInitStruct.TIM_ICFilter = 0x0;             // 无滤波
    TIM_ICInit(TIM3, &TIM_ICInitStruct);
    // 6. 启动 TIM3
    TIM_Cmd(TIM3, ENABLE);

}

// 定时器7
void TIM7_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	
	TIM_TimeBaseInitStructure.TIM_Period = 1000-1;      //1ms
	TIM_TimeBaseInitStructure.TIM_Prescaler=84-1;       //84分频=1MHz
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM7,ENABLE);
}
int time_cnt = 0;
int enc_cnt = 0;

typedef struct Val
{
	int val1;
	int val2;
}Valt;

char JS_RTT_UpBuff[4096];
u8 JS_RTT_Channel=1;
char JS_RTT_DownBuff[4096];

float Calculate_Temperature(uint16_t adc_value) {
	float voltage = (float)adc_value * VREF / ADC_RES;
    float resistance = (3.3 - voltage) * 10000 / voltage;
    float steinhart = resistance / 10000.0;
    steinhart = log(steinhart);
    steinhart = 1.0 / (0.001129148 + 0.000234125 * steinhart +
                      0.0000000876741 * pow(steinhart, 3));
    return steinhart - 273.15; // 开尔文转摄氏度
}

// ADC值转换为温度函数
float adc_to_temperature(uint16_t adc_value) {
    // 1. 将ADC值转换为电压
    float voltage = (float)adc_value * VREF / ADC_RES;
    
    // 2. 计算NTC当前电阻值
    // 公式: Rntc = Rdiv * (Vref - Vntc) / Vntc
    float ntc_resistance = VOLTAGE_DIVIDER_R * (VREF - voltage) / voltage;
    
    // 3. 使用Steinhart-Hart方程计算温度(开尔文)
    // 1/T = 1/T0 + (1/B) * ln(R/R0)
    float steinhart;
    steinhart = (1.0f / NTC_T25) + (1.0f / NTC_BETA) * logf(ntc_resistance / NTC_R25);
    float temp_kelvin = 1.0f / steinhart;
    
    // 4. 转换为摄氏度
    float temp_celsius = temp_kelvin - 273.15f;
    
    return temp_celsius;
}

void TIM7_IRQHandler(void)
{
	Valt test;
	static u8 c = 0;
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{	
		if(c == 0){
		    c = 1;
			//SEGGER_RTT_ConfigUpBuffer(JS_RTT_Channel, "JScope_t4i4i4", &JS_RTT_UpBuff[0], sizeof(JS_RTT_UpBuff), SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
		}
		float Rt=0;   //NTC电阻
		float R=10000; //10K固定阻值电阻
		float T0=273.15+25;//转换为开尔文温度
		float B=3450; //B值
		float Ka=273.15; //K值
		float VR=0;//电压值
		float temp=0;//温度值

		VR=(float) (adc1_value[0]/4095*3.3); //转换成电压值
   		Rt=(3.3-VR)*10000/VR;//计算Rt
   		temp=1/(1/T0+log(Rt/R)/B)-Ka+0.5; //计算温度

		test.val1 +=8;
		test.val2 +=4;
		// SEGGER_RTT_Write(JS_RTT_Channel, &test, sizeof(test));	/* 上传数据 */
		time_cnt++;
		//enc_cnt = HALL_GETState();
		

		// 位置估计
		 //OD_CANSendData(CAN1,OD_CANID,MSG_GET_ENCODER_ESTIMATES,8,pos_estimate_,&ODSendData);
		OD_CANSendData_2(CAN1,OD_CANID,MSG_GET_ENCODER_ESTIMATES,8,pos_estimate_,vel_estimate_,&ODSendData);
		// 母线电流
		//can_SendFloatData(CAN1, 0x303, 4, Ibus, &ODSendData);

		
		// 速度估计
		// OD_CANSendData(CAN1,OD_CANID,MSG_GET_ENCODER_ESTIMATES,8,pos_estimate_,&ODSendData);
		
		if(time_cnt > 20){
			time_cnt = 0;
			// 进入闭环控制状态才发送
			if(current_state_ == AXIS_STATE_CLOSED_LOOP_CONTROL){
				vofaFrame.fdata[0] = vel_estimate_;
				vofaFrame.fdata[1] = input_vel_;
				
				vofaFrame.fdata[2] = pos_estimate_;
				vofaFrame.fdata[3] = input_pos_;
				vofaFrame.fdata[4] = encoder_config.pre_calibrated;
				vofaFrame.fdata[5] = current_meas_.phA;
				vofaFrame.fdata[6] = current_meas_.phB;
				vofaFrame.fdata[7] = current_meas_.phC;
				vofaFrame.fdata[8] = Ibus;
				vofaFrame.fdata[9] = vbus_voltage;
				vofaFrame.fdata[10] = Iq_measured;
				vofaFrame.fdata[11] = input_torque_; 
//				// vofaFrame.fdata[12] = adc_to_temperature(adc1_value[2]); // 计算温度值，单位摄氏度
//				vofaFrame.fdata[13] = Calculate_Temperature(adc1_value[0]); // Vbus
				vofa_printf_USB();
			}
		}
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		// 在这里添加定时器7的中断处理代码
	}
}
