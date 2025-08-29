#include "MyProject.h"


/*****************************************************************************
****************************************************************************/
__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;
/****************************************************************************/
uint32_t timecntr_pre=0;

uint8_t usb_recbuff[128];
uint8_t usb_sndbuff[128];
uint32_t usb_rcv_count;
extern  bool dc_calib_valid;
/******************************************************************************/
void commander_run(void);
void USBcommander_run(void);
/******************************************************************************/
//us计时，每71.5分钟溢出循环一次
uint32_t timecount(void)
{
	uint32_t  diff,now_us;
	
	now_us = micros();    //0xFFFFFFFF=4294967295 us=71.5分钟
	if(now_us>=timecntr_pre)diff = now_us - timecntr_pre;   //us
	else
		diff = 0xFFFFFFFF - timecntr_pre + now_us;
	timecntr_pre = now_us;
	
	return diff;
}
/****************************************************************************/
uint32_t print_flag;

void control_loop_cb(void)
{
	encoder_update();
	controller_update();
	openloop_controller_update();
	motor_update();
	foc_update();
	
	if((motor_error!=0)&&(print_flag==0))
	{
		print_flag=1;  //只打印一次
		disarm();
		
		USART2_SendDMA(sprintf(snd2_buff,"error:0x%X\r\n", motor_error));      //串口打印
		uint32_t len=sprintf((char *)usb_sndbuff,"error:0x%X\r\n", motor_error); //虚拟串口打印
		usb_send(usb_sndbuff, len);
	}
}
/****************************************************************************/
void GPIO_Configure(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIO
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOA,ENABLE);//使能GPIOD,GPIOC,GPIOB,GPIOA;AFIO;
	
	//指示灯为PD2，低电平灯亮，高电平灯灭，
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;  //IO输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_2MHz; 
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; 
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_SetBits(GPIOD,GPIO_Pin_2);   //上电灭，待进入抗齿槽功能后闪烁，controller.c第325行
}
/*****************************************************************************/
//擦除指令，擦除扇区后复位
//看门狗使用32KHz时钟，32分频，计数500，大概0.5秒复位
void IWDG_Init(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //取消寄存器写保护
	IWDG_SetPrescaler(IWDG_Prescaler_32); //设置 IWDG 分频系数
	IWDG_SetReload(500);     //设置 IWDG 装载值
	IWDG_ReloadCounter();    //reload
	IWDG_Enable();           //使能看门狗
}
/*****************************************************************************/
//本示例支持USB通信和USART2通信，
//USART2接线：GPIO3接USB转串口的RXD，GPIO4接USB转串口的TXD，GND共地
//本节增加了参数保存功能
/*****************************************************************************/
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	GPIO_Configure();
	USART2_Init(115200);        //排针的GPIO3为TXD2，GPIO4为RXD2，GND共地
	// uart2_init(115200); //初始化串口2
	tim14_InitTick();           //1ms中断初始化，为系统提供计时
	TIM1_PWM_Init();            //M0接口PWM配置，但没有使能
	ADC_Common_Init();          //初始化ADC的引脚和通用配置，设置中断
	ADC1_DMA_Init();            //配置注入组，检测vbus，等待TIM1触发
	ADC2_TRGO_Init();           //配置注入组，检测m0_phB，等待TIM1触发
	ADC3_TRGO_Init();           //配置注入组，检测m0_phC，等待TIM1触发
	TIM2_Init();                //触发ADC1的规则转换
	TIM7_Init();
	CAN1_Init();				//CAN1初始化
	//参数设置在MyProject.h中，更多参数设置请进入以下几个函数中设置
	motor_para_init();
	motor_setup();
	MagneticSensor_Init();
	controller_config_default();//电机控制参数上电默认值
	trapTraj_config_default();  //梯形轨迹参数上电默认值
	controller_para_init();     //配置控制参数
	anticogging_init();         //抗齿槽参数默认值
	
	flash_para_read();          //读取参数
	if(encoder_config.pre_calibrated && axis_config.startup_closed_loop_control)current_state_ = AXIS_STATE_CLOSED_LOOP_CONTROL;  //如果已经校准并且配置上电闭环，设置闭环模式
	
	delay_us(500000);
	USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc, &USBD_CDC_cb, &USR_cb);  //USB转串口,COM174
	TIM_Cmd(TIM1, ENABLE);   //TIM1触发中断，AD开始转换。但是PWM还没有输出
	
	for(uint32_t i=0; i<2000; i++)   //最多等待2秒。main.cpp文件第565行
	{
		if(dc_calib_valid)break;       //等待电流校准完成后退出
		delay_us(1000);
	}
	
	USART2_SendDMA(sprintf(snd2_buff,"Motor Ready!\r\n"));
	//在MyProject.h中设置电机参数和控制模式。参数设置与官方代码一致，请先熟悉官方odrivetool的操作
	//发送指令“C”，测量电机的电阻电感并校准电机，发送指令“G”进入闭环，然后设置对应的速度或者位置
	while(1)
	{
		run_state_machine_loop();
		delay_us(1000);     //1ms，延时增加通信的可靠性
		
	}
}
/*****************************************************************************/
/*****************************************************************************/
//串口2通信，排针的GPIO3为TXD2，GPIO4为RXD2，GND共地。
//因为闭环控制时run_closed_loop_control_loop()代码会进入while循环，所以通信指令放在了接收中断里处理，usart2.c文件中，
//当电机转速较高时，比如T100，串口通信会影响电机的转动，此时请使用USB通信
void commander_run(void)
{
	switch(rcv2_buff[0])
	{
		case 'H': {
			USART2_SendDMA(sprintf(snd2_buff,"Closeloop2 Encoder:%d\r\n", encoder_config.mode));
		} break;
		
		case 'C': {  //测量电阻电感，3秒后电机“嘀”一声。然后进入校准，正反转8个电角度
			current_state_ = AXIS_STATE_MOTOR_CALIBRATION;
		} break;
		case 'G': {  //闭环控制
			current_state_ = AXIS_STATE_CLOSED_LOOP_CONTROL;
		} break;
		case 'I': {  //空闲模式
			input_pos_ = 0;   //清零输入值
			input_vel_ = 0;
			input_torque_ = 0;
			controller_reset();
			disarm();
			current_state_ = AXIS_STATE_IDLE;
		} break;
		
		case 'D': {  //开始抗齿槽校准
			start_anticogging_calibration();
			USART2_SendDMA(sprintf(snd2_buff,"start anticogging cal\r\n"));
		} break;
		case 'J': {  //读取校准到哪个位置
			USART2_SendDMA(sprintf(snd2_buff,"index=%d\r\n", anticogging.index));
		} break;
		
		case 'S': {  //设置力矩 S0.1
			input_torque_ = atof((const char *)(rcv2_buff+1));
			USART2_SendDMA(sprintf(snd2_buff,"S=%.2f\r\n", input_torque_));
		} break;
		case 'T': {  //设置速度 T10，单位 turn/s
			input_vel_ = atof((const char *)(rcv2_buff+1));
			USART2_SendDMA(sprintf(snd2_buff,"T=%.2f\r\n", input_vel_));
		} break;
		case 'K': {  //设置位置 K2，单位 turn
			input_pos_ = atof((const char *)(rcv2_buff+1));
			USART2_SendDMA(sprintf(snd2_buff,"K=%.2f\r\n", input_pos_));
			input_pos_updated_ = true;  //针对梯形轨迹模式，更新目标位置
		} break;
		case 'V': {  //读取实际速度，单位 turn/s
			USART2_SendDMA(sprintf(snd2_buff,"vel=%.2f\r\n", vel_estimate_));
		} break;
		case 'P': {  //当前绝对位置，单位 turn
			USART2_SendDMA(sprintf(snd2_buff,"pos=%.2f\r\n", pos_estimate_));
		} break;
		case 'Q': {  //读取错误标志
			USART2_SendDMA(sprintf(snd2_buff,"error=%X\r\n", motor_error));
		} break;
		
		case 'A': {  //配置是否已经校准  A1="pre_calibrated = True", A0="pre_calibrated = False",
			encoder_config.pre_calibrated = atoi((const char *)(rcv2_buff+1));
			USART2_SendDMA(sprintf(snd2_buff,"pre_calibrated = %d\r\n", encoder_config.pre_calibrated));
		} break;
		case 'B': {  //配置上电进入闭环模式  B1="startup_closed_loop_control = True", B0="startup_closed_loop_control = False"
			axis_config.startup_closed_loop_control = atoi((const char *)(rcv2_buff+1));
			USART2_SendDMA(sprintf(snd2_buff,"startup_closed_loop_control = %d\r\n", axis_config.startup_closed_loop_control));
		} break;
		case 'F': {  //保存参数
			flash_para_write();  //下载程序是不会擦除这个扇区，一旦保存不用时请及时擦除
			USART2_SendDMA(sprintf(snd2_buff,"flash save!\r\n"));
		} break;
		case 'E': {  //擦除扇区
			disarm();
			flash_para_erase();
			IWDG_Init();//擦除后重启
			USART2_SendDMA(sprintf(snd2_buff,"flash erase!\r\n"));
		} break;
		case 'R': {  //复位
			flash_para_write();  //下载程序是不会擦除这个扇区，一旦保存不用时请及时擦除
			IWDG_Init();//使能看门狗，等待复位
			USART2_SendDMA(sprintf(snd2_buff,"reset!\r\n"));
		} break;
	}
	memset(rcv2_buff,0,16);  //USART2_BUFFER_SIZE //清空接收数组,长度覆盖接收的字节数即可
}
/*****************************************************************************/
//USB虚拟串口通信
//放在USB接收中断里处理 App/usbd_cdc_vcp.c文件中的VCP_DataRx()函数中
void USBcommander_run(void)
{
	uint32_t len;
	
	if(usb_rcv_count != 0)
	{
		switch(usb_recbuff[0])
		{
			case 'H':
				len=sprintf((char *)usb_sndbuff, "Closeloop2 Encoder:%d\r\n", encoder_config.mode);
				usb_send(usb_sndbuff, len);
				break;
				
			case 'C': {  //测量电阻电感，3秒后电机“嘀”一声
				current_state_ = AXIS_STATE_MOTOR_CALIBRATION;
			} break;
			case 'G': {  //闭环控制
				current_state_ = AXIS_STATE_CLOSED_LOOP_CONTROL;
			} break;
			case 'I': {  //空闲模式
				input_pos_ = 0;   //清零输入值
				input_vel_ = 0;
				input_torque_ = 0;
				controller_reset();
				disarm();
				current_state_ = AXIS_STATE_IDLE;
			} break;
			
			case 'D': {  //开始抗齿槽校准
				start_anticogging_calibration();
				len=sprintf((char *)usb_sndbuff, "start anticogging cal\r\n");
				usb_send(usb_sndbuff, len);
			} break;
			case 'J': {  //读取校准到哪个位置
				len=sprintf((char *)usb_sndbuff, "index=%d\r\n", anticogging.index);
				usb_send(usb_sndbuff, len);
			} break;
			
			case 'S': {  //设置力矩 S0.1
				input_torque_ = atof((const char *)(usb_recbuff+1));
				len=sprintf((char *)usb_sndbuff,"S=%.2f\r\n", input_torque_);
				usb_send(usb_sndbuff, len);
			} break;
			case 'T': {  //设置速度 T10，单位turn/s
				input_vel_ = atof((const char *)(usb_recbuff+1));
				len=sprintf((char *)usb_sndbuff,"T=%.2f\r\n", input_vel_);
				usb_send(usb_sndbuff, len);
			} break;
			case 'K': {  //设置位置 K2，单位turn
				input_pos_ = atof((const char *)(usb_recbuff+1));
				len=sprintf((char *)usb_sndbuff, "K=%.2f\r\n", input_pos_);
				usb_send(usb_sndbuff, len);
				input_pos_updated_ = true;  //针对梯形轨迹模式，更新目标位置
			} break;
			case 'V': {  //读取当前速度 turn/s
				len=sprintf((char *)usb_sndbuff, "vel=%.4f\r\n", vel_estimate_);
				usb_send(usb_sndbuff, len);
			} break;
			case 'P': {  //读取当前位置 turn
				len=sprintf((char *)usb_sndbuff, "pos=%.4f\r\n", pos_estimate_);
				usb_send(usb_sndbuff, len);
			} break;
			case 'Q': {  //读取错误标志
				len=sprintf((char *)usb_sndbuff, "error=%X\r\n", motor_error);
				usb_send(usb_sndbuff, len);
			} break;
			
			case 'A': {  //配置是否已经校准  A1="pre_calibrated = True", A0="pre_calibrated = False",
				encoder_config.pre_calibrated = atoi((const char *)(usb_recbuff+1));
				len=sprintf((char *)usb_sndbuff, "pre_calibrated = %d\r\n", encoder_config.pre_calibrated);
				usb_send(usb_sndbuff, len);
			} break;
			case 'B': {  //配置上电进入闭环模式  B1="startup_closed_loop_control = True", B0="startup_closed_loop_control = False"
				axis_config.startup_closed_loop_control = atoi((const char *)(usb_recbuff+1));
				len=sprintf((char *)usb_sndbuff, "startup_closed_loop_control = %d\r\n", axis_config.startup_closed_loop_control);
				usb_send(usb_sndbuff, len);
			} break;
			case 'F': {  //保存参数
				flash_para_write();  //下载程序是不会擦除这个扇区，一旦保存不用时请及时擦除
				len=sprintf((char *)usb_sndbuff, "flash save!\r\n");
				usb_send(usb_sndbuff, len);
			} break;
			case 'E': {  //擦除扇区
				disarm();
				flash_para_erase();
				IWDG_Init();//擦除后重启
				len=sprintf((char *)usb_sndbuff, "flash erase!\r\n");
				usb_send(usb_sndbuff, len);
			} break;
			case 'R': {  //复位
				IWDG_Init();//使能看门狗，等待复位
				len=sprintf((char *)usb_sndbuff, "reset!\r\n");
				usb_send(usb_sndbuff, len);
			} break;
		}
		
		memset(usb_recbuff,0,usb_rcv_count); //清空接收数组
		usb_rcv_count = 0;
	}
}
/*****************************************************************************/


