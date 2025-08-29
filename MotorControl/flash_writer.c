#include "MyProject.h"
#include "stm32f4xx_flash.h"
#include "flash_writer.h"
/*
STM32F405RGT6, 1M flash, 中文数据手册P.59
扇区 0   0x0800 0000 - 0x0800 3FFF 16 KB
扇区 1   0x0800 4000 - 0x0800 7FFF 16 KB
扇区 2   0x0800 8000 - 0x0800 BFFF 16 KB
扇区 3   0x0800 C000 - 0x0800 FFFF 16 KB
扇区 4   0x0801 0000 - 0x0801 FFFF 64 KB
扇区 5   0x0802 0000 - 0x0803 FFFF 128 KB
扇区 6   0x0804 0000 - 0x0805 FFFF 128 KB
扇区 7   0x0806 0000 - 0x0807 FFFF 128 KB
扇区 8   0x0808 0000 - 0x0809 FFFF 128 KB
扇区 9   0x080A 0000 - 0x080B FFFF 128 KB
扇区 10  0x080C 0000 - 0x080D FFFF 128 KB
扇区 11  0x080E 0000 - 0x080F FFFF 128 KB

#define FLASH_Sector_0     ((uint16_t)0x0000)
#define FLASH_Sector_1     ((uint16_t)0x0008)
#define FLASH_Sector_2     ((uint16_t)0x0010)
#define FLASH_Sector_3     ((uint16_t)0x0018)
#define FLASH_Sector_4     ((uint16_t)0x0020)
#define FLASH_Sector_5     ((uint16_t)0x0028)
#define FLASH_Sector_6     ((uint16_t)0x0030)
#define FLASH_Sector_7     ((uint16_t)0x0038)
#define FLASH_Sector_8     ((uint16_t)0x0040)
#define FLASH_Sector_9     ((uint16_t)0x0048)
#define FLASH_Sector_10    ((uint16_t)0x0050)
#define FLASH_Sector_11    ((uint16_t)0x0058)
*/
//第一次烧写固件前，请先整片擦除单片机，因为之前代码可能也使用了这个扇区保存参数
//比如官方odrive代码，好像就是使用的这个扇区，如果当时保存了参数，
//再运行当前代码，会把之前保存的参数用于当前代码中导致错误。


#define FLASH_Sector_11    ((uint16_t)0x0058)
#define Flash_Addr         0x080E0000     //保存参数的flash起始地址, 地址必须4字节对齐
#define NUMBER_PARA_        19
#define Flash_AntiCogging_Addr    0x080F0000     //保存抗齿槽校准参数的flash起始地址
/*****************************************************************************/
uint32_t flash_reg[128];
/*****************************************************************************/
typedef union
{
	float f;
	uint32_t da;
} UN32;

uint32_t float2uint(float var)
{
	UN32 a;
	
	a.f = var;
	return a.da;
}
float uint2float(uint32_t var)
{
	UN32 a;
	
	a.da = var;
	return a.f;
}
/*****************************************************************************/
void flash_para_read(void)
{
	uint32_t i;
	
	for(i=0; i<NUMBER_PARA_ ;i++)
	{
		flash_reg[i] = *(uint32_t*)(Flash_Addr+ i*4);
	}
	if(flash_reg[5]==1)   //已经校准
	{
		motor_config.phase_resistance = uint2float(flash_reg[0]);    //相电阻
		motor_config.phase_inductance = uint2float(flash_reg[1]);    //相电感   如果是云台电机设置为0
		encoder_config.direction = flash_reg[2];                     //方向
		encoder_config.phase_offset = flash_reg[3];                  //偏移值 整数部分
		encoder_config.phase_offset_float = uint2float(flash_reg[4]);//偏移值 小数部分
		encoder_config.pre_calibrated = flash_reg[5];                //是否已经校准
		axis_config.startup_closed_loop_control = flash_reg[6];      //是否上电后进入闭环
		anticogging_valid_ = flash_reg[7];                           //抗齿槽校准数据是否正常标志位

		motor_config.pole_pairs = flash_reg[8];  //极对数
		ctrl_config.control_mode = flash_reg[9];         //控制模式
		ctrl_config.input_mode = flash_reg[10];           //输入模式
		ctrl_config.pos_gain = uint2float(flash_reg[11]);                   //位置增益
		ctrl_config.vel_gain = uint2float(flash_reg[12]);                   //速度增益
		ctrl_config.vel_integrator_gain = uint2float(flash_reg[13]);        //速度积分增益
		ctrl_config.vel_limit = uint2float(flash_reg[14]);                  //速度限制
		ctrl_config.vel_limit_tolerance = uint2float(flash_reg[15]);        //速度限制容差
		ctrl_config.vel_integrator_limit = uint2float(flash_reg[16]);       //速度积分限制
		ctrl_config.vel_ramp_rate = uint2float(flash_reg[17]);              //速度斜坡率
		ctrl_config.torque_ramp_rate = uint2float(flash_reg[18]); 			//力矩斜坡率
		
		update_current_controller_gains();
		is_calibrated_ = 1;
		
//		一共保存了8个参数
//		printf("phase_resistance=%.4f\r\n", motor_config.phase_resistance);
//		printf("phase_inductance=%f\r\n", motor_config.phase_inductance);
//		printf("direction=%d\r\n", encoder_config.direction);
//		printf("phase_offset=%d\r\n", encoder_config.phase_offset);
//		printf("phase_offset_float=%.4f\r\n", encoder_config.phase_offset_float);
//		printf("pre_calibrated=%d\r\n", encoder_config.pre_calibrated);
//		printf("startup_closed_loop_control=%d\r\n", axis_config.startup_closed_loop_control);
//		printf("anticogging_valid=%d\r\n", anticogging_valid_);
	}
	
	if(anticogging_valid_)   //如果抗齿槽校准数据正常，读出全部数据
	{
		for(i=0; i<COG_num ;i++)
		{
			anticogging.cogging_map[i] = *(float*)(Flash_AntiCogging_Addr+i*4);
		}
	}
}
/*****************************************************************************/
void flash_para_write(void)
{
	uint32_t i;
	
	FLASH_Unlock();
	FLASH_DataCacheCmd(DISABLE);  //禁止数据缓存
	
	FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);  //擦除整个扇区128K字节
	
	flash_reg[0] = float2uint(motor_config.phase_resistance);    //相电阻
	flash_reg[1] = float2uint(motor_config.phase_inductance);    //相电感
	flash_reg[2] = encoder_config.direction;                     //方向
	flash_reg[3] = encoder_config.phase_offset;                  //偏移值 整数部分
	flash_reg[4] = float2uint(encoder_config.phase_offset_float);//偏移值 小数部分
	flash_reg[5] = encoder_config.pre_calibrated;                //是否已经校准
	flash_reg[6] = axis_config.startup_closed_loop_control;      //是否上电后进入闭环
	flash_reg[7] = anticogging_valid_;                           //抗齿槽校准数据是否正常
	
	flash_reg[8] = motor_config.pole_pairs;  							//极对数
	flash_reg[9] = ctrl_config.control_mode;         					//控制模式
	flash_reg[10] = ctrl_config.input_mode;           					//输入模式
	flash_reg[11] = float2uint(ctrl_config.pos_gain);                   //位置增益
	flash_reg[12] = float2uint(ctrl_config.vel_gain);                   //速度增益
	flash_reg[13] = float2uint(ctrl_config.vel_integrator_gain);        //速度积分增益
	flash_reg[14] = float2uint(ctrl_config.vel_limit);                  //速度限制
	flash_reg[15] = float2uint(ctrl_config.vel_limit_tolerance);        //速度限制容差
	flash_reg[16] = float2uint(ctrl_config.vel_integrator_limit);       //速度积分限制
	flash_reg[17] = float2uint(ctrl_config.vel_ramp_rate);              //速度斜坡率
	flash_reg[18] = float2uint(ctrl_config.torque_ramp_rate);		    //力矩斜坡率
	
	for(i=0; i<NUMBER_PARA_; i++)  //保存8个参数
	{
		FLASH_ProgramWord(Flash_Addr+4*i, flash_reg[i]);
	}
	
	if(anticogging_valid_)  //如果抗齿槽校准数据正常，保存
	{
		for(i=0;i<COG_num;i++)
		{
			FLASH_ProgramWord(Flash_AntiCogging_Addr+4*i, float2uint(anticogging.cogging_map[i]));
		}
	}
	
	FLASH_DataCacheCmd(ENABLE);   //开启数据缓存
	FLASH_Lock();
}
/*****************************************************************************/
void flash_para_erase(void)
{
	FLASH_Unlock();
	FLASH_DataCacheCmd(DISABLE);  //禁止数据缓存
	FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);  //擦除整个扇区128K字节
	FLASH_DataCacheCmd(ENABLE);   //开启数据缓存
	FLASH_Lock();
}
/*****************************************************************************/




