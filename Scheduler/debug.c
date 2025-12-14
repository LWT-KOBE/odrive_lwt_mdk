#include "debug.h"


void debug_vofa_task(void){
    // if(current_state_ == AXIS_STATE_CLOSED_LOOP_CONTROL){
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
        vofaFrame.fdata[11] = Idq_setpoint_.q; 
        vofaFrame.fdata[12] = 0; // 计算温度值，单位摄氏度
        vofaFrame.fdata[13] = 0; // Vbus
        vofaFrame.fdata[14] = adc1_value[0]; // ADC原始值
        vofaFrame.fdata[15] = adc1_value[1]; // ADC原始值
        vofa_printf_USB();
    // }
}

