#ifndef _SCHEDULER_H
#define _SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

// 定义任务最大数量
#define MAX_TASKS 5 

// 任务结构体
typedef struct {
    void (*p_task_func)(void);  // 任务函数指针
    uint16_t period_ms;         // 任务执行周期 (ms)
    volatile uint16_t time_cnt; // 倒计时计数器
    uint8_t run_flag;           // 运行标志位 (1=就绪, 0=等待)
} Task_t;

// 函数声明
void Scheduler_Init(void);
void Scheduler_Register(uint8_t index, void (*func)(void), uint16_t period);
void Scheduler_Dispatch(void);
void Scheduler_Tick_Handler(void);

#endif // _SCHEDULER_H


