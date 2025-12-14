#include "scheduler.h"

// 任务列表数组
static Task_t task_list[MAX_TASKS];

// 初始化任务列表
void Scheduler_Init(void) {
    for(int i = 0; i < MAX_TASKS; i++) {
        task_list[i].p_task_func = 0;
        task_list[i].period_ms = 0;
        task_list[i].time_cnt = 0;
        task_list[i].run_flag = 0;
    }
}

// 注册任务
// index: 任务ID (0 ~ MAX_TASKS-1)
// func:  任务函数名
// period: 执行间隔(ms)
void Scheduler_Register(uint8_t index, void (*func)(void), uint16_t period) {
    if(index < MAX_TASKS) {
        task_list[index].p_task_func = func;
        task_list[index].period_ms = period;
        task_list[index].time_cnt = period; // 初始倒计时
        task_list[index].run_flag = 0;
    }
}

// 此函数放在定时器中断中，每1ms调用一次
void Scheduler_Tick_Handler(void) {
    for(int i = 0; i < MAX_TASKS; i++) {
        // 如果该任务有效
        if(task_list[i].p_task_func != 0) {
            if(task_list[i].time_cnt > 0) {
                task_list[i].time_cnt--; // 倒计时减1
            }
            if(task_list[i].time_cnt == 0) {
                task_list[i].run_flag = 1; // 标记为可运行
                task_list[i].time_cnt = task_list[i].period_ms; // 重置倒计时
            }
        }
    }
}

// 主调度器，放在 main 的 while(1) 中
void Scheduler_Dispatch(void) {
    for(int i = 0; i < MAX_TASKS; i++) {
        if(task_list[i].run_flag == 1) {
            // 执行任务
            task_list[i].p_task_func();
            // 清除标志
            task_list[i].run_flag = 0;
        }
    }
}

