#ifndef _FREERTOS_APP_H
#define _FREERTOS_APP_H

/* 调度器启动前：互斥量 / 共享快照等 RTOS 对象 */
void App_SharedInit(void);

/* 创建全部应用 Task（不启动调度器） */
void App_TasksCreate(void);

/* SharedInit → TasksCreate → vTaskStartScheduler */
void RTOS_AppStart(void);

#endif
