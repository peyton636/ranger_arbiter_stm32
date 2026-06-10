#ifndef _APP_BOOT_H
#define _APP_BOOT_H

/* 调度器启动前：运动控制相关外设初始化（Jetson/GPS/测距/CAN/Arbiter） */
void App_MotionHwInit(void);

/* 调度器启动前：LCD 占位画面；调度器启动后由 UiTask 接管刷新 */
void App_ShowBootSplash(void);

#endif
