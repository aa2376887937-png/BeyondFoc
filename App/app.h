#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化外设和模块（在 main() 里、osKernelInitialize() 之前调用） */
void All_Init(void);

/* 创建应用任务（在 osKernelInitialize() 之后、osKernelStart() 之前调用） */
void App_Start(void);

#ifdef __cplusplus
}
#endif
