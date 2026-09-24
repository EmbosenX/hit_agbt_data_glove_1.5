#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* 确保仅编译器使用，汇编器不引用 */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include <stdint.h>
  void xPortSysTickHandler(void);
#endif

/* 内核基础配置 */
#define configENABLE_FPU                         1
#define configENABLE_MPU                         0
/* 钩子函数配置 */
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configUSE_DAEMON_TASK_STARTUP_HOOK       0
#define configUSE_PREEMPTION                     1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1
#define configUSE_TICKLESS_IDLE                  0
#define configUSE_TIME_SLICING                   1
#define configIDLE_SHOULD_YIELD                  1
#define configUSE_16_BIT_TICKS                   0

/* 时钟与节拍配置 - 对应HC32F460 200MHz主频 */
#define configCPU_CLOCK_HZ                       ( 200000000UL )
#define configTICK_RATE_HZ                       ((TickType_t)1000)

/* 任务与内存配置 */
#define configMAX_PRIORITIES                     ( 32 )
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
#define configMAX_TASK_NAME_LEN                  ( 16 )
#define configMESSAGE_BUFFER_LENGTH_TYPE         size_t

#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configTOTAL_HEAP_SIZE                    ((size_t)(64 * 1024))
#define configAPPLICATION_ALLOCATED_HEAP         0

/* 内核功能开关 */
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_TASK_NOTIFICATIONS             1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES    1
#define configUSE_QUEUE_SETS                     0
#define configQUEUE_REGISTRY_SIZE                8

/* 软件定时器配置
 * 守护任务栈：周期节拍(125Hz 上报 / 10ms caliber 上传)的回调在守护任务上下文执行，
 * 调用链含组帧 + 整帧拷贝入队，实测栈深 1.2~1.3KB，故由 *2(1KB) 加大到 *4(2KB)。
 * 注意：静态分配时守护任务栈实际由 main.c 的 uxTimerTaskStack[] 提供，两处须同步 */
#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                 10
#define configTIMER_TASK_STACK_DEPTH             ( configMINIMAL_STACK_SIZE * 4 )

/* 协程（一般不用） */
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          ( 2 )

/* ===================== 中断优先级核心配置 =====================
   HC32F460: 4位抢占优先级，共16级（0最高，15最低）
   优先级 0~4:  高优先级，不被临界段屏蔽，禁止调用FreeRTOS API
   优先级 5~15: 可安全调用 FromISR 系列API
*/
#ifdef __NVIC_PRIO_BITS
 #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
 #define configPRIO_BITS         4
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY        15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY   5

#define configKERNEL_INTERRUPT_PRIORITY        ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY   ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* 调试与断言 */
#define configCHECK_FOR_STACK_OVERFLOW         0
#define configUSE_MALLOC_FAILED_HOOK           0
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}

/* 运行时统计 - 已关闭，避免未定义报错
   需要开启时，配套实现高频定时器驱动即可
*/
#define configUSE_TRACE_FACILITY               0
#define configGENERATE_RUN_TIME_STATS          0
#define configUSE_STATS_FORMATTING_FUNCTIONS   0

/* API 函数裁剪 */
#define INCLUDE_vTaskPrioritySet               1
#define INCLUDE_uxTaskPriorityGet              1
#define INCLUDE_vTaskDelete                    1
#define INCLUDE_vTaskCleanUpResources          0
#define INCLUDE_vTaskSuspend                   1
#define INCLUDE_vTaskDelayUntil                1
#define INCLUDE_vTaskDelay                     1
#define INCLUDE_xTaskGetSchedulerState         1
#define INCLUDE_xTaskGetCurrentTaskHandle      1
#define INCLUDE_uxTaskGetStackHighWaterMark    1
#define INCLUDE_xTimerPendFunctionCall         1

/* 中断函数重映射 - 解决与启动文件的符号冲突 */
#define vPortSVCHandler        SVC_Handler
#define xPortPendSVHandler     PendSV_Handler
#define xPortSysTickHandler    SysTick_Handler

#endif /* FREERTOS_CONFIG_H */


