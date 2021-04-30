/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 * 2018-11-19     flybreak     add stm32f407-atk-explorer bsp
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "psdk_core.h"
#include "psdk_platform.h"
#include "psdk_product_info.h"

/* defined the LED0 pin: PF9 */
#define LED0_PIN    GET_PIN(F, 9)

rt_device_t dev3 = RT_NULL;

#define USER_APP_NAME               "INYYO 210R"
#define USER_APP_ID                 "99732"
#define USER_APP_KEY                "bd0662289b3cb3a11421b4b65fe58bc"
#define USER_DEVELOPER_ACCOUNT      "779213943@qq.com"
#define USER_UTIL_MIN(a, b)         (((a) < (b)) ? (a) : (b))

T_PsdkReturnCode Hal_UartInit(void)
{
    dev3 = rt_device_find("uart3");
    RT_ASSERT(dev3 != RT_NULL);
    rt_device_open(dev3, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
	struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_460800;
    rt_device_control(dev3, RT_DEVICE_CTRL_CONFIG, &config);

    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_PsdkReturnCode Hal_UartSendData(const uint8_t *buf, uint16_t len)
{
    rt_device_write(dev3, 0, buf, len);

    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_PsdkReturnCode Hal_UartReadData(uint8_t *buf, uint16_t len, uint16_t *realLen)
{
    *realLen = rt_device_read(dev3, 0, buf, len);

    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
T_PsdkReturnCode Osal_TaskCreate(T_PsdkTaskHandle *task, void *(*taskFunc)(void *), const char *name, uint32_t stackSize, void *arg)
{
	static rt_thread_t tid1 = RT_NULL;
	tid1=rt_thread_create(name,(void *)taskFunc, (void*)1,stackSize,10, 20);
	if (tid1 != RT_NULL)
        rt_thread_startup(tid1);
	task=(T_PsdkTaskHandle *)tid1;
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_PsdkReturnCode Osal_TaskDestroy(T_PsdkTaskHandle task)
{
	rt_thread_delete((rt_thread_t) task);
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_PsdkReturnCode Osal_TaskSleepMs(uint32_t timeMs)
{
	rt_thread_mdelay(timeMs);
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_PsdkReturnCode Osal_MutexCreate(T_PsdkMutexHandle *mutex)
{
	static rt_mutex_t dynamic_mutex = RT_NULL;
	dynamic_mutex =rt_mutex_create ("mutex", RT_IPC_FLAG_FIFO);
	mutex=(T_PsdkMutexHandle *)dynamic_mutex;
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_PsdkReturnCode Osal_MutexDestroy(T_PsdkMutexHandle mutex)
{
	rt_mutex_delete ((rt_mutex_t ) mutex);
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_PsdkReturnCode Osal_MutexLock(T_PsdkMutexHandle mutex)
{
	rt_mutex_take ((rt_mutex_t ) mutex, RT_WAITING_FOREVER);
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_PsdkReturnCode Osal_MutexUnlock(T_PsdkMutexHandle mutex)
{
	rt_mutex_release((rt_mutex_t ) mutex);
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_PsdkReturnCode Osal_SemaphoreCreate(T_PsdkSemHandle *semaphore, uint32_t initValue)
{
	static rt_sem_t dynamic_sem = RT_NULL;
	dynamic_sem = rt_sem_create("dsem", 0, RT_IPC_FLAG_FIFO);
	semaphore=(T_PsdkSemHandle *)dynamic_sem;
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_PsdkReturnCode Osal_SemaphoreDestroy(T_PsdkSemHandle semaphore)
{
	rt_sem_delete((rt_sem_t ) semaphore);
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_PsdkReturnCode Osal_SemaphoreWait(T_PsdkSemHandle semaphore)
{
	rt_sem_trytake((rt_sem_t ) semaphore);
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_PsdkReturnCode Osal_SemaphoreTimedWait(T_PsdkSemHandle semaphore, uint32_t waitTimeMs)
{
	rt_sem_take ((rt_sem_t ) semaphore, waitTimeMs);
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_PsdkReturnCode Osal_SemaphorePost(T_PsdkSemHandle semaphore)
{
	rt_sem_release((rt_sem_t ) semaphore);
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_PsdkReturnCode Osal_GetTimeMs(uint32_t *ms)
{
	*ms=rt_tick_get()/RT_TICK_PER_SECOND;
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void *Osal_Malloc(uint32_t size)
{
    return rt_malloc(size);
}

void Osal_Free(void *ptr)
{

}
static T_PsdkReturnCode PsdkUser_FillInUserInfo(T_PsdkUserInfo *userInfo)
{
    memset(userInfo->appName, 0, sizeof(userInfo->appName));
    memset(userInfo->appId, 0, sizeof(userInfo->appId));
    memset(userInfo->appKey, 0, sizeof(userInfo->appKey));
    memset(userInfo->developerAccount, 0, sizeof(userInfo->developerAccount));

    if (strlen(USER_APP_NAME) >= sizeof(userInfo->appName) ||
        strlen(USER_APP_ID) > sizeof(userInfo->appId) ||
        strlen(USER_APP_KEY) > sizeof(userInfo->appKey) ||
        strlen(USER_DEVELOPER_ACCOUNT) >= sizeof(userInfo->developerAccount)) {
		rt_kprintf("Length of user information string is beyond limit!!! \n");
        return PSDK_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    if (!strcmp(USER_APP_NAME, "your_app_name") ||
        !strcmp(USER_APP_ID, "your_app_id") ||
        !strcmp(USER_APP_KEY, "your_app_key") ||
        !strcmp(USER_DEVELOPER_ACCOUNT, "your_developer_account")) {
		rt_kprintf("Please fill in correct user information!!! \n");
        return PSDK_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    strncpy(userInfo->appName, USER_APP_NAME, sizeof(userInfo->appName) - 1);
    memcpy(userInfo->appId, USER_APP_ID, USER_UTIL_MIN(sizeof(userInfo->appId), strlen(USER_APP_ID)));
    memcpy(userInfo->appKey, USER_APP_KEY, USER_UTIL_MIN(sizeof(userInfo->appKey), strlen(USER_APP_KEY)));
    strncpy(userInfo->developerAccount, USER_DEVELOPER_ACCOUNT, sizeof(userInfo->developerAccount) - 1);

    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

int main(void)
{	
    int count = 1;
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
	rt_kprintf("000 error!!! \n");
	T_PsdkUserInfo userInfo;
	T_PsdkHalUartHandler halUartHandler = {
        .UartInit = Hal_UartInit,
        .UartWriteData = Hal_UartSendData,
        .UartReadData = Hal_UartReadData,
    };
	T_PsdkOsalHandler osalHandler = {
        .TaskCreate = Osal_TaskCreate,
        .TaskDestroy = Osal_TaskDestroy,
        .TaskSleepMs = Osal_TaskSleepMs,
        .MutexCreate = Osal_MutexCreate,
        .MutexDestroy = Osal_MutexDestroy,
        .MutexLock = Osal_MutexLock,
        .MutexUnlock = Osal_MutexUnlock,
        .SemaphoreCreate = Osal_SemaphoreCreate,
        .SemaphoreDestroy = Osal_SemaphoreDestroy,
        .SemaphoreWait = Osal_SemaphoreWait,
        .SemaphoreTimedWait = Osal_SemaphoreTimedWait,
        .SemaphorePost = Osal_SemaphorePost,
        .GetTimeMs = Osal_GetTimeMs,
        .Malloc = Osal_Malloc,
        .Free = Osal_Free,
    };
	rt_kprintf("111 error!!! \n");
	if (PsdkPlatform_RegHalUartHandler(&halUartHandler) != PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		rt_kprintf("PsdkPlatform_RegHalUartHandler error!!! \n");
	}
	rt_kprintf("222 error!!! \n");
	if (PsdkUser_FillInUserInfo(&userInfo) != PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS) 
	{
        rt_kprintf("PsdkUser_FillInUserInfo error!!! \n");
    }
	rt_kprintf("333 error!!! \n");
	if (PsdkCore_Init(&userInfo) != PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS) 
	{
        rt_kprintf("PsdkCore_Init error!!! \n");
    }
	rt_kprintf("444 error!!! \n");
	if (PsdkProductInfo_SetAlias("ZINGTO_SUN_TEST") != PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS) 
	{
        rt_kprintf("PsdkProductInfo_SetAlias error!!! \n");
    }
	rt_kprintf("555 error!!! \n");
	if (PsdkCore_ApplicationStart() != PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS) 
	{
        rt_kprintf("PsdkCore_ApplicationStart error!!! \n");
    }
	rt_kprintf("666 error!!! \n");

    while (1)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }

    return RT_EOK;
}
