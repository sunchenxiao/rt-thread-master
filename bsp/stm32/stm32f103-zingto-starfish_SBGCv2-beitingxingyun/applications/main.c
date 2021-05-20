/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "guardian.h"

/* defined the PoWer SiWtch pin: PC4 */
#define PWSW_PIN        GET_PIN(C, 4)

#define LED_PIN         GET_PIN(A, 15)

static  rt_device_t pWDT = RT_NULL;

static struct guardian_environment env;

static void idle_hook(void)
{
    rt_device_control(pWDT, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
//    rt_kprintf("feed the dog!\n ");
}

int main(void)
{
	rt_thread_t pthread = RT_NULL;	
    rt_err_t result = RT_EOK;
    rt_size_t       timeout = 1;
    
    pWDT = rt_device_find("wdt");
    RT_ASSERT(pWDT != RT_NULL);
    
    result = rt_device_init(pWDT);
    result = rt_device_control(pWDT, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout);
    result = rt_device_control(pWDT, RT_DEVICE_CTRL_WDT_START, RT_NULL);
    rt_thread_idle_sethook(idle_hook);
    rt_kprintf("Set WatchDog Complate!\n");
    
    /* initialization */
    
    rt_memset(&env, 0x00, sizeof(struct guardian_environment));
	for (int i = 0; i < SBUS_CHANNEL_NUMBER; i++)
    {
        env.ch_value[i] = SBUS_VALUE_MEDIAN;
        env.ch_status[i] = SBUS_IDLE;
    }
	
	pthread = rt_thread_create("tPTZ", pantilt_resolving_entry, &env, 2048, 6, 20);
    RT_ASSERT(pthread != RT_NULL);
    result = rt_thread_startup(pthread);
    RT_ASSERT(result == RT_EOK);
	
	pthread = rt_thread_create("tZINGTO", zingto_resolving_entry, &env, 2048, 3, 20);
	RT_ASSERT(pthread != RT_NULL);
	result = rt_thread_startup(pthread);
	RT_ASSERT(result == RT_EOK);
	
	pthread = rt_thread_create("tCamera", camera_resolving_entry, &env, 2048, 9, 50);
    RT_ASSERT(pthread != RT_NULL);
    result = rt_thread_startup(pthread);
    RT_ASSERT(result == RT_EOK);
    
    /* set LED0 pin mode to output */
    rt_pin_write(PWSW_PIN, PIN_HIGH);
    rt_pin_mode (PWSW_PIN, PIN_MODE_OUTPUT);
    
    rt_pin_mode (LED_PIN, PIN_MODE_OUTPUT);
    
    
    while (1)
    {
        rt_pin_write(LED_PIN, PIN_HIGH);
        rt_thread_mdelay(RT_TICK_PER_SECOND / 5);
        rt_pin_write(LED_PIN, PIN_LOW);
        rt_thread_mdelay(RT_TICK_PER_SECOND * 4 / 5);
    }

    // Never reach here.
}
