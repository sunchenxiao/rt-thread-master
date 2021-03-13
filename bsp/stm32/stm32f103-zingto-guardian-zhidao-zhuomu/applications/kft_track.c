/*
 * Copyright (c) 2006-2018, ZINGTO UAV
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-03-04     serni        first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "guardian.h"
#include "general_pid.h"

#define DBG_ENABLE
#define DBG_SECTION_NAME "Track"
#define DBG_LEVEL DBG_LOG
#define DBG_COLOR
#include <rtdbg.h>

#define TRACK_UARTPORT_NAME "uart3"
#define TRACK_SEMAPHORE_NAME "shTRCK"
#define TRACK_SEMAPHORE_RX_NAME "shTRCKrx"

#define TRACK_SEND_MP_NAME "mpPTZtx"
#define TRACK_SEND_MB_NAME "mbPTZtx"

/* defined the LED pin: PA0 */
#define LED_PIN    GET_PIN(A, 0)

#define TRACK_BUFFER_SIZE       (64)

#define TRACK_ACK_PKT_SIZE      (13)
#define TRACK_ACK_PKT_HEADER    (0x91)
#define TRACK_ACK_PKT_ADDR      (0x0D)

#define TRACK_SIZE     	 (18)
#define DETECT_SIZE      (6)
rt_uint8_t TRACK_PREPARE_XY[] 	= {0x90,0xeb,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x82,0x38,0x03,0x28,0x02,0x74};
rt_uint8_t TRACK_PREPARE_WH[] 	= {0x90,0xeb,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x83,0x60,0x00,0x60,0x00,0xd0};
rt_uint8_t TRACK_START[] 		= {0x90,0xeb,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x60,0x00,0x60,0x00,0xcd};
rt_uint8_t TRACK_STOP[] 		= {0x90,0xeb,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x81,0x60,0x00,0x60,0x00,0xce};
rt_uint8_t DETECT_START[] 		= {0x90,0xeb,0x06,0x84,0x01,0x06};
rt_uint8_t DETECT_STOP[] 		= {0x90,0xeb,0x06,0x84,0x00,0x05};
rt_uint8_t DETECT_TRACK_START[] = {0x90,0xeb,0x06,0x92,0x00,0x13};
rt_uint8_t DETECT_TRACK_STOP[] 	= {0x90,0xeb,0x06,0x92,0x01,0x14};

static rt_sem_t semaph = RT_NULL;

static rt_mailbox_t mailbox = RT_NULL;
static rt_mp_t mempool = RT_NULL;

rt_err_t uart_hook_callback(rt_device_t dev, rt_size_t sz)
{
    rt_sem_release(semaph);    
    return RT_EOK;
}

static void track_data_recv_entry(void* parameter)
{
    struct guardian_environment *env = RT_NULL;
    rt_uint8_t* pbuf;
    rt_size_t szbuf = 0;;
    rt_err_t result;
    rt_device_t dev = RT_NULL;
    rt_size_t pktsz;
    rt_bool_t on_tracing = RT_FALSE;
    
    PID_t pid_x, pid_y;
    
    env = (struct guardian_environment*)parameter;
    RT_ASSERT(env != RT_NULL);
    
    pbuf = rt_malloc(TRACK_BUFFER_SIZE);
    RT_ASSERT(pbuf != RT_NULL);
    
    dev = rt_device_find(TRACK_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
		
    pid_init(&pid_x, 0.7f, 0.035f, 0.8f);
    pid_init(&pid_y, 0.7f, 0.035f, 0.8f);
    
    pid_setThreshold(&pid_x, 250.0f, 250.0f, 0.02f);
    pid_setThreshold(&pid_y, 250.0f, 250.0f, 0.02f);
    
    pid_setSetpoint(&pid_x, 0.0f);
    pid_setSetpoint(&pid_y, 0.0f);
    
    pid_enable(&pid_x, RT_TRUE);
    pid_enable(&pid_y, RT_TRUE);
    
    LOG_I("recv sub-thread, start!");
    
    while (1)
    {
        result = rt_sem_take(semaph, RT_WAITING_FOREVER);
        
        if(result == -RT_ETIMEOUT)
            continue; 
        
        switch (szbuf){
        case 0:
        case 1:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, 1);
            if (pbuf[0] != TRACK_ACK_PKT_HEADER)
                szbuf = 0;
            break;
		case 2:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, 1);
            if (pbuf[1] != TRACK_ACK_PKT_ADDR)
                szbuf = 0;
            break;
		case 3:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, 1);
            if (pbuf[2] != 0x3f)
                szbuf = 0;
            break;
        default:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, TRACK_ACK_PKT_SIZE - szbuf);
            break;
        }
        // should never happened.
        if (szbuf != TRACK_ACK_PKT_SIZE)
            continue;
        
        szbuf = 0;
		
		float deffer_x, deffer_y;
		
		deffer_x = (float)*(rt_int16_t*)&pbuf[4]-960;
		deffer_y = (float)*(rt_int16_t*)&pbuf[6]-540;
		
		env->trck_err_x = pid_update(&pid_x, deffer_x);
		env->trck_err_y = pid_update(&pid_y, deffer_y);
		
		if (pbuf[3] == 0x02||pbuf[3] == 0x03)
		{
			env->trck_err_x = 0;
            env->trck_err_y = 0;
			rt_sem_release(env->sh_ptz);
			
			rt_thread_mdelay(100);
			
            on_tracing = RT_FALSE;     
            env->trck_incharge = RT_FALSE;
            env->trck_lost = RT_TRUE;
		}
            
        rt_sem_release(env->sh_ptz);
        
    }
}

void track_resolving_entry(void* parameter)
{
    static rt_device_t dev = RT_NULL;
    static struct guardian_environment *env = RT_NULL;
    rt_thread_t pthread = RT_NULL;
    static rt_err_t result = RT_EOK;
    
    env = (struct guardian_environment *)parameter;
    RT_ASSERT(env != RT_NULL);
    
    dev = rt_device_find(TRACK_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
	
	rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    semaph = rt_sem_create(TRACK_SEMAPHORE_RX_NAME, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(semaph != RT_NULL);
    env->sh_track = rt_sem_create(TRACK_SEMAPHORE_NAME, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(env->sh_track != RT_NULL);
    
    mempool = rt_mp_create(TRACK_SEND_MP_NAME, 8, TRACK_BUFFER_SIZE);
    RT_ASSERT(mempool != RT_NULL);
    mailbox = rt_mb_create(TRACK_SEND_MB_NAME, 8, RT_IPC_FLAG_FIFO);
    RT_ASSERT(mailbox != RT_NULL);
    
    rt_device_set_rx_indicate(dev, uart_hook_callback);

    pthread = rt_thread_create("tTRCKrx", track_data_recv_entry, env, 2048, 9, 20);
    RT_ASSERT(pthread != RT_NULL);
    result = rt_thread_startup(pthread);
    RT_ASSERT(result == RT_EOK);
    
    LOG_I("initialization finish, start!");
    
    env->trck_incharge = RT_FALSE;

    while (1)
    {
        result = rt_sem_take(env->sh_track, RT_WAITING_FOREVER);
        
        if (env->trck_action == TRACK_ACTION_POINT_START)
        {
			TRACK_PREPARE_XY[13]=(env->trck_offset_x-30)%256;
			TRACK_PREPARE_XY[14]=(env->trck_offset_x-30)/256;
			TRACK_PREPARE_XY[15]=(env->trck_offset_y-30)%256;
			TRACK_PREPARE_XY[16]=(env->trck_offset_y-30)/256;
			TRACK_PREPARE_XY[17]=0;
			for(int i = 0; i < TRACK_SIZE - 1; i++)
				TRACK_PREPARE_XY[17] += TRACK_PREPARE_XY[i];      
			rt_device_write(dev, 0, TRACK_PREPARE_XY,TRACK_SIZE);
			
			rt_thread_mdelay(100);
			
			rt_device_write(dev, 0, TRACK_PREPARE_WH,TRACK_SIZE);
			
			rt_thread_mdelay(100);

			rt_device_write(dev, 0, TRACK_START,TRACK_SIZE);
        
			env->trck_action = TRACK_ACTION_NULL;
			continue;
        }
		else if (env->trck_action == TRACK_ACTION_TRACE_STOP)
        {
			env->trck_prepare = RT_FALSE;
            env->trck_incharge = RT_FALSE;
            rt_device_write(dev, 0, TRACK_STOP,TRACK_SIZE);
        }
		else if (env->trck_action == TRACK_ACTION_DETECT_START)
        {
            rt_device_write(dev, 0, DETECT_START,DETECT_SIZE);
        }
		else if (env->trck_action == TRACK_ACTION_DETECT_STOP)
        {
            rt_device_write(dev, 0, DETECT_STOP,DETECT_SIZE);
        }
		else if (env->trck_action == TRACK_ACTION_DETECT_TRACK_START)
        {
            rt_device_write(dev, 0, DETECT_TRACK_START,DETECT_SIZE);
        }
		else if (env->trck_action == TRACK_ACTION_DETECT_TRACK_STOP)
        {
            rt_device_write(dev, 0, DETECT_TRACK_STOP,DETECT_SIZE);
        }
        else
            continue;
         
        env->trck_action = TRACK_ACTION_NULL;
    }
    
    // never be here.
}

