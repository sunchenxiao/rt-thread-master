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

#define TRACK_BUFFER_SIZE       (128)
#define MIN_RX_INTERVAL    	    (10)

#define TRACK_PKT_SIZE			(11)
#define TRACK_PKT_HEADER		(0x55)

static rt_sem_t semaph = RT_NULL;

rt_err_t uart_hook_callback(rt_device_t dev, rt_size_t sz)
{
    rt_sem_release(semaph);
    
    return RT_EOK;
}

void track_resolving_entry(void* parameter)
{
    static rt_device_t dev = RT_NULL;
    static rt_uint8_t *pbuf = RT_NULL;
    static struct guardian_environment *env = RT_NULL;
    rt_size_t szbuf = 0;
    static rt_err_t result = RT_EOK;
    
    env = (struct guardian_environment *)parameter;
    RT_ASSERT(env != RT_NULL);
	
	pbuf = rt_malloc(TRACK_BUFFER_SIZE);
    RT_ASSERT(pbuf != RT_NULL);
    
    dev = rt_device_find(TRACK_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
	
	rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    semaph = rt_sem_create(TRACK_SEMAPHORE_NAME, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(semaph != RT_NULL);

    rt_device_set_rx_indicate(dev, uart_hook_callback);

    while (1)
    {
		result = rt_sem_take(semaph, RT_WAITING_FOREVER);
        
        if(result == -RT_ETIMEOUT)
            continue;
        
        switch (szbuf){
        case 0:
        case 1:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, 1);
            if (pbuf[0] != TRACK_PKT_HEADER)
                szbuf = 0;
            break;
        default:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, TRACK_PKT_SIZE - szbuf);
            break;
        }
        // should nerver happened
        if (szbuf != TRACK_PKT_SIZE)
            continue;
        
        szbuf = 0;
        
        if (pbuf[1] == 0x53)
		{
			env->droneyaw_L=pbuf[6];
			env->droneyaw_H=pbuf[7];
			env->droneyaw=(float)(pbuf[6]+pbuf[7]*256)/32768*180;
		}
		else if(pbuf[1]==0x56)
		{
			env->dronealt=(float)((pbuf[6]+pbuf[7]*256)+(pbuf[8]+pbuf[9]*256)*65536)/100;
		}
		else if(pbuf[1]==0x57)
		{
			rt_uint32_t dronelat1=(pbuf[6]+pbuf[7]*256)+(pbuf[8]+pbuf[9]*256)*65536;
			rt_uint32_t dronelng1=(pbuf[2]+pbuf[3]*256)+(pbuf[4]+pbuf[5]*256)*65536;
			env->dronelat=(float)(dronelat1/10000000)+(float)(dronelat1%10000000)/100000/60;
			env->dronelng=(float)(dronelng1/10000000)+(float)(dronelng1%10000000)/100000/60;
//			env->dronelat=(float)((pbuf[6]+pbuf[7]*256)+(pbuf[8]+pbuf[9]*256)*65536)*0.0000001f;
//			env->dronelng=(float)((pbuf[2]+pbuf[3]*256)+(pbuf[4]+pbuf[5]*256)*65536)*0.0000001f;
			
//			env->dronelat=(float)(dronelat1/10000000)+(float)((dronelat1%10000000)/100000)/100+(float)((dronelat1%100000))/10000000;
//			env->dronelng=(float)(dronelng1/10000000)+(float)((dronelng1%10000000)/100000)/100+(float)((dronelng1%100000))/10000000;
		}
		else
		{
			continue;
		}     
    }
    
    // never be here.
}

