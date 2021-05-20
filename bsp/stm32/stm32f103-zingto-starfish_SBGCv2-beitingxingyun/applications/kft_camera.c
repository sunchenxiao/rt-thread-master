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

#define DBG_ENABLE
#define DBG_SECTION_NAME "Camera"
#define DBG_LEVEL DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

#define CAMERA_UARTPORT_NAME "uart4"
#define CAMERA_SEMAPHORE_NAME "shCAM"

#define CAMERA_BUFFER_SIZE              (128)
#define CAMERA_RX_TIMEOUT               (10)

#define ZINGTO_PKT_HEADER				(0xFF)
#define ZINGTO_PKT_SIZE   				(7)

static rt_sem_t semaph = RT_NULL;

static rt_err_t uart_hook_callback(rt_device_t dev, rt_size_t sz)
{
    rt_sem_release(semaph);
    
    return RT_EOK;
}

void camera_resolving_entry(void* parameter)
{
    rt_device_t dev = RT_NULL;
	rt_device_t dev2 = RT_NULL;
    struct guardian_environment *env = RT_NULL;
    rt_err_t result = RT_EOK;
    rt_uint8_t *pbuf = RT_NULL;
    rt_size_t szbuf = 0;
    rt_uint8_t opcode, speedlv;
    
    env = (struct guardian_environment *)parameter;
    RT_ASSERT(env != RT_NULL);
    
    pbuf = rt_malloc(CAMERA_BUFFER_SIZE);
    RT_ASSERT(pbuf != RT_NULL);
    
    dev = rt_device_find(CAMERA_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
    rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
	
	dev2 = rt_device_find("uart2");
    RT_ASSERT(dev2 != RT_NULL);

    semaph = rt_sem_create(CAMERA_SEMAPHORE_NAME, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(semaph != RT_NULL);
    
    // set uart in 115200, 8N1.
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &config);
    
    rt_device_set_rx_indicate(dev, uart_hook_callback);
	
	env->send_flag=0;
	env->ask_laser_distance=0;
    
    LOG_I("initialization finish, start!");

    while (1)
    {
        result = rt_sem_take(semaph, RT_WAITING_FOREVER);
        
        if(result == -RT_ETIMEOUT)
            continue;
        
        switch (szbuf){
        case 0:
        case 1:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, 1);
            if (pbuf[0] != ZINGTO_PKT_HEADER)
                szbuf = 0;
            break;
        default:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, ZINGTO_PKT_SIZE - szbuf);
            break;
        }
        // should nerver happened
        if (szbuf != ZINGTO_PKT_SIZE)
            continue;
        
        szbuf = 0;
        
        rt_bool_t ptz_request = RT_FALSE;
        rt_bool_t cam_request = RT_FALSE;
        rt_bool_t trck_request = RT_FALSE;
        rt_uint32_t cam_eval;
        
        if(pbuf[2]==0x00&&pbuf[3]==0x00) //stop	
		{
			env->ch_value[0] = SBUS_VALUE_MEDIAN;
            env->ch_value[1] = SBUS_VALUE_MEDIAN;
            env->ch_value[3] = SBUS_VALUE_MEDIAN;
			env->ptz_action = PANTILT_ACTION_NULL;
			ptz_request = RT_TRUE;
		}
		else if(pbuf[2]==0x00&&pbuf[3]==0x10) //pitch up
		{
			env->ch_value[1] = SBUS_VALUE_MEDIAN + (SBUS_VALUE_MAXIMUM - SBUS_VALUE_MEDIAN) * pbuf[5] / 200.f;
			env->ptz_action = PANTILT_ACTION_NULL;
			ptz_request = RT_TRUE;
		}
		else if(pbuf[2]==0x00&&pbuf[3]==0x08) //pitch down
		{
			env->ch_value[1] = SBUS_VALUE_MEDIAN + (SBUS_VALUE_MININUM - SBUS_VALUE_MEDIAN) * pbuf[5] / 200.f;
			env->ptz_action = PANTILT_ACTION_NULL;
			ptz_request = RT_TRUE;
		}
		else if(pbuf[2]==0x00&&pbuf[3]==0x02) //yaw left
		{
			env->ch_value[3] = SBUS_VALUE_MEDIAN + (SBUS_VALUE_MAXIMUM - SBUS_VALUE_MEDIAN) * pbuf[4] / 200.f;
			env->ptz_action = PANTILT_ACTION_NULL;
			ptz_request = RT_TRUE;
		}
		else if(pbuf[2]==0x00&&pbuf[3]==0x04) //yaw right
		{
			env->ch_value[3] = SBUS_VALUE_MEDIAN + (SBUS_VALUE_MININUM - SBUS_VALUE_MEDIAN) * pbuf[4] / 200.f;
			env->ptz_action = PANTILT_ACTION_NULL;
			ptz_request = RT_TRUE;
		}
		else if(pbuf[2]==0x11&&pbuf[3]==0x00) //point track
		{
			rt_device_write(dev2, 0, pbuf, ZINGTO_PKT_SIZE);
		}
		else if(pbuf[2]==0x11&&pbuf[3]==0x01) //stop track
		{
			rt_device_write(dev2, 0, pbuf, ZINGTO_PKT_SIZE);
		}
		else if(pbuf[2]==0x13&&pbuf[3]==0x00) //header free
		{
			env->ptz_mode = PANTILT_MODE_HEADFREE;
            ptz_request = RT_TRUE;
		}
		else if(pbuf[2]==0x13&&pbuf[3]==0x01) //header lock
		{
			env->ptz_mode = PANTILT_MODE_HEADLOCK;
            ptz_request = RT_TRUE;
		}
		else if(pbuf[2]==0x13&&pbuf[3]==0x02) //header down
		{
			env->ptz_mode = PANTILT_MODE_HEADDOWN;
            ptz_request = RT_TRUE;
		}
		else if(pbuf[2]==0x13&&pbuf[3]==0x03) //homing
		{
			env->ptz_action = PANTILT_ACTION_HOMING;
            ptz_request = RT_TRUE;
		}
		else
		{
			
		}
        
        if (ptz_request == RT_TRUE)
        {
            ptz_request = RT_FALSE;
            if (env->sh_ptz != RT_NULL)
            {
                env->user_incharge = RT_TRUE;
                rt_sem_release(env->sh_ptz); // notify the PanTiltZoom.
            }
        }
        else {
            env->user_incharge = RT_FALSE;
        }
        
        if (trck_request == RT_TRUE)
        {
            trck_request = RT_FALSE;
            if (env->sh_track != RT_NULL)
                rt_sem_release(env->sh_track);
        }
        
        if (cam_request == RT_TRUE)
        {
            cam_request = RT_FALSE;
            if (env->ev_camera != RT_NULL)
                rt_event_send(env->ev_camera, cam_eval); // notify the Camera.
        }        
        
    }
    
    // never be here.
}

