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
#include <stdlib.h>
#include <math.h>

#include "guardian.h"

#define DBG_ENABLE
#define DBG_SECTION_NAME "PanTilt"
#define DBG_LEVEL DBG_INFO       // DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

#pragma pack(1)
typedef struct __PTZ_SendPacket
{
    rt_uint32_t     HEADER;             // 4
    rt_int16_t      roll;               // 6  #1
    rt_int16_t      pitch;              // 8  #2
    rt_uint16_t     __reserved1;        // 10 #3
    rt_int16_t      yaw;                // 12 #4
    rt_uint16_t     mode;               // 14 #5
    rt_uint16_t     __reserved2;        // 16 #6
    rt_uint16_t     __reserved3;        // 18 #7
    rt_uint16_t     homing;             // 20 #8
    rt_uint16_t     __reserved4;        // 22
    rt_uint8_t      __reserved5[68-22]; // 68
    rt_uint8_t      checksum;           // 69
}ptz_serialctrlpkt;
#pragma pack(4)

//30
const float ZOOM2RATIO[30] = {  1.0f,   0.6f,   0.5f,   0.4f,   0.35f,   0.3f,
                                0.3f,   0.3f,   0.3f,   0.25f,  0.25f,  0.25f,
                                0.2f,   0.2f,   0.2f,   0.15f,  0.15f,  0.15f,
                                0.1f,   0.1f,   0.1f,   0.1f,   0.1f,   0.1f,
                                0.1f,   0.1f,   0.1f,   0.1f,   0.1f,   0.1f
                             };
const float IR2RATIO = 0.2f;

#define IRSENSOR_COLOR_PKT_SIZE (9)                    
                             
rt_uint8_t irs_serialctrlpkt[IRSENSOR_COLOR_PKT_SIZE] = {0xAA, 0x05, 0x01, 0x42, 0x02, 0x00, 0xF4, 0xEB, 0xAA};

rt_uint8_t laser_a5[1] = {0xA5};

rt_uint8_t laser_dis[6] = {0xEE, 0x16, 0x02, 0x03, 0x02, 0x05};

rt_uint8_t laser1_dis[8] = {0xAE, 0xA7, 0x04, 0x00, 0x05, 0x09, 0xBC, 0xBE};

#define PTZ_ASK_PKT_SIZE (5)  

rt_uint8_t ptz_askctrlpkt[PTZ_ASK_PKT_SIZE] = {0x3e,0x19,0x00,0x19,0x00};

#define SEND_PKT_DATA_SIZE		(9)
rt_uint8_t send_angle_data[SEND_PKT_DATA_SIZE]={0xEE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

#define IRSENSOR_ZOOM_PKT_SIZE  (16)

rt_uint8_t irs_zoom[8][IRSENSOR_ZOOM_PKT_SIZE] = {
    {0xAA, 0x0C, 0x01, 0x40, 0x02, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x01, 0x1F, 0x01, 0x99, 0xEB, 0xAA},
    {0xAA, 0x0C, 0x01, 0x40, 0x02, 0x60, 0x00, 0x48, 0x00, 0x1F, 0x01, 0xD7, 0x00, 0x98, 0xEB, 0xAA},
    {0xAA, 0x0C, 0x01, 0x40, 0x02, 0x80, 0x00, 0x60, 0x00, 0xFF, 0x00, 0xBF, 0x00, 0x97, 0xEB, 0xAA},
    {0xAA, 0x0C, 0x01, 0x40, 0x02, 0x90, 0x00, 0x6C, 0x00, 0xEF, 0x00, 0xB3, 0x00, 0x97, 0xEB, 0xAA},
    {0xAA, 0x0C, 0x01, 0x40, 0x02, 0x9A, 0x00, 0x73, 0x00, 0xE5, 0x00, 0xAB, 0x00, 0x96, 0xEB, 0xAA},
    {0xAA, 0x0C, 0x01, 0x40, 0x02, 0xA0, 0x00, 0x78, 0x00, 0xDF, 0x00, 0xA7, 0x00, 0x97, 0xEB, 0xAA},
    {0xAA, 0x0C, 0x01, 0x40, 0x02, 0xA5, 0x00, 0x7B, 0x00, 0xDA, 0x00, 0xA3, 0x00, 0x96, 0xEB, 0xAA},
    {0xAA, 0x0C, 0x01, 0x40, 0x02, 0xA8, 0x00, 0x7E, 0x00, 0xD7, 0x00, 0xA1, 0x00, 0x97, 0xEB, 0xAA}
};

#define PANTILT_CALIB_PKT_SIZE (5)

rt_uint8_t calib_protcol[4][PANTILT_CALIB_PKT_SIZE] = {
    {0xE1, 0x1E, 0x08, 0xF1, 0x1F},
    {0xE1, 0x1E, 0x09, 0xF1, 0x1F},
    {0xE1, 0x1E, 0x08, 0xF1, 0x1F},
    {0xE1, 0x1E, 0x09, 0xF1, 0x1F},
};
        

#define PANTILT_UARTPORT_NAME "uart1"
#define PANTILT_SEMAPHORE_NAME "shPTZ"
#define PANTILT_SEMAPHORE_RX_NAME "shPTZrx"

#define PANTILT_SEND_MP_NAME "mpPTZtx"
#define PANTILT_SEND_MB_NAME "mbPTZtx"

#define PANTILT_BUFFER_SIZE     	(128)
#define PANTILT_RX_TIMEOUT      	(10)

#define PANTILT_PKT_HEADER          (0x6D402D3E)
#define IRSENSOR_COLOR_PKT_HEADER   (0x05AA)
#define IRSENSOR_ZOOM_PKT_HEADER    (0x0CAA)
#define PTZ_ASK_PKT_HEADER    		(0x193E)

#define PANTILT_VALUE_MAXIMUM   	(500)
#define PANTILT_VALUE_MININUM   	(-500)

//30???
#define PANTILT_VALUE_RATIO     	(0.7142857f)

#define ANSWER_PKT_HEADER0			(0x3E)
#define ANSWER_PKT_SIZE0			(129)
#define ANSWER_PKT_HEADER1			(0xEE)
#define ANSWER_PKT_SIZE1			(10)
#define ANSWER_PKT_HEADER2			(0xAE)
#define ANSWER_PKT_HEADER3			(0xA7)

/* defined the LED pin: PA0 */
#define LED_PIN    GET_PIN(A, 0)

static rt_sem_t semaph = RT_NULL;
static rt_mailbox_t mailbox = RT_NULL;
static rt_mp_t      mempool = RT_NULL;

static rt_err_t pantilt_update_checksum(ptz_serialctrlpkt *pkt)
{
    rt_uint8_t *ptr = (rt_uint8_t*)pkt;
    rt_size_t pktsz = sizeof(ptz_serialctrlpkt);
    
    for (int i = 4; i < pktsz - 1; i++)
        pkt->checksum += *(ptr + i);
    
    return RT_EOK;
}

static rt_err_t uart_hook_callback(rt_device_t dev, rt_size_t sz)
{
    rt_sem_release(semaph);
    
    return RT_EOK;
}

static void pantilt_data_send_entry(void* parameter)
{
    struct guardian_environment *env = RT_NULL;
    rt_ubase_t mail;
    rt_uint8_t* pbuf;
    rt_device_t dev = RT_NULL;
    rt_uint32_t ubase32 = 0;
    rt_uint16_t ubase16 = 0;
    
    env = (struct guardian_environment*)parameter;
    RT_ASSERT(env != RT_NULL);
    
    dev = rt_device_find(PANTILT_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
    
    LOG_I("send sub-thread, start!");
    
    while (1)
    {
        rt_mb_recv(mailbox, &mail, RT_WAITING_FOREVER);
        LOG_D("mb recv %0X", mail);
        if (mail == RT_NULL)
            continue;
        pbuf = (rt_uint8_t*)mail;
        
        ubase32 = *(rt_uint32_t*)pbuf;
        ubase16 = *(rt_uint16_t*)pbuf;
        
        if (ubase32 == PANTILT_PKT_HEADER)
        {
            LOG_D("send to pantilt");
            rt_device_write(dev, 0, pbuf, sizeof(ptz_serialctrlpkt));
        }
        else if(ubase16 == IRSENSOR_COLOR_PKT_HEADER)
        {
            LOG_D("send to irsensor color");
            rt_device_write(dev, 0, pbuf, IRSENSOR_COLOR_PKT_SIZE);
        }
        else if(ubase16 == IRSENSOR_ZOOM_PKT_HEADER)
        {
            LOG_D("send to irsensor zoom");
            rt_device_write(dev, 0, pbuf, IRSENSOR_ZOOM_PKT_SIZE);
        }
		else if(ubase16 == PTZ_ASK_PKT_HEADER)
        { 
			rt_device_write(dev, 0, ptz_askctrlpkt, PTZ_ASK_PKT_SIZE);
			//rt_kprintf("ask_500ms \n");
			
			//Erboli jiguangqi
//            rt_device_write(dev, 0, ptz_askctrlpkt, PTZ_ASK_PKT_SIZE);
//			rt_thread_mdelay(20);
//			rt_device_write(dev, 0, laser_a5, 1);
//			rt_thread_mdelay(20);
//			rt_device_write(dev, 0, laser_dis, 6);
			
			//laokuan jiguangqi
//			rt_device_write(dev, 0, ptz_askctrlpkt, PTZ_ASK_PKT_SIZE);
//			rt_thread_mdelay(30);
//			rt_device_write(dev, 0, laser1_dis, 8);
        }
        
        rt_mp_free(pbuf);
        
        rt_thread_delay(1);
    }
}

static void pantilt_data_recv_entry(void* parameter)
{
//    struct guardian_environment *env = RT_NULL;
//    rt_uint8_t* pbuf;
//    rt_device_t dev = RT_NULL;
//	rt_err_t result = RT_EOK;
//	rt_size_t szbuf = 0;
//    
//    env = (struct guardian_environment*)parameter;
//    RT_ASSERT(env != RT_NULL);
//    
//    pbuf = rt_malloc(PANTILT_BUFFER_SIZE);
//    RT_ASSERT(pbuf != RT_NULL);
//    
//    dev = rt_device_find(PANTILT_UARTPORT_NAME);
//    RT_ASSERT(dev != RT_NULL);
//  
//    LOG_I("recv sub-thread, start!");
// 
//    while (1)
//    {
//		result = rt_sem_take(semaph, 5);
//        
//        if(result != -RT_ETIMEOUT) {
//            /* read 1 byte form uart fifo ringbuffer */
//            szbuf += rt_device_read(dev, 0, pbuf + szbuf, 1);
//            continue;
//        }
//        else {
//            if (szbuf == 0)
//                continue;       // ignore idle frame.
//        }
//		if(pbuf[0] == ANSWER_PKT_HEADER0 && szbuf > (ANSWER_PKT_SIZE0-1))
//		{
//			//yaw 71 72     pitch 69 70
//			send_angle_data[1]=0;
//			send_angle_data[2]=0;
//			send_angle_data[3]=pbuf[69];
//			send_angle_data[4]=pbuf[70];
//			send_angle_data[5]=pbuf[71];
//			send_angle_data[6]=pbuf[72];
//			send_angle_data[7]=env->cam_zoom_pos;
//			//rt_kprintf("receive_500ms  %d  %d  \n",send_angle_data[5],send_angle_data[6]);
//			rt_int16_t  temp = 0;
//			temp = *(rt_int16_t*)&pbuf[71];
//			float yaw = temp * 0.02197265625f;
//			yaw = yaw - floor(yaw / 360.0f) * 360.0f;
//			if ( yaw > 180.0f ) yaw = yaw - 360.0f;// range -180 and 180 degree.
//			
//			env->ptz_yaw = yaw;
//			
//			temp = *(rt_int16_t*)&pbuf[69];
//			float pitch = temp * 0.02197265625f;
//			pitch = pitch - floor(pitch / 360.0f) * 360.0f;
//			if ( pitch > 180.0f ) pitch = pitch - 360.0f;
//			pitch = pitch * -1.0f;
//			
//			env->ptz_pitch = pitch;
//			
//			env->ptz_roll = .0f;
//			
//			LOG_D("PTZ: %d %d", *(rt_int16_t*)&pbuf[69], *(rt_int16_t*)&pbuf[71]);
//			
//			//rt_device_write(dev2, 0, send_angle_data, SEND_PKT_DATA_SIZE);
//		}
//		//Erboli jiguangqi
////		else if(pbuf[0] == ANSWER_PKT_HEADER1 && szbuf == ANSWER_PKT_SIZE1)
////		{
////			send_data[0]=pbuf[7];
////			send_data[1]=pbuf[6];
////			rt_device_write(dev5, 0, send_data, 9);
////		}
//		//laokuan jiguangqi
////		else if(pbuf[0] == ANSWER_PKT_HEADER2 && pbuf[1] == ANSWER_PKT_HEADER3 && szbuf > 8)
////		{
////			send_data[0]=pbuf[8];
////			send_data[1]=pbuf[7];
////			
////			env->laser_dis=((float)(pbuf[7]*256+pbuf[8]))/10;
////			
////			rt_device_write(dev2, 0, send_data, 9);
////		}
//		else
//		{
//			
//		}
//		
//		szbuf = 0;
//		
//    }
}

void pantilt_resolving_entry(void* parameter)
{
    rt_device_t dev = RT_NULL;
    struct guardian_environment *env = RT_NULL;
    ptz_serialctrlpkt ctrlpkt;
    rt_uint8_t *pbuf = RT_NULL;
    rt_size_t   pktsz;
    rt_err_t result = RT_EOK;
    rt_thread_t pthread = RT_NULL;
    
    env = (struct guardian_environment *)parameter;
    RT_ASSERT(env != RT_NULL);
    
    dev = rt_device_find(PANTILT_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
    rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    
    semaph = rt_sem_create(PANTILT_SEMAPHORE_RX_NAME, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(semaph != RT_NULL);
    
    env->sh_ptz = rt_sem_create(PANTILT_SEMAPHORE_NAME, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(env->sh_ptz != RT_NULL);
    mempool = rt_mp_create(PANTILT_SEND_MP_NAME, 8, PANTILT_BUFFER_SIZE);
    RT_ASSERT(mempool != RT_NULL);
    mailbox = rt_mb_create(PANTILT_SEND_MB_NAME, 8, RT_IPC_FLAG_FIFO);
    RT_ASSERT(mailbox != RT_NULL);
    
    // set uart in 115200, 8N1.
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &config);
    
    rt_device_set_rx_indicate(dev, uart_hook_callback);
    
    pthread = rt_thread_create("tPTZtx", pantilt_data_send_entry, env, 2048, 10, 20);
    RT_ASSERT(pthread != RT_NULL);
    result = rt_thread_startup(pthread);
    RT_ASSERT(result == RT_EOK);
    
    pthread = rt_thread_create("tPTZrx", pantilt_data_recv_entry, env, 2048, 10, 20);
    RT_ASSERT(pthread != RT_NULL);
    result = rt_thread_startup(pthread);
    RT_ASSERT(result == RT_EOK);
    
    LOG_I("initialization finish, start!");

    while (1)
    {
        result = rt_sem_take(env->sh_ptz, RT_WAITING_FOREVER);

		//location
//		if (env->ptz_action == PANTILT_ACTION_ASK)
//		{
//			env->ask_laser_distance=1;
//			env->ptz_action = PANTILT_ACTION_NULL;
//			pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
//			rt_memcpy(pbuf, ptz_askctrlpkt, PTZ_ASK_PKT_SIZE);
//			rt_mb_send(mailbox, (rt_ubase_t)pbuf);
//			continue;			
//		}

        if (env->ptz_action == PANTILT_ACTION_IRCOLOR)
        {
            env->ptz_action = PANTILT_ACTION_NULL;
            LOG_D("PANTILT_ACTION_IRCOLOR");
            
            pktsz = sizeof(irs_serialctrlpkt);
            
            switch(env->irs_color)
            {
                case 0:
                    irs_serialctrlpkt[5] = 0;
                    break;
                case 1:
                    irs_serialctrlpkt[5] = 1;
                    break;
                case 2:
                    irs_serialctrlpkt[5] = 2;
                    break;
                default:
                    irs_serialctrlpkt[5] = 4;
                    break;
            }
            
            irs_serialctrlpkt[6] = 0x00;
            
            for (int i = 0; i < 6; i++)
                irs_serialctrlpkt[6] += irs_serialctrlpkt[i];
            
            pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
            rt_memcpy(pbuf, irs_serialctrlpkt, pktsz);
            rt_mb_send(mailbox, (rt_ubase_t)pbuf);
  
        }
        else if (env->ptz_action == PANTILT_ACTION_CALIBRATE)
        {
            LOG_D("PANTILT_ACTION_CALIBRATE");
            env->ptz_action = PANTILT_ACTION_NULL;
            
            for (int i = 0; i < 4; i++)
            {
                pktsz = sizeof(ptz_serialctrlpkt);
                rt_memset(&ctrlpkt, 0x00, pktsz);
                ctrlpkt.HEADER = PANTILT_PKT_HEADER;
                
                switch(i) {
                    case 0:
                    case 2:
                    default:
                        ctrlpkt.mode = 0x6400;
                        break;
                    case 1:
                    case 3:
                        ctrlpkt.mode = 0x9BFE;
                        break;
                }         
                
                pantilt_update_checksum(&ctrlpkt);
                pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
                rt_memcpy(pbuf, &ctrlpkt, pktsz);
                rt_mb_send(mailbox, (rt_ubase_t)pbuf);
                
                rt_thread_delay(200);
            }
        }
        else if (env->ptz_action == PANTILT_ACTION_HOMING)
        {
            LOG_D("PANTILT_ACTION_HOMING");
            env->ptz_action = PANTILT_ACTION_NULL;
            
            pktsz = sizeof(ptz_serialctrlpkt);
            rt_memset(&ctrlpkt, 0x00, pktsz);
            ctrlpkt.HEADER = PANTILT_PKT_HEADER;
            
            if (env->ptz_mode == PANTILT_MODE_HEADFREE)
                ctrlpkt.mode = 0x0000;
            else if (env->ptz_mode == PANTILT_MODE_HEADLOCK)
                ctrlpkt.mode = 0x6400;               
            else if (env->ptz_mode == PANTILT_MODE_HEADDOWN)
                ctrlpkt.mode = 0x9BFE;
            
            ctrlpkt.homing = 0x9BFE;
            pantilt_update_checksum(&ctrlpkt);
            pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
            rt_memcpy(pbuf, &ctrlpkt, pktsz);
            rt_mb_send(mailbox, (rt_ubase_t)pbuf);
            
            rt_thread_delay(100);
            
            rt_memset(&ctrlpkt, 0x00, pktsz);
            ctrlpkt.HEADER = PANTILT_PKT_HEADER;

            if (env->ptz_mode == PANTILT_MODE_HEADFREE)
                ctrlpkt.mode = 0x0000;
            else if (env->ptz_mode == PANTILT_MODE_HEADLOCK)
                ctrlpkt.mode = 0x6400;               
            else if (env->ptz_mode == PANTILT_MODE_HEADDOWN)
                ctrlpkt.mode = 0x9BFE;
                
            ctrlpkt.homing = 0x0000;
            pantilt_update_checksum(&ctrlpkt);
            pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
            rt_memcpy(pbuf, &ctrlpkt, pktsz);
            rt_mb_send(mailbox, (rt_ubase_t)pbuf);
            
        }
        else {
            
            if (env->trck_incharge)
            {
                pktsz = sizeof(ptz_serialctrlpkt);
                rt_memset(&ctrlpkt, 0x00, pktsz);

                ctrlpkt.HEADER = PANTILT_PKT_HEADER;
                
                ctrlpkt.pitch   = -env->trck_err_y * ZOOM2RATIO[env->cam_zoom_pos];
                ctrlpkt.yaw     = -env->trck_err_x * ZOOM2RATIO[env->cam_zoom_pos];
                
                if (env->ptz_mode == PANTILT_MODE_HEADFREE)
                    ctrlpkt.mode = 0x0000;
                else if (env->ptz_mode == PANTILT_MODE_HEADLOCK)
                    ctrlpkt.mode = 0x6400;               
                else if (env->ptz_mode == PANTILT_MODE_HEADDOWN)
                    ctrlpkt.mode = 0x9BFE;

                pantilt_update_checksum(&ctrlpkt);
                   
                pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
                rt_memcpy(pbuf, &ctrlpkt, pktsz);
                rt_mb_send(mailbox, (rt_ubase_t)pbuf);

                if (env->trck_lost == RT_TRUE)
                {
                    env->trck_incharge = RT_FALSE;
                    env->trck_lost = RT_FALSE;
                    
//                    pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
//                    rt_memcpy(pbuf, &ctrlpkt, pktsz);
//                    rt_mb_send(mailbox, (rt_ubase_t)pbuf);
//                    
//                    pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
//                    rt_memcpy(pbuf, &ctrlpkt, pktsz);
//                    rt_mb_send(mailbox, (rt_ubase_t)pbuf);
                }
                // todo.
            }
            else if (env->sbus_incharge)
            {
                env->sbus_incharge = RT_FALSE;
                
                {
                    rt_int16_t dval_pitch, dval_yaw, dval_roll = 0;
                    LOG_D("PANTILT_ACTION_OTHER, %d", env->ptz_action);
                    
                    pktsz = sizeof(ptz_serialctrlpkt);
                    rt_memset(&ctrlpkt, 0x00, pktsz);
                                
                    ctrlpkt.HEADER = PANTILT_PKT_HEADER;

                    dval_roll = env->ch_value[0] - SBUS_VALUE_MEDIAN;    // roll
                    
                    if (abs(dval_roll) < SBUS_VALUE_IGNORE * 3)
                        dval_roll = 0;
                    else
                    {
                        if (dval_roll < 0)
                            dval_roll = PANTILT_VALUE_MININUM;
                        else
                            dval_roll = PANTILT_VALUE_MAXIMUM;
                    }
                    
                    dval_pitch = env->ch_value[1] - SBUS_VALUE_MEDIAN;    // pitch
                    if (abs(dval_pitch) < SBUS_VALUE_IGNORE)
                        dval_pitch = 0;
                   
                    dval_yaw = env->ch_value[3] - SBUS_VALUE_MEDIAN;    // yaw
                    if (abs(dval_yaw) < SBUS_VALUE_IGNORE)
                        dval_yaw = 0;
                    
                    ctrlpkt.pitch   = dval_pitch * ZOOM2RATIO[env->cam_zoom_pos];
                    ctrlpkt.yaw     = dval_yaw * ZOOM2RATIO[env->cam_zoom_pos];
                    
                    ctrlpkt.roll = dval_roll;
                    
                    if (env->ptz_mode == PANTILT_MODE_HEADFREE)
                        ctrlpkt.mode = 0x0000;
                    else if (env->ptz_mode == PANTILT_MODE_HEADLOCK)
                        ctrlpkt.mode = 0x6400;               
                    else if (env->ptz_mode == PANTILT_MODE_HEADDOWN)
                        ctrlpkt.mode = 0x9BFE;
                    
                    pantilt_update_checksum(&ctrlpkt);
                       
                    pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
                    rt_memcpy(pbuf, &ctrlpkt, pktsz);
                    rt_mb_send(mailbox, (rt_ubase_t)pbuf);
                    
//                    if ((ctrlpkt.roll == 0) || (ctrlpkt.pitch == 0) || (ctrlpkt.yaw == 0)) // send again, ensure stop.
//                    {
//                        rt_thread_delay(10);
//                        pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
//                        rt_memcpy(pbuf, &ctrlpkt, pktsz);
//                        rt_mb_send(mailbox,  (rt_ubase_t)pbuf);
//                        rt_thread_delay(10);
//                        pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
//                        rt_memcpy(pbuf, &ctrlpkt, pktsz);
//                        rt_mb_send(mailbox,  (rt_ubase_t)pbuf);
//                    }
                }
            }
            else if (env->user_incharge)
            {
                env->user_incharge = RT_FALSE;
                
                {
                    rt_int16_t dval_pitch, dval_yaw, dval_roll = 0;
                    LOG_D("PANTILT_ACTION_OTHER, %d", env->ptz_action);
                    
                    pktsz = sizeof(ptz_serialctrlpkt);
                    rt_memset(&ctrlpkt, 0x00, pktsz);
                                
                    ctrlpkt.HEADER = PANTILT_PKT_HEADER;

                    dval_roll = env->ch_value[0] - SBUS_VALUE_MEDIAN;    // roll
                    
                    if (abs(dval_roll) < SBUS_VALUE_IGNORE)
                        dval_roll = 0;
                    else
                    {
                        if (dval_roll < 0)
                            dval_roll = PANTILT_VALUE_MININUM;
                        else
                            dval_roll = PANTILT_VALUE_MAXIMUM;
                    }
                    
                    dval_pitch = env->ch_value[1] - SBUS_VALUE_MEDIAN;    // pitch
                    if (abs(dval_pitch) < SBUS_VALUE_IGNORE)
                        dval_pitch = 0;
                   
                    dval_yaw = env->ch_value[3] - SBUS_VALUE_MEDIAN;    // yaw
                    if (abs(dval_yaw) < SBUS_VALUE_IGNORE)
                        dval_yaw = 0;
                    
                    ctrlpkt.pitch   = dval_pitch * ZOOM2RATIO[env->cam_zoom_pos];
                    ctrlpkt.yaw     = dval_yaw * ZOOM2RATIO[env->cam_zoom_pos];
                    
                    ctrlpkt.roll = dval_roll;
                    
                    if (env->ptz_mode == PANTILT_MODE_HEADFREE)
                        ctrlpkt.mode = 0x0000;
                    else if (env->ptz_mode == PANTILT_MODE_HEADLOCK)
                        ctrlpkt.mode = 0x6400;               
                    else if (env->ptz_mode == PANTILT_MODE_HEADDOWN)
                        ctrlpkt.mode = 0x9BFE;
                    
                    pantilt_update_checksum(&ctrlpkt);
                       
                    pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
                    rt_memcpy(pbuf, &ctrlpkt, pktsz);
                    rt_mb_send(mailbox, (rt_ubase_t)pbuf);
                    
                }
            }
            
        }
    }
    // never be here.
}

