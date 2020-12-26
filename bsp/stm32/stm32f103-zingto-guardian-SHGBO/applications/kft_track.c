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

#define TRACK_UARTPORT_NAME "uart1"
#define TRACK_SEMAPHORE_NAME "shTRCK"
#define TRACK_SEMAPHORE_RX_NAME "shTRCKrx"

#define TRACK_SEND_MP_NAME "mpPTZtx"
#define TRACK_SEND_MB_NAME "mbPTZtx"

/* defined the LED pin: PA0 */
#define LED_PIN    GET_PIN(A, 0)

#define TRACK_BUFFER_SIZE       (64)
#define MIN_RX_INTERVAL    	    (10)

#define TRACK_ACK_PKT_SIZE      (8)
#define TRACK_ACK_PKT_HEADER    (0xBB)
#define TRACK_ACK_PKT_ADDR      (0x01)
#define ZOOM_ACK_PKT_SIZE       (7)
#define ZOOM_ACK_PKT_HEADER     (0x90)
#define ZOOM_ACK_PKT_ADDR       (0x50)

#pragma pack(1)
typedef struct __SHB30X_SendPacket
{
	rt_uint16_t		HEADER;				//b1 Always be 0x7E7E.
	rt_uint8_t		ADDR;				//b3 Always be 0x44.
	rt_uint8_t		__reserved1;		//b4 reserved.
	rt_uint8_t		__reserved2;		//b5 reserved.
	rt_uint8_t		set_mode;			//b6 0x71,
	rt_uint8_t		set_fuction;		//b7 0xfe,
	rt_uint16_t		set_offset_x;       //b8
	rt_uint16_t		set_offset_y;       //b10
	rt_uint8_t		start_trace;        //b12 0x01,
	rt_uint8_t		__reserved3;		//b13 0x01,
	rt_uint8_t		set_trace_mode;		//b14 0x3c,
	rt_uint8_t		set_video_source;   //b15 0x00 - 0x03
	rt_uint8_t		set_ircolor;		//b16
	rt_uint8_t		set_zoom;			//b17
	rt_uint8_t		__reserved6;		//b18
	rt_uint8_t		__reserved7;		//b19
	rt_uint8_t		__reserved8[28];	//b20
	rt_uint8_t		checksum;			//b48
}shb_serialctrlpkt;
#pragma pack()

static rt_sem_t semaph = RT_NULL;

static rt_mailbox_t mailbox = RT_NULL;
static rt_mp_t mempool = RT_NULL;
#define VISCA_SET_ZOOMPOS_CMD_SIZE      (9)
const rt_uint8_t VISCA_ZOOM[41][VISCA_SET_ZOOMPOS_CMD_SIZE] = 
{
	{0x81, 0x01, 0x04, 0x47, 0x00, 0x00, 0x00, 0x00, 0xFF},	//1X
	{0x81, 0x01, 0x04, 0x47, 0x01, 0x06, 0x0a, 0x01, 0xFF},	//2X
	{0x81, 0x01, 0x04, 0x47, 0x02, 0x00, 0x06, 0x03, 0xFF},	//3X
	{0x81, 0x01, 0x04, 0x47, 0x02, 0x06, 0x02, 0x08, 0xFF},	//4X
	{0x81, 0x01, 0x04, 0x47, 0x02, 0x0a, 0x01, 0x0d, 0xFF},	//5X
	{0x81, 0x01, 0x04, 0x47, 0x02, 0x0d, 0x01, 0x03, 0xFF},	//6X
	{0x81, 0x01, 0x04, 0x47, 0x02, 0x0f, 0x06, 0x0d, 0xFF},	//7X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x01, 0x06, 0x01, 0xFF},	//8X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x03, 0x00, 0x0d, 0xFF},	//9X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x04, 0x08, 0x06, 0xFF},	//10X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x05, 0x0d, 0x07, 0xFF},	//11X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x07, 0x00, 0x09, 0xFF},	//12X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x08, 0x02, 0x00, 0xFF},	//13X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x09, 0x02, 0x00, 0xFF},	//14X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0a, 0x00, 0x0a, 0xFF},	//15X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0a, 0x0d, 0x0d, 0xFF},	//16X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0b, 0x09, 0x0c, 0xFF},	//17X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0c, 0x04, 0x06, 0xFF},	//18X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0c, 0x0d, 0x0c, 0xFF},	//19X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0d, 0x06, 0x00, 0xFF},	//20X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0d, 0x0d, 0x04, 0xFF},	//21X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0e, 0x03, 0x09, 0xFF},	//22X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0e, 0x09, 0x00, 0xFF},	//23X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0e, 0x0d, 0x0c, 0xFF},	//24X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x01, 0x0e, 0xFF},	//25X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x05, 0x07, 0xFF},	//26X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x08, 0x0a, 0xFF},	//27X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x0b, 0x06, 0xFF},	//28X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x0d, 0x0c, 0xFF},	//29X
	{0x81, 0x01, 0x04, 0x47, 0x04, 0x00, 0x00, 0x00, 0xFF},	//30X * 1X
    {0x81, 0x01, 0x04, 0x47, 0x06, 0x00, 0x00, 0x00, 0xFF}, //30X * 2X
    {0x81, 0x01, 0x04, 0x47, 0x06, 0x0a, 0x08, 0x00, 0xFF}, //30X * 3X
    {0x81, 0x01, 0x04, 0x47, 0x07, 0x00, 0x00, 0x00, 0xFF}, //30X * 4X
    {0x81, 0x01, 0x04, 0x47, 0x07, 0x03, 0x00, 0x00, 0xFF}, //30X * 5X
    {0x81, 0x01, 0x04, 0x47, 0x07, 0x05, 0x04, 0x00, 0xFF}, //30X * 6X
    {0x81, 0x01, 0x04, 0x47, 0x07, 0x06, 0x0c, 0x00, 0xFF}, //30X * 7X
    {0x81, 0x01, 0x04, 0x47, 0x07, 0x08, 0x00, 0x00, 0xFF}, //30X * 8X
    {0x81, 0x01, 0x04, 0x47, 0x07, 0x08, 0x0c, 0x00, 0xFF}, //30X * 9X
    {0x81, 0x01, 0x04, 0x47, 0x07, 0x09, 0x08, 0x00, 0xFF}, //30X * 10X
    {0x81, 0x01, 0x04, 0x47, 0x07, 0x0a, 0x00, 0x00, 0xFF}, //30X * 11X
    {0x81, 0x01, 0x04, 0x47, 0x07, 0x0a, 0x0c, 0x00, 0xFF}  //30X * 12X
};

rt_err_t uart_hook_callback(rt_device_t dev, rt_size_t sz)
{
    rt_sem_release(semaph);
    
    return RT_EOK;
}

static void track_data_send_entry(void* parameter)
{
    struct guardian_environment *env = RT_NULL;
    rt_ubase_t mail;
    rt_uint8_t* pbuf;
    rt_device_t dev = RT_NULL;
    
    env = (struct guardian_environment*)parameter;
    RT_ASSERT(env != RT_NULL);
    
    dev = rt_device_find(TRACK_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
    
    LOG_I("send sub-thread, start!");
    
    while (1)
    {
        rt_mb_recv(mailbox, &mail, RT_WAITING_FOREVER);
        LOG_D("mb recv %0X", mail);
        if (mail == RT_NULL)
            continue;
        pbuf = (rt_uint8_t*)mail;
        
        LOG_D("send to shb30x");
        rt_device_write(dev, 0, pbuf, sizeof(shb_serialctrlpkt));
        
        rt_mp_free(pbuf);
        
        rt_thread_delay(1);
    }
}

static void track_data_recv_entry(void* parameter)
{
    struct guardian_environment *env = RT_NULL;
    rt_uint8_t* pbuf;
    rt_size_t szbuf = 0;
	rt_size_t SIZE = TRACK_ACK_PKT_SIZE;
	rt_uint8_t HEADER = TRACK_ACK_PKT_HEADER;
	rt_uint8_t ADDR = TRACK_ACK_PKT_ADDR;
    rt_err_t result;
    rt_device_t dev = RT_NULL;
    static shb_serialctrlpkt ctrlpkt;
    rt_size_t pktsz;
    rt_uint8_t lost_count = 0;
	rt_uint8_t ctrl_count = 0;
    rt_bool_t on_tracing = RT_FALSE;
    
    PID_t pid_x, pid_y;
    
    env = (struct guardian_environment*)parameter;
    RT_ASSERT(env != RT_NULL);
    
    pbuf = rt_malloc(TRACK_BUFFER_SIZE);
    RT_ASSERT(pbuf != RT_NULL);
    
    dev = rt_device_find(TRACK_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
		
    pid_init(&pid_x, 1.0f, 0.02f, 0.7f);
    pid_init(&pid_y, 1.0f, 0.02f, 0.7f);
    
    pid_setThreshold(&pid_x, 500.0f, 200.0f, 0.02f);
    pid_setThreshold(&pid_y, 500.0f, 200.0f, 0.02f);
    
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
            if (pbuf[0] == TRACK_ACK_PKT_HEADER)
            {
				SIZE = TRACK_ACK_PKT_SIZE;
				HEADER = TRACK_ACK_PKT_HEADER;
				ADDR = TRACK_ACK_PKT_ADDR;
			}
			else if(pbuf[0] == ZOOM_ACK_PKT_HEADER)
			{
				SIZE = ZOOM_ACK_PKT_SIZE;
				HEADER = ZOOM_ACK_PKT_HEADER;
				ADDR = ZOOM_ACK_PKT_ADDR;
			}
			else
			{
				szbuf = 0;
			}
            break;
        default:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, SIZE - szbuf);
            break;
        }
		// should never happened.
        if (szbuf != SIZE)
            continue;
        
        szbuf = 0;
        // check packet addr byte, shouled 0x01.
        if (pbuf[1] != ADDR)
            continue;
		if(pbuf[0] == TRACK_ACK_PKT_HEADER && pbuf[1] == TRACK_ACK_PKT_ADDR)
		{
			if ((pbuf[6] & 0x02) != 0x00)
			{
				on_tracing = RT_TRUE;
				lost_count = 0;
				
				float deffer_x, deffer_y;
				
				deffer_x = (float)*(rt_int16_t*)&pbuf[2];
				deffer_y = (float)*(rt_int16_t*)&pbuf[4];
				
				env->trck_err_x = pid_update(&pid_x, deffer_x);
				env->trck_err_y = pid_update(&pid_y, deffer_y);
				
				LOG_D("tracing %d %d", env->trck_err_x, env->trck_err_y);
				
				if ( ctrl_count < 11)
					ctrl_count++;
				else {
					rt_sem_release(env->sh_ptz);
					ctrl_count = 0;
				}
			}
			else
			{
				if (on_tracing == RT_FALSE)
					continue;
				
				if (lost_count++ < 90)
					continue;
				
				on_tracing = RT_FALSE;
				
				if (env->trck_incharge == RT_TRUE)
				{
					env->trck_lost = RT_TRUE;
					
					/* stop Tracing module. */
					pktsz = sizeof(shb_serialctrlpkt);
					rt_memset(&ctrlpkt, 0x00, pktsz);

					ctrlpkt.HEADER = 0x7E7E;
					ctrlpkt.ADDR = 0x44;
					ctrlpkt.set_mode = 0x26;
					
					rt_uint8_t *ptr = (rt_uint8_t*)&ctrlpkt;
					for(int i = 0; i < pktsz - 1; i++)
						ctrlpkt.checksum += *(ptr + i);
					
					ptr = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
					rt_memcpy(ptr, &ctrlpkt, pktsz);
					rt_mb_send(mailbox, (rt_ubase_t)ptr);	
				}
				/* stop PanTiltZoom */
				env->trck_err_x = 0;
				env->trck_err_y = 0;
				rt_sem_release(env->sh_ptz);
				
				LOG_D("tracing target lost");
			}
		}
		else if(pbuf[0] == 0x90 && pbuf[1] == 0x50)
		{
			rt_uint8_t nZoom = 0;
			rt_uint16_t nZoomNow, nZoomPrev, nZoomNext;

			nZoomNow = ((pbuf[2] & 0x0F) << 12) | \
					   ((pbuf[3] & 0x0F) << 8) | \
					   ((pbuf[4] & 0x0F) << 4) | \
					   ((pbuf[5] & 0x0F) << 0);

			for ( nZoom = 0; nZoom < 41; nZoom++)
			{
				int res = rt_memcmp(&VISCA_ZOOM[nZoom][4], &pbuf[2], 4);
				if (0 == res)
					break;
				if (res > 0)
				{
					if (nZoom == 40)
						continue;
					else
					{
						nZoomPrev = ((VISCA_ZOOM[nZoom - 1][4] & 0x0F) << 12) | \
									((VISCA_ZOOM[nZoom - 1][5] & 0x0F) << 8) | \
									((VISCA_ZOOM[nZoom - 1][6] & 0x0F) << 4) | \
									((VISCA_ZOOM[nZoom - 1][7] & 0x0F) << 0);
						
						nZoomNext = ((VISCA_ZOOM[nZoom][4] & 0x0F) << 12) | \
									((VISCA_ZOOM[nZoom][5] & 0x0F) << 8) | \
									((VISCA_ZOOM[nZoom][6] & 0x0F) << 4) | \
									((VISCA_ZOOM[nZoom][7] & 0x0F) << 0);
						
						if ((nZoomNext - nZoomNow) > (nZoomNow - nZoomPrev))
						{
							nZoom--;
							break;
						}		
						else
							break;
					}
				}
			}
			
			if (nZoom > 29) 
			{
				nZoom = 29;
			}
			
			env->cam_zoom_pos = nZoom;
			rt_kprintf("env->cam_zoom_pos  %d \n",env->cam_zoom_pos);
			
			// notice the tracker thread to show.
			env->trck_action = TRACK_ACTION_ZOOM_SHOW;
			rt_sem_release(env->sh_track);
			
			env->cam_getpos_tick = rt_tick_get();
		}
		else
		{
			
		}
    }
}

void track_resolving_entry(void* parameter)
{
    static rt_device_t dev = RT_NULL;
    static rt_uint8_t *pbuf = RT_NULL;
    static struct guardian_environment *env = RT_NULL;
    static shb_serialctrlpkt ctrlpkt; 
    rt_size_t pktsz;
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
    
    // set uart3 in 115200, 8E1.
//    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
//    config.data_bits = DATA_BITS_8;
//    config.parity = PARITY_NONE;
//    rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &config);

		//uart3 baud_rate 57600  parity_even
		/*struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_57600;
		config.data_bits = DATA_BITS_9;
		config.parity    = PARITY_EVEN;
    rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &config);*/
    
    rt_device_set_rx_indicate(dev, uart_hook_callback);
    
    pthread = rt_thread_create("tTRCKtx", track_data_send_entry, env, 2048, 9, 20);
    RT_ASSERT(pthread != RT_NULL);
    result = rt_thread_startup(pthread);
    RT_ASSERT(result == RT_EOK);

    pthread = rt_thread_create("tTRCKrx", track_data_recv_entry, env, 2048, 9, 20);
    RT_ASSERT(pthread != RT_NULL);
    result = rt_thread_startup(pthread);
    RT_ASSERT(result == RT_EOK);
    
    LOG_I("initialization finish, start!");
    
    env->trck_incharge = RT_FALSE;

    while (1)
    {
        result = rt_sem_take(env->sh_track, RT_WAITING_FOREVER);
        
        pktsz = sizeof(shb_serialctrlpkt);
        
        rt_memset(&ctrlpkt, 0x00, pktsz);
        
        ctrlpkt.HEADER = 0x7E7E;
        ctrlpkt.ADDR = 0x44;
        
		if (env->trck_action == TRACK_ACTION_ZOOM_SHOW)
        {
			ctrlpkt.__reserved1 = 0x10;
			ctrlpkt.set_mode = 0x83;
			ctrlpkt.set_fuction = 0x30;
            
            float  zoomf32 = 0.f;
            if (env->cam_zoom_pos < 30)
                zoomf32 = env->cam_zoom_pos + 1;                    // Optical ZOOM.
            else             
                zoomf32 = 30;     // Optical ZOOM 30X combine Digital ZOOM.
            
			rt_memcpy(ctrlpkt.__reserved8 + 4, &zoomf32, 4);
            rt_memcpy(&ctrlpkt.set_ircolor, &env->ptz_yaw, sizeof(float));
			rt_memcpy(ctrlpkt.__reserved8, &env->ptz_pitch, sizeof(float));		
        }
        else if (env->trck_action == TRACK_ACTION_PREPARE)
        {
            ctrlpkt.set_mode = 0x71;
            ctrlpkt.set_fuction = 0xFF;
            
            LOG_D("tracker searching target.");
            
            env->trck_prepare = RT_TRUE;
        }
        else if (env->trck_action == TRACK_ACTION_TRACE_START)
        {
			ctrlpkt.set_mode = 0x71;			// 0x71 Trace Mode.
			ctrlpkt.set_fuction = 0xFE;
			ctrlpkt.start_trace = 0x01;			// 0: OFF; 1: ON.
			ctrlpkt.__reserved3 = 0x01;
			ctrlpkt.set_trace_mode = 0x38;		// medium size trace window.   // 0x3C: S,M,L     0x38: M,L    0x2C: S,M
            
            LOG_D("tracker start tracing.");
            
            env->trck_prepare = RT_FALSE;
            env->trck_incharge = RT_TRUE;
        }
        else if (env->trck_action == TRACK_ACTION_TRACE_STOP)
        {
            ctrlpkt.set_mode = 0x26;			// 0x71 Trace Mode.
            
            LOG_D("tracker stop tracing.");
            
            env->trck_prepare = RT_FALSE;
            env->trck_incharge = RT_FALSE;
        }
		else if (env->trck_action == TRACK_ACTION_POINT_START)
        {
			ctrlpkt.set_mode = 0x26;
			env->trck_prepare = RT_FALSE;
            env->trck_incharge = RT_FALSE;
			pbuf = (rt_uint8_t*)&ctrlpkt;
			for(int i = 0; i < pktsz - 1; i++)
				ctrlpkt.checksum += *(pbuf + i);      
			pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
			rt_memcpy(pbuf, &ctrlpkt, pktsz);
			rt_mb_send(mailbox, (rt_ubase_t)pbuf);
			
			rt_thread_mdelay(100);
			
			rt_memset(&ctrlpkt, 0x00, pktsz);
			ctrlpkt.HEADER = 0x7E7E;
			ctrlpkt.ADDR = 0x44;
			ctrlpkt.set_mode = 0x71;
            ctrlpkt.set_fuction = 0xFF;
			ctrlpkt.set_offset_x=env->trck_offset_x;
			ctrlpkt.set_offset_y=env->trck_offset_y;            
            env->trck_prepare = RT_TRUE;
			pbuf = (rt_uint8_t*)&ctrlpkt;
			for(int i = 0; i < pktsz - 1; i++)
				ctrlpkt.checksum += *(pbuf + i);      
			pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
			rt_memcpy(pbuf, &ctrlpkt, pktsz);
			rt_mb_send(mailbox, (rt_ubase_t)pbuf);
			
			rt_thread_mdelay(100);
			
			rt_memset(&ctrlpkt, 0x00, pktsz);
			ctrlpkt.HEADER = 0x7E7E;
			ctrlpkt.ADDR = 0x44;
			ctrlpkt.set_mode = 0x71;			// 0x71 Trace Mode.
			ctrlpkt.set_fuction = 0xFE;
			ctrlpkt.start_trace = 0x01;			// 0: OFF; 1: ON.
			ctrlpkt.__reserved3 = 0x01;
			ctrlpkt.set_trace_mode = 0x3C;		// medium size trace window.   // 0x3C: S,M,L     0x38: M,L    0x2C: S,M          
            env->trck_prepare = RT_FALSE;
            env->trck_incharge = RT_TRUE;
			pbuf = (rt_uint8_t*)&ctrlpkt;
			for(int i = 0; i < pktsz - 1; i++)
				ctrlpkt.checksum += *(pbuf + i);      
			pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
			rt_memcpy(pbuf, &ctrlpkt, pktsz);
			rt_mb_send(mailbox, (rt_ubase_t)pbuf);
        
			env->trck_action = TRACK_ACTION_NULL;
			continue;
        }
        else if (env->trck_action == TRACK_ACTION_RECORD_ON)
        {
            ctrlpkt.set_mode = 0x7C;
            ctrlpkt.set_fuction = 0x01;
            ctrlpkt.set_offset_x = 0x0258;
            
            LOG_D("tracker start recording video.");
        }
        else if (env->trck_action == TRACK_ACTION_RECORD_OFF)
        {
            ctrlpkt.set_mode = 0x7C;
            
            LOG_D("tracker stop recording video.");
        }
		else if (env->trck_action == TRACK_ACTION_CAPTURE)
        {
            ctrlpkt.set_mode = 0x7C;
            ctrlpkt.set_fuction = 0x02;
            
            LOG_D("TRACK_ACTION_CAPTURE");
        }
        else if (env->trck_action == TRACK_ACTION_PIP_MODE)
        {
			ctrlpkt.set_mode = 0x85;
            if(env->cam_pip_mode == 0)
			{
				ctrlpkt.set_video_source=0x00;
			}
			else if(env->cam_pip_mode == 1)
			{
				ctrlpkt.set_video_source=0x02;
			}
			else if(env->cam_pip_mode == 2)
			{
				ctrlpkt.set_video_source=0x03;
			}
			else
			{
				ctrlpkt.set_video_source=0x01;
			}
        }
		else if (env->trck_action == TRACK_ACTION_IRCOLOR)
        {
			ctrlpkt.set_mode = 0x84;
            if(env->irs_color == 0)
			{
				ctrlpkt.set_fuction=0x00;
				ctrlpkt.set_ircolor=0x00;
			}
			else if(env->irs_color == 1)
			{
				ctrlpkt.set_fuction=0x00;
				ctrlpkt.set_ircolor=0x01;
			}
			else if(env->irs_color == 2)
			{
				ctrlpkt.set_fuction=0x04;
			}
			else if(env->irs_color == 3)
			{
				ctrlpkt.set_fuction=0x02;
			}
			else if(env->irs_color == 4)
			{
				ctrlpkt.set_fuction=0x01;
			}
			else
			{
				ctrlpkt.set_fuction=0x03;
			}
        }
		else if (env->trck_action == TRACK_ACTION_IRZOOM)
        {
			ctrlpkt.set_mode = 0x78;
            if(env->irs_zoom == 0)
			{
				ctrlpkt.set_zoom=0x01;
			}
			else if(env->irs_zoom == 1)
			{
				ctrlpkt.set_zoom=0x02;
			}
			else if(env->irs_zoom == 2)
			{
				ctrlpkt.set_zoom=0x03;
			}
			else if(env->irs_zoom == 3)
			{
				ctrlpkt.set_zoom=0x04;
			}
			else
			{
				ctrlpkt.set_zoom=0x05;
			}
			
			//pip mode
			if(env->cam_pip_mode == 0)
			{
				ctrlpkt.set_video_source=0x00;
			}
			else if(env->cam_pip_mode == 1)
			{
				ctrlpkt.set_video_source=0x02;
			}
			else if(env->cam_pip_mode == 2)
			{
				ctrlpkt.set_video_source=0x03;
			}
			else
			{
				ctrlpkt.set_video_source=0x01;
			}
			//irs color
			if(env->irs_color == 0)
			{
				ctrlpkt.set_fuction=0x00;
				ctrlpkt.set_ircolor=0x00;
			}
			else if(env->irs_color == 1)
			{
				ctrlpkt.set_fuction=0x00;
				ctrlpkt.set_ircolor=0x01;
			}
			else if(env->irs_color == 2)
			{
				ctrlpkt.set_fuction=0x04;
			}
			else if(env->irs_color == 3)
			{
				ctrlpkt.set_fuction=0x02;
			}
			else if(env->irs_color == 4)
			{
				ctrlpkt.set_fuction=0x01;
			}
			else
			{
				ctrlpkt.set_fuction=0x03;
			}
        }
        else
            continue;
        
        pbuf = (rt_uint8_t*)&ctrlpkt;
        for(int i = 0; i < pktsz - 1; i++)
            ctrlpkt.checksum += *(pbuf + i);
        
        pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
        rt_memcpy(pbuf, &ctrlpkt, pktsz);
        rt_mb_send(mailbox, (rt_ubase_t)pbuf);
        
        env->trck_action = TRACK_ACTION_NULL;
    }
    
    // never be here.
}

