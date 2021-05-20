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

const char XUANZHAN_CMD_SNAP[] = "<snap quality=\"10\" ></snap>\r\n";
const char XUANZHAN_CMD_RECORD_ON[] = "<?xml version=\"1.0\" encoding=\"GB2312\" ?><?xml version=\"1.0\" encoding=\"GB2312\" ?><record cmd=\"start\"></record>\r\n";
const char XUANZHAN_CMD_RECORD_OFF[] = "<?xml version=\"1.0\" encoding=\"GB2312\" ?><?xml version=\"1.0\" encoding=\"GB2312\" ?><record cmd=\"stop\"></record>\r\n";

const char XUANZHAN_LOGO_RECORD_ON[] = "<?xml version=\"1.0\" encoding=\"GB2312\" ?><icon name=\"record_l\"  posx=\"10\" posy=\"10\" cmd=\"show\" ></icon>\r\n";
const char XUANZHAN_LOGO_RECORD_OFF[] = "<?xml version=\"1.0\" encoding=\"GB2312\" ?><icon name=\"record_l\"  posx=\"10\" posy=\"10\" cmd=\"hide\" ></icon>\r\n";
const char XUANZHAN_LOGO_SNAP_ON[] = "<?xml version=\"1.0\" encoding=\"GB2312\" ?><icon name=\"photo_l\"  posx=\"10\" posy=\"10\" cmd=\"show\" ></icon>\r\n";
const char XUANZHAN_LOGO_SNAP_OFF[] = "<?xml version=\"1.0\" encoding=\"GB2312\" ?><icon name=\"photo_l\"  posx=\"10\" posy=\"10\" cmd=\"hide\" ></icon>\r\n";

#define CAMERA_UARTPORT_NAME "uart1"
#define CAMERA_SEMAPHORE_NAME "shCAM"
#define CAMERA_EVENT_NAME "evCAM"

#define CAMERA_BUFFER_SIZE              (32)
#define CAMERA_RX_TIMEOUT               (RT_TICK_PER_SECOND)  // almost 0.3s

#define VISCA_ZOOM_SPEED                (2)
#define VISCA_SET_ZOOMPOS_CMD_SIZE      (9)
#define VISCA_GET_ZOOMPOS_ACK_SIZE      (7)

rt_uint8_t CAMERA_VISCA_CAPTURE[]     	  = {0x81, 0x01, 0x04, 0x24, 0x0A, 0x00, 0x00, 0x01, 0xFF};
rt_uint8_t CAMERA_VISCA_RECORD_ON[]        = {0x81, 0x01, 0x04, 0x24, 0x0A, 0x00, 0x00, 0x02, 0xFF};
rt_uint8_t CAMERA_VISCA_RECORD_OFF[]    = {0x81, 0x01, 0x04, 0x24, 0x0A, 0x00, 0x00, 0x03, 0xFF};
rt_uint8_t VISCA_ICR_ON[]     	  = {0x81, 0x01, 0x04, 0x01, 0x02, 0xFF};
rt_uint8_t VISCA_ICR_OFF[]        = {0x81, 0x01, 0x04, 0x01, 0x03, 0xFF};
rt_uint8_t VISCA_AUTOICR_OFF[]    = {0x81, 0x01, 0x04, 0x51, 0x03, 0xFF};
rt_uint8_t VISCA_ZOOM_IN[]        = {0x81, 0x01, 0x04, 0x07, 0x20, 0xFF};
rt_uint8_t VISCA_ZOOM_OUT[]       = {0x81, 0x01, 0x04, 0x07, 0x30, 0xFF};
const rt_uint8_t VISCA_ZOOM_STOP[]      = {0x81, 0x01, 0x04, 0x07, 0x00, 0xFF};
const rt_uint8_t VISCA_GET_ZOOMPOS[]    = {0x81, 0x09, 0x04, 0x47, 0xFF};
const rt_uint8_t VISCA_GET_BRIGHTPOS[]  = {0x81, 0x09, 0x04, 0x4D, 0xFF};
const rt_uint8_t VISCA_SET_1080P50HZ[]  = {0x81, 0x01, 0x04, 0x24, 0x72, 0x01, 0x04, 0xFF, 0x90, 0x41, 0xFF, 0x90, 0x51, 0xFF};
const rt_uint8_t VISCA_SET_1080P60HZ[]  = {0x81, 0x01, 0x04, 0x24, 0x72, 0x01, 0x05, 0xFF, 0x90, 0x41, 0xFF, 0x90, 0x51, 0xFF};

#define HI3521D_CMD_CAPTURE     "CAPTURE:ON\n"
#define HI3521D_CMD_RECORD_ON   "RECORD:ON\n"
#define HI3521D_CMD_RECORD_OFF  "RECORD:OFF\n"
#define HI3521D_CMD_PIP_MODE1   "SHOWCHANNEL:S1\n"
#define HI3521D_CMD_PIP_MODE2   "SHOWCHANNEL:S2\n"
#define HI3521D_CMD_PIP_MODE3   "SHOWCHANNEL:S3\n"
#define HI3521D_CMD_PIP_MODE4   "SHOWCHANNEL:S4\n"

#define HI3521D_ACK_CAPTURE     "CAPTURE:OK\n"
#define HI3521D_ACK_RECORD      "RECORD:OK\n"
#define HI3521D_ACK_PIP_MODE    "SHOWCHANNEL:OK\n"

/* defined the LED pin: PA0 */
#define LED_PIN    GET_PIN(A, 0)

static rt_err_t uart_send_with_block(rt_device_t dev, void * buffer, rt_size_t size)
{
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);
    RT_ASSERT(size > 0);
    
    return rt_device_write(dev, 0, buffer, size);
}

static rt_err_t uart_clean_recv_buff(rt_device_t dev, void * buffer)
{
    rt_size_t sz_recv = 0;
    
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);
    
    while (1)
    {
        sz_recv = rt_device_read(dev, 0, buffer, CAMERA_BUFFER_SIZE);
        
        if (sz_recv == 0)
            break;
    }
    
    return RT_EOK;
}


static rt_tick_t tm_snap;

void snap_logo_manager_entry(void* parameter)
{
    rt_device_t udev = (rt_device_t)parameter;
    
    tm_snap = RT_TICK_MAX;
    
    while(1)
    {
        if (tm_snap != RT_TICK_MAX) {
            if (rt_tick_get() - tm_snap > RT_TICK_PER_SECOND) {
                uart_send_with_block(udev, (void *)XUANZHAN_LOGO_SNAP_OFF, sizeof(XUANZHAN_LOGO_SNAP_OFF));
                tm_snap = RT_TICK_MAX;
            }
        }
        
        rt_thread_delay(RT_TICK_PER_SECOND / 100);
    }
}

void camera_resolving_entry(void* parameter)
{
    rt_device_t dev = RT_NULL;
    rt_uint8_t *pbuf = RT_NULL;
    struct guardian_environment *env = RT_NULL;
    rt_err_t result = RT_EOK;
    rt_uint32_t opcode;
    rt_thread_t pthread;
    
    env = (struct guardian_environment *)parameter;
    RT_ASSERT(env != RT_NULL);
    
    pbuf = rt_malloc(CAMERA_BUFFER_SIZE);
    RT_ASSERT(pbuf != RT_NULL);
    
    dev = rt_device_find(CAMERA_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
    
    rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    
    env->ev_camera = rt_event_create(CAMERA_EVENT_NAME, RT_IPC_FLAG_FIFO);
    RT_ASSERT(env->ev_camera != RT_NULL);
    
    // set uart in 115200     , 8N1.
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_115200;
    rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &config);
    
    pthread = rt_thread_create("tXZlogo", snap_logo_manager_entry, dev, 2048, 11, 20);
    RT_ASSERT(pthread != RT_NULL);
    result = rt_thread_startup(pthread);
    RT_ASSERT(result == RT_EOK);
    
    LOG_I("initialization finish, start!");
    
    env->cam_recording = RT_FALSE;

    while (1)
    {
        result = rt_event_recv(env->ev_camera, CAMERA_CMD_MASK, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &opcode);
        
        if ( result != RT_EOK)
        {
            LOG_E("event receive failed");
            continue;
        }
        
        if (opcode & CAMERA_CMD_ZOOM_STOP)
        {
            opcode = CAMERA_CMD_ZOOM_STOP;
        }
        
        switch(opcode & ~CAMERA_CMD_ZOOM_GETPOS) {
            case CAMERA_CMD_CAPTURE:
              
				uart_send_with_block(dev, (void *)CAMERA_VISCA_CAPTURE, sizeof(CAMERA_VISCA_CAPTURE));
				LOG_D("VISCA_CMD_CAPTURE");
		
				uart_clean_recv_buff(dev, pbuf);
				uart_send_with_block(dev, HI3521D_CMD_CAPTURE, sizeof(HI3521D_CMD_CAPTURE));
				LOG_D("HI3521D_CMD_CAPTURE");
				uart_send_with_block(dev, HI3521D_CMD_CAPTURE, sizeof(HI3521D_CMD_CAPTURE));
				LOG_D("HI3521D_CMD_CAPTURE");
             
                break;
            case CAMERA_CMD_RECORD_ON:

				uart_send_with_block(dev, (void *)CAMERA_VISCA_RECORD_ON, sizeof(CAMERA_VISCA_RECORD_ON));
				LOG_D("VISCA_CMD_CAPTURE");
						
				uart_clean_recv_buff(dev, pbuf);
				uart_send_with_block(dev, HI3521D_CMD_RECORD_ON, sizeof(HI3521D_CMD_RECORD_ON));
				LOG_D("HI3521D_CMD_RECORD_ON");
            
                break;
            case CAMERA_CMD_RECORD_OFF:
             
				uart_send_with_block(dev, (void *)CAMERA_VISCA_RECORD_OFF, sizeof(CAMERA_VISCA_RECORD_OFF));
				LOG_D("VISCA_CMD_CAPTURE");
		  
				uart_clean_recv_buff(dev, pbuf);
				uart_send_with_block(dev, HI3521D_CMD_RECORD_OFF, sizeof(HI3521D_CMD_RECORD_OFF));
				LOG_D("HI3521D_CMD_RECORD_OFF");
             
                break;
            case CAMERA_CMD_PIP_MODE1:
                uart_clean_recv_buff(dev, pbuf);
            
                uart_send_with_block(dev, HI3521D_CMD_PIP_MODE1, sizeof(HI3521D_CMD_PIP_MODE1));
                LOG_D("HI3521D_CMD_PIP1");
                break;
            case CAMERA_CMD_PIP_MODE2:
                uart_clean_recv_buff(dev, pbuf);
            
                uart_send_with_block(dev, HI3521D_CMD_PIP_MODE2, sizeof(HI3521D_CMD_PIP_MODE2));
                LOG_D("HI3521D_CMD_PIP2");
                break;
            case CAMERA_CMD_PIP_MODE3:
                uart_clean_recv_buff(dev, pbuf);
            
                uart_send_with_block(dev, HI3521D_CMD_PIP_MODE3, sizeof(HI3521D_CMD_PIP_MODE3));
                LOG_D("HI3521D_CMD_PIP3");
                break;
            case CAMERA_CMD_PIP_MODE4:
                uart_clean_recv_buff(dev, pbuf);
            
                uart_send_with_block(dev, HI3521D_CMD_PIP_MODE4, sizeof(HI3521D_CMD_PIP_MODE4));
                LOG_D("HI3521D_CMD_PIP4");
                break;
            
            // visca command
            case CAMERA_CMD_ZOOM_IN:
                uart_clean_recv_buff(dev, pbuf);
                VISCA_ZOOM_IN[4] &= 0xF0;
                VISCA_ZOOM_IN[4] |= env->cam_zoom_speed;
                uart_send_with_block(dev, (rt_uint8_t*)VISCA_ZOOM_IN, sizeof(VISCA_ZOOM_IN));
                LOG_D("VISCA_ZOOM_IN");
                break;
            case CAMERA_CMD_ZOOM_OUT:
                uart_clean_recv_buff(dev, pbuf);
                VISCA_ZOOM_OUT[4] &= 0xF0;
                VISCA_ZOOM_OUT[4] |= env->cam_zoom_speed;
                uart_send_with_block(dev, (rt_uint8_t*)VISCA_ZOOM_OUT, sizeof(VISCA_ZOOM_OUT));
                LOG_D("VISCA_ZOOM_OUT");
                break;
            case CAMERA_CMD_ZOOM_STOP:
                
                uart_clean_recv_buff(dev, pbuf);
                uart_send_with_block(dev, (rt_uint8_t*)VISCA_ZOOM_STOP, sizeof(VISCA_ZOOM_STOP));
                LOG_D("VISCA_ZOOM_STOP");
                break;
			case CAMERA_CMD_ICR_ON:
                
                uart_clean_recv_buff(dev, pbuf);
                uart_send_with_block(dev, (rt_uint8_t*)VISCA_ICR_ON, sizeof(VISCA_ICR_ON));
                LOG_D("VISCA_ICR_ON");
                break;
			case CAMERA_CMD_ICR_OFF:
                
                uart_clean_recv_buff(dev, pbuf);
                uart_send_with_block(dev, (rt_uint8_t*)VISCA_ICR_OFF, sizeof(VISCA_ICR_OFF));
                LOG_D("VISCA_ICR_OFF");
                break;
			case CAMERA_CMD_BRIGHT_GETPOS:
				uart_clean_recv_buff(dev, pbuf);
				uart_send_with_block(dev, (rt_uint8_t*)VISCA_GET_BRIGHTPOS, sizeof(VISCA_GET_BRIGHTPOS));
				LOG_D("CAMERA_CMD_BRIGHT_GETPOS");
				break;
            case 0:
                break;
            default:
                LOG_W("undefined opcode, %08X", opcode);
                break;
        }
        
        if (opcode & CAMERA_CMD_ZOOM_GETPOS)
        {
            uart_clean_recv_buff(dev, pbuf);
            uart_send_with_block(dev, (rt_uint8_t*)VISCA_GET_ZOOMPOS, sizeof(VISCA_GET_ZOOMPOS));
            LOG_D("VISCA_GET_ZOOM_POS");
        }     
        // wait the next.
    }
    
    // never be here.
}

