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
#include <math.h>

/* defined the LED0 pin: PE4 */
#define LED0_PIN    GET_PIN(E, 4)

#define RXBUFF_SIZE 						(1024)
#define PLANE_R_DATA_SIZE				(20)
#define PLANE_R_DATA_HEADER			(0xEE)
#define PTZ_R_DATA_SIZE					(8)
#define PTZ_R_DATA_HEADER				(0xEE)
#define GCS_R_DATA_SIZE					(5)
#define GCS_R_DATA_HEADER				(0xE1)

static rt_device_t  pUart2 = RT_NULL;
static rt_sem_t     sUart2 = RT_NULL;
static rt_device_t  pUart3 = RT_NULL;
static rt_sem_t     sUart3 = RT_NULL;
static rt_device_t  pUart6 = RT_NULL;
static rt_sem_t     sUart6 = RT_NULL;

static rt_uint8_t uart2_rx_stack[ 1024 ];
static struct rt_thread uart2_rx_thread;
static rt_uint8_t uart3_rx_stack[ 1024 ];
static struct rt_thread uart3_rx_thread;
static rt_uint8_t uart6_rx_stack[ 1024 ];
static struct rt_thread uart6_rx_thread;

rt_uint8_t data_send_plane[20]={0xEE,0x90,0x51,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
rt_uint8_t data_send_gcs[10]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
rt_uint8_t data_effective=0x00;

double PI=3.141592654;
double factor_lng=1.0;
float dronelat=20.0f;
float dronelng=100.0f;
float dronealt=200.0f;
float droneyaw=0.0f;
float ptzpitch=0.0f;
float ptzyaw=0.0f;
float ptzdistance=0.0f;
float locationlat=0.0f;
float locationlng=0.0f;
float locationalt=0.0f;

//将经纬度转换为距离
float calculateLineDistance(double lat0,double lng0,double lat1,double lng1) {
	long double var4 = lng0;
	long double var6 = lat0;
	long double var8 = lng1;
	long double var10 = lat1;
	var4 *= 0.01745329251994329;
	var6 *= 0.01745329251994329;
	var8 *= 0.01745329251994329;
	var10 *= 0.01745329251994329;
	long double var12 = sin(var4);
	long double var14 = sin(var6);
	long double var16 = cos(var4);
	long double var18 = cos(var6);
	long double var20 = sin(var8);
	long double var22 = sin(var10);
	long double var24 = cos(var8);
	long double var26 = cos(var10);
	long double var28[3] = {0};
	long double var29[3] = {0};
	var28[0] = var18 * var16;
	var28[1] = var18 * var12;
	var28[2] = var14;
	var29[0] = var26 * var24;
	var29[1] = var26 * var20;
	var29[2] = var22;
	double var30 = sqrt((var28[0] - var29[0]) * (var28[0] - var29[0]) + (var28[1] - var29[1]) * (var28[1] - var29[1]) + (var28[2] - var29[2]) * (var28[2] - var29[2]));
	return (float)(asin(var30 / 2.0) * 1.27420015798544E7);
}
static rt_err_t rx2_hook(rt_device_t dev, rt_size_t sz)
{
	rt_sem_release(sUart2);   
	return RT_EOK;
}
static rt_err_t rx3_hook(rt_device_t dev, rt_size_t sz)
{
	rt_sem_release(sUart3);   
	return RT_EOK;
}
static rt_err_t rx6_hook(rt_device_t dev, rt_size_t sz)
{
	rt_sem_release(sUart6);   
	return RT_EOK;
}
//发送消息到地面站
static void uart2_send_entry(void* parameter)
{
	while(RT_TRUE)
	{
		rt_uint16_t locationalt_uint16=(rt_uint16_t)locationalt;
		rt_uint32_t locationlng_uint32=(rt_uint32_t)(locationlng*10000000);
		rt_uint32_t locationlat_uint32=(rt_uint32_t)(locationlat*10000000);
		rt_memcpy(&data_send_gcs[0],&locationalt_uint16,sizeof(rt_uint16_t));
		rt_memcpy(&data_send_gcs[2],&locationlng_uint32,sizeof(rt_uint32_t));
		rt_memcpy(&data_send_gcs[6],&locationlat_uint32,sizeof(rt_uint32_t));
		if(locationlat>0.0f&&locationlat<180.0f&&locationlng>0.0f&&locationlng<180.0f)
		{
			rt_device_write(pUart2, 0, data_send_gcs, 10);
		}
		rt_thread_mdelay(300);
	}
}
//接收地面站消息
static void uart2_rx_entry(void* parameter)
{
	rt_err_t retval=RT_EOK;
	rt_size_t szbuf=0;
	rt_uint8_t *pbuf=RT_NULL;
	rt_thread_t tidU2Tx = RT_NULL;
	
	pUart2 = rt_device_find("uart2");
  RT_ASSERT(pUart2 != RT_NULL);
  rt_device_open(pUart2, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
	
	sUart2 = rt_sem_create("u2rx", 0, RT_IPC_FLAG_FIFO);
  rt_device_set_rx_indicate(pUart2, rx2_hook);
	
	tidU2Tx = rt_thread_create("tTX2", uart2_send_entry, NULL, 1024, 11, RT_TICK_PER_SECOND/20);
  if (tidU2Tx)
	{
		rt_thread_startup(tidU2Tx);
	}
	
	while(RT_TRUE)
	{
		retval=rt_sem_take(sUart2,RT_WAITING_FOREVER);
		if(retval==-RT_ETIMEOUT)
			continue;
		switch(szbuf){
			case 0:
			case 1:
				szbuf+=rt_device_read(pUart2,0,pbuf+szbuf,1);
				if (pbuf[0] != GCS_R_DATA_HEADER)
					szbuf = 0;
				break;
			default:
				szbuf += rt_device_read(pUart2, 0, pbuf + szbuf, GCS_R_DATA_SIZE - szbuf);
				break;
		}
		if (szbuf != GCS_R_DATA_SIZE)
			continue; 
		szbuf = 0;
		rt_kprintf("get uart2 data! \n");
		rt_device_write(pUart6, 0, pbuf, GCS_R_DATA_SIZE);
	}
	
}
//发送消息到飞控
static void uart3_send_entry(void* parameter)
{
	while(RT_TRUE)
	{
		rt_int32_t locationlng_int32=(rt_int32_t)(locationlng*10000000.0f);
		rt_int32_t locationlat_int32=(rt_int32_t)(locationlat*10000000.0f);
		rt_uint8_t data1=0x00;
		rt_uint8_t data2=0x00;
		
		rt_memset(&data_send_plane, 0x00, 20);
		data_send_plane[0]=0xEE;
		data_send_plane[1]=0x90;
		data_send_plane[2]=0x51;
		
		rt_memcpy(&data_send_plane[3],&locationlng_int32,sizeof(rt_int32_t));
		rt_memcpy(&data_send_plane[7],&locationlat_int32,sizeof(rt_int32_t));
		data1=data_send_plane[3];
		data2=data_send_plane[4];
		data_send_plane[3]=data_send_plane[6];
		data_send_plane[4]=data_send_plane[5];
		data_send_plane[5]=data2;
		data_send_plane[6]=data1;
		data1=data_send_plane[7];
		data2=data_send_plane[8];
		data_send_plane[7]=data_send_plane[10];
		data_send_plane[8]=data_send_plane[9];
		data_send_plane[9]=data2;
		data_send_plane[10]=data1;
		
		rt_int16_t dronealt_int=(rt_int16_t)((locationalt-2000.0f)*10.0f);
		rt_memcpy(&data_send_plane[11],&dronealt_int,2);
		data1=data_send_plane[11];
		data_send_plane[11]=data_send_plane[12];
		data_send_plane[12]=data1;
		rt_int16_t ptzdistance_int=(rt_int16_t)(ptzdistance*10.0f);
		data_send_plane[13]=data_effective;
		rt_memcpy(&data_send_plane[14],&ptzdistance_int,2);
		data1=data_send_plane[14];
		data_send_plane[14]=data_send_plane[15];
		data_send_plane[15]=data1;
		rt_int16_t ptzyaw_int=(rt_int16_t)(ptzyaw*100.0f);
		rt_memcpy(&data_send_plane[16],&ptzyaw_int,2);
		data1=data_send_plane[16];
		data_send_plane[16]=data_send_plane[17];
		data_send_plane[17]=data1;
		for(int i = 0; i < 18; i++)
		{
		 data_send_plane[18]+=data_send_plane[i];
		}
		data_send_plane[19]=data_send_plane[0];
		for(int i = 1; i < 19; i++)
		{
		 data_send_plane[19]^=data_send_plane[i];
		}
		if(locationlat>0.0f&&locationlat<180.0f&&locationlng>0.0f&&locationlng<180.0f)
		{
			rt_device_write(pUart3, 0, data_send_plane, 20);
		}
		rt_thread_mdelay(40);
	}
}
//接收飞控指令，给dronelat/dronelng/dronealt/droneyaw赋值
static void uart3_rx_entry(void* parameter)
{
	rt_err_t retval = RT_EOK;
	rt_size_t szbuf = 0;
	rt_uint8_t *pbuf = RT_NULL;
	rt_thread_t tidU3Tx = RT_NULL;

	pUart3 = rt_device_find("uart3");
	RT_ASSERT(pUart3 != RT_NULL);
	rt_device_open(pUart3, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

	sUart3 = rt_sem_create("u3rx", 0, RT_IPC_FLAG_FIFO);
	rt_device_set_rx_indicate(pUart3, rx3_hook);

	tidU3Tx = rt_thread_create("tTX3", uart3_send_entry, NULL, 1024, 11, RT_TICK_PER_SECOND/20);
	if (tidU3Tx)
	{
		rt_thread_startup(tidU3Tx);
	}

	pbuf = rt_malloc(RXBUFF_SIZE);
	rt_memset(pbuf, 0x00, RXBUFF_SIZE);

	while(RT_TRUE)
	{
		retval = rt_sem_take(sUart3, RT_WAITING_FOREVER); 
		if(retval == -RT_ETIMEOUT)  
			continue; 	
		switch (szbuf){
			case 0:
			case 1:
				szbuf += rt_device_read(pUart3, 0, pbuf + szbuf, 1);
				if (pbuf[0] != PLANE_R_DATA_HEADER)
					szbuf = 0;
				break;
			default:
				szbuf += rt_device_read(pUart3, 0, pbuf + szbuf, PLANE_R_DATA_SIZE - szbuf);
				break;
		}
		if (szbuf != PLANE_R_DATA_SIZE)
			continue; 
		szbuf = 0;
		rt_kprintf("get uart3 data! \n");
		rt_int32_t dronelng_int32=0x00;
		rt_int32_t dronelat_int32=0x00;
		rt_uint8_t data1=0x00;
		rt_uint8_t data2=0x00;
		
		data1=pbuf[3];
		data2=pbuf[4];
		pbuf[3]=pbuf[6];
		pbuf[4]=pbuf[5];
		pbuf[5]=data2;
		pbuf[6]=data1;
		data1=pbuf[7];
		data2=pbuf[8];
		pbuf[7]=pbuf[10];
		pbuf[8]=pbuf[9];
		pbuf[9]=data2;
		pbuf[10]=data1;
		rt_memcpy(&dronelng_int32,&pbuf[3],sizeof(float));
		rt_memcpy(&dronelat_int32,&pbuf[7],sizeof(float));
		dronelng=(float)dronelng_int32*0.0000001f;
		dronelat=(float)dronelat_int32*0.0000001f;
		
		rt_int16_t dronealt_int=0;
		data1=pbuf[11];
		pbuf[11]=pbuf[12];
		pbuf[12]=data1;
		rt_memcpy(&dronealt_int,&pbuf[11],2);
		dronealt=(float)dronealt_int*0.1f+2000.0f;
		//rt_kprintf("lat:%d  lng:%d  alt%d \n",(rt_uint32_t)(dronelat*1000000),(rt_uint32_t)(dronelng*1000000),(rt_int16_t)dronealt);
		rt_int16_t droneyaw_int=0;
		data1=pbuf[14];
		pbuf[14]=pbuf[15];
		pbuf[15]=data1;
		rt_memcpy(&droneyaw_int,&pbuf[14],2);
		droneyaw=(float)droneyaw_int*0.01f;
	}
}
//接收吊舱框架角和激光数据并计算结果
static void uart6_rx_entry(void* parameter)
{
	rt_err_t retval = RT_EOK;
	rt_size_t szbuf = 0;
	rt_uint8_t *pbuf = RT_NULL;

	pUart6 = rt_device_find("uart6");
	RT_ASSERT(pUart6 != RT_NULL);
	rt_device_open(pUart6, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

	sUart6 = rt_sem_create("u6rx", 0, RT_IPC_FLAG_FIFO);
	rt_device_set_rx_indicate(pUart6, rx6_hook);

	pbuf = rt_malloc(RXBUFF_SIZE);
	rt_memset(pbuf, 0x00, RXBUFF_SIZE);

	while(RT_TRUE)
	{
		retval = rt_sem_take(sUart6, RT_WAITING_FOREVER); 
		if(retval == -RT_ETIMEOUT)  
				continue; 
		switch (szbuf){
		case 0:
		case 1:
			szbuf += rt_device_read(pUart6, 0, pbuf + szbuf, 1);
			if (pbuf[0] != PTZ_R_DATA_HEADER)
					szbuf = 0;
			break;
		default:
			szbuf += rt_device_read(pUart6, 0, pbuf + szbuf, PTZ_R_DATA_SIZE - szbuf);
			break;
		}
		if (szbuf != PTZ_R_DATA_SIZE)
			continue;
		szbuf = 0;
		ptzdistance=(float)(pbuf[1]+pbuf[2]*256);
		ptzdistance=ptzdistance*0.1f;
		ptzpitch=(float)(pbuf[3]+pbuf[4]*256);
		ptzyaw=(float)(pbuf[5]+pbuf[6]*256);
		ptzpitch = ptzpitch * 0.02197265625;
		ptzyaw = ptzyaw * 0.02197265625;
		
		data_effective=pbuf[7];

		//计算当前经度向东走0.01度为多少距离，单位为厘米
		factor_lng = calculateLineDistance(dronelat,dronelng,dronelat,(dronelng + 0.01f)) * 100.0f;
		float allyaw=droneyaw+ptzyaw;
		
		rt_kprintf("ptzyaw:%d  ptzpitch:%d  ptzdistance:%d \n",(rt_uint16_t)ptzyaw,(rt_uint16_t)ptzpitch,(rt_uint8_t)ptzdistance);
		float x = ptzdistance*sin(PI / 180.0f * (90-ptzpitch)) * sin(PI / 180.0f * allyaw);
		float y = ptzdistance*sin(PI / 180.0f * (90-ptzpitch)) * cos(PI / 180.0f * allyaw);
		locationlat = dronelat + y * 180.0f / (6371000.0f * PI);
		locationlng = dronelng + x / factor_lng;
		locationalt= dronealt-ptzdistance*cos(PI / 180.0f * (90-ptzpitch));
		//rt_kprintf("lat:%d  lng:%d  alt%d \n",(rt_uint32_t)(locationlat*1000000),(rt_uint32_t)(locationlng*1000000),(rt_uint8_t)locationalt);
	}
}

int main(void)
{
	rt_err_t retval = RT_EOK;

	/* init uart2 rx thread */
	retval = rt_thread_init(&uart2_rx_thread,
													"tRX2",
													uart2_rx_entry,
													RT_NULL,
													(rt_uint8_t*)&uart2_rx_stack[0],
													sizeof(uart2_rx_stack), 12, 5);
	if (retval == RT_EOK)
	{
		rt_kprintf("thread \"uart2_rx\"start.\n");
		rt_thread_startup(&uart2_rx_thread);
	}
	/* init uart3 rx thread */
	retval = rt_thread_init(&uart3_rx_thread,
													"tRX3",
													uart3_rx_entry,
													RT_NULL,
													(rt_uint8_t*)&uart3_rx_stack[0],
													sizeof(uart3_rx_stack), 12, 5);
	if (retval == RT_EOK)
	{
		rt_kprintf("thread \"uart3_rx\"start.\n");
		rt_thread_startup(&uart3_rx_thread);
	}
	/* init uart6 rx thread */
	retval = rt_thread_init(&uart6_rx_thread,
													"tRX6",
													uart6_rx_entry,
													RT_NULL,
													(rt_uint8_t*)&uart6_rx_stack[0],
													sizeof(uart6_rx_stack), 12, 5);
	if (retval == RT_EOK)
	{
		rt_kprintf("thread \"uart6_rx\"start.\n");
		rt_thread_startup(&uart6_rx_thread);
	}
	/* set LED0 pin mode to output */
	rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
	
	while (RT_TRUE)
	{
		rt_pin_write(LED0_PIN, PIN_HIGH);
		rt_thread_mdelay(500);
		rt_pin_write(LED0_PIN, PIN_LOW);
		rt_thread_mdelay(500);
	}

	// Never reach here.
}
