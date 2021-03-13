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

/* defined the LED0 pin: PE4 */
#define LED0_PIN    GET_PIN(E, 4)

#define PWM_IN_PIN    GET_PIN(A, 3)

#define PWM_DEV_NAME        "pwm2"
#define PWM_DEV_CHANNEL     3
struct rt_device_pwm *pwm_dev;

rt_uint16_t count=0;
int status=0,laststatus=0,pwmflag=0;

//void read_pwm(void *args)
//{
//    rt_kprintf("read_pwm %d!\n",count);
//	count=0;
//}

static void set_pwm_entry(void *parameter)
{
    while (1)
    {
        rt_hw_us_delay(100);
		status=rt_pin_read(PWM_IN_PIN);
		if(laststatus!=status)
		{
			//rt_kprintf("read_pwm %d!\n",count);
			if(count<100)
			{
				if(count>14)
				{
					pwmflag=1;
				}
				else
				{
					pwmflag=0;
				}
			}
			count=0;
		}
		laststatus=status;
		count++;
    }
}

int main(void)
{   
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
	
	rt_uint32_t period;
	period = 20000000;
	
	pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("pwm sample run failed!\n");
    }
	else
	{
		rt_kprintf("pwm sample run succeed!\n");
	}
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1200000);
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);
	
	rt_pin_mode(PWM_IN_PIN, PIN_MODE_INPUT);
//    rt_pin_attach_irq(PWM_IN_PIN, PIN_IRQ_MODE_RISING_FALLING, read_pwm, RT_NULL);
//    rt_pin_irq_enable(PWM_IN_PIN, PIN_IRQ_ENABLE);
	
	rt_thread_t thread = rt_thread_create("setpwm", set_pwm_entry, RT_NULL, 1024, 25, 10);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
	rt_uint8_t pwm_value=120;
    while (1)
    {
		rt_pin_write(LED0_PIN, PIN_HIGH);
		if(pwmflag==1)
		{
			pwm_value=120;
			for(int i=0;i<27;i++)
			{
				rt_thread_mdelay(30);
				rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, pwm_value*10000);
				pwm_value++;
			}
		}
		else
		{
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
		}
		rt_pin_write(LED0_PIN, PIN_LOW);
		if(pwmflag==1)
		{
			pwm_value=147;
			for(int i=0;i<27;i++)
			{
				rt_thread_mdelay(30);
				rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, pwm_value*10000);
				pwm_value--;
			}
		}
		else
		{
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
			rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, 1470000);
			rt_thread_mdelay(50);
		}
    }
}
