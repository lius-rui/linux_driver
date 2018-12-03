/*************字符设备驱动模板---内核中断和时钟操作**********************/
/************************************
 *
 * 中断底半部主要用法
 * 1.tasklet(不允许睡眠)
 * void my_tasklet_func(unsigned long);
 * DECLARE_TASKLET(my_tasklet,my_tasklet_func,data);
 * tasklet_schedule(&my_tasklet); //调度tasklet
 *
 * 2.工作队列（可以睡眠）
 * struct work_struct my_wq;   //定义一个工作队列
 * void my_wq_func(struct work_struct *work);// 定义一个处理函数
 * INIT_WORK(&my_wq, my_wq_func);   //初始化并绑定处理函数
 * schdeule_work(&my_wq);      //调度工作队列
 *
 *4.软中断(不可以睡眠)
 *
 * /
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/jiffies.h>
#include <linux/sysfs.h>
