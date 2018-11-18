#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <asm/io.h>
#include <linux/mm_types.h>
#include <linux/vmalloc.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/jiffies.h>
#include <linux/sysfs.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/of_device.h>

#include "ad7606.h"
#define SPIDEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);

#define DRIVER_NAME "ad7606"

unsigned int num_flag = 0; //表示采集多少个数据
unsigned int start_flag = 0; //开始采集表示
unsigned int read_flag = 0; //单次采集完成
static struct hrtimer timer; //内核高精度定时器
ktime_t kt;


struct spidev_data  ad7606_spidev;
static struct fasync_struct *fasync_queue;  //用于通知信号

static void start_ad(unsigned int sec,unsigned int nsec);
static void stop_ad(void);

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

int ad7606_reset(struct spidev_data*st)
{
	if (gpio_is_valid(ad7606_spidev.pdata.gpio_reset)) {
		gpio_set_value(ad7606_spidev.pdata.gpio_reset, 1);
		ndelay(100); /* t_reset >= 100ns */
		gpio_set_value(ad7606_spidev.pdata.gpio_reset, 0);
		printk("start reset!\r\n");
		//return 0;
	}

	gpio_set_value(ad7606_spidev.pdata.gpio_convst, 0);
	ndelay(100);
	gpio_set_value(ad7606_spidev.pdata.gpio_convst, 1);
	udelay(1000);
	spi_read(ad7606_spidev.spi1,ad7606_spidev.rx_buffer,8);

	gpio_set_value(ad7606_spidev.pdata.gpio_convst2, 0);
	ndelay(100);
	gpio_set_value(ad7606_spidev.pdata.gpio_convst2, 1);
	udelay(1000);
	spi_read(ad7606_spidev.spi2,ad7606_spidev.rx_buffer,8);


	return -ENODEV;
}
//进行一次采样读数
static void ad7606_scan_direct(struct spidev_data*st)
{
		int ret;
		gpio_set_value(ad7606_spidev.pdata.gpio_convst, 0);
		ndelay(100);
		gpio_set_value(ad7606_spidev.pdata.gpio_convst, 1);

		while(gpio_get_value(ad7606_spidev.pdata.gpio_busy));
		udelay(4);
		num_flag++;
		spi_read(ad7606_spidev.spi1,ad7606_spidev.rx_buffer,8);

		ad7606_spidev.buf_5k[num_flag-1] = be16_to_cpu(*(unsigned short*)&ad7606_spidev.rx_buffer[2]);
		ad7606_spidev.buf_c[num_flag-1] = be16_to_cpu(*(unsigned short*)&ad7606_spidev.rx_buffer[0]);

		//ad7606_spidev.buf_c[num_flag-1] =  be16_to_cpu(*(unsigned short*)&ad7606_spidev.rx_buffer[0]);
		printk("ad7606_spidev1.buf_5k[num_flag-1] = %d \r\n",ad7606_spidev.buf_5k[num_flag-1]);
		printk("ad7606_spidev1.buf_c[num_flag-1] = %d \r\n",ad7606_spidev.buf_c[num_flag-1]);

		//第二片AD采样
		gpio_set_value(ad7606_spidev.pdata.gpio_convst2, 0);
		ndelay(100);
		gpio_set_value(ad7606_spidev.pdata.gpio_convst2, 1);

		while(gpio_get_value(ad7606_spidev.pdata.gpio_busy2));
		udelay(4);
		num_flag++;
		spi_read(ad7606_spidev.spi2,ad7606_spidev.rx_buffer,8);

		ad7606_spidev.buf_5k[num_flag-1] = be16_to_cpu(*(unsigned short*)&ad7606_spidev.rx_buffer[2]);
		ad7606_spidev.buf_c[num_flag-1] = be16_to_cpu(*(unsigned short*)&ad7606_spidev.rx_buffer[0]);

		//ad7606_spidev.buf_c[num_flag-1] =  be16_to_cpu(*(unsigned short*)&ad7606_spidev.rx_buffer[0]);
		printk("ad7606_spidev2.buf_5k[num_flag-1] = %d \r\n",ad7606_spidev.buf_5k[num_flag-1]);
		printk("ad7606_spidev2.buf_c[num_flag-1] = %d \r\n",ad7606_spidev.buf_c[num_flag-1]);}

static DECLARE_WORK(my_work, ad7606_scan_direct);
/*中断处理函数*/
static enum hrtimer_restart hrtimer_handler(struct hrtimer *timer) {

	int ret;
	struct spidev_data*st;
	if(start_flag == 1)
	{
		 schedule_work(&my_work);
		 //ad7606_scan_direct(&ad7606_spidev);//采样并读数
	}
	hrtimer_forward(timer, timer->base->get_time(), kt);
	if(num_flag == 10000)
	{
		printk("spi_cs = %d \r\n",ad7606_spidev.spi1->chip_select);
		//printk("*************************end\r\n");

		num_flag = 0;
		if (fasync_queue)
			kill_fasync(&fasync_queue, SIGIO, POLL_IN);
		start_flag = 0;  //关闭采数
		return IRQ_HANDLED;
	}
	return HRTIMER_RESTART;
}



static ssize_t ad7606_write(struct file *file, const char __user *buf, size_t count, loff_t *offset){

	return 0;
}
static int ad7606_open(struct inode *node,struct file *filp){
	filp->private_data = &ad7606_spidev;
	return 0;
}
static int ad7606_fasync(int fd, struct file * filp, int on)
/*fasync方法的实现*/
{
    int retval;
    retval=fasync_helper(fd,filp,on,&fasync_queue);
    /*将该设备登记到fasync_queue队列中去*/
    if(retval<0)
      return retval;
    return 0;
}
static int ad7606_release(struct inode *node,struct file *filp){
	filp->private_data = NULL;
	ad7606_fasync(-1, filp, 0);
	return 0;
}

static ssize_t ad7606_read(struct file *filp,char *buf, size_t count, loff_t *offset){
	return 0;
}

/*使用ioctl发送报文数据*/
static int ad7606_ioctl(struct file *filp,unsigned int cmd, unsigned long arg){
         //printk("ad7606_ioctl start\n");
       struct spidev_data *ad7606 = filp->private_data;
      // printk("fff = %d \r\n",ad7606_spidev.speed_hz);
       int err,rc;
       void __user *argp = (void __user *)arg;
       struct mem_config kern_config;
       struct mem_config *config = &kern_config;
       err = copy_from_user(config, argp, sizeof(kern_config));  //从应用层接收数据
       if (err) {
			err = -EFAULT;
                    printk("copy_from_user err\n");
			return err;
       }
		switch (cmd) {

		case READ_DATA:
			{
				rc = copy_to_user(config->out_c,ad7606->buf_c, 20000);
				//printk("ad7606->buf = %d \r\n",ad7606->buf_c[99]);
				rc = copy_to_user(config->out_5k, ad7606->buf_5k,20000);

			}
			break;
		case START_AD:
			{
			//	printk("**********************START_AD\r\n");
				start_flag = 1;
			}
			break;
		case STOP_AD:
			{
				printk("STOP_AD\r\n");
			}
			break;
		default:
			break;
		}
		return 0;
}

static int ad7606_request_gpios(struct spidev_data*st)
{
	int ret;
	//申请启动采样管脚IO
	//printk("1111111111 = %d \r\n",ad7606_spidev.pdata.gpio_convst);
	if (gpio_is_valid(ad7606_spidev.pdata.gpio_convst)) {
	//	printk("convst = %d \r\n",ad7606_spidev.pdata.gpio_convst);
		ret = gpio_request(ad7606_spidev.pdata.gpio_convst,"AD7606_CONVST");
		ret = gpio_direction_output(ad7606_spidev.pdata.gpio_convst,0);
		if (ret) {
			dev_err(st->spi1, "failed to request GPIO CONVST\n");
			goto error_ret;
		}

	} else {
		ret = -EIO;
		goto error_ret;
	}

	if (gpio_is_valid(ad7606_spidev.pdata.gpio_convst2)) {
	//	printk("convst = %d \r\n",ad7606_spidev.pdata.gpio_convst);
		ret = gpio_request(ad7606_spidev.pdata.gpio_convst2,"AD7606_CONVST2");
		ret = gpio_direction_output(ad7606_spidev.pdata.gpio_convst2,0);
		if (ret) {
			dev_err(st->spi2, "failed to request GPIO CONVST\n");
			goto error_ret;
		}

	} else {
		ret = -EIO;
		goto error_ret;
	}

  //申请设置过采样管脚
	//printk("22222222222222 = %d \r\n",ad7606_spidev.pdata.gpio_os0);
	if (gpio_is_valid(ad7606_spidev.pdata.gpio_os0) &&
	    gpio_is_valid(ad7606_spidev.pdata.gpio_os1) &&
	    gpio_is_valid(ad7606_spidev.pdata.gpio_os2)) {
		ret = gpio_request(ad7606_spidev.pdata.gpio_os0,"AD7606_OS0");
		ret = gpio_direction_output(ad7606_spidev.pdata.gpio_os0,0);
		ret = gpio_request(ad7606_spidev.pdata.gpio_os1,"AD7606_OS1");
		ret = gpio_direction_output(ad7606_spidev.pdata.gpio_os1,0);
		ret = gpio_request(ad7606_spidev.pdata.gpio_os2,"AD7606_OS2");
		ret = gpio_direction_output(ad7606_spidev.pdata.gpio_os2,0);
		if (ret < 0)
			goto error_free_convst;
	}
  //申请设置复位管脚
	if (gpio_is_valid(ad7606_spidev.pdata.gpio_reset)) {
		ret = gpio_request(ad7606_spidev.pdata.gpio_reset,"AD7606_RESET");
		ret = gpio_direction_output(ad7606_spidev.pdata.gpio_reset,0);
		if (ret < 0)
			goto error_free_os;
	}
	//设置采样范围 +-10v
	if (gpio_is_valid(ad7606_spidev.pdata.gpio_range)) {
		ret = gpio_request(ad7606_spidev.pdata.gpio_range,"AD7606_RANGE");
		ret = gpio_direction_output(ad7606_spidev.pdata.gpio_range,1); //+-10v
		if (ret < 0)
			goto error_free_reset;
	}
  //申请设置busy管脚
	if (gpio_is_valid(ad7606_spidev.pdata.gpio_busy)) {
		ret = gpio_request(ad7606_spidev.pdata.gpio_busy,"AD7606_BUSY");
		ret = gpio_direction_input(ad7606_spidev.pdata.gpio_busy);
		if (ret < 0)
			goto error_free_busy;
	}

	if (gpio_is_valid(ad7606_spidev.pdata.gpio_busy2)) {
		ret = gpio_request(ad7606_spidev.pdata.gpio_busy2,"AD7606_BUSY2");
		ret = gpio_direction_input(ad7606_spidev.pdata.gpio_busy2);
		if (ret < 0)
			goto error_free_busy;
	}

	ret = ad7606_reset(st);


	return 0;

error_free_busy:
	if (gpio_is_valid(ad7606_spidev.pdata.gpio_busy))
		gpio_free(ad7606_spidev.pdata.gpio_busy);
	if (gpio_is_valid(ad7606_spidev.pdata.gpio_busy2))
		gpio_free(ad7606_spidev.pdata.gpio_busy2);

error_free_range:
	if (gpio_is_valid(ad7606_spidev.pdata.gpio_range))
		gpio_free(ad7606_spidev.pdata.gpio_range);
error_free_reset:
	if (gpio_is_valid(ad7606_spidev.pdata.gpio_reset))
		gpio_free(ad7606_spidev.pdata.gpio_reset);
error_free_os:
	if (gpio_is_valid(ad7606_spidev.pdata.gpio_os0) &&
	    gpio_is_valid(ad7606_spidev.pdata.gpio_os1) &&
	    gpio_is_valid(ad7606_spidev.pdata.gpio_os2))
		{
			gpio_free(ad7606_spidev.pdata.gpio_os0);
			gpio_free(ad7606_spidev.pdata.gpio_os1);
			gpio_free(ad7606_spidev.pdata.gpio_os2);
		}
error_free_convst:
	gpio_free(ad7606_spidev.pdata.gpio_convst);
	gpio_free(ad7606_spidev.pdata.gpio_convst2);

error_ret:
	return ret;
}

static int ad7606_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct ad7606_platform_data *pdata;
	int ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	ret = of_property_read_u8(np, "default-os",
			(u8 *) &pdata->default_os);
	if (ret < 0)
		pdata->default_os = 0;
	ad7606_spidev.pdata.default_os = pdata->default_os;
	//printk("pdata->default_os = %x \r\n",ad7606_spidev.pdata.default_os);

	ret = of_property_read_u16(np, "default-range",
			(u16 *) &pdata->default_range);
	if (ret < 0)
		pdata->default_range = 10000;
	ad7606_spidev.pdata.default_range = 10000;
	//printk("pdata->default_range = %x \r\n",ad7606_spidev.pdata.default_range);

	ret = of_get_named_gpio(np, "convst-gpio", 0);
	if (ret < 0) {
		dev_err(dev, "convst-gpio property not found\n");
		goto error_free;
	}
	pdata->gpio_convst = ret;
	ad7606_spidev.pdata.gpio_convst = pdata->gpio_convst;

	ret = of_get_named_gpio(np, "convst2-gpio", 0);
	if (ret < 0) {
		dev_err(dev, "convst2-gpio property not found\n");
		goto error_free;
	}
	pdata->gpio_convst2 = ret;
	ad7606_spidev.pdata.gpio_convst2 = pdata->gpio_convst2;

	ret = of_get_named_gpio(np, "reset-gpio", 0);
	if (ret < 0)
		pdata->gpio_reset = -1;
	else
		pdata->gpio_reset = ret;
	ad7606_spidev.pdata.gpio_reset = pdata->gpio_reset;

	ret = of_get_named_gpio(np, "range-gpio", 0);
	if (ret < 0)
		pdata->gpio_range = -1;
	else
		pdata->gpio_range = ret;
	ad7606_spidev.pdata.gpio_range = pdata->gpio_range;

	ret = of_get_named_gpio(np, "os0-gpio", 0);
	if (ret < 0)
		pdata->gpio_os0 = -1;
	else
		pdata->gpio_os0 = ret;
	ad7606_spidev.pdata.gpio_os0 = pdata->gpio_os0;

	ret = of_get_named_gpio(np, "os1-gpio", 0);
	if (ret < 0)
		pdata->gpio_os1 = -1;
	else
		pdata->gpio_os1 = ret;
	ad7606_spidev.pdata.gpio_os1 = pdata->gpio_os1;

	ret = of_get_named_gpio(np, "os2-gpio", 0);
	if (ret < 0)
		pdata->gpio_os2 = -1;
	else
		pdata->gpio_os2 = ret;
	ad7606_spidev.pdata.gpio_os2 = pdata->gpio_os2;

	ret = of_get_named_gpio(np, "busy-gpio", 0);
	if (ret < 0)
		pdata->gpio_busy = -1;
	else
		pdata->gpio_busy = ret;
	ad7606_spidev.pdata.gpio_busy = pdata->gpio_busy;

	ret = of_get_named_gpio(np, "busy2-gpio", 0);
	if (ret < 0)
		pdata->gpio_busy2 = -1;
	else
		pdata->gpio_busy2 = ret;
	ad7606_spidev.pdata.gpio_busy2 = pdata->gpio_busy2;

	dev->platform_data = pdata;

	return 1;

error_free:
	devm_kfree(dev, pdata);
	return ret;
};


static void start_ad(unsigned int sec,unsigned int nsec)
{
	kt = ktime_set(sec, nsec); /* 1 sec, 10 nsec */
	hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer.function = hrtimer_handler;
	hrtimer_start(&timer, kt, HRTIMER_MODE_REL);
}

static struct class *spidev_class;
static const struct of_device_id spidev_dt_ids[] = {
	{ .compatible = "rohm,dh2228fv" },
	{ .compatible = "lineartechnology,ltc2488" },
	{},
};
MODULE_DEVICE_TABLE(of, spidev_dt_ids);

static void ad7606_free_gpios(struct spidev_data *st)
{
	if (gpio_is_valid(ad7606_spidev.pdata.gpio_range))
		gpio_free(ad7606_spidev.pdata.gpio_range);
	if (gpio_is_valid(ad7606_spidev.pdata.gpio_reset))
		gpio_free(ad7606_spidev.pdata.gpio_reset);
	if (gpio_is_valid(ad7606_spidev.pdata.gpio_os0) &&
	    gpio_is_valid(ad7606_spidev.pdata.gpio_os1) &&
	    gpio_is_valid(ad7606_spidev.pdata.gpio_os2)) {
		gpio_free(ad7606_spidev.pdata.gpio_os0);
		gpio_free(ad7606_spidev.pdata.gpio_os1);
		gpio_free(ad7606_spidev.pdata.gpio_os2);
	}
	gpio_free(ad7606_spidev.pdata.gpio_convst);
	gpio_free(ad7606_spidev.pdata.gpio_busy);
}

static int spidev_probe(struct spi_device *spi)
{
	int ret;
	struct spidev_data	*spidev;
	//struct ad7606_platform_data *pdata;
	int			status;
	unsigned long		minor;

	ret = ad7606_parse_dt(spi);
	if (ret < 0)
		return ERR_PTR(ret);
	//pdata = spi->dev.platform_data;;
	/*
	 * spidev should never be referenced in DT without a specific
	 * compatible string, it is a Linux implementation thing
	 * rather than a description of the hardware.
	 */
	if (spi->dev.of_node && !of_match_device(spidev_dt_ids, &spi->dev)) {
		dev_err(&spi->dev, "buggy DT: spidev listed directly in DT\n");
		WARN_ON(spi->dev.of_node &&
			!of_match_device(spidev_dt_ids, &spi->dev));
	}

	/* Allocate driver data */
//	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
//	if (!spidev)
//		return -ENOMEM;

	/* Initialize the driver data */
	//spidev->spi = spi;
	printk("spi = %d \r\n",spi->chip_select);
	if(spi->chip_select==0)ad7606_spidev.spi1 = spi;
	else if(spi->chip_select==1) ad7606_spidev.spi2 = spi;
	//INIT_LIST_HEAD(&spidev->device_entry);
	INIT_LIST_HEAD(&ad7606_spidev.device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;
		ad7606_spidev.devt = MKDEV(SPIDEV_MAJOR, minor);
		//spidev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(spidev_class, &spi->dev, ad7606_spidev.devt,
				    &ad7606_spidev, "spidev%d.%d",
				    spi->master->bus_num, spi->chip_select);
		status = PTR_ERR_OR_ZERO(dev);
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
	//	list_add(&spidev->device_entry, &device_list);
		list_add(&ad7606_spidev.device_entry, &device_list);

	}
	//spidev->speed_hz = spi->max_speed_hz;
	ad7606_spidev.speed_hz = spi->max_speed_hz;

	ad7606_request_gpios(&ad7606_spidev);
	ad7606_reset(&ad7606_spidev);

//	printk("ad7606_spidev = %x \r\n",ad7606_spidev.rx_buffer);
//	if (status == 0)
//		//spi_set_drvdata(spi, &ad7606_spidev);
//	else
//		;//kfree(spidev);
	return status;
}

static int spidev_remove(struct spi_device *spi)
{
	struct spidev_data	*spidev = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spidev->spi_lock);
	spidev->spi1 = NULL;
	spin_unlock_irq(&spidev->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&spidev->device_entry);
	device_destroy(spidev_class, spidev->devt);
	clear_bit(MINOR(spidev->devt), minors);
	if (spidev->users == 0)
		kfree(spidev);
	mutex_unlock(&device_list_lock);

		return 0;
}

static const struct file_operations spidev_fops = {

	.owner =	THIS_MODULE,
	.write = ad7606_write,
	.read =	ad7606_read,
	.unlocked_ioctl = ad7606_ioctl,
	.open =		ad7606_open,
	.release =	ad7606_release,
	.fasync  = ad7606_fasync,
};




static struct spi_driver spidev_spi_driver = {
	.driver = {
		.name =	"spidev",
		.of_match_table = of_match_ptr(spidev_dt_ids),
	},
	.probe =	spidev_probe,
	.remove =	spidev_remove,
};



static int __init spidev_init(void)
{
	int status;

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, "spi", &spidev_fops);
	if (status < 0)
		return status;
	printk("start ad sample\n");
	start_ad(0,100000);
	spidev_class = class_create(THIS_MODULE, "spidev");
	if (IS_ERR(spidev_class)) {
		unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
		return PTR_ERR(spidev_class);
	}

	status = spi_register_driver(&spidev_spi_driver);
	if (status < 0) {
		class_destroy(spidev_class);
		unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
	}
	return status;
}
module_init(spidev_init);

static void __exit spidev_exit(void)
{
	spi_unregister_driver(&spidev_spi_driver);
	class_destroy(spidev_class);
	hrtimer_cancel(&timer); //取消定时器
	unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
}
module_exit(spidev_exit);

MODULE_AUTHOR("liushuang <liushuang@lhrtt.com>");
MODULE_DESCRIPTION("Analog Devices AD7606 ADC");
MODULE_LICENSE("GPL v2");
