#ifndef __AD7606_H__
#define __AD7606_H__

struct mem_config{
	unsigned int enumtype;
	unsigned short*out_c;
	unsigned short*out_5k;

};
//ad7606相关配置管脚
struct ad7606_platform_data {
	unsigned	default_os;
	unsigned	default_range;
	unsigned	gpio_convst;
	unsigned	gpio_convst2;
	unsigned	gpio_reset;
	unsigned	gpio_range;
	unsigned	gpio_os0;
	unsigned	gpio_os1;
	unsigned	gpio_os2;
	unsigned	gpio_frstdata;
	unsigned	gpio_stby;
	unsigned    gpio_busy;
	unsigned    gpio_busy2;

};

struct spidev_data {
	unsigned short buf_c[10000];
	unsigned short buf_5k[10000];
	struct spi_device	*spi1;
	struct spi_device	*spi2;
	struct ad7606_platform_data	pdata;
	unsigned			range;
	unsigned			oversampling;
	dev_t			devt;
	spinlock_t		spi_lock;
	struct list_head	device_entry;

	/* TX/RX buffers are NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	unsigned char			tx_buffer[8];
	unsigned char			rx_buffer[8];
	unsigned int 			speed_hz;

};


#define AD7606_IOCTL_MAGIC		'x'		//定义幻数
#define AD7606_IOCTL_MAX_NR		 3		//定义命令的最大值
#define READ_DATA  _IOWR(AD7606_IOCTL_MAGIC, 1, struct mem_config)
#define START_AD   _IOWR(AD7606_IOCTL_MAGIC, 2,  struct mem_config)
#define STOP_AD    _IOWR(AD7606_IOCTL_MAGIC, 3,  struct mem_config)

#endif

