/*************字符设备驱动模板**********************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define MEM_SIZE  0x1000 //４Ｋ
#define MEM_CLEAR  0x01
#define GLOBALMEM_MAJOR   230 //主设备号
#define GLOBALMEM_MAGIC  'g'  //幻数
#define MEM_CLEAR  _IO(GLOBALMEM_MAGIC,0)  //清空内存的ioctl命令
#define DEVICE_NUM  10  //最多同时支持10个同类设备

static int globalmem_major = GLOBALMEM_MAJOR;
module_param(globalmem_major,int,S_IRUGO);

//自定义的字符设备结构体

struct globalmem_dev
{
	struct cdev cdev;//内核自带的字符设备
	unsigned char mem[MEM_SIZE];  //开辟的内存空间大小
};

struct globalmem_dev *globalmem_devp;  //设备结构体指针，后面会赋值给私有数据使用



static ssize_t globalmem_read(struct  file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	unsigned long p = *ppos;
	unsigned int count  = size;
	int ret = 0;
	struct globalmem_dev  *dev = filp->private_data;

	if(p >= MEM_SIZE)
		return 0;
	if(count > MEM_SIZE - p) //最大读取数据量
		count  = MEM_SIZE - p;

	if(copy_to_user(buf,dev->mem + p, count))
	{
		return -EFAULT;
	}
	else{

		*ppos += count;
		ret = count;
		printk(KERN_INFO"read data ok!\n");
	}
	return ret;
}

static ssize_t globalmem_write(struct file *filp, const char __user *buf,size_t size, loff_t *ppos)
{
	unsigned long p = *ppos;
	unsigned int count = size;
	int ret = 0;
	struct globalmem_dev *dev = filp->private_data;

	if(p >= MEM_SIZE)
		return 0;
	if(count > MEM_SIZE - p)
		count = MEM_SIZE -p;
	if(copy_from_user(dev->mem, buf, count))
	{
		return -EFAULT;
	}
	else
	{
		*ppos += count;
		ret = count;
		printk(KERN_INFO"write data ok!\r\n");
	}
	return ret;
}

static loff_t globalmem_llseek(struct file *filp, loff_t offset , int orig)
{
	loff_t ret = 0;
	switch(orig)
	{
	case 0 :  //从文件开头位置seek,SEEK_SET
		if(offset < 0 || ((unsigned int)offset > MEM_SIZE))
		{
			ret = -EINVAL;
			break;
		}
		filp->f_pos = (unsigned int)offset;
		ret = filp->f_pos;
		break;
	case 1:   //从文件当前位置开始seek,　SEEK_CUR
		if((filp->f_pos + (unsigned int)offset) > MEM_SIZE || (filp->f_pos + (unsigned int)offset) < 0)
		{
			ret = -EINVAL;
			break;
		}
		filp->f_pos += (unsigned int)offset;
		ret = filp->f_pos;
		break;
	default:
		ret = -EINVAL;
		 break;
	}

	return ret;
}

static long globalmem_ioctl(struct file *filp,unsigned int cmd,unsigned long arg)
{
	struct globalmem_dev *dev = filp->private_data;
	switch(cmd)
	{
	case MEM_CLEAR:
		memset(dev->mem,0,MEM_SIZE);
		printk(KERN_INFO"globalmem is set zero\n");
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int globalmem_open(struct inode *inode, struct file *filp)
{
	//filp->private_data = globalmem_devp;
	struct globalmem_dev *dev = container_of(inode->i_cdev,struct globalmem_dev,cdev);
	filp->private_data = dev;
	return 0;
}

static int globalmem_release(struct inode *inode, struct file *filp)
{
	return 0;
}


//设备驱动到系统调用的映射,由ＶＦＳ完成
static const struct file_operations globalmem_fops = {

	.owner = THIS_MODULE,
	.llseek = globalmem_llseek,
	.read = globalmem_read,
	.write = globalmem_write,
	.unlocked_ioctl = globalmem_ioctl,
	.open = globalmem_open,
	.release = globalmem_release,


};


static void globalmem_setup_cdev(struct globalmem_dev*dev,int index)
{
	int err,devno = MKDEV(globalmem_major,index);

	cdev_init(&dev->cdev,&globalmem_fops);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&dev->cdev,devno,1);
	if(err)
		printk(KERN_NOTICE"add globalmem err\n");

}


//驱动模块加载函数
static int __init globalmem_init(void)
{

	int ret,i;
	dev_t devno = MKDEV(globalmem_major,0); //合成设备号
	if(globalmem_major)
		ret = register_chrdev_region(devno,DEVICE_NUM,"globalmem");
	else{
		ret = alloc_chrdev_region(&devno,0,DEVICE_NUM,"globalmem");
		globalmem_major = MAJOR(devno);
	}

	if(ret < 0){
		return ret;
	}

	globalmem_devp = kmalloc(sizeof(struct globalmem_dev)*DEVICE_NUM,GFP_KERNEL);
	if(!globalmem_devp){

		ret = -ENOMEM;
		goto fail_malloc;

	}
	for(i = 0; i<DEVICE_NUM; i++)
		globalmem_setup_cdev(globalmem_devp + i,i);
	return  0;

fail_malloc:
	unregister_chrdev_region(devno,1);
	return ret;
}

module_init(globalmem_init);

static void __exit globalmem_exit(void)
{
	int i = 0;
	for(i=0; i<DEVICE_NUM; i++)
		cdev_del(&(globalmem_devp + i)->cdev);
	kfree(globalmem_devp);
	unregister_chrdev_region(MKDEV(globalmem_major,0),DEVICE_NUM);
}
module_exit(globalmem_exit);

MODULE_AUTHOR("liushuang <269620154@qq.com>");
MODULE_LICENSE("GPL v2");
