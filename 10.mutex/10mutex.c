/**
 * @file 10mutex.c
 * @author wls (ufo281@outlook.com) 
 * @brief 
 *      mutex互斥锁:
 *      
 *      在使用 mutex 之前要先定义一个 mutex 变量。在使用 mutex 的时候要注意如下几点：
    1. mutex 可以导致休眠，因此不能在中断中使用 mutex，中断中只能使用自旋锁。
    2. 和信号量一样，mutex 保护的临界区可以调用引起阻塞的 API 函数。
    3. 因为一次只有一个线程可以持有 mutex，因此，必须由 mutex 的持有者释放 mutex。并
    且 mutex 不能递归上锁和解锁。
 *      
    DEFINE_MUTEX(name) 定义并初始化一个 mutex 变量。
    void mutex_init(mutex *lock) 初始化 mutex。
    void mutex_lock(struct mutex *lock) 获取 mutex，也就是给 mutex 上锁。如果获
    取不到就进休眠。
    void mutex_unlock(struct mutex *lock) 释放 mutex，也就给 mutex 解锁。
    int mutex_trylock(struct mutex *lock) 尝试获取 mutex，如果成功就返回 1，如果失
    败就返回 0。
    int mutex_is_locked(struct mutex *lock) 判断 mutex 是否被获取，如果是的话就返回
    1，否则返回 0。

    使用此函数获取信号量失败进入休眠以后可以被信号打断。
    int mutex_lock_interruptible(struct mutex *lock) 

    Example:
    互斥体的使用如下所示：
        示例代码 47.5.2.1 互斥体使用示例
        struct mutex lock; // 定义一个互斥体 
        mutex_init(&lock); // 初始化互斥体 

        mutex_lock(&lock); // 上锁 
        // 临界区
        mutex_unlock(&lock); // 解锁 


 * 
 * @version 1.0
 * @date 2024-05-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>


/*--------------------------------------------------------------------*/
#define NEW_CHR_CNT     1   /*设备个数*/
#define NEW_CHR_Name    "mutex-led0" /* 设备名*/

#define BEPP_ON      1   
#define BEPP_OFF     0   




/**
 * @brief linux设备结构体
 * 
 */
typedef struct linuxDEV
{
    dev_t devid;    /*设备号*/
    struct cdev char_dev;   /*字符类设备*/
    struct class *class;    /*类*/
    struct device *devices; /*设备*/
    int major;  /*主设备号*/
    int minor;  /*次设备号*/
    struct device_node *nd; /*设备节点*/
    int led_gpio;   /*led 所使用的GPIO编号*/
    atomic_t atlock; /*原子锁*/
    int dev_stats; /*设备状态，0，设备未使用：>0，设备已经被使用*/
    spinlock_t spinlock; /*自旋锁*/
    struct semaphore sema0; /*信号量*/
    struct mutex mutex_lock;  /*互斥锁*/

}gpio_dev;

gpio_dev led;




/**
 * @brief 打开led设备，main函数里调用open函数的时候就会调用这个函数
 * 
 * @param inode 传递驱动的inode
 * @param filp  设备文件，file 结构体有个叫做 private_data 的成员变量
                一般在 open 的时候将 private_data 指向设备结构体。
 * @return      0 成功;其他 失败
 */
static int devopen(struct inode *inode, struct file *filp)
{

    printk("Driver: devopen 2! \r\n");
    filp->private_data = &led; /*设置私有数据*/
    
    /*互斥锁 上锁,上锁失败可以被信号打断*/
    if (mutex_lock_interruptible(&led.mutex_lock))
    {
        return -ERESTARTSYS;
    }
    

#if 0

    /*互斥锁 上锁,上锁失败 不可以被信号打断*/
    mutex_lock(&led.mutex_lock);

#endif


    return 0;
	
}



/**
 * @brief 从设备读取数据 
 * 
 * @param filp 要打开的设备文件(文件描述符)
 * @param buf 返回给用户空间的数据缓冲区
 * @param cnt 要读取的数据长度
 * @param offt 相对于文件首地址的偏移
 * @return  读取的字节数，如果为负值，表示读取失败
 */
static ssize_t devread(    struct file *filp, 
                            char __user *buf, 
                            size_t cnt,
                            loff_t *offt
                        )
{   

	printk("Driver: devread 3!\r\n");


    return 0;

}




/**
 * @brief 向设备写数据
 * 
 * @param filp 设备文件，表示打开的文件描述符
 * @param buf 写入的数据
 * @param cnt 写入数据长度
 * @param offt 相对于文件的首地址偏移
 * @return 要写入的字节数，如果为负值，表示写入失败
 */
static ssize_t devwrite(   struct file *filp, 
                            const char __user *buf, 
                            size_t cnt, 
                            loff_t *offt
                        )
{

    int ret = 0;
    unsigned char databuf[1];
    unsigned char gpio_stat;
    gpio_dev *dev = filp->private_data;

	printk("Driver: devwrite 4! \r\n");


    /*向内核空间写数据 buf -> databuf */
    ret = copy_from_user(databuf,buf,cnt);
    if (ret<0 )
    {
        printk("Driver: kernel write data failed! \r\n");
        return  -EFAULT;
    }

    gpio_stat = databuf[0];   /*获取状态值*/

    if (gpio_stat == BEPP_ON)
    {
        // devswitch(BEPP_ON);
        gpio_set_value(dev->led_gpio,0);  /*打开LED灯*/
        printk("Driver: len on! \r\n");

    }
    else if (gpio_stat == BEPP_OFF)
    {
        gpio_set_value(dev->led_gpio,1); /*关闭LED灯*/
        // devswitch(BEPP_OFF);
        printk("Driver: len off! \r\n");

    }
    

    return 0;     

}




/**
 * @brief 关闭/释放设备，main函数的调用close函数时候就会调用这个函数
 * 
 * @param inode 
 * @param filp 文件描述符
 * @return int 0：OK, other：failed!
 */
static int devrelease(struct inode *inode, struct file *filp)
{
    gpio_dev *dev = filp->private_data;

    /*互斥锁 解锁*/
    mutex_unlock(&dev->mutex_lock);

	printk("Driver: devrelease! 7 \r\n");

    return 0;

}



/**
 * @brief 设备操作函数结构体
 * 
 */
static struct file_operations devfops = {
    
    .owner = THIS_MODULE,
    .open = devopen,
    .read = devread,
    .write = devwrite,
    .release = devrelease

};



/**
 * @brief 驱动入口函数
    执行modprobe 7.ko指令,加载驱动模块时候,就会调用这个函数
    
    Use example:
        /lib/modules/4.1.15 # modprobe 7.ko
        Driver: led OK!
        Driver: led-gpio num = 3!
        Driver: DEV_ID Register OK! led.major:244 minor:0 !
        Driver: wlsdevinit 1!
 * 
 * @return int 
 */
static int __init wlsdevinit(void)
{

    // unsigned int value = 0;
    int ret = 0;
    // unsigned int regdata[14];
    // const char *status_value;
    // struct property *proper;
    // u8 i = 0;

	printk("Driver: wlsdevinit 1! \r\n");

    /*初始化 互斥锁*/
    mutex_init(&led.mutex_lock);

    /*1. 获取led所使用的GPIO*/
    led.nd = of_find_node_by_path("/wgpioled");
    if (led.nd==NULL)
    {
        printk("Driver: led node Not found!\r\n");
        return -ENAVAIL;
    }
    else
    {
        printk("Driver: led OK!\r\n");
        
    }
    
    /*2. 获取设备树中的GPIO属性，得到LED所使用的LED编号*/
    led.led_gpio = of_get_named_gpio(led.nd,"led-gpio",0);
    if (led.led_gpio<0)
    {
        printk("Driver: can't get led-gpio!\n");
        return -ENAVAIL;
    }
    else
    {
        printk("Driver: led-gpio num = %d!\n",led.led_gpio);     
    }

    /*3.设置GPIO1_IO03为输出，并且输出高低电平，默认关闭LED灯*/        
    ret = gpio_direction_output(led.led_gpio,0);
    if (ret<0)
    {
        printk("Driver: can't set gpio!!\r\n");
    }


    /*注册字符设备驱动*/
    /*1. 创建设备号*/
    if (led.major) /*申请了设备号*/
    {
        /*将给定的主设备号和次设备号的值组合成 dev_t 类型的设备号*/
        led.devid = MKDEV(led.major,0);/* 次设备号0*/
        
        /**
         * @brief 注销字符设备之后要释放掉设备号,设备号释放函数
         * 
         * @param from 要释放的设备号
         * @param count 表示从 from 开始，要释放的设备号数量。
         */
        register_chrdev_region(led.devid,NEW_CHR_CNT,NEW_CHR_Name);/*注册设备号*/
    }
    else
    {

        /**
         * @brief 动态申请 linux 设备号
         * 
         * @param dev 保存申请到的设备号
         * @param baseminor 次设备号起始地址，alloc_chrdev_region 可以申请一段连续的多个设备号，这
                些设备号的主设备号一样，但是次设备号不同，次设备号以 baseminor 为起始地址地址开始递
                增。一般 baseminor 为 0，也就是说次设备号从 0 开始。
        * @param count 要申请的设备号数量。
        * @param name 设备名字。
        * @return int 
        */
        alloc_chrdev_region(&led.devid,0,NEW_CHR_CNT,NEW_CHR_Name);
        
        /*宏 MAJOR 用于从 dev_t 中获取主设备号，将 dev_t 右移 20 位即可*/
        led.major = MAJOR(led.devid);
        
        /*宏 MINOR 用于从 dev_t 中获取次设备号，取 dev_t 的低 20 位的值即可*/
        led.minor = MINOR(led.devid);

    }
    
    printk("Driver: DEV_ID Register OK! led.major:%d minor:%d !\r\n",
            led.major,led.minor);

    /*2. 初始化 char_dev*/
    led.char_dev.owner = THIS_MODULE;
    cdev_init(  (struct cdev *)&led.char_dev,
                (const struct file_operations *)&devfops
             );/*初始化char类型设备的结构体变量*/

    /*3. 添加一个 char_dev*/
    cdev_add(&led.char_dev,led.devid,NEW_CHR_CNT);

    /*4. 创建设备类*/
    led.class = class_create ((struct module *)THIS_MODULE, 
                                    (const char *)NEW_CHR_Name);

    if ( IS_ERR(led.class) )
    {
        return PTR_ERR(led.class);
    }


    /*5. 创建设备*/
    led.devices = device_create(led.class,NULL,led.devid,NULL,NEW_CHR_Name);
    if ( IS_ERR(led.devices) )
    {
        return PTR_ERR(led.devices);
    }


    return 0;
	
}



/**
 * @brief 模块驱动出口函数
 *        rmmod gpio_wls.ko，执行这个指令卸载驱动模块的时候调用这个函数     
 * 
 * @return int 
 */
static void __exit wlsdev_exit(void)
{

    printk("Driver: wlsdev_exit!\r\n");

    /*注销字符设备*/
    cdev_del(&led.char_dev);/*删除char dev设备*/

    unregister_chrdev_region(led.devid,NEW_CHR_CNT); /*注销设备号*/

    /*删除设备*/
    device_destroy(led.class,led.devid);
    
    /*删除类*/
    class_destroy(led.class);

}




/**
 * @brief 使用“modprobe -r”命令卸载驱动，比如要卸载 drv.ko，命令如下：
                    modprobe -r drv.ko
        会调用这个接口
 * 
 */
module_init(wlsdevinit);


/**
 * @brief 驱动模块的卸载使用命令“rmmod”即可，比如要卸载 drv.ko，使用如下命令即可：
                rmmod drv.ko
        会调用这个接口
 */
module_exit(wlsdev_exit);



/**
 * @brief LICENSE 和作者信息
 * 
 */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("wls");



