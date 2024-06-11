/**
 * @file platdriver.c
 * @author wls (ufo281@outlook.com) 
 * @brief 
 * @version 1.0
 * @date 2024-06-10
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
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/fcntl.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>



/*--------------------------------------------------------------------*/

#define MISCBEEP_Name       "miscbeep" /* 设备名*/
#define MISCBEEP_MINOR      144      /*子设备号*/

#define BEEPOFF        0
#define BEPPON         1



/**
 * @brief 中断IO描述结构体
 * 
 */
typedef struct IRQ_L
{
    int gpio;   /*gpio*/
    int irq_label;  /*中断号*/
    unsigned char value; /*按键对应的值*/
    char name[10];
    irqreturn_t (*handler_f)(int,void *);/*ISR*/     
    
}IRQ_IO;




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
    int gpio_number;   /*所使用的GPIO编号 eg:GPIO1_IO05*/
    int dev_stats; /*设备状态，0，设备未使用：>0，设备已经被使用*/
    spinlock_t spinlock; /*自旋锁*/
    struct semaphore sema0; /*信号量*/
    struct mutex mutex_lock;  /*互斥锁*/
    atomic_t keyvalue;  /*有效的按键值*/
    atomic_t releasekey;    /*标记是否一次完成的按键*/
    struct timer_list timer;
    IRQ_IO  irqkeydesc[1];/*按键描述数组*/
    unsigned char curkeynum;    /*当前的按键号*/
    wait_queue_head_t   r_wait; /*读等待队列头*/
    struct fasync_struct  *async_queue; /*异步相关结构体*/ 


}gpio_dev;

// gpio_dev beep;
gpio_dev miscbeep;
// gpio_dev key;





/**
 * @brief 打开beep设备，main函数里调用open函数的时候就会调用这个函数
 * 
 * @param inode 传递驱动的inode
 * @param filp  设备文件，file 结构体有个叫做 private_data 的成员变量
                一般在 open 的时候将 private_data 指向设备结构体。
 * @return      0 成功;其他 失败
 */
static int devopen(struct inode *inode, struct file *filp)
{

    printk("Driver: devopen 2! \r\n");
    filp->private_data = &miscbeep; /*设置私有数据*/

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

    int ret = 6;
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

    if (gpio_stat == BEPPON)
    {
        gpio_set_value(dev->gpio_number,0);
        printk("Driver: beep on! \r\n");

    }
    else if (gpio_stat == BEEPOFF)
    {
        gpio_set_value(dev->gpio_number,1);
        printk("Driver: beep off! \r\n");

    }
    
    return 0;     

}




/**
 * @brief 关闭/释放设备，main函数的调用close函数时候就会调用这个函数
 *          释放 flatform 设备模块的时候此函数会执行
 * 
 * @param inode 
 * @param filp 文件描述符
 * @return int 0：OK, other：failed!
 */
static int devrelease(struct inode *inode, struct file *filp)
{

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
    // .read = devread,
    .write = devwrite,
    // .release = devrelease,
    // .poll = wimx6ull_poll,
    // .fasync = imx6ull_fasync,

};


static struct miscdevice beep_miscdev = {

    .minor = MISCBEEP_MINOR,
    .name = MISCBEEP_Name,
    .fops = &devfops,
};




/**
 * @brief flatform 驱动的 probe 函数，当驱动与设备,匹配以后此函数就会执行
 * 
 * @param dev platform 设备
 * @return int 0，成功;其他负值,失败
 */
static int beep_probe(struct platform_device  *dev)
{
    int ret;
    printk("Driver: beep_probe 1! \r\n");

    printk("beep driver and device has matched!\r\n"); 

    /*config gpio*/

    /*1. 初始化GPIO*/
    miscbeep.nd = of_find_node_by_path("/beep"); /*根据设备树的路径 获取设备节点*/
    if (miscbeep.nd == NULL)
    {
        printk("Driver: wgpioled node not found\r\n");
        return -EINVAL;
    }
    
    miscbeep.gpio_number = of_get_named_gpio(miscbeep.nd, "beep-gpio", 0); /* 根据设备树节点和属性 获取GPIO号*/
    if (miscbeep.gpio_number <0 )
    {
        printk("Driver: can't get gpio number\r\n");
        return -EINVAL;
    }

    /**
     * @brief gpio_request 函数用于申请一个 GPIO 管脚，
     * 在使用一个 GPIO 之前一定要使用 gpio_request进行申请
     * 
     * gpio：要申请的 gpio 标号，使用 of_get_named_gpio 函数
     * 从设备树获取指定 GPIO 属性信息，此函数会返回这个 GPIO 的标号
     * 
     * label：给 gpio 设置个名字
     * 
     */
    gpio_request(miscbeep.gpio_number,"beep"); 
    gpio_direction_output(miscbeep.gpio_number,1);/*设置为输出，默认高电平*/


    /**
     * @brief 一般情况下会注册对应的字符设备，但是这里我们使用MISC设备
     * 所以我们不需要自己注册字符驱动，只需要注册MISC设备驱动即可  
     * 
     */
    ret = misc_register(&beep_miscdev);
    if (ret<0)
    {
        printk("Driver: misc device register failed! \r\n");
        return -EINVAL;

    }
    

    return 0;

}


/**
 * @brief 移除 platform 驱动的时候此函数会执行
 * 
 * @param dev platform 设备
 * @return int 0，成功;其他负值,失败
 */
static int beep_remove(struct platform_device *dev)
{

    /*注销设备的时候关闭蜂鸣器*/
    gpio_set_value(miscbeep.gpio_number,1);
    gpio_free(miscbeep.gpio_number);

    misc_deregister(&beep_miscdev);

    return 0;

}

/*匹配表*/
static const struct of_device_id beep_of_match[]={

    {.compatible = "wls-beep"},
    {}

};



static struct platform_driver beep_driver = {

    .driver = {
        .name = "wls-beep", /*驱动名字，用于和设备匹配*/   
        .of_match_table = beep_of_match, /*设备树匹配表*/
    },
    .probe = beep_probe,
    .remove = beep_remove

}; 


/**
 * @brief 驱动入口函数
    执行modprobe 7.ko指令,加载驱动模块时候,就会调用这个函数
    
    Use example:
        /lib/modules/4.1.15 # modprobe 7.ko
        Driver: beep OK!
        Driver: beep-gpio num = 3!
        Driver: DEV_ID Register OK! beep.major:244 minor:0 !
        Driver: wlsdevinit 1!
 * 
 * @return int 
 */
static int __init wlsdev_driverinit(void)
{
 
    return platform_driver_register(&beep_driver);

}



/**
 * @brief 模块驱动出口函数
 *        rmmod gpio_wls.ko，执行这个指令卸载驱动模块的时候调用这个函数     
 * 
 * @return int 
 */
static void __exit wlsdev_driverexit(void)
{

    platform_driver_unregister(&beep_driver);

}







/**
 * @brief 使用“modprobe -r”命令卸载驱动，比如要卸载 drv.ko，命令如下：
                    modprobe -r drv.ko
        会调用这个接口
 * 
 */
module_init(wlsdev_driverinit);


/**
 * @brief 驱动模块的卸载使用命令“rmmod”即可，比如要卸载 drv.ko，使用如下命令即可：
                rmmod drv.ko
        会调用这个接口
 */
module_exit(wlsdev_driverexit);



/**
 * @brief LICENSE 和作者信息
 * 
 */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("wls");
