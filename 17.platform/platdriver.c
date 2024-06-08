/**
 * @file platdriver.c
 * @author wls (ufo281@outlook.com) 
 * @brief platform 设备驱动实验 不支持设备树版本的
 * 
    我们在前面几章编写的设备驱动都非常的简单，都是对 IO 进行最简单的读写操作。像 I2C、
    SPI、LCD 等这些复杂外设的驱动就不能这么去写了，Linux 系统要考虑到驱动的可重用性，因
    此提出了驱动的分离与分层这样的软件思路，在这个思路下诞生了我们将来最常打交道的
    platform 设备驱动，也叫做平台设备驱动。本章我们就来学习一下 Linux 下的驱动分离与分层，
    以及 platform 框架下的设备驱动该如何编写。


    本章实验我们需要编写一个驱动模块和一个设备模块，其中驱动模块是 platform 驱动程序，
    设备模块是 platform 的设备信息。当这两个模块都加载成功以后就会匹配成功，然后 platform
    驱动模块中的 probe 函数就会执行，probe 函数中就是传统的字符设备驱动那一套。

 * 
 * @version 1.0
 * @date 2024-06-06
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
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>



/*--------------------------------------------------------------------*/

#define LinCdev_CNT     1           /*设备个数*/
#define LinCdev_Name    "platled" /* 设备名*/

#define LEDOFF        0
#define LEDON         1



/* 寄存器名 */
static void __iomem *IMX6U_CCM_CCGR1;
static void __iomem *SW_MUX_GPIO1_IO03;
static void __iomem *SW_PAD_GPIO1_IO03;
static void __iomem *GPIO1_DR;
static void __iomem *GPIO1_GDIR;


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

gpio_dev led;
// gpio_dev key;



/**
 * @brief LED 打开/关闭
 * 
 * LEDON(0) 打开 LED，LEDOFF(1) 关闭 LED
 * @param sta 
 */
void led0_switch(u8 sta)
{
    u32 val = 0;
    if(sta == LEDON){

        val = readl(GPIO1_DR);
        val &= ~(1 << 3);

        writel(val, GPIO1_DR);
    }else if(sta == LEDOFF){
        val = readl(GPIO1_DR);
        val|= (1 << 3);
        writel(val, GPIO1_DR);
    }

}





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
    // gpio_dev *dev = filp->private_data;

    printk("Driver: devwrite 4! \r\n");


    /*向内核空间写数据 buf -> databuf */
    ret = copy_from_user(databuf,buf,cnt);
    if (ret<0 )
    {
        printk("Driver: kernel write data failed! \r\n");
        return  -EFAULT;
    }

    gpio_stat = databuf[0];   /*获取状态值*/

    if (gpio_stat == LEDON)
    {
        led0_switch(LEDON);
        // gpio_set_value(dev->gpio_number,0);  /*打开LED灯*/
        printk("Driver: led on! \r\n");

    }
    else if (gpio_stat == LEDOFF)
    {
        // gpio_set_value(dev->gpio_number,1); /*关闭LED灯*/
        led0_switch(LEDOFF);
        printk("Driver: led off! \r\n");

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


/**
 * @brief flatform 驱动的 probe 函数，当驱动与设备,匹配以后此函数就会执行
 * 
 * @param dev platform 设备
 * @return int 0，成功;其他负值,失败
 */
static int led_probe(struct platform_device  *dev)
{
    int i = 0;
    int ressize[5];
    u32 val = 0;
    struct resource *led_source[5];
    printk("Driver: led_probe 1! \r\n");

    printk("led driver and device has matched!\r\n"); 

    /*获取资源*/
    for ( i = 0; i < 5; i++)
    {
        led_source[i] = platform_get_resource(dev,IORESOURCE_MEM,i);
        if (!led_source[i])
        {
            dev_err(&dev->dev,"No MEM resource for always on \r\n");
            return -ENXIO;
        }
        ressize[i] = resource_size(led_source[i]);
        
    }
    
    /* 2、初始化 LED */
    /* 寄存器地址映射 */
    SW_MUX_GPIO1_IO03 = ioremap(led_source[1]->start, ressize[1]);
    SW_PAD_GPIO1_IO03 = ioremap(led_source[2]->start, ressize[2]);
    IMX6U_CCM_CCGR1 = ioremap(led_source[0]->start, ressize[0]);
    GPIO1_DR = ioremap(led_source[3]->start, ressize[3]);
    GPIO1_GDIR = ioremap(led_source[4]->start, ressize[4]);

    val = readl(IMX6U_CCM_CCGR1);
    val &= ~(3 << 26); /* 清除以前的设置 */
    val |= (3 << 26); /* 设置新值 */
    writel(val, IMX6U_CCM_CCGR1);

    /* 设置 GPIO1_IO03 复用功能，将其复用为 GPIO1_IO03 */
    writel(5, SW_MUX_GPIO1_IO03);
    writel(0x10B0, SW_PAD_GPIO1_IO03);

    /* 设置 GPIO1_IO03 为输出功能 */
    val = readl(GPIO1_GDIR);
    val &= ~(1 << 3); /* 清除以前的设置 */
    val |= (1 << 3); /* 设置为输出 */
    writel(val, GPIO1_GDIR);

    /* 默认关闭 LED1 */
    val = readl(GPIO1_DR);
    val |= (1 << 3) ;
    writel(val, GPIO1_DR);


    /*注册字符设备驱动*/

    /*1. 申请创建设备号*/
    if (led.major) 
    {
        /*将给定的主设备号和次设备号的值组合成 dev_t 类型的设备号*/
        led.devid = MKDEV(led.major,0);/* 次设备号0*/
        
        /**
         * @brief 注销字符设备之后要释放掉设备号,设备号释放函数
         * 
         * @param from 要释放的设备号
         * @param count 表示从 from 开始，要释放的设备号数量。
         */
        register_chrdev_region(led.devid,LinCdev_CNT,LinCdev_Name);/*注册设备号*/
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
        alloc_chrdev_region(&led.devid,0,LinCdev_CNT,LinCdev_Name);
        
        /*宏 MAJOR 用于从 dev_t 中获取主设备号，将 dev_t 右移 20 位即可*/
        led.major = MAJOR(led.devid);
        
        /*宏 MINOR 用于从 dev_t 中获取次设备号，取 dev_t 的低 20 位的值即可*/
        led.minor = MINOR(led.devid);

    }
    
    printk("Driver: DEV_ID Register OK! led.major:%d minor:%d !\r\n",
            led.major,led.minor);

    /*2. 注册 字符设备*/
    led.char_dev.owner = THIS_MODULE;
    cdev_init(  (struct cdev *)&led.char_dev,
                (const struct file_operations *)&devfops
             );/*初始化char类型设备的结构体变量*/

    /*3. 添加一个 char_dev*/
    cdev_add(&led.char_dev,led.devid,LinCdev_CNT);

    /*4. 创建设备类*/
    led.class = class_create ((struct module *)THIS_MODULE, 
                                    (const char *)LinCdev_Name);

    if ( IS_ERR(led.class) )
    {
        return PTR_ERR(led.class);
    }


    /*5. 创建设备*/
    led.devices = device_create(led.class,NULL,led.devid,NULL,LinCdev_Name);
    if ( IS_ERR(led.devices) )
    {   
        return PTR_ERR(led.devices);
    }

    return 0;
}


/**
 * @brief 移除 platform 驱动的时候此函数会执行
 * 
 * @param dev platform 设备
 * @return int 0，成功;其他负值,失败
 */
static int led_remove(struct platform_device *dev)
{

    iounmap(IMX6U_CCM_CCGR1);
    iounmap(SW_MUX_GPIO1_IO03);
    iounmap(SW_PAD_GPIO1_IO03);
    iounmap(GPIO1_DR);
    iounmap(GPIO1_GDIR);

    cdev_del(&led.char_dev); /* 删除 cdev */
    unregister_chrdev_region(led.devid, LinCdev_CNT);
    device_destroy(led.class, led.devid);
    class_destroy(led.class);

    return 0;

}




static struct platform_driver led_driver = {

    .driver = {
        .name = "imx6ull_led",   

    },
    .probe = led_probe,
    .remove = led_remove

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
static int __init wlsdev_driverinit(void)
{
 
    return platform_driver_register(&led_driver);

}



/**
 * @brief 模块驱动出口函数
 *        rmmod gpio_wls.ko，执行这个指令卸载驱动模块的时候调用这个函数     
 * 
 * @return int 
 */
static void __exit wlsdev_driverexit(void)
{

    platform_driver_unregister(&led_driver);

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
