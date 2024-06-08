/**
 * @file platdev.c
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

#define CCM_CCGR1_BASE              (0X020C406C) 
#define SW_MUX_GPIO1_IO03_BASE      (0X020E0068) 
#define SW_PAD_GPIO1_IO03_BASE      (0X020E02F4) 
#define GPIO1_DR_BASE               (0X0209C000) 
#define GPIO1_GDIR_BASE             (0X0209C004) 
#define REGISTER_LENGTH             (4) 



// #define LinCdev_CNT     1           /*设备个数*/
// #define LinCdev_Name    "noi.mx6ull-wls" /* 设备名*/

// #define KEY_VALUE     0X66  /*按键值*/  
// #define INvakey       0x55  /*无效的按键值*/   


/**
 * @brief 中断IO描述结构体
 * 
 */
// typedef struct IRQ_L
// {
//     int gpio;   /*gpio*/
//     int irq_label;  /*中断号*/
//     unsigned char value; /*按键对应的值*/
//     char name[10];
//     irqreturn_t (*handler_f)(int,void *);/*ISR*/     
    
// }IRQ_IO;






// /**
//  * @brief linux设备结构体
//  * 
//  */
// typedef struct linuxDEV
// {
//     dev_t devid;    /*设备号*/
//     struct cdev char_dev;   /*字符类设备*/
//     struct class *class;    /*类*/
//     struct device *devices; /*设备*/
//     int major;  /*主设备号*/
//     int minor;  /*次设备号*/
//     struct device_node *nd; /*设备节点*/
//     int gpio_number;   /*所使用的GPIO编号 eg:GPIO1_IO05*/
//     int dev_stats; /*设备状态，0，设备未使用：>0，设备已经被使用*/
//     spinlock_t spinlock; /*自旋锁*/
//     struct semaphore sema0; /*信号量*/
//     struct mutex mutex_lock;  /*互斥锁*/
//     atomic_t keyvalue;  /*有效的按键值*/
//     atomic_t releasekey;    /*标记是否一次完成的按键*/
//     struct timer_list timer;
//     IRQ_IO  irqkeydesc[1];/*按键描述数组*/
//     unsigned char curkeynum;    /*当前的按键号*/
//     wait_queue_head_t   r_wait; /*读等待队列头*/
//     struct fasync_struct  *async_queue; /*异步相关结构体*/ 


// }gpio_dev;

// // gpio_dev led;
// gpio_dev key;



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
    // gpio_dev *dev = filp->private_data;

    /*互斥锁 解锁*/
    // mutex_unlock(&dev->mutex_lock);

    printk("Driver: devrelease! 7 \r\n");

    // return imx6ull_fasync(-1,filp,0);/* 删除异步通知 */


}



static struct resource led_resources[] = {

    [0] ={
        .start = CCM_CCGR1_BASE,
        .end = (CCM_CCGR1_BASE +REGISTER_LENGTH - 1 ),
        .flags = IORESOURCE_MEM

    },
    [1] = {
        .start = SW_MUX_GPIO1_IO03_BASE,
        .end = (SW_MUX_GPIO1_IO03_BASE + REGISTER_LENGTH - 1),
        .flags = IORESOURCE_MEM
    },
    [2] = {
        .start = SW_PAD_GPIO1_IO03_BASE,
        .end = (SW_PAD_GPIO1_IO03_BASE + REGISTER_LENGTH - 1),
        .flags = IORESOURCE_MEM
    },
    [3] = {
        .start = GPIO1_DR_BASE,
        .end = (GPIO1_DR_BASE + REGISTER_LENGTH - 1),
        .flags = IORESOURCE_MEM
    },
    [4] = {
        .start = GPIO1_GDIR_BASE,
        .end = (GPIO1_GDIR_BASE + REGISTER_LENGTH - 1),
        .flags = IORESOURCE_MEM
    },

};



static struct platform_device leddevice = {

    .name = "imx6ull_led",
    .id = -1,
    .dev ={
        .release = &devrelease,
    },
    .num_resources = ARRAY_SIZE(led_resources),
    .resource = led_resources,

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
    printk("Driver: wlsdevinit!\r\n");
    
    return platform_device_register(&leddevice);

}



/**
 * @brief 模块驱动出口函数
 *        rmmod gpio_wls.ko，执行这个指令卸载驱动模块的时候调用这个函数     
 * 
 * @return int 
 */
static void __exit wlsdev_exit(void)
{

    // size_t i = 0;
    printk("Driver: wlsdev_exit!\r\n");
    platform_device_unregister(&leddevice);

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
