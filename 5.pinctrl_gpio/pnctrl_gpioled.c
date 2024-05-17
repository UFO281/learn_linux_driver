/**
 * @file newgpio_wls.c
 * @author wls (ufo281@outlook.com) 
 * @brief 使用新的linux驱动API，并且具有自动创建驱动设备节点文件的功能
 * @version 1.0
 * @date 2024-05-12
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
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>


/*--------------------------------------------------------------------*/
#define NEW_CHR_CNT     1   /*设备个数*/
#define NEW_CHR_Name    "pinctrl_gpio_led" /* 设备名*/

#define LED_ON      1   
#define LED_OFF     0   


/**
 * @brief 寄存器物理地址
 * 
 */
// #define CMM_CCGR1_BASE                  (0X020C406C)
// #define SW_MUX_GPIO1_IO03_BASE          (0X020E0068)
// #define SW_PAD_GPIO1_IO03_BASE          (0X020E02F4)
// #define GPIO01_DR_BASE                  (0X0209C000)
// #define GPIO01_GDIR_BASE                (0X0209C004)


/**
 * @brief linux OS 映射后的寄存器虚拟地址
 * 
 */
// static void __iomem *IMX6U_CCM_CCGR1 = NULL;
// static void __iomem *SW_MUX_GPIO1_IO03 = NULL;
// static void __iomem *SW_PAD_GPIO1_IO03 = NULL;
// static void __iomem *GPIO01_DR = NULL;
// static void __iomem *GPIO01_GDIR = NULL;



/**
 * @brief led设备结构体
 * 
 */
typedef struct LEDDEV
{
    dev_t devid;    /*设备号*/
    struct cdev char_dev;   /*字符类设备*/
    struct class *class;    /*类*/
    struct device *devices; /*设备*/
    int major;  /*主设备号*/
    int minor;  /*次设备号*/
    struct device_node *nd; /*设备节点*/
    int led_gpio;   /*led 所使用的GPIO编号*/

}led_dev;

led_dev led0;




/**
 * @brief 打开led设备，main函数里调用open函数的时候就会调用这个函数
 * 
 * @param inode 传递驱动的inode
 * @param filp  设备文件，file 结构体有个叫做 private_data 的成员变量
                一般在 open 的时候将 private_data 指向设备结构体。
 * @return      0 成功;其他 失败
 */
static int led_open(struct inode *inode, struct file *filp)
{

	printk("led_open  2! \r\n");

    filp->private_data = &led0; /*设置私有数据*/
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
static ssize_t led_read(    struct file *filp, 
                            char __user *buf, 
                            size_t cnt,
                            loff_t *offt
                        )
{   

	printk("led_read  3! \r\n");


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
static ssize_t led_write(   struct file *filp, 
                            const char __user *buf, 
                            size_t cnt, 
                            loff_t *offt
                        )
{

    int ret = 0;
    unsigned char databuf[1];
    unsigned char ledstat;
    led_dev *dev = filp->private_data;

	printk("led_write start 4! \r\n");


    /*向内核空间写数据 buf -> databuf */
    ret = copy_from_user(databuf,buf,cnt);
    if (ret<0 )
    {
        printk("kernel write data failed! \r\n");
        return  -EFAULT;
    }

    ledstat = databuf[0];   /*获取状态值*/

    if (ledstat == LED_ON)
    {
        // led_switch(LED_ON);
        gpio_set_value(dev->led_gpio,0);  /*打开LED灯*/
        printk("len on! \r\n");

    }
    else if (ledstat == LED_OFF)
    {
        gpio_set_value(dev->led_gpio,1); /*关闭LED灯*/
        // led_switch(LED_OFF);
        printk("len off! \r\n");

    }
    

	printk("led_write endoff 4! \r\n");

    return 0;     

}




/**
 * @brief 关闭/释放设备，main函数的调用close函数时候就会调用这个函数
 * 
 * @param inode 
 * @param filp 文件描述符
 * @return int 0：OK, other：failed!
 */
static int led_release(struct inode *inode, struct file *filp)
{

	printk("led release! 7 \r\n");

    return 0;

}



/**
 * @brief 设备操作函数结构体
 * 
 */
static struct file_operations led_fops = {
    
    .owner = THIS_MODULE,
    .open = led_open,
    .read = led_read,
    .write = led_write,
    .release = led_release

};




/**
 * @brief 驱动入口函数  
 *      执行这个指令 加载驱动模块时候 modprobe gpio_wls.ko，调用这个函数
 *      
 *      配置GPIO属性；eg: 开启GPIO外设时钟，输入/输出模式，上下拉，速度等等
 * 
 * @return int 
 */
static int __init wlsled_init(void)
{

    // unsigned int value = 0;
    int ret = 0;
    // unsigned int regdata[14];
    // const char *status_value;
    // struct property *proper;
    // u8 i = 0;

	printk("wlsled_init start 1! \r\n");

    /*1. 获取led所使用的GPIO*/
    led0.nd = of_find_node_by_path("/wgpioled");
    if (led0.nd==NULL)
    {
        printk("wls_led node Not found!\r\n");
        return -ENAVAIL;
    }
    else
    {
        printk("wls_led node found!\r\n");
        
    }
    
    /*2. 获取设备树中的GPIO属性，得到LED所使用的LED编号*/
    led0.led_gpio = of_get_named_gpio(led0.nd,"led-gpio",0);
    if (led0.led_gpio<0)
    {
        printk("can't get led-gpio!\n");
        return -ENAVAIL;
    }
    else
    {
        printk("led-gpio num = %d!\n",led0.led_gpio);     
    }

    /*3.设置GPIO1_IO03为输出，并且输出高低电平，默认关闭LED灯*/        
    ret = gpio_direction_output(led0.led_gpio,1);
    if (ret<0)
    {
        printk("can't set gpio!!\r\n");
    }


    /*注册字符设备驱动*/
    /*1. 创建设备号*/
    if (led0.major) /*申请了设备号*/
    {
        /*将给定的主设备号和次设备号的值组合成 dev_t 类型的设备号*/
        led0.devid = MKDEV(led0.major,0);/* 次设备号0*/
        
        /**
         * @brief 注销字符设备之后要释放掉设备号,设备号释放函数
         * 
         * @param from 要释放的设备号
         * @param count 表示从 from 开始，要释放的设备号数量。
         */
        register_chrdev_region(led0.devid,NEW_CHR_CNT,NEW_CHR_Name);/*注册设备号*/
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
        alloc_chrdev_region(&led0.devid,0,NEW_CHR_CNT,NEW_CHR_Name);
        
        /*宏 MAJOR 用于从 dev_t 中获取主设备号，将 dev_t 右移 20 位即可*/
        led0.major = MAJOR(led0.devid);
        
        /*宏 MINOR 用于从 dev_t 中获取次设备号，取 dev_t 的低 20 位的值即可*/
        led0.minor = MINOR(led0.devid);

    }
    
    printk("\nDEV_ID Register OK! led0.major:%d minor:%d !\r\n",
            led0.major,led0.minor);

    /*2. 初始化 char_dev*/
    led0.char_dev.owner = THIS_MODULE;
    cdev_init(  (struct cdev *)&led0.char_dev,
                (const struct file_operations *)&led_fops
             );/*初始化char类型设备的结构体变量*/

    /*3. 添加一个 char_dev*/
    cdev_add(&led0.char_dev,led0.devid,NEW_CHR_CNT);

    /*4. 创建设备类*/
    led0.class = class_create ((struct module *)THIS_MODULE, 
                                    (const char *)NEW_CHR_Name);

    if ( IS_ERR(led0.class) )
    {
        return PTR_ERR(led0.class);
    }


    /*5. 创建设备*/
    led0.devices = device_create(led0.class,NULL,led0.devid,NULL,NEW_CHR_Name);
    if ( IS_ERR(led0.devices) )
    {
        return PTR_ERR(led0.devices);
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

    /*注销字符设备*/
    cdev_del(&led0.char_dev);/*删除char dev设备*/

    unregister_chrdev_region(led0.devid,NEW_CHR_CNT); /*注销设备号*/

    /*删除设备*/
    device_destroy(led0.class,led0.devid);
    
    /*删除类*/
    class_destroy(led0.class);

    printk("wlsdev_exit()6 unregister succesfull!\r\n");

}




/**
 * @brief 使用“modprobe -r”命令卸载驱动，比如要卸载 drv.ko，命令如下：
                    modprobe -r drv.ko
        会调用这个接口
 * 
 */
module_init(wlsled_init);


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




