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
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>


/*--------------------------------------------------------------------*/
#define LED_MAJOR     200             /* 主设备号*/
#define Char_Dev_Base_Name      "linux_led"           /* 设备名*/

#define LED_ON      1   
#define LED_OFF     0   


/**
 * @brief 寄存器物理地址
 * 
 */
#define CMM_CCGR1_BASE                  (0X020C406C)
#define SW_MUX_GPIO1_IO03_BASE          (0X020E0068)
#define SW_PAD_GPIO1_IO03_BASE          (0X020E02F4)
#define GPIO01_DR_BASE                  (0X0209C000)
#define GPIO01_GDIR_BASE                (0X0209C004)


/**
 * @brief linux OS 映射后的寄存器虚拟地址
 * 
 */
static void __iomem *IMX6U_CCM_CCGR1 = NULL;
static void __iomem *SW_MUX_GPIO1_IO03 = NULL;
static void __iomem *SW_PAD_GPIO1_IO03 = NULL;
static void __iomem *GPIO01_DR = NULL;
static void __iomem *GPIO01_GDIR = NULL;


/**
 * @brief led open/off
 * 
 * @param sta   LED_ON:  OPEN LED
 *              LED_OFF: OFF LED
 */
void led_switch(u8 sta)
{


    u32 value =0;
    if (sta == LED_ON)
    {
        value = readl(GPIO01_DR);
        value &= ~(1<<3);
        writel(value,GPIO01_DR);
    }
    else if ( sta == LED_OFF )
    {
        value = readl(GPIO01_DR);
        value |= (1<<3);
        writel(value,GPIO01_DR); 
    }
    
	printk("led_switch  5! \r\n");
}




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

    // int retvalue = 0; 

    // /*向空间发送数据*/
    // memcpy(read_buf,kernel_buf,sizeof(kernel_buf));
    // retvalue = copy_to_user(buf,read_buf,cnt);

    // if (retvalue ==0 )
    // {    
    //     printk("kernel send data ok!\r\n");
    // }
    // else
    // {
    //     printk("kernel send data failed!\r\n");

    // }
 

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
        led_switch(LED_ON);
        printk("len on! \r\n");

    }
    else if (ledstat == LED_OFF)
    {
        led_switch(LED_OFF);
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

    int ret = 0;
    unsigned int value = 0;

	printk("wlsled_init start 1! \r\n");

    /*初始化LED*/
    /*1.虚拟地址映射，输入物理地址获得虚拟地址*/
    IMX6U_CCM_CCGR1 = ioremap(CMM_CCGR1_BASE,4);
    SW_MUX_GPIO1_IO03 = ioremap(SW_MUX_GPIO1_IO03_BASE,4);
    SW_PAD_GPIO1_IO03 = ioremap(SW_PAD_GPIO1_IO03_BASE,4);
    GPIO01_DR = ioremap(GPIO01_DR_BASE,4);
    GPIO01_GDIR = ioremap(GPIO01_GDIR_BASE,4);

    /*2. 使能GPIO1时钟*/
    value = readl(IMX6U_CCM_CCGR1);
    value &= ~(3<<26); /* clear before setting*/
	value |= (3<<26); /*set new value */
	writel(value,IMX6U_CCM_CCGR1);


	/*3. 设置GPIO01_IO03的复用功能
		复用为GPIO1_IO03，最后设置IO属性
	*/	
	writel(5,SW_MUX_GPIO1_IO03);
	

	/*寄存器SW_PAD_GPIO1_IO03 设置IO属性
    上下拉，输出速度等等*/
	writel(0x10B0,SW_PAD_GPIO1_IO03);	


	/*4. 设置GPIO1_IO03为输出功能*/
	value = readl(GPIO01_GDIR);
	value &= ~(1<<3);/*清楚以前的设置*/
	value |= (1<<3); /*设置为输出*/
	writel(value,GPIO01_GDIR);
    

	/*5. 默认关闭LED 输出高电平*/
	value = readl(GPIO01_DR);
	value |= (1<<3);
	writel(value,GPIO01_DR);

	
    /**
     * @brief 函数用于注册字符设备
     * 
     * @param major 主设备号，Linux 下每个设备都有一个设备号，设备号分为主设备号和次设备号两
            部分，关于设备号后面会详细讲解。
    * @param name 设备名字，指向一串字符串。
    * @param fops 结构体 file_operations 类型指针，指向设备的操作函数集合变量。
    * @return int 
    */
    ret = register_chrdev(LED_MAJOR,Char_Dev_Base_Name,&led_fops);
    if ( ret<0 )
    {
        /*char device register failed!*/
        printk("led register failed!\r\n");
		return -EIO;
    }
    else
    {
        printk("wlsled_init() register succesfull!\r\n");
    }
       
	printk("wlsled_init endoff 1! \r\n");
    

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


	/*取消虚拟映射*/
	iounmap(IMX6U_CCM_CCGR1);
	iounmap(SW_MUX_GPIO1_IO03);
	iounmap(SW_PAD_GPIO1_IO03);
	iounmap(GPIO01_DR);
	iounmap(GPIO01_GDIR);

    /**
     * @brief 函数用户注销字符设备
     * 
     * @param major 要注销的设备对应的主设备号
     * @param name 要注销的设备对应的设备名
     */
    unregister_chrdev(LED_MAJOR,Char_Dev_Base_Name);
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
// int alloc_chrdev_region(dev_t *dev, unsigned baseminor, unsigned count, const char *name);



/**
 * @brief 注销字符设备之后要释放掉设备号,设备号释放函数
 * 
 * @param from 要释放的设备号
 * @param count 表示从 from 开始，要释放的设备号数量。
 */
// void unregister_chrdev_region(dev_t from, unsigned count);


