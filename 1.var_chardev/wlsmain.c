/**
 * @file wlsmain.c
 * @author wls (ufo281@outlook.com) 
 * @brief 这个是linux的驱动文件,makefile 只编译此文件
 * @version 1.0
 * @date 2024-05-10
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


#define Char_Dev_Base_Major     200             /* 主设备号*/
#define Char_Dev_Base_Name      "chardev1"      /* 设备名*/


static char read_buf[100];   /*读缓冲区*/
static char write_buf[100];   /*写缓冲区*/
static char kernel_buf[] = {"kernel data!"};




/**
 * @brief 打开设备
 * 
 * @param inode 传递驱动的inode
 * @param filp  设备文件，file 结构体有个叫做 private_data 的成员变量
                一般在 open 的时候将 private_data 指向设备结构体。
 * @return      0 成功;其他 失败
 */
static int chartest_open(struct inode *inode, struct file *filp)
{


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
static ssize_t chartest_read(  struct file *filp, 
                                char __user *buf, 
                                size_t cnt,
                                loff_t *offt
                            )
{   

    int retvalue = 0; 

    /*向空间发送数据*/
    memcpy(read_buf,kernel_buf,sizeof(kernel_buf));
    retvalue = copy_to_user(buf,read_buf,cnt);

    if (retvalue ==0 )
    {
        printk("kernel send data ok!\r\n");
    }
    else
    {
        printk("kernel send data failed!\r\n");

    }

    return 0;

}




/**
 * @brief 向设备写数据
 * 
 * @param filp 设备文件，表示打开的文件描述符
 * @param buf 设备写入数据
 * @param cnt 写入数据长度
 * @param offt 相对于文件的首地址偏移
 * @return 要写入的字节数，如果为负值，表示写入失败
 */
static ssize_t chartest_write(  struct file *filp, 
                                const char __user *buf, 
                                size_t cnt, 
                                loff_t *offt
                            )
{
    int ret = 0;

    /*向内核空间写数据*/
    ret = copy_from_user(write_buf,buf,cnt);
    if (ret ==0 )
    {
        printk("kernel RX data: %s\r\n",write_buf);
    }
    else
    {
        printk("kernel RX data failed!\r\n");

    }

    return 0;     
}




/**
 * @brief 关闭/释放设备
 * 
 * @param inode 
 * @param filp 文件描述符
 * @return int 0：OK, other：failed!
 */
static int chardev_release(struct inode *inode, struct file *filp)
{

	printk("chrdevbase release! \r\n");

    return 0;

}


/**
 * @brief 设备操作函数结构体
 * 
 */
static struct file_operations chrdev_base_fops = {
    
    .owner = THIS_MODULE,
    .open = chartest_open,
    .read = chartest_read,
    .write = chartest_write,
    .release = chardev_release

};


/**
 * @brief 驱动入口函数
 * 
 * @return int 
 */
static int __init wlsdev_init(void)
{

    int ret = 0;

    /**
     * @brief 函数用于注册字符设备
     * 
     * @param major 主设备号，Linux 下每个设备都有一个设备号，设备号分为主设备号和次设备号两
                    部分，关于设备号后面会详细讲解。
    * @param name 设备名字，指向一串字符串。
    * @param fops 结构体 file_operations 类型指针，指向设备的操作函数集合变量。
    * @return int 
    */
    ret = register_chrdev(Char_Dev_Base_Major,"chartest",&chrdev_base_fops);
    if ( ret<0 )
    {
        /*char device register failed!*/
        printk("chardev register failed!\r\n");
    }
    else
    {
        printk("wlsdev_init() register succesfull!\r\n");
    }
       

    return 0;
}



/**
 * @brief 模块驱动出口函数
 * 
 * @return int 
 */
static void __exit wlsdev_exit(void)
{

    /**
     * @brief 函数用户注销字符设备
     * 
     * @param major 要注销的设备对应的主设备号
     * @param name 要注销的设备对应的设备名
     */
    unregister_chrdev(Char_Dev_Base_Major,Char_Dev_Base_Name);
    printk("wlsdev_exit() unregister succesfull!\r\n");

}




/**
 * @brief 使用“modprobe -r”命令卸载驱动，比如要卸载 drv.ko，命令如下：
                    modprobe -r drv.ko
        会调用这个接口
 * 
 */
module_init(wlsdev_init);


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


