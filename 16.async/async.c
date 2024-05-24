/**
 * @file 10mutex.c
 * @author wls (ufo281@outlook.com) 
 * @brief 
 *      mutex互斥锁:+ linux timer + 异步通知
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
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/fcntl.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>



/*--------------------------------------------------------------------*/


#define LinCdev_CNT     1           /*设备个数*/
#define LinCdev_Name    "noi.mx6ull-wls" /* 设备名*/

#define KEY_VALUE     0X66  /*按键值*/  
#define INvakey       0x55  /*无效的按键值*/   


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

// gpio_dev led;
gpio_dev key;



/**
 * @brief IO ISR 开启定时器，延时10ms
 *          定时按键用于消抖
 * @param irq 中断号
 * @param dev_id 设备结构
 * @return irqreturn_t 中断执行结果
 */
static irqreturn_t key0_handler(int irq,void *dev_id)
{
 
    gpio_dev *dev = (gpio_dev *)dev_id;
    dev->curkeynum = 0;

    /*将设备的标识信息（dev_id）存储到定时器结构体中的数据字段 (data) 中，
    以便在定时器回调函数中能够访问到这个设备数据*/
    dev->timer.data = (volatile long)dev_id;


    /*使用 mod_timer函数启动定时器，定时器周期为 10ms*/
    mod_timer(&dev->timer,jiffies+msecs_to_jiffies(10));

    /*这行代码的作用是在中断处理程序执行完成后，
    向内核表明这个中断已经被成功处理。这有助于内核正确管理中断，
    并防止同一中断被重复处理*/
    return IRQ_RETVAL(IRQ_HANDLED);

}



/**
 * @brief 定时器服务函数，用于消抖，定时器到了之后
 *      再次读取按键值，如果按键还是处于按下状态就表示按键有效
 * 
 * @param arg 设备结构变量
 */
void timer_function(unsigned long arg)
{

    unsigned char value;
    unsigned char num;
    IRQ_IO *keydesc;
    gpio_dev *dev = (gpio_dev *)arg;

    num = dev->curkeynum;
    keydesc = &dev->irqkeydesc[num];

    value = gpio_get_value(keydesc->gpio);
    if (value == 0)
    {
        atomic_set(&dev->keyvalue,keydesc->value);
    }
    else
    {
        atomic_set(&dev->keyvalue,0x80|keydesc->value); /*按键松开*/
        atomic_set(&dev->releasekey,1);/*标记松开按键*/
    }

    /*按键成功按下,唤醒进程*/
    if (atomic_read(&dev->releasekey))
    {
        // wake_up_interruptible(&dev->r_wait);
        if (dev->async_queue)
        {
            /*当设备可以访问的时候，驱动程序需要向应用程序发出信号，
            相当于产生“中断”。kill_fasync函数负责发送指定的信号*/
            kill_fasync(&dev->async_queue,SIGIO,POLL_IN );
        }
        
    }    

}



/**
 * @brief 
 * 
 * @param fd 
 * @param filp 
 * @param on 
 * @return int 
 */
static int imx6ull_fasync(int fd, struct file *filp, int on)
{
    gpio_dev *dev = (gpio_dev*)filp->private_data;

    return fasync_helper(fd, filp, on, &dev->async_queue);

}


static int keyio_init(void)
{
    int ret;
    int i=0;

    /*1. 获取key在设备树中的节点*/
    key.nd = of_find_node_by_path("/key");
    if (key.nd==NULL)
    {
        printk("Driver: key node Not found!\r\n");
        return -ENAVAIL;
    }
    else
    {
        printk("Driver: key OK!\r\n");
        
    }

    /*2. 获取设备树中的GPIO属性，得到key所使用的key编号*/
    for ( i = 0; i < 1; i++)
    {
        key.irqkeydesc[i].gpio = of_get_named_gpio(key.nd,"key-gpio",i);
        if (key.irqkeydesc[i].gpio<0)
        {
            printk(" can't get key%d \r\n",i);
        }
        
    }

    /*3.设置key IO mode for int*/
    for ( i = 0; i < 1; i++)
    {
        memset(key.irqkeydesc[i].name ,0, sizeof(key.irqkeydesc[i].name) );
        sprintf(key.irqkeydesc[i].name,"key%d",i);
        gpio_request(key.irqkeydesc[i].gpio, key.irqkeydesc->name);   /*请求GPIO*/
        ret = gpio_direction_input(key.irqkeydesc[i].gpio);
        if (ret<0) 
        {
            printk("Driver: can't set gpio!!\r\n");
        }

        /*从设备树中获取按键 IO 对应的中断号*/
        key.irqkeydesc[i].irq_label = irq_of_parse_and_map(key.nd,i);
        // key.irqkeydesc[i].irq_label = gpio_to_irq(key.irqkeydesc[i].gpio);
        printk("key%d:gpio=%d,irq_label=%d \r\n",i,key.irqkeydesc[i].gpio,
                                                key.irqkeydesc[i].irq_label);

    }
            
    /*申请中断*/
    key.irqkeydesc[0].handler_f = key0_handler;
    key.irqkeydesc[0].value = KEY_VALUE;

    for ( i = 0; i < 1; i++)
    {
        ret = request_irq(  key.irqkeydesc[i].irq_label,
                            key.irqkeydesc[i].handler_f,
                            IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
                            key.irqkeydesc[i].name,
                            &key            
                        );
        if (ret<0)
        {
            printk("irq %d request failed! \r\n",key.irqkeydesc[i].irq_label);
            return -EFAULT;
        }
        
    }
    
    /*创建定时器*/
    init_timer(&key.timer);
    key.timer.function = timer_function;

    /*初始化等待队列头*/
    init_waitqueue_head(&key.r_wait);
    return 0;
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
    filp->private_data = &key; /*设置私有数据*/

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

    int ret;
    unsigned char value=0;
    unsigned char releasekey=0;
    gpio_dev *dev = (gpio_dev *)filp->private_data;

#if 0

    /*加入等待队列，等待被唤醒，也就是有按键按下*/
    ret = wait_event_interruptible(dev->r_wait,atomic_read(&dev->releasekey));
    if (ret)
    {
        set_current_state(TASK_RUNNING); /* 设置任务为运行态*/
        remove_wait_queue(&dev->r_wait,&wait);  /*将等待队列移除*/
        return ret;
    }
    
#endif

    // DECLARE_WAITQUEUE(wait,current);/*定义一个等待队列*/
    if (filp->f_flags & O_NONBLOCK) /*非阻塞式访问*/
    {
        if (atomic_read(&dev->releasekey)==0) /*按键未按下*/
        {
            return -EAGAIN;
        }
        else
        {
            /*加入等待队列，等待被唤醒，也就是有按键按下*/
            ret = wait_event_interruptible(dev->r_wait, 
                                        atomic_read(&dev->releasekey));
            if (ret)
            {
                return ret;
            }
        }        
        
    }
    
    value = atomic_read(&dev->keyvalue);
    releasekey = atomic_read(&dev->releasekey);

    if (releasekey)
    {
        if (value & 0x80)
        {
            value &= ~0x80;
            ret = copy_to_user(buf, &value, sizeof(value) );    
        }
        else
        {
            return -EINVAL;
        }
        atomic_set(&dev->releasekey,0);/*按下标志清零 */

    }
    else    
    {
        return -EINVAL;
    }

    return 0;

}


/**
 * @brief poll 函数，用于处理非阻塞访问
 * 
 * @param filp 要打开的设备文件(文件描述符)
 * @param wait 等待列表(poll_table)
 * @return unsigned int 设备或者资源状态
 */
unsigned int wimx6ull_poll( struct file *filp,
                            struct poll_table_struct *wait
                        )
{
    unsigned int mask = 0;
    gpio_dev *dev = (struct gpio*)filp->private_data;

    poll_wait(filp, &dev->r_wait, wait);

    if (atomic_read(&dev->releasekey) ) /*按键按下*/
    {
        mask = POLLIN | POLLRDNORM;     /*返回PLLN*/
    }
    
    return mask;

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

    // int ret = 0;
    // unsigned char databuf[1];
    // unsigned char gpio_stat;
    // gpio_dev *dev = filp->private_data;

    printk("Driver: devwrite 4! \r\n");


    // /*向内核空间写数据 buf -> databuf */
    // ret = copy_from_user(databuf,buf,cnt);
    // if (ret<0 )
    // {
    //     printk("Driver: kernel write data failed! \r\n");
    //     return  -EFAULT;
    // }

    // gpio_stat = databuf[0];   /*获取状态值*/

    // if (gpio_stat == LED_ON)
    // {
    //     // devswitch(LED_ON);
    //     gpio_set_value(dev->gpio_number,0);  /*打开LED灯*/
    //     printk("Driver: len on! \r\n");

    // }
    // else if (gpio_stat == LED_OFF)
    // {
    //     gpio_set_value(dev->gpio_number,1); /*关闭LED灯*/
    //     // devswitch(LED_OFF);
    //     printk("Driver: len off! \r\n");

    // }
    
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
    // gpio_dev *dev = filp->private_data;

    /*互斥锁 解锁*/
    // mutex_unlock(&dev->mutex_lock);

    printk("Driver: devrelease! 7 \r\n");

    return imx6ull_fasync(-1,filp,0);/* 删除异步通知 */


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
    .release = devrelease,
    .poll = wimx6ull_poll,
    .fasync = imx6ull_fasync,

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
    
    // int ret = 0;

    /*初始化原子变量*/
    atomic_set(&key.keyvalue,INvakey);/*原子变量初始值为1*/
    atomic_set(&key.releasekey,0);
    keyio_init();

    printk("Driver: wlsdevinit 1! \r\n");

    /*初始化 互斥锁*/
    // mutex_init(&key.mutex_lock);

    /*注册字符设备驱动*/

    /*1. 申请创建设备号*/
    if (key.major) 
    {
        /*将给定的主设备号和次设备号的值组合成 dev_t 类型的设备号*/
        key.devid = MKDEV(key.major,0);/* 次设备号0*/
        
        /**
         * @brief 注销字符设备之后要释放掉设备号,设备号释放函数
         * 
         * @param from 要释放的设备号
         * @param count 表示从 from 开始，要释放的设备号数量。
         */
        register_chrdev_region(key.devid,LinCdev_CNT,LinCdev_Name);/*注册设备号*/
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
        alloc_chrdev_region(&key.devid,0,LinCdev_CNT,LinCdev_Name);
        
        /*宏 MAJOR 用于从 dev_t 中获取主设备号，将 dev_t 右移 20 位即可*/
        key.major = MAJOR(key.devid);
        
        /*宏 MINOR 用于从 dev_t 中获取次设备号，取 dev_t 的低 20 位的值即可*/
        key.minor = MINOR(key.devid);

    }
    
    printk("Driver: DEV_ID Register OK! key.major:%d minor:%d !\r\n",
            key.major,key.minor);

    /*2. 注册 字符设备*/
    key.char_dev.owner = THIS_MODULE;
    cdev_init(  (struct cdev *)&key.char_dev,
                (const struct file_operations *)&devfops
             );/*初始化char类型设备的结构体变量*/

    /*3. 添加一个 char_dev*/
    cdev_add(&key.char_dev,key.devid,LinCdev_CNT);

    /*4. 创建设备类*/
    key.class = class_create ((struct module *)THIS_MODULE, 
                                    (const char *)LinCdev_Name);

    if ( IS_ERR(key.class) )
    {
        return PTR_ERR(key.class);
    }


    /*5. 创建设备*/
    key.devices = device_create(key.class,NULL,key.devid,NULL,LinCdev_Name);
    if ( IS_ERR(key.devices) )
    {   
        return PTR_ERR(key.devices);
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

    size_t i = 0;
    printk("Driver: wlsdev_exit!\r\n");

    /*删除定时器*/
    del_timer_sync(&key.timer);

    /*释放 中断*/
    for (i = 0; i < 1; i++)
    {
        free_irq(key.irqkeydesc[i].irq_label,&key);
        gpio_free(key.irqkeydesc[i].gpio);
        
    }

    cdev_del(&key.char_dev);/*删除char dev设备*/

    unregister_chrdev_region(key.devid,LinCdev_CNT); /*注销设备号*/

    /*删除设备*/
    device_destroy(key.class,key.devid);
    
    /*删除类*/
    class_destroy(key.class);

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
