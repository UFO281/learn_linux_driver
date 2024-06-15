/**
 * @file keyinput.c
 * @author wls (ufo281@outlook.com) 
 * @brief 第五十八章 Linux INPUT 子系统实验
 *  按键、鼠标、键盘、触摸屏等都属于输入(input)设备，Linux 内核为此专门做了一个叫做 input
子系统的框架来处理输入事件。输入设备本质上还是字符设备，只是在此基础上套上了 input 框
架，用户只需要负责上报输入事件，比如按键值、坐标等信息，input 核心层负责处理这些事件。
本章我们就来学习一下 Linux 内核中的 input 子系统。
 * 
 * 
 * @version 1.0
 * @date 2024-06-15
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
#include <linux/input.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/fcntl.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>



/*--------------------------------------------------------------------*/
#define KEYINPUT_CNT    1   /*设备号个数*/
#define KEYINPUT_NAME   "keyinput"  /* 设备名*/
#define KEY0VALUE       0x01    /*KEY0按键值*/
#define INVAKEY         0xFF    /*无效的按键值*/
#define KEY_NUM         1   /*按键数量*/    



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
    IRQ_IO  irqkeydesc[1];  /*按键描述数组*/
    unsigned char curkeynum;    /*当前的按键号*/
    wait_queue_head_t   r_wait; /*读等待队列头*/
    struct fasync_struct  *async_queue; /*异步相关结构体*/ 
    struct input_dev *inputdev;     /*input结构体*/


}gpio_dev;

// gpio_dev beep;
// gpio_dev miscbeep;
gpio_dev key_inputdev;


/**
 * @brief 
 * 
 * @param irq 
 * @param dev_id 
 * @return irqreturn_t 
 */
static irqreturn_t key0_handler(int irq,void *dev_id)
{
    gpio_dev *dev = (gpio_dev*)dev_id;
    dev->curkeynum = 0;
    dev->timer.data = (volatile long)dev_id;
    mod_timer( &dev->timer,jiffies+msecs_to_jiffies(10) );/*10ms 后启动timer*/

    return IRQ_RETVAL(IRQ_HANDLED);

}


/**
 * @brief 定时器服务函数，用于按键消抖，定时器到了以后
 *      再次读取按键值，如果按键还是处于按下状态就表示按键有效。
 * 
 * @param arg 设备结构变量
 */
void timer_function(unsigned long arg)
{
    unsigned char value;
    unsigned char num;
    IRQ_IO *keydec;
    gpio_dev *dev = (gpio_dev*)arg; 

    num = dev->curkeynum;
    keydec = &dev->irqkeydesc[1];
    value = gpio_get_value(keydec->gpio);
    if ( value ==0 )
    {
        input_report_key(dev->inputdev,keydec->value,1); /*1 按下*/
        input_sync(dev->inputdev);   
    }
    else{

        input_report_key(dev->inputdev,keydec->value,0); /*0 没按下*/
        input_sync(dev->inputdev);   
    }


}


/**
 * @brief 按键 IO 初始化
 * 
 * @return int 
 */
static int keyio_init(void)
{

    char name[10];
    int ret = 0; 

    key_inputdev.nd = of_find_node_by_path("/key");
    if (  key_inputdev.nd == NULL)
    {
        printk("key node not find! \r\n");
        return -EINVAL;
    }

    key_inputdev.irqkeydesc[0].gpio = 
        of_get_named_gpio(key_inputdev.nd,"key-gpio",0); /*根据设备树节点名字得到IO*/
    if ( key_inputdev.irqkeydesc[0].gpio<0 )
    {
        printk(" Can't get key%d! \r\n");
    }
    
    memset(key_inputdev.irqkeydesc[0].name,0,sizeof(name));
    sprintf(key_inputdev.irqkeydesc[0].name,"key%d",0);  
    gpio_request(key_inputdev.irqkeydesc[0].gpio,
                key_inputdev.irqkeydesc[0].name);
    gpio_direction_input(key_inputdev.irqkeydesc[0].gpio);
    key_inputdev.irqkeydesc[0].irq_label = irq_of_parse_and_map(key_inputdev.nd,0);
    key_inputdev.irqkeydesc[0].handler_f = key0_handler;
    key_inputdev.irqkeydesc[0].value = KEY_0;

    ret = request_irq(  key_inputdev.irqkeydesc[0].irq_label,
                        key_inputdev.irqkeydesc[0].handler_f,
                        IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                        key_inputdev.irqkeydesc[0].name,
                        &key_inputdev
                        );

    if ( ret<0 )
    {
        printk(" irq %d  request failed ! \r\n",key_inputdev.irqkeydesc[0].irq_label);
        return -EFAULT;
    }
    
    /*creat soft timer*/
    init_timer(&key_inputdev.timer);
    key_inputdev.timer.function = timer_function;

    /* 申请input_dev*/
    key_inputdev.inputdev = input_allocate_device();
    key_inputdev.inputdev->name = KEYINPUT_NAME;

    /* 设置成key事件*/
    key_inputdev.inputdev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);

    /*调用 input_set_capability 函数设置 EV_KEY 事件以及 KEY 的按键类型，也就
    是 KEY 作为哪个按键？我们会在设备树里面设置指定的 KEY 作为哪个按键*/
    input_set_capability(key_inputdev.inputdev,EV_KEY,KEY_0);

    /*注册设备*/
    ret = input_register_device(key_inputdev.inputdev);
    if (ret)
    {
        printk("register input device failed! \r\n");
        return ret;
    }
    
    return 0;

}


/**
 * @brief 驱动入口函数
 * 
 * @return int 
 */
static int __init keyinput_init(void)
{

    keyio_init();
    return 0;
}



/**
 * @brief 驱动出口函数
 * 
 */
static void __exit keyinput_exit(void)
{

    del_timer_sync(&key_inputdev.timer);

    free_irq(key_inputdev.irqkeydesc[0].irq_label,&key_inputdev);
    gpio_free(key_inputdev.irqkeydesc[0].gpio);

    /*释放input dev*/
    input_unregister_device(key_inputdev.inputdev);
    input_free_device(key_inputdev.inputdev);

}


/**
 * @brief 使用“modprobe -r”命令卸载驱动，比如要卸载 drv.ko，命令如下：
                    modprobe -r drv.ko
        会调用这个接口
 * 
 */
module_init(keyinput_init);


/**
 * @brief 驱动模块的卸载使用命令“rmmod”即可，比如要卸载 drv.ko，使用如下命令即可：
                rmmod drv.ko
        会调用这个接口
 */
module_exit(keyinput_exit);



/**
 * @brief LICENSE 和作者信息
 * 
 */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("wls");
