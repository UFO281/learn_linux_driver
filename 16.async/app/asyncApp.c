/**
 * @file newapp_gpio.c
 * @author wls (ufo281@outlook.com) 
 * @brief 
 * @version 1.0
 * @date 2024-05-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
// #include <fcntl.h>
#include "stdlib.h"
#include "string.h"
#include "poll.h"
#include "sys/select.h"
#include "sys/time.h"
#include "linux/ioctl.h"
#include "signal.h"



#define CLOSE_CMD       (_IO(0XEF,0X1))  /*关闭定时器*/
#define OPEN_CMD        (_IO(0XEF,0X2))  /*打开定时器*/
#define SETPERIOD_CMD   (_IO(0XEF,0X3))  /*设置定时器周期指令*/


static int fd = 0; /*文件描述符*/


/**
 * @brief SIGIO信号处理函数
 * 
 * @param signum 信号值
 */
static void sigio_signal_func(int signum)
{
    int err = 0;
    unsigned int keyvalue = 0;
    err = read( fd, &keyvalue, sizeof(keyvalue));
    if (err<0)
    {
        /*read error*/
    
    }else{
        
        printf("sigio signal! key value = %d \r\n",keyvalue);

    }

    

}



/**
 * @brief 
 * 
 * @param count 数组元素的个数
 * @param str 具体参数
 * @return int 
 */
int main(int count,char **str)
{

    printf("APP: Running ok! \r\n");

    int flags = 0;
    char *file_name;

    file_name = str[1];
    fd = open(file_name,O_RDWR);
    if (fd<0)
    {
        printf("can't open file %s \r\n",file_name);
        return -1;
    }

    /*设置信号SIGIO的处理函数*/
    signal(SIGIO,sigio_signal_func);

    fcntl(fd,8,getpid()); /*将当前进程的进程号告诉内核*/
    flags = fcntl(fd,F_GETFD); /*获取当前的进程状态 */
    fcntl(fd,F_SETFL,flags|00020000); /*设置进程启用异步通知功能*/

    while (1)
    {
        sleep(2);
    }
    


    if ( close(fd)<0 ) /*关闭设备*/
    {
        printf("APP: Can't close file %s \r\n",file_name);   
        return -1;
    }
    

    return 0;
}

