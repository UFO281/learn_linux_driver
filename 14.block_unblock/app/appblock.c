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
#include "stdlib.h"
#include "string.h"
#include "linux/ioctl.h"



#define CLOSE_CMD       (_IO(0XEF,0X1))  /*关闭定时器*/
#define OPEN_CMD        (_IO(0XEF,0X2))  /*打开定时器*/
#define SETPERIOD_CMD   (_IO(0XEF,0X3))  /*设置定时器周期指令*/


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
    int fd;
    int ret;
    char *file_name=NULL;
    unsigned char data;

    // if (count!=3)
    // {
    //     printf("APP: error usage! \r\n");
    //     return -1;
    // }

    file_name = str[1];
    
    /*打开led驱动文件*/
    fd = open(file_name,O_RDWR);/*默认为阻塞方式打开*/
    // fd = open(file_name,O_RDWR|O_NONBLOCK);/*以非阻塞方式打开*/
    if (fd<0 )
    {
        printf("APP: can't open file %s\r\n",file_name);
        return -1;
    }
    

    // /*要执行的操作*/
    // databuf[0]= atoi(str[2]);

    // /*向/dev/led文件写入数据*/
    // ret = write( fd,databuf,sizeof(databuf) );
    // if (ret<0)
    // {
    //     printf("APP: LED Control failed! \r\n");   
    //     /*关闭设备*/
    //     close(fd);
    //     return -1;
    
    // }

    while (1)
    {
        ret = read(fd, &data, sizeof(data) );
        if (ret<0)  /*数据读错或者无效*/
        {

        }
        else{ /*数据正确*/

            if (data)
            {
                printf("key value = %#X\r\n",data);
            }
            
        }        
        
    }
    
    /*关闭设备*/
    ret = close(fd);
    if (ret<0)
    {
        printf("APP: Can't close file %s \r\n",file_name);   
        return -1;
    }
    

    return 0;
}

