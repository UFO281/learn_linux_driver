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


#define KEY_VALUE     0XF0  /*按键值*/  
#define INvakey       0x00  /*无效的按键值*/   



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
    int fd,ret;
    char *file_name=NULL;
    unsigned char databuf[1];
    int cnt=0;
    unsigned char  keyvalue;

    // if (count!=3)
    // {
    //     printf("APP: error usage! \r\n");
    //     return -1;
    // }

    file_name = str[1];
    
    /*打开led驱动文件*/
    fd = open(file_name,O_RDWR);
    if (fd<0 )
    {
        printf("APP: can't open file %s\r\n",file_name);
        return -1;
    }
    else
    {
        printf("APP: open file OK! %s\r\n",file_name);
        
    }
    

    /*要执行的操作*/
    databuf[0]= atoi(str[2]);

    /*向/dev/led文件写入数据*/
    ret = write( fd,databuf,sizeof(databuf) );
    if (ret<0)
    {
        printf("APP: LED Control failed! \r\n");   
        /*关闭设备*/
        close(fd);
        return -1;
    
    }

    while (1)
    {
        read(fd,&keyvalue,sizeof(keyvalue));
        if ( keyvalue == KEY_VALUE )
        {
            printf("KEY0 Press, value = %#X\r\n",keyvalue);/*按下*/
        }

    }
    
    /*关闭设备*/
    ret = close(fd);
    if (ret<0)
    {

        printf("APP: Can't close file %s \r\n",file_name);   

        return -1;
    }
    else{
        printf("APP: close file ok! %s \r\n",file_name);   

    }
    

    return 0;
}

