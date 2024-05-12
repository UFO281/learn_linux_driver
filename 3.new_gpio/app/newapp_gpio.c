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




/**
 * @brief 
 * 
 * @param count 数组元素的个数
 * @param str 具体参数
 * @return int 
 */
int main(int count,char **str)
{
    printf("\r\nhh\r\n");
    int fd,ret;
    char *file_name=NULL;
    unsigned char databuf[1];

    if (count!=3)
    {
        printf("error usage! \r\n");
        return -1;
    }

    file_name = str[1];
    
    /*打开led驱动文件*/
    fd = open(file_name,O_RDWR);
    if (fd<0 )
    {
        printf("\r\ncan't open file %s\r\n",file_name);
        return -1;
    }
    else
    {
        printf("\r\nopen file OK! %s\r\n",file_name);
        
    }
    

    /*要执行的操作*/
    databuf[0]= atoi(str[2]);

    // /*向/dev/led文件写入数据*/
    // ret = write( fd,databuf,sizeof(databuf) );
    // if (ret<0)
    // {
    //     printf("LED Control failed! \r\n");   
    //     /*关闭设备*/
    //     close(fd);
    //     return -1;
    
    // }
    

    // if ( atoi(str[2])==1 )
    // {/*从驱动文件读取数据*/

    //     ret = read(fd,read_buf,50);
    //     if (ret<0)
    //     {
    //         printf("can't open file %s\r\n",file_name);
    //     }
    //     else
    //     {   /*读取成功打印数据*/
    //         printf("read data: %s\r\n",read_buf);
    //     } 

    // }
    
    // if ( atoi(str[2])==2 )
    // {/*向设备驱动写数据*/

    //     memcpy(write_buf,usrdata,sizeof(usrdata));
    //     ret = write(fd,write_buf,50);
    //     if (ret<0)
    //     {
    //         printf("write file : %s failed! \r\n",file_name);   
    //     }

    // }

    /*关闭设备*/
    ret = close(fd);
    if (ret<0)
    {

        printf("\r\nCan't close file %s \r\n",file_name);   

        return -1;
    }
    else{
        printf("\r\nclose file ok! %s \r\n",file_name);   

    }
    

    return 0;
}

