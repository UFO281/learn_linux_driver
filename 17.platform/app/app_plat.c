/**
 * @file app_plat.c
 * @author wls (ufo281@outlook.com) 
 * @brief 
 * @version 1.0
 * @date 2024-06-08
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
#include "poll.h"
#include "sys/select.h"
#include "sys/time.h"
#include "linux/ioctl.h"
#include "signal.h"


#define LEDOFF  0
#define LEDON   1




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

    int fd, retvalue;
    char *file_name;
    unsigned char databuf[2];

    file_name = str[1];

    fd = open(file_name,O_RDWR);
    if (fd<0)
    {
        printf("can't open file %s \r\n",file_name);
        return -1;
    }

    databuf[0] = atoi(str[2]);/*操作 open/close*/
    retvalue = write(fd,databuf,sizeof(databuf));
    if (retvalue<0)
    {
        printf("APP: LED Control Failed! \r\n");   
        close(fd);
        return -1;
    }
    

    if ( close(fd)<0 ) /*关闭设备*/
    {
        printf("APP: Can't close file %s \r\n",file_name);   
        return -1;
    }
    

    return 0;
}

