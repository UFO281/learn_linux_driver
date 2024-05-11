/**
 * @file app_wlsmain.c
 * @author wls (ufo281@outlook.com) 
 * @brief 
 * @version 1.0
 * @date 2024-05-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>


static char usrdata[] = {"usr data!"};



/**
 * @brief 
 * 
 * @param count 数组元素的个数
 * @param str 具体参数
 * @return int 
 */
int main(int count,char **str)
{
    int fd,ret;
    char *file_name=NULL;
    char read_buf[100]={0};
    char write_buf[100]={0};
    if (count!=3)
    {
        printf("error usage! \r\n");
        return -1;
    }

    file_name = str[1];

    /*打开驱动文件*/
    fd = open(file_name,O_RDWR);
    if (fd<0 )
    {
        printf("can't open file %s\r\n",file_name);
        return -1;
    }

    if ( atoi(str[2])==1 )
    {/*从驱动文件读取数据*/

        ret = read(fd,read_buf,50);
        if (ret<0)
        {
            printf("can't open file %s\r\n",file_name);
        }
        else
        {   /*读取成功打印数据*/
            printf("read data: %s\r\n",read_buf);
        } 

    }
    
    if ( atoi(str[2])==2 )
    {/*向设备驱动写数据*/

        memcpy(write_buf,usrdata,sizeof(usrdata));
        ret = write(fd,write_buf,50);
        if (ret<0)
        {
            printf("write file : %s failed! \r\n",file_name);   
        }

    }

    /*关闭设备*/
    ret = close(fd);
    if (ret<0)
    {

        printf("Can't close file %s \r\n",file_name);   

        return -1;
    }
    

    return 0;
}

