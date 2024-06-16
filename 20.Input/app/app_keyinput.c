/**
 * @file app_keyinput.c
 * @author wls (ufo281@outlook.com) 
 * @brief 
 * @version 1.0
 * @date 2024-06-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "sys/ioctl.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include <linux/input.h>



/* 定义一个input_event变量,存放输入事件信息  */
static struct input_event inputevent;



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
    int err = 0;
    char *file_name;

    file_name = str[1];

    fd = open(file_name,O_RDWR);
    if (fd<0)
    {
        printf("can't open file %s \r\n",file_name);
        return -1;
    }

 
    while (1)
    {
        printf("APP: Action read! \r\n");

        err = read(fd,&inputevent,sizeof(inputevent));
        if (err>0) /*read ok*/
        {
            switch (inputevent.type)
            {
                case EV_KEY:
                    if (inputevent.code<BTN_MISC) /*键盘值*/
                    {
                        printf("key %d %s \r\n",inputevent.code,
                                inputevent.value?"press":"release");
                    }
                    else
                    {
                        printf("button %d %s \r\n",inputevent.code,
                            inputevent.value?"press":"release");
                    }
                    break;

                case EV_REL:
                    break;
                case EV_ABS:
                    break;
                case EV_MSC:
                    break;
                case EV_SW:
                    break;

            }
        }
        else
        {
            printf("APP: read data error!\r\n");
        }



    }


    if ( close(fd)<0 ) /*关闭设备*/
    {
        printf("APP: Can't close file %s \r\n",file_name);   
        return -1;
    }
    

    return 0;
}

