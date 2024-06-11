# <center> 第五十七章 Linux MISC 驱动实验
>misc 的意思是混合、杂项的，因此 MISC 驱动也叫做杂项驱动，也就是当我们板子上的某
些外设无法进行分类的时候就可以使用 MISC 驱动。MISC 驱动其实就是最简单的字符设备驱
动，通常嵌套在 platform 总线驱动中，实现复杂的驱动，本章我们就来学习一下 MISC 驱动的
编写。
---
## 57.1 MISC 设备驱动简介
所有的 MISC 设备驱动的主设备号都为 10，不同的设备使用不同的从设备号。随着 Linux
字符设备驱动的不断增加，设备号变得越来越紧张，尤其是主设备号，MISC 设备驱动就用于解
决此问题。MISC 设备会自动创建 cdev，不需要像我们以前那样手动创建，因此采用 MISC 设
备驱动可以简化字符设备驱动的编写。我们需要向 Linux 注册一个 miscdevice 设备，miscdevice
是一个结构体，定义在文件 include/linux/miscdevice.h 中，内容如下：   

>示例代码 57.1.1 miscdevice 结构体代码
57 struct miscdevice {
58 int minor; /* 子设备号 */
59 const char *name; /* 设备名字 */
60 const struct file_operations *fops; /* 设备操作集 */
61 struct list_head list;
62 struct device *parent;
63 struct device *this_device;
64 const struct attribute_group **groups;
65 const char *nodename;
66 umode_t mode;
67 };



