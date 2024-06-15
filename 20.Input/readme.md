# <center> 第五十八章 Linux INPUT 子系统实验
>按键、鼠标、键盘、触摸屏等都属于输入(input)设备，Linux 内核为此专门做了一个叫做 input子系统的框架来处理输入事件。输入设备本质上还是字符设备，只是在此基础上套上了 input 框架，用户只需要负责上报输入事件，比如按键值、坐标等信息，input 核心层负责处理这些事件。本章我们就来学习一下 Linux 内核中的 input 子系统。
---
## 58.1 input 子系统
### 58.1.1 input 子系统简
input 就是输入的意思，因此 input 子系统就是管理输入的子系统，和 pinctrl、gpio 子系统
一样，都是 Linux 内核针对某一类设备而创建的框架。比如按键输入、键盘、鼠标、触摸屏等
等这些都属于输入设备，不同的输入设备所代表的含义不同，按键和键盘就是代表按键信息，
鼠标和触摸屏代表坐标信息，因此在应用层的处理就不同，对于驱动编写者而言不需要去关心
应用层的事情，我们只需要按照要求上报这些输入事件即可。为此 input 子系统分为 input 驱动
层、input 核心层、input 事件处理层，最终给用户空间提供可访问的设备节点，input 子系统框
架如图 58.1.1.1 所示：
