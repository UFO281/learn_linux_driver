#!/bin/bash

# 检查输入参数是否为1或0
if [ $# -eq 1 ]; then
    
    # 如果参数为1，执行命令：sudo ifconfig ens33 up
    if [ "$1" -eq 1 ]; then
        sudo ifconfig ens33 up
        echo "Interface ens33 is up."
        ifconfig

    # 如果参数为0，执行命令：sudo ifconfig ens33 down
    elif [ "$1" -eq 0 ]; then
        sudo ifconfig ens33 down
        echo "Interface ens33 is down."
        ifconfig

    # 如果参数既不是1也不是0，则打印错误消息
    else
        echo "Invalid argument. Please provide 1 or 0 as argument."
        ifconfig
    fi
else
    echo "Usage: $0 <argument>"
    echo "Argument 1: bring up interface ens33"
    echo "Argument 0: bring down interface ens33"
fi

