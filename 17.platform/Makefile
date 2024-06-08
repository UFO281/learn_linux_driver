# 设置内核源码目录和当前路径
KERNELDIR := /home/wls/linux/os_src/linux-imx-rel_imx_4.1.15_2.1.0_ga

CURRENT_PATH := $(shell pwd)

# 设置目标文件名
obj-m := platdev.o platdriver.o

# build目标
build: driver_modules app_modules

# 编译内核模块
driver_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

# 编译应用程序模块
app_modules:
	$(MAKE) -C app/ 

# 清理
clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
	$(MAKE) -C app/ clean
	echo "KERNELDIR: $(KERNELDIR)"
	echo "CURRENT_PATH: $(CURRENT_PATH)/app"
	echo "MAKE: $(MAKE)"
