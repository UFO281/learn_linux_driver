# KERNELDIR 表示开发板所使用的 Linux 内核源码目录，使用绝对路径，大家根
# 据自己的实际情况填写即可。
KERNELDIR := /home/wls/linux/os_src/linux-imx-rel_imx_4.1.15_2.1.0_ga

#CURRENT_PATH 表示当前路径，直接通过运行“pwd”命令来获取当前所处路径。
CURRENT_PATH := $(shell pwd) 

# obj-m 表示将 chrdevbase.c 这个文件编译为 chrdevbase.ko 模块
obj-m := wlsmain.o 


# `build: kernel_modules` 这一句是 Makefile 中的一个规则，
# 它定义了一个名为 `build` 的目标（target），并指定了该目标依赖于 `kernel_modules` 
# 这个目标。
# ---------------------------------------------------------------------------
# 在这个 Makefile 中，`build` 目标用于编译模块，而 `kernel_modules` 
# 目标是用来编译内核模块的。因此，通过这个规则，当你运行 `make build` 命令时，
# Make 工具会自动执行 `kernel_modules` 目标所定义的命令，从而编译内核模块。
build: kernel_modules

kernel_modules:
# 具体的编译命令，后面的 modules 表示编译模块，-C 表示将当前的工作目录切
# 换到指定目录中，也就是 KERNERLDIR 目录。M 表示模块源码目录，“make modules”命令
# 中加入 M=dir 以后程序会自动到指定的 dir 目录中读取模块的源码并将其编译为.ko 文件
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
	echo "KERNELDIR: $(KERNELDIR)"
	echo "CURRENT_PATH: $(CURRENT_PATH)"
	echo "MAKE: $(MAKE)"
