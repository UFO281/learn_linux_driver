# 定义编译器变量，这里使用 GCC
CC=arm-linux-gnueabihf-gcc

# 设置编译器标志变量，'-Wall' 用于显示所有警告 '-g' 添加调试信息
CFLAGS= -Wall -g -Wextra -std=c11

# 定义最终可执行文件的名称
TARGET = appblock

# 默认目标 all，依赖于 $(TARGET) 即可执行文件
all: $(TARGET)

# 规则用于构建最终的目标文件 $(TARGET)，它依赖于对象文件 appwlsmain.o
# 该规则会调用 $(CC) 执行链接
$(TARGET): $(TARGET).o
	$(CC) $(CFLAGS) -o $(TARGET) $(TARGET).o

# 规则告诉 make 如何生成 appwlsmain.o 从 appwlsmain.c 文件
# 使用 $(CC) 和 $(CFLAGS) 编译源代码
appwlsmain.o: $(TARGET).c
	$(CC) $(CFLAGS) -c $(TARGET).c

# clean 目标用于清除所有编译生成的文件，保持目录干净
# 'rm -f' 用于删除文件，'-f' 标志忽略不存在的文件，避免错误提示
clean:
	rm -f *.o $(TARGET)

# 声明伪目标，告诉 make 这些目标不对应文件
.PHONY: all clean




# CROSS_COMPILE 	?= arm-linux-gnueabihf-
# TARGET		  	?= main

# CC 				:= $(CROSS_COMPILE)gcc
# LD				:= $(CROSS_COMPILE)ld
# OBJCOPY 		:= $(CROSS_COMPILE)objcopy
# OBJDUMP 		:= $(CROSS_COMPILE)objdump

# LIBPATH			:= -lgcc -L /usr/local/arm/gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf/lib/gcc/arm-linux-gnueabihf/4.9.4


# INCDIRS 		:= imx6ul \
# 				   bsp/clk \
# 				   bsp/led \
# 				   bsp/delay  \
# 				   bsp/beep \
# 				   bsp/gpio \
# 				   bsp/key \
# 				   bsp/exit \
# 				   bsp/int \
# 				   bsp/epittimer \
# 				   bsp/keyfilter \
# 				   bsp/uart 
				   			   
# SRCDIRS			:= project \
# 				   bsp/clk \
# 				   bsp/led \
# 				   bsp/delay \
# 				   bsp/beep \
# 				   bsp/gpio \
# 				   bsp/key \
# 				   bsp/exit \
# 				   bsp/int \
# 				   bsp/epittimer \
# 				   bsp/keyfilter \
# 				   bsp/uart 
				   
				   
# INCLUDE			:= $(patsubst %, -I %, $(INCDIRS))

# SFILES			:= $(foreach dir, $(SRCDIRS), $(wildcard $(dir)/*.S))
# CFILES			:= $(foreach dir, $(SRCDIRS), $(wildcard $(dir)/*.c))

# SFILENDIR		:= $(notdir  $(SFILES))
# CFILENDIR		:= $(notdir  $(CFILES))

# SOBJS			:= $(patsubst %, obj/%, $(SFILENDIR:.S=.o))
# COBJS			:= $(patsubst %, obj/%, $(CFILENDIR:.c=.o))
# OBJS			:= $(SOBJS) $(COBJS)

# VPATH			:= $(SRCDIRS)

# .PHONY: c
	
# $(TARGET).bin : $(OBJS)
# 	$(LD) -Timx6ul.lds -o $(TARGET).elf $^ $(LIBPATH)
# 	$(OBJCOPY) -O binary -S $(TARGET).elf $@
# 	$(OBJDUMP) -D -m arm $(TARGET).elf > $(TARGET).dis

# $(SOBJS) : obj/%.o : %.S
# 	$(CC) -Wall -nostdlib -fno-builtin -c -O2  $(INCLUDE) -o $@ $<

# $(COBJS) : obj/%.o : %.c
# 	$(CC) -Wall -nostdlib -fno-builtin -c -O2  $(INCLUDE) -o $@ $<
	
# c:
# 	rm -rf $(TARGET).elf $(TARGET).dis $(TARGET).bin $(COBJS) $(SOBJS)

	
