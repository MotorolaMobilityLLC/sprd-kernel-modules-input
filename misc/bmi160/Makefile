ifeq ($(BUILD),debug)
ccflags-y += -DDEBUG
endif


#
#Makefile for Miramems Motion Sensor driver
#
#obj - $(CONFIG_INPUT_MIR3DA) += mir3da.o
obj-m += bmi160sum.o
bmi160sum-objs :=bmi160_driver.o bmi160.o bmi160_i2c.o


.PHONY:modules

modules:
	make ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

modules_install:
	make ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` INSTALL_MOD_PATH=$(BSP_MODULES_OUT) $@

clean:
	make -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@
