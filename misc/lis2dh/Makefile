ifeq ($(BUILD),debug)
ccflags-y += -DDEBUG
endif


#
#Makefile for the input misc STM acc lis2dh driver
#
obj-m += lis2dh.o
lis2dh-objs := lis2dh_acc.o lis2dh_i2c.o


.PHONY:modules

modules:
	$(MAKE) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

modules_install:
	$(MAKE) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` INSTALL_MOD_PATH=$(BSP_MODULES_OUT) $@

clean:
	$(MAKE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@
