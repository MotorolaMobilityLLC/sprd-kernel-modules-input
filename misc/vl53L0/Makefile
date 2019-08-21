
LOCAL_PATH := $(PWD)

ccflags-y += -DDEBUG

ccflags-y += -I$(LOCAL_PATH)/inc

obj-m += stmvl53l0.o

stmvl53l0-objs := stmvl53l0_module.o \
                stmvl53l0_module-i2c.o \
                stmvl53l0_module-cci.o \
                src/vl53l0_api_calibration.o \
                src/vl53l0_api_core.o \
                src/vl53l0_api_histogram.o \
                src/vl53l0_api_ranging.o \
                src/vl53l0_api_strings.o \
                src/vl53l0_api.o \
                src/vl53l0_platform.o \
                src/vl53l0_i2c_platform.o \
                src/vl53l0_port_i2c.o

.PHONY:modules

modules:
	make $(BSP_MAKE_EXTRA_ARGS) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

modules_install:
	make $(BSP_MAKE_EXTRA_ARGS) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` INSTALL_MOD_PATH=$(BSP_MODULES_OUT) $@

clean:
	make -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@