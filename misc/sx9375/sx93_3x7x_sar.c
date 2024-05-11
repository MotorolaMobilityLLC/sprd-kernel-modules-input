/*! \file sx93_3x7x_sar.c
 * \brief  Semtech CAP Sensor Driver for SAR
 *
 * Copyright (c) 2023 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/sort.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/notifier.h>
#include <linux/usb.h>
#include <linux/power_supply.h>
#include <linux/version.h>

//#include "sx933x.h"
#include "sx937x.h"

#define CONFIG_CAPSENSE_USB_CAL 1
#ifdef CONFIG_CAPSENSE_USB_CAL
#define USB_POWER_SUPPLY_NAME   "battery"
#define CONFIG_USE_POWER_SUPPLY_ONLINE 1
#endif
#define HEADSET_CALI 1
#ifdef HEADSET_CALI
extern int headset_register_notifier(struct notifier_block *nb);
extern int headset_unregister_notifier(struct notifier_block *nb);
#endif

#define USE_THREAD_IRQ
//#define ENABLE_STAY_AWAKE
//#define ENABLE_ESD_RECOVERY

#define DISABLE_DURING_SUSPEND      0
#define PAUSE_DURING_SUSPEND        1
#define WAKE_BY_IRQ                 0

#if (WAKE_BY_IRQ + PAUSE_DURING_SUSPEND + DISABLE_DURING_SUSPEND) > 1
#error Only enable one or none of them
#endif

#ifdef ENABLE_STAY_AWAKE
#ifdef CONFIG_PM
#include <linux/pm_wakeup.h>
#include <linux/pm_wakeirq.h>
#define WAKE_TMOUT 2000 //milliseconds
#else
#error CONFIG_PM is not enabled in the Kconfig
#endif
#endif

#define I2C_M_WR_SMTC           0 /* for i2c Write */
#define I2C_M_RD_SMTC           1 /* for i2c Read */
#define NUM_IRQ_BITS            8
#define NUM_RETRY_ON_I2C_ERR    3
#define SLEEP_BETWEEN_RETRY     50 //in ms
#define I2C_DELAY_AFTER_RESUME  50 //in ms

//Refer to register 0x8000
typedef enum {
    RELEASED = 0,
    PROX1 = 1,  //SX933X: PROXSTAT
    PROX2 = 2,  //SX933X: TABLESTAT
    PROX3 = 3,  //SX933X: BODYSTAT
    PROX4 = 4   //SX933X: Not used
}PROX_STATUS;

typedef enum {
    NOT_USED = 0,   //this phase is not used.
    MAIN = 1,       //its CS is connected to an antenna for detecting human proximity.
    REF = 2         //Reference phase. Used to correct the temperature drift of the correspoinding MAIN phase
}PHASE_USAGE;

//Refer to register 0x4280
typedef enum{
    CMD_ACTIVATE    = 0xF,
    CMD_COMPENSATE  = 0xE,
    CMD_PAUSE       = 0xD,
    CMD_RESUME      = 0xC,

}COMMANDS;

typedef struct reg_addr_val_s
{
    //declare it as u32 is for parsing the dts, though regiter addr is 16 bits,
    u32 addr;
    u32 val;
}reg_addr_val_t, *reg_addr_val_p;

typedef struct phase_s
{
    char *name; //Name to show in the input getevent
    struct input_dev *input;

    //Refer to register 0x8000, prox4_mask is not used by the sx933x
    u32 prox4_mask;
    u32 prox3_mask;
    u32 prox2_mask;
    u32 prox1_mask;

    bool enabled;
    PHASE_USAGE usage;
    PROX_STATUS state;
}phase_t, *phase_p;

typedef struct variables_s
{
    u16 reading_reg;
    u32 irq_mask;
    u32 enabled_phases;
}variables_t, *variables_p;

typedef struct debug_flag_s
{
    //log raw data (useful, diff, etc.) when proximity status is changed
    char log_raw_data;
    char log_dbg_data;
    char log_hex_data;
}debug_flag_t;

typedef struct self_s
{
    struct i2c_client *client;
    int irq_id; /* irq number used */
    int gpio;
    int num_regs;   //Number of registers need to be configured. Parsed from dts
    reg_addr_val_t *regs_addr_val;

    phase_t *phases;

    /*Each bit represents whether its correspoding phase is used for(1) this runction or not(0)
    for example: main_phases=0x5=0b0101, means PHASE 2 and 0 are used as the main phase*/
    u32 main_phases;
    u32 ref_phases;

    //phases are for temperature reference correction
    int ref_phase_a;
	int ref_phase_b;
    int ref_phase_c;

    variables_t variables;
    debug_flag_t dbg_flag;

    spinlock_t lock; /* Spin Lock used for nirq worker function */
#ifndef USE_THREAD_IRQ
    struct delayed_work worker; /* work struct for worker function */
#endif

#ifdef ENABLE_STAY_AWAKE
    struct wakeup_source *wake_lock;
#endif

    struct mutex phen_lock;

#ifdef CONFIG_CAPSENSE_USB_CAL
        struct work_struct ps_notify_work;
        struct notifier_block ps_notif;
        bool ps_is_present;
#endif
#ifdef HEADSET_CALI
        struct work_struct hs_notify_work;
        struct notifier_block hs_notif;
        int hs_is_plug;
#endif

    //Function that will be called back when corresponding IRQ is raised
    //Refer to register 0x4000
    void (*irq_handler[NUM_IRQ_BITS])(struct self_s* self);
}self_t, *Self;

#define SMTC_LOG_DBG(fmt, args...)   pr_debug("[DBG]"  SMTC_CHIP_NAME ".%s(%d):" fmt "\n", __func__, __LINE__, ##args)
#define SMTC_LOG_INF(fmt, args...)   pr_info ("[INF]"  SMTC_CHIP_NAME ".%s(%d):" fmt "\n", __func__, __LINE__, ##args)
#define SMTC_LOG_WRN(fmt, args...)   pr_warn ("[WRN]"  SMTC_CHIP_NAME ".%s(%d):" fmt "\n", __func__, __LINE__, ##args)
#define SMTC_LOG_ERR(fmt, args...)   pr_err  ("[ERR]"  SMTC_CHIP_NAME ".%s(%d):" fmt "\n", __func__, __LINE__, ##args)

static void smtc_log_raw_data(Self self);

//#################################################################################################

//The value of all other variables not configured in the table are 0.
#if defined(SMTC_SX933X)
static phase_t smtc_phase_table[] =
{
	{
        .name = "SMTC SAR Sensor CH0",
		.prox1_mask = 1 << 24,
		.prox2_mask = 1 << 16,
		.prox3_mask = 1 << 8
	},
	{
		.name = "SMTC SAR Sensor CH1",
		.prox1_mask = 1 << 25,
		.prox2_mask = 1 << 17,
		.prox3_mask = 1 << 9
	},
	{
		.name = "SMTC SAR Sensor CH2",
		.prox1_mask = 1 << 26,
		.prox2_mask = 1 << 18,
		.prox3_mask = 1 << 10
	},
	{
		.name = "SMTC SAR Sensor CH3",
		.prox1_mask = 1 << 27,
		.prox2_mask = 1 << 19,
		.prox3_mask = 1 << 11,
	},
	{
		.name = "SMTC SAR Sensor CH4",
		.prox1_mask = 1 << 28,
		.prox2_mask = 1 << 20,
		.prox3_mask = 1 << 12
	},
	{
		.name = "SMTC SAR Sensor CH5",
		.prox1_mask = 1 << 29,
		.prox2_mask = 1 << 21,
		.prox3_mask = 1 << 13
	}
};

#else
static phase_t smtc_phase_table[] =
{
	{
        .name = "SMTC SAR Sensor CH0",
		.prox1_mask = 1 << 24,
		.prox2_mask = 1 << 16,
		.prox3_mask = 1 << 8,
		.prox4_mask = 1 << 0
	},
	{
		.name = "SMTC SAR Sensor CH1",
		.prox1_mask = 1 << 25,
		.prox2_mask = 1 << 17,
		.prox3_mask = 1 << 9,
		.prox4_mask = 1 << 1
	},
	{
		.name = "SMTC SAR Sensor CH2",
		.prox1_mask = 1 << 26,
		.prox2_mask = 1 << 18,
		.prox3_mask = 1 << 10,
		.prox4_mask = 1 << 2
	},
	{
		.name = "SMTC SAR Sensor CH3",
		.prox1_mask = 1 << 27,
		.prox2_mask = 1 << 19,
		.prox3_mask = 1 << 11,
		.prox4_mask = 1 << 3
	},
	{
		.name = "SMTC SAR Sensor CH4",
		.prox1_mask = 1 << 28,
		.prox2_mask = 1 << 20,
		.prox3_mask = 1 << 12,
		.prox4_mask = 1 << 4
	},
	{
		.name = "SMTC SAR Sensor CH5",
		.prox1_mask = 1 << 29,
		.prox2_mask = 1 << 21,
		.prox3_mask = 1 << 13,
		.prox4_mask = 1 << 5
	},
	{
		.name = "SMTC SAR Sensor CH6",
		.prox1_mask = 1 << 30,
		.prox2_mask = 1 << 22,
		.prox3_mask = 1 << 14,
		.prox4_mask = 1 << 6
	},
	{
		.name = "SMTC SAR Sensor CH7",
		.prox1_mask = 1 << 31,
		.prox2_mask = 1 << 23,
		.prox3_mask = 1 << 15,
		.prox4_mask = 1 << 7
	}
};
#endif

static inline u32 smtc_get_chip_id(u32 chip_id){
    return (chip_id >> 8) & 0xFFFF;
}
//#################################################################################################
static int smtc_i2c_write(Self self, u16 reg_addr, u32 reg_val)
{
    int ret = -ENOMEM;
    int num_retried = 0;
    struct i2c_client *i2c = 0;
    struct i2c_msg msg;
    unsigned char w_buf[6];

    i2c = self->client;
    w_buf[0] = (u8)(reg_addr>>8);
    w_buf[1] = (u8)(reg_addr);
    w_buf[2] = (u8)(reg_val>>24);
    w_buf[3] = (u8)(reg_val>>16);
    w_buf[4] = (u8)(reg_val>>8);
    w_buf[5] = (u8)(reg_val);

    msg.addr = i2c->addr;
    msg.flags = I2C_M_WR_SMTC;
    msg.len = 6; //2 bytes regaddr + 4 bytes data
    msg.buf = (u8 *)w_buf;

    while(true)
    {
        ret = i2c_transfer(i2c->adapter, &msg, 1);
        if (ret > 0)
            break;

        if (num_retried++ < NUM_RETRY_ON_I2C_ERR)
        {
            SMTC_LOG_ERR("i2c write reg 0x%x error %d. Goint to retry", reg_addr, ret);
            if(SLEEP_BETWEEN_RETRY != 0)
                msleep(SLEEP_BETWEEN_RETRY);
        }
        else{
            SMTC_LOG_ERR("i2c write reg 0x%x error %d after retried %d times", reg_addr, ret, NUM_RETRY_ON_I2C_ERR);
             break;
        }
    }

    return ret;
}

//=================================================================================================
//return number of bytes read from the i2c device or error codes
static int smtc_i2c_read(Self self, u16 reg_addr, u32 *reg_val)
{
    int ret = -ENOMEM;
    int num_retried = 0;
    struct i2c_client *i2c = 0;
    struct i2c_msg msg[2];
    u8 w_buf[2];
    u8 buf[4];


    if (self && self->client)
    {
        i2c = self->client;
        w_buf[0] = (u8)(reg_addr>>8);
        w_buf[1] = (u8)(reg_addr);

        msg[0].addr = i2c->addr;
        msg[0].flags = I2C_M_WR_SMTC;
        msg[0].len = 2;
        msg[0].buf = (u8 *)w_buf;

        msg[1].addr = i2c->addr;;
        msg[1].flags = I2C_M_RD_SMTC;
        msg[1].len = 4;
        msg[1].buf = (u8 *)buf;

        while(true)
        {
            ret = i2c_transfer(i2c->adapter, msg, 2);
            if (ret > 0){
                *reg_val = (u32)buf[0]<<24 | (u32)buf[1]<<16 | (u32)buf[2]<<8 | (u32)buf[3];
                break;
            }

            if (num_retried++ < NUM_RETRY_ON_I2C_ERR)
            {
                SMTC_LOG_ERR("i2c read reg 0x%x error %d. Goint to retry", reg_addr, ret);
                if(SLEEP_BETWEEN_RETRY != 0)
                    msleep(SLEEP_BETWEEN_RETRY);
            }
            else{
                SMTC_LOG_ERR("i2c read reg 0x%x error %d after retried %d times",
                    reg_addr, ret, NUM_RETRY_ON_I2C_ERR);
                 break;
            }
       }

    }

    if (reg_addr == 0x4000){
        SMTC_LOG_DBG("reg_addr=0x%X reg_val=0x%X ret=%d", reg_addr, *reg_val, ret);
    }
    return ret;
}

//=================================================================================================
static int smtc_send_cmd(Self self, COMMANDS cmd)
{
    int retry_times=100, state, ret;
    while(1)
    {
        ret = smtc_i2c_read(self, REG_CMD_STATE, &state);
        if (ret < 0){
            SMTC_LOG_ERR("Failed to send command.");
            return -EIO;
        }
        if ((state & 1) == 0){
            SMTC_LOG_DBG("Chip is free and capable to process new command.");
            break;
        }

        if (--retry_times == 0){
            SMTC_LOG_WRN("Chip keeps busy after 10 times retry.");
            break;
        }
        msleep(10);
        SMTC_LOG_DBG("Chip is busy, go to retry.");
    }

    ret = smtc_i2c_write(self, REG_CMD, (u32)cmd);
    if (ret < 0){
        SMTC_LOG_ERR("Failed to send command.");
        return -EIO;
    }

    return 0;
}

static int smtc_enable_phase(Self self, u32 phases)
{
    int ret;
    SMTC_LOG_DBG("phases=0x%X", phases);

    mutex_lock(&self->phen_lock);
    self->variables.enabled_phases = phases;
    ret = smtc_i2c_write(self, REG_PHEN, phases);
    mutex_unlock(&self->phen_lock);

    return ret;
}
//#################################################################################################
static int smtc_read_and_clear_irq(Self self, u32 *irq_src)
{
    int ret = 0;
    u32 reg_val;

    ret = smtc_i2c_read(self, REG_IRQ_SRC, &reg_val);
    if (ret > 0)
    {
        reg_val &= 0xFF;
        SMTC_LOG_DBG("irq_src= 0x%X", reg_val);

        if (irq_src){
            *irq_src = reg_val;
        }
        ret = 0;
    }
    else if (ret==0){
        ret = -EIO;
    }

    return ret;
}

//=================================================================================================
static int smtc_calibrate(Self self)
{
    int ret = 0;
	SMTC_LOG_INF("Enter");

    ret = smtc_send_cmd(self, CMD_COMPENSATE);
    if (ret){
        SMTC_LOG_ERR("Failed to calibrate. ret=%d", ret);
    }
    return ret;
}

//=================================================================================================
static int smtc_is_irq_low(Self self)
{
    return !gpio_get_value(self->gpio);
}


//#################################################################################################
static ssize_t smtc_calibration_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int i, count=0, ph, shift;
    u32 reg_val = 0;
    u16 offset;
    Self self = dev_get_drvdata(dev);

    for (i=0; i<60; i++)
    {
        smtc_i2c_read(self, 0x8004,  &reg_val);
        if (reg_val & COMPENSATING_MASK){
            msleep(50);
        }else{
            break;
        }
    }

    if (i==30){
        count = sprintf(buf, "%s", "chip is still compensating after waiting for 3 seconds.");
        SMTC_LOG_WRN("%s", buf);
        return count;
    }else if (i!=0){
        SMTC_LOG_INF("chip completed the compensation after waiting for %d ms", i*100);
    }

    for(ph =0; ph < NUM_PHASES; ph++)
    {
        shift = ph*4;
        smtc_i2c_read(self, REG_PH0_OFFSET + shift*OFFSET_PH_REG_SHIFT,  &reg_val);
		offset = (u16)(reg_val & OFFSET_VAL_MASK);

        count += sprintf(buf+count, "PH%d=%d ", ph, offset);
    }
    count += sprintf(buf+count, "\n");
    SMTC_LOG_INF("%s", buf);
    return count;
}

//-------------------------------------------------------------------------------------------------
static ssize_t smtc_calibration_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    Self self = dev_get_drvdata(dev);
    SMTC_LOG_INF("Enter");
    smtc_calibrate(self);
    return count;
}

//=================================================================================================
static ssize_t smtc_reg_write_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    u32 reg_addr = 0, reg_val = 0;
    Self self = dev_get_drvdata(dev);

    if (sscanf(buf, "%x,%x", &reg_addr, &reg_val) != 2)
    {
        //users are always confused whether there is a blank character or not
        //handle both case here
        if (sscanf(buf, "%x, %x", &reg_addr, &reg_val) != 2){
            SMTC_LOG_ERR("Invalid command format. Example: ehco '0x4280,0xE' > reg_write");
            return -EINVAL;
        }
    }
    SMTC_LOG_INF("0x%X= 0x%X", reg_addr, reg_val);

    if (reg_addr == REG_PHEN)
    {
        mutex_lock(&self->phen_lock);
        self->variables.enabled_phases = reg_val;
        smtc_i2c_write(self, reg_addr, reg_val);
        mutex_unlock(&self->phen_lock);
    }
    else{
        smtc_i2c_write(self, reg_addr, reg_val);
    }

    return count;
}
//-------------------------------------------------------------------------------------------------
static ssize_t smtc_reg_write_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf,
        "\nUsage: echo reg_addr,reg_val > reg_write\n"
        "Example: echo 0x4004,0x74 > reg_write\n");
}

//=================================================================================================
static ssize_t smtc_reg_read_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    u32 reg_val=0;
    u16 reg_addr = 0;
    Self self = dev_get_drvdata(dev);

    if (sscanf(buf, "%x", &reg_addr) != 1)
    {
        SMTC_LOG_ERR(
            "echo reg_addr > reg_read; cat reg_read\n"
            "Example: echo 0x4000 > reg_read; cat reg_read\n");
        return -EINVAL;
    }

    self->variables.reading_reg = reg_addr;
    ret = smtc_i2c_read(self, reg_addr, &reg_val);

    if (ret < 0){
        SMTC_LOG_ERR("Failed to read register 0x%X", reg_addr);
    }else{
        SMTC_LOG_INF("0x%X= 0x%X", reg_addr, reg_val);
    }

    return count;
}
//-------------------------------------------------------------------------------------------------
static ssize_t smtc_reg_read_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int count=0, ret;
    u32 reg_val = 0, reading_reg;
    Self self = NULL;
    self = dev_get_drvdata(dev);
    reading_reg = self->variables.reading_reg;

    if (reading_reg == 0xFFFF){
        count = sprintf(buf,
            "\nUsage: echo reg_addr > reg_read; cat reg_read\n"
            "Example: echo 0x4000 > reg_read; cat reg_read\n");
    }else{
        ret = smtc_i2c_read(self, reading_reg, &reg_val);

        if (ret < 0){
            count = sprintf(buf, "Failed to read reg=0x%X\n", reading_reg);
        }else{
            count = sprintf(buf, "0x%X= 0x%X\n", reading_reg, reg_val);
        }
    }

    return count;
}

//=================================================================================================
static ssize_t smtc_raw_data_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ph, bytes=0;
	u32 reg_val;
	u16 offset;
    s32 useful, average, diff;
    Self self = dev_get_drvdata(dev);

    for (ph=0; ph<NUM_PHASES; ph++)
    {
        int shift = ph*4;
        smtc_i2c_read(self, REG_PH0_USEFUL  + shift,    &reg_val);
        useful = (s32)reg_val>>10;

        smtc_i2c_read(self, REG_PH0_AVERAGE + shift,    &reg_val);
        average = (s32)reg_val>>10;

        smtc_i2c_read(self, REG_PH0_DIFF    + shift,    &reg_val);
        diff = (s32)reg_val>>10;

        smtc_i2c_read(self, REG_PH0_OFFSET + shift*OFFSET_PH_REG_SHIFT,  &reg_val);
        offset = (u16)(reg_val & OFFSET_VAL_MASK);

        bytes += sprintf(buf+bytes, "ph= %d useful= %d average= %d diff= %d offset= %d\n",
            ph, useful, average, diff, offset);

    }
    smtc_log_raw_data(self);

    return bytes;
}

static ssize_t smtc_batch_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    int flag = 0;
    int handle;
    int report_rate;
    s64 batch_timeout;

    SMTC_LOG_ERR("buf=%s\n", buf);
    if (sscanf(buf, "%d %d %d %lld\n",
                   &handle, &flag,
                   &report_rate,
                   &batch_timeout) != 4)
                return -EINVAL;
    SMTC_LOG_ERR("handle = %d, rate = %d, batch_latency = %lld\n",
                 handle,
                 report_rate, batch_timeout);

        return count;
}

static void flush_touch_status(int handle)
{
    int ph;
    phase_p phase;
    struct input_dev *input;
    static bool flush_flag = false;
    flush_flag = !flush_flag;

    for (ph = 0; ph < NUM_PHASES; ph++)
    {
        phase = &smtc_phase_table[ph];
        if (phase->usage != MAIN)
            continue;

        if(ph != (handle - 161))
	    continue;

        input = phase->input;

        SMTC_LOG_DBG("%s reports RELEASED in flush", phase->name);
	if (!flush_flag) {
            input_report_abs(input, ABS_DISTANCE, 3);
            input_sync(input);
	    break;
        } else {
            input_report_abs(input, ABS_DISTANCE, 4);
            input_sync(input);
	    break;
        }
    }
}

static ssize_t smtc_flush_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    int handle;

    SMTC_LOG_ERR("buf=%s\n", buf);
    if (sscanf(buf, "%d\n", &handle) != 1)
                return -EINVAL;
    flush_touch_status(handle);
    SMTC_LOG_ERR("handle = %d\n", handle);

        return count;
}

//=================================================================================================
//      Enable/Disable Channels/Phases
//=================================================================================================
static ssize_t smtc_enable_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    u32 phase=0, enable=0, enabled=0;
	u32 reg_val=0;
    Self self = dev_get_drvdata(dev);
    int handle = -1;

    SMTC_LOG_ERR("buf=%s", buf);
    /* Enable/Disable ONE phase at a time
    command: echo phase,enable > enable,
    example:
        echo 2,1 > enable #enable phase 2
        echo 3,0 > enable #disable phase 3
    */
    if ((sscanf(buf, "%d %d\n", &handle, &enable) == 2) || (sscanf(buf, "%d,%d", &phase, &enable) == 2))
    {
	if(handle >= 161)
	    phase = (handle - 161);

        SMTC_LOG_INF("%s phase=%d", enable ? "Enable" : "Disable", phase);
        if (phase >= NUM_PHASES){
            SMTC_LOG_ERR("phase=%d is not in the range [0, %d]", phase, NUM_PHASES-1);
            return -EINVAL;
        }

        smtc_i2c_read(self, REG_PHEN, &reg_val);
        enabled = reg_val & 1<<phase;
        if (enable)
        {
            if (enabled)
            {
                SMTC_LOG_WRN("phase=%d has alredy been enabled", phase, NUM_PHASES);
                SMTC_LOG_ERR("count=%d", count);
                return count;
            }

            reg_val |= 1<<phase;
        }
        else
        {
            if (!enabled)
            {
                SMTC_LOG_WRN("phase=%d has alredy been disabled", phase, NUM_PHASES);
                SMTC_LOG_ERR("count=%d", count);
                return count;
            }

            reg_val &= ~(1<<phase);
        }
    }
    /* Enable/Disable MULTIPLE phases at a time
    command: echo phases > enable,
    example:
        echo 0x1 > enable #Enable phase(0) and with all other phases disbled. 0x1 = 0b0001
        echo 0x5 > enable #Enable phase(0,2) and with all other phases disbled. 0x5 = 0b0101
    */
    else
    {
        if (sscanf(buf, "0x%x", &phase) != 1){
            SMTC_LOG_ERR("Invalid format, cat enable for the usage");
            return -EINVAL;
        }

        if (phase & ~((1<<NUM_PHASES)-1))
        {
            SMTC_LOG_ERR("Invalid phase=0x%X, max phases supported=%d", phase, NUM_PHASES);
            return -EINVAL;
        }

        SMTC_LOG_INF("phase= 0x%X", phase);
        reg_val &= ~0xFF;
	    reg_val |= phase;
    }

    smtc_enable_phase(self, reg_val);
    smtc_calibrate(self);
    return count;
}

//-------------------------------------------------------------------------------------------------
static ssize_t smtc_enable_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    u32 reg_val;
    int ph=0, ret=0, bytes=0;
    Self self = dev_get_drvdata(dev);

    ret = smtc_i2c_read(self, REG_PHEN, &reg_val);
    if (ret < 0){
        bytes = sprintf(buf, "Failed to read reg=0x%X\n", REG_PHEN);
        SMTC_LOG_ERR("%s", buf);
        return bytes;
    }

    bytes += sprintf(buf+bytes, "-----------------------------------------------------------\n");
    bytes += sprintf(buf+bytes, "Current enabled phases: reg_val=0x%X, phases=", reg_val&0xFF);
    if (reg_val & 0xFF)
    {
        for (ph=0; ph<NUM_PHASES; ph++)
        {
            if (reg_val & 1<<ph){
                bytes += sprintf(buf+bytes, "%d ", ph);
            }
        }
    }else{
        bytes += sprintf(buf+bytes, "none");
    }
    bytes += sprintf(buf+bytes, "\n");

    bytes += sprintf(buf+bytes, "-----------------------------------------------------------\n");
    bytes += sprintf(buf+bytes,
        "Usage:\n"
        "1. Enable/Disable ONE phase at a time\n"
        "   command: echo phase,enable > enable\n"
        "   example:\n"
        "       echo 2,1 > enable #enable phase 2\n"
        "       echo 3,0 > enable #disable phase 3\n\n"

        "2. Enable/Disable MULTIPLE phases at a time\n"
        "   command: echo phases > enable\n"
        "   example:\n"
        "       echo 0x1 > enable #Enable phase(0) and with all other phases disbled. 0x1 = 0b0001\n"
        "       echo 0x5 > enable #Enable phase(0,2) and with all other phases disbled. 0x5 = 0b0101\n\n"
    );
    return bytes;
}

//=================================================================================================
// Enable/Disable the specified phase from the dedicated NODE
//=================================================================================================
static ssize_t smtc_enable_ph0_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    u32 enable=0;
	u32 reg_val;
    Self self = dev_get_drvdata(dev);

    SMTC_LOG_INF("count=%d buf=%d", count, buf[0]);

    if (sscanf(buf, "%d", &enable) != 1)
    {
        SMTC_LOG_ERR("Invalid command format. Usage: ehco '0' | '1' > enable_ch0");
        return -EINVAL;
    }
    smtc_i2c_read(self, REG_PHEN, &reg_val);

    if (enable)
    {
        if (reg_val & 1<<0){
            SMTC_LOG_WRN("CH0 has already been enabled");
        }else{
            SMTC_LOG_INF("Enable CH0");
            reg_val |= 1<<0;
            smtc_enable_phase(self, reg_val);
            smtc_calibrate(self);
        }
    }
    else
    {
        if ((reg_val & 1<<0)==0){
            SMTC_LOG_WRN("CH0 has already been disabled");
        }else{
            SMTC_LOG_INF("Disable CH0");
            reg_val &= ~(1<<0);
            smtc_enable_phase(self, reg_val);
        }
    }

    return count;
}

static ssize_t smtc_enable_ph0_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int bytes=0;

    bytes = sprintf(buf, "%s",
            "Usage: echo arg > enable_ch0\n"
            "Disable(arg=0) or Enable(arg=1) Phase 0\n\n"
    );
    return bytes;
}

//=================================================================================================
static ssize_t smtc_enable_ph1_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    u32 enable=0;
	u32 reg_val;
    Self self = dev_get_drvdata(dev);

    if (sscanf(buf, "%d", &enable) != 1)
    {
        SMTC_LOG_ERR("Invalid command format. Usage: ehco '0' | '1' > enable_ph1");
        return -EINVAL;
    }
    smtc_i2c_read(self, REG_PHEN, &reg_val);

    if (enable)
    {
        if (reg_val & 1<<1){
            SMTC_LOG_WRN("Phase 1 has already been enabled");
        }else{
            SMTC_LOG_INF("Enable phase 1");
            reg_val |= 1<<1;
            smtc_enable_phase(self, reg_val);
            smtc_calibrate(self);
        }
    }
    else
    {
        if ((reg_val & 1<<1)==0){
            SMTC_LOG_WRN("phase 1 has already been disabled");
        }else{
            SMTC_LOG_INF("Disable phase 1");
            reg_val &= ~(1<<1);
            smtc_enable_phase(self, reg_val);
        }
    }

    return count;
}
static ssize_t smtc_enable_ph1_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int bytes=0;

    bytes = sprintf(buf, "%s",
            "Usage: echo arg > enable_ph1\n"
            "Disable(arg=0) or Enable(arg=1) phase 1\n\n"
    );
    return bytes;
}

//=================================================================================================
static void smtc_test_func(Self self)
{
    SMTC_LOG_INF("You can modify this function freely to test any of driver functionality.");
}

static ssize_t smtc_debug_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    int cmd, arg;
    Self self = dev_get_drvdata(dev);

    if (sscanf(buf, "%c,%c", &cmd, &arg) != 2)
    {
        if (sscanf(buf, "%c, %c", &cmd, &arg) != 2){
            SMTC_LOG_ERR("Invalid command format. Use 'cat debug' to show the usages.");
            return -EINVAL;
        }
    }

    switch (cmd){
    case 0:
        smtc_test_func(self);
        break;
    case 1:
        self->dbg_flag.log_raw_data = arg;
        SMTC_LOG_INF("%s log raw data when proximity status is changed", arg ? "Enable" : "Disable");
        break;
    case 2:
        self->dbg_flag.log_dbg_data = arg;
        SMTC_LOG_INF("%s log debug data", arg ? "Enable" : "Disable");
        break;

    case 3:
        self->dbg_flag.log_hex_data = arg;
        SMTC_LOG_INF("%s log hex data", arg ? "Enable" : "Disable");
        break;

   default:
        SMTC_LOG_ERR("Invalid command=%d. Use 'cat debug' to show the usages.", cmd);
    }

    return count;
}

//-------------------------------------------------------------------------------------------------
static ssize_t smtc_debug_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s",
        "Usage: echo 'cmd,arg' > debug\n"
        "cmd:\n"
        "1: Turn on(arg=1) | off(arg=0), default=0\n"
        "   Log raw data when proximity status is changed"
        "2: Turn on(arg=1) | off(arg=0), default=1\n"
        "   Log debug data, such as PROX RAW, UseFilter DeltaVar, etc."
        "3: Turn on(arg=1) | off(arg=0), default=0\n"
        "   Log some of registers value(Useful, average, etc.) in hex format.\n"
    );
}

//=================================================================================================
//read and print the value of registers in the dts
static ssize_t smtc_dump_reg_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int i, bytes=0;
    u32 addr, val;
    Self self = dev_get_drvdata(dev);

    for (i=0; i<self->num_regs; i++)
    {
        addr = self->regs_addr_val[i].addr;

        if (smtc_i2c_read(self, addr, &val) > 0){
            bytes += sprintf(buf+bytes, "0x%X= 0x%08X\n", addr, val);
        }else{
            bytes += sprintf(buf+bytes, "0x%X= FAILED\n", addr);
        }
    }
    return bytes;
}

static ssize_t smtc_irq_gpio_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    Self self = dev_get_drvdata(dev);
    int value = gpio_get_value(self->gpio);
    return sprintf(buf, "GPIO-%d=%d %s\n", self->gpio, value, value ? "hi" : "lo");
}

//=================================================================================================
static DEVICE_ATTR(cali,           0664, smtc_calibration_show,    smtc_calibration_store);
static DEVICE_ATTR(reg_write,      0664, smtc_reg_write_show,      smtc_reg_write_store);
static DEVICE_ATTR(reg_read,       0664, smtc_reg_read_show,       smtc_reg_read_store);
static DEVICE_ATTR(raw_data,       0444, smtc_raw_data_show,       NULL);
static DEVICE_ATTR(enable,         0664, smtc_enable_show,         smtc_enable_store);
static DEVICE_ATTR(enable_ph0,     0664, smtc_enable_ph0_show,     smtc_enable_ph0_store);
static DEVICE_ATTR(enable_ph1,     0664, smtc_enable_ph1_show,     smtc_enable_ph1_store);
static DEVICE_ATTR(debug,          0664, smtc_debug_show,          smtc_debug_store);
static DEVICE_ATTR(dump_reg,       0444, smtc_dump_reg_show,       NULL);
static DEVICE_ATTR(irq_gpio,       0444, smtc_irq_gpio_show,       NULL);
static DEVICE_ATTR(batch,          0664, NULL,                     smtc_batch_store);
static DEVICE_ATTR(flush,          0664, NULL,                     smtc_flush_store);

static struct attribute *smtc_sysfs_nodes_attr[] =
{
    &dev_attr_cali.attr,
    &dev_attr_reg_write.attr,
    &dev_attr_reg_read.attr,
    &dev_attr_raw_data.attr,
    &dev_attr_enable.attr,
    &dev_attr_enable_ph0.attr,
    &dev_attr_enable_ph1.attr,
    &dev_attr_debug.attr,
    &dev_attr_dump_reg.attr,
    &dev_attr_irq_gpio.attr,
    &dev_attr_batch.attr,
    &dev_attr_flush.attr,

    NULL,
};
static struct attribute_group smtc_sysfs_nodes =
{
    .attrs = smtc_sysfs_nodes_attr,
};

//#################################################################################################
static void smtc_get_ph_prox_state(u32 prox_reg_val, PROX_STATUS ph_state[NUM_PHASES])
{
    int ph;
    phase_p phase;

	for (ph=0; ph<NUM_PHASES; ph++)
	{
        phase = &smtc_phase_table[ph];
        //The prox4_mask of sx933x is always 0
        if (prox_reg_val & phase->prox4_mask){
            ph_state[ph] = PROX4;
        }
        else if (prox_reg_val & phase->prox3_mask){
            ph_state[ph] = PROX3;
        }
        else if (prox_reg_val & phase->prox2_mask){
            ph_state[ph] = PROX2;
        }
        else if (prox_reg_val & phase->prox1_mask){
            ph_state[ph] = PROX1;
        }
        else{
            ph_state[ph] = RELEASED;
        }
	}
}

static void smtc_init_variables(Self self)
{
    self->dbg_flag.log_raw_data = 0;
    self->dbg_flag.log_dbg_data = 1;
    self->dbg_flag.log_hex_data = 0;
    self->variables.reading_reg = REG_WHO_AMI;

    self->variables.irq_mask = 0x70;
    self->variables.enabled_phases = 0x0;
}

//#################################################################################################
static int smtc_parse_dts(Self self)
{
    int ph;
    struct device_node *of_node = self->client->dev.of_node;
    enum of_gpio_flags flags;

    if (!of_node)
    {
        SMTC_LOG_ERR("of_node == NULL");
        return -EINVAL;
    }

    self->gpio = of_get_named_gpio_flags(of_node, "semtech,nirq-gpio", 0, &flags);
    SMTC_LOG_DBG("irq_gpio= %d", self->gpio);

    if (self->gpio < 0)
    {
        SMTC_LOG_ERR("Failed to get irq_gpio.");
        return -EINVAL;
    }

    //.............................................................................................
    //load main and reference phase setup
    if (of_property_read_u32(of_node, "semtech,main-phases", &self->main_phases) )
	{
		SMTC_LOG_ERR("Failed to get main-phases");
        return -EINVAL;
	}
    SMTC_LOG_DBG("main-phases= 0x%X", self->main_phases);


	if (of_property_read_u32(of_node, "semtech,ref-phases", &self->ref_phases) )
	{
		SMTC_LOG_ERR("Failed to get ref-phases");
        return -EINVAL;
	}
    SMTC_LOG_DBG("ref-phases= 0x%X", self->ref_phases);

    self->ref_phase_a = -1;
	self->ref_phase_b = -1;
    self->ref_phase_c = -1;

    for (ph=0; ph<NUM_PHASES; ph++)
    {
        if (1<<ph & self->ref_phases){
            if (self->ref_phase_a==-1){
                self->ref_phase_a = ph;
            }
            else if (self->ref_phase_b==-1){
                self->ref_phase_b = ph;
            }
            else if (self->ref_phase_c==-1){
                self->ref_phase_c = ph;
            }
            else{
                SMTC_LOG_ERR("Max 3 reference phases are supported. ref_phases passed from dts= 0x%X",
                    self->ref_phases);
                return -EINVAL;
            }
        }
    }
	SMTC_LOG_INF("ref_phase_a= %d ref_phase_b= %d ref_phase_c= %d",
		self->ref_phase_a, self->ref_phase_b, self->ref_phase_c);

    //.............................................................................................
    //load register settings
    self->num_regs = of_property_count_u32_elems(of_node, "semtech,reg-init");
    SMTC_LOG_DBG("number of registers= %d", self->num_regs);

    if (unlikely(self->num_regs <= 0 || self->num_regs %2 != 0))
    {
        SMTC_LOG_ERR("Invalid reg_num= %d, please check dtsi config", self->num_regs);
        return -EINVAL;
    }
    else
    {
        self->num_regs /= 2;
        self->regs_addr_val = devm_kzalloc(&self->client->dev, sizeof(reg_addr_val_t)*self->num_regs, GFP_KERNEL);
        if (unlikely(!self->regs_addr_val))
        {
            SMTC_LOG_ERR("Failed to alloc memory, num_reg= %d", self->num_regs);
            return -ENOMEM;
        }

        if (of_property_read_u32_array(of_node, "semtech,reg-init",
                                (u32*)self->regs_addr_val,
                                sizeof(reg_addr_val_t) * self->num_regs/sizeof(u32)))
       {
            SMTC_LOG_ERR("Failed to load registers from the dts");
            return -EINVAL;
        }
    }

    return 0;
}

//=================================================================================================
static int smtc_check_hardware(Self self)
{
    int ret;
    u32 chip_id;

    ret = smtc_i2c_read(self, REG_WHO_AMI, &chip_id);
    if(ret < 0)
    {
        SMTC_LOG_ERR("Failed to read chip id. ret= %d.", ret);
        return ret;
    }

    chip_id = smtc_get_chip_id(chip_id);
    if (chip_id != SMTC_CHIP_ID){
        SMTC_LOG_ERR("chip id 0x%X != 0x%X expected id.", chip_id, SMTC_CHIP_ID);
        return -ENODEV;
    }

    SMTC_LOG_INF("chip_id= 0x%X", chip_id);
    return 0;
}

//=================================================================================================
static int smtc_init_irq_gpio(Self self)
{
	int ret = 0;
    SMTC_LOG_DBG("");

	if (!gpio_is_valid(self->gpio))
	{
        SMTC_LOG_ERR("Invalid irq_gpio= %d. Please check DTS stup.", self->gpio);
		return -EINVAL;
	}

    ret = gpio_request(self->gpio, "smtc_nirq_gpio");
    if (ret < 0)
    {
		SMTC_LOG_ERR("Failed to request GPIO= %d", self->gpio);
        return ret;
    }

    ret = gpio_direction_input(self->gpio);
    if (ret < 0)
    {
        SMTC_LOG_ERR("Failed to set GPIO= %d as input", self->gpio);
        gpio_free(self->gpio);
        return ret;
    }

    self->irq_id = self->client->irq = gpio_to_irq(self->gpio);
    SMTC_LOG_DBG("Get irq= %d from gpio= %d", self->irq_id, self->gpio);

#if WAKE_BY_IRQ
    SMTC_LOG_INF("Enable wake up function on gpio=%d irq=%d", self->gpio, self->irq_id);
    device_init_wakeup(&self->client->dev, true);
    ret = dev_pm_set_wake_irq(&self->client->dev, self->irq_id);
    if (ret < 0){
        SMTC_LOG_INF("Failed to nable wake up function on gpio=%d irq=%d", self->gpio, self->irq_id);
    }
#endif

    return ret;
}

//=================================================================================================
static int smtc_init_registers(Self self)
{
    int i = 0, ret=0;
    u16 reg_addr;
    u32 reg_val;

    SMTC_LOG_INF("Going to init registers passed from the DTS");
    for (i=0; i < self->num_regs; i++)
    {
        reg_addr = self->regs_addr_val[i].addr;
        reg_val  = self->regs_addr_val[i].val;
        SMTC_LOG_DBG("0x%X= 0x%X", reg_addr, reg_val);

        if (likely(reg_addr != REG_PHEN)){
            ret = smtc_i2c_write(self, reg_addr, reg_val);
        }else{
            mutex_lock(&self->phen_lock);
            self->variables.enabled_phases = reg_val;
            ret = smtc_i2c_write(self, reg_addr, reg_val);
            mutex_unlock(&self->phen_lock);
        }

        if (ret <= 0){
            SMTC_LOG_ERR("Failed to write reg=0x%x value=0x%X", reg_addr, reg_val);
            return ret;
        }
    }

    /*Chip must be activated once after reset.
    You can disable all phases in the register settings if your product need to
    put the SAR sensor into the sleep mode during startup
    */
    ret = smtc_send_cmd(self, CMD_ACTIVATE);
    if (ret){return ret;}

    return 0;
}

//=================================================================================================
static int smtc_wait_reset_done(Self self)
{
    int ret = 0, i;
    u32 irq_src=0;

    for (i=0; i<5; i++)
    {
        msleep(2);
        ret = smtc_i2c_read(self, REG_IRQ_SRC, &irq_src);
        if (ret > 0 && irq_src != 0)
            return 0;
    }

    SMTC_LOG_ERR("Failed to read reset IRQ");
    return -EIO;
}

static int smtc_reset_and_init_chip(Self self)
{
    int ret = 0;
    SMTC_LOG_INF("Enter");

    ret = smtc_i2c_write(self, REG_RESET, 0xDE);
    if(ret<0){return ret;}

    ret = smtc_wait_reset_done(self);
    if(ret){return ret;}

    ret = smtc_init_registers(self);
    if(ret){return ret;}

    //make sure no interrupts are pending
    ret = smtc_read_and_clear_irq(self, NULL);
    return ret;
}

//=================================================================================================
#ifdef ENABLE_ESD_RECOVERY
static int smtc_recover_esd_reset(Self self)
{
    int ret = 0;
    u32 irq_mask=0;
    ret = smtc_i2c_read(self, REG_IRQ_MASK, &irq_mask);
    if (ret < 0){
        SMTC_LOG_ERR("Failed to read IRQ MASK");
        return ret;
    }
    SMTC_LOG_DBG("irq_mask=0x%X", irq_mask);
    if ((irq_mask & 1<<4) != 0){
        return 0;
    }
    SMTC_LOG_INF("Compensation IRQ mask is cleard, try to reset and re-init chip");
    ret = smtc_reset_and_init_chip(self);
    if (ret < 0){
        SMTC_LOG_ERR("Failed to recover the chip");
    }else{
        SMTC_LOG_ERR("PASSED to recover the chip");
    }
    return ret;
}
#endif

//#################################################################################################
static void smtc_log_dbg_data(Self self)
{
    u16 off;
	int ph, state;
	u32 reg_val;
    s32 avg, diff, main_use, main_raw, dlt_var;
    PROX_STATUS ph_prox_state[NUM_PHASES];

    int ref_ph_a, ref_ph_b, ref_ph_c;
	s32 ref_a_use=0, ref_b_use=0, ref_c_use=0;
    char ref_a_name[] = "na";
    char ref_b_name[] = "na";
    char ref_c_name[] = "na";

	ref_ph_a = self->ref_phase_a;
	ref_ph_b = self->ref_phase_b;
	ref_ph_c = self->ref_phase_c;

    smtc_i2c_read(self, REG_DBG_SEL, &reg_val);
    ph = reg_val>>3 & 0x7;

	smtc_i2c_read(self, REG_PROX_STATUS, &reg_val);
    SMTC_LOG_DBG("REG_PROX_STATUS=0x%X", reg_val);
	smtc_get_ph_prox_state(reg_val, ph_prox_state);
    state = ph_prox_state[ph];

	if(ref_ph_a != -1)
	{
		smtc_i2c_read(self, REG_PH0_USEFUL + ref_ph_a*4, &reg_val);
		ref_a_use = (s32)reg_val >> 10;
        sprintf(ref_a_name, "%d", ref_ph_a);
	}
	if(ref_ph_b != -1)
	{
		smtc_i2c_read(self, REG_PH0_USEFUL + ref_ph_b*4, &reg_val);
		ref_b_use = (s32)reg_val >> 10;
        sprintf(ref_b_name, "%d", ref_ph_b);
	}
	if(ref_ph_c != -1)
	{
		smtc_i2c_read(self, REG_PH0_USEFUL + ref_ph_c*4, &reg_val);
		ref_c_use = (s32)reg_val >> 10;
        sprintf(ref_c_name, "%d", ref_ph_c);
	}

	smtc_i2c_read(self, REG_RAW_DATA, &reg_val);
	main_raw = (s32)reg_val>>10;

	smtc_i2c_read(self, REG_DLT_VAR, &reg_val);
	dlt_var = (s32)reg_val>>4;

	smtc_i2c_read(self, REG_PH0_USEFUL + ph*4, &reg_val);
	main_use = (s32)reg_val>>10;

	smtc_i2c_read(self, REG_PH0_AVERAGE + ph*4, &reg_val);
	avg = (s32)reg_val>>10;

	smtc_i2c_read(self, REG_PH0_DIFF + ph*4, &reg_val);
	diff = (s32)reg_val>>10;

	smtc_i2c_read(self, REG_PH0_OFFSET + ph*4*OFFSET_PH_REG_SHIFT, &reg_val);
	off = (u16)(reg_val & OFFSET_VAL_MASK);

	pr_info(
	"SMTC_DBG PH=%d USE=%d RAW=%d PH%s_USE=%d PH%s_USE=%d PH%s_USE=%d STATE=%d AVG=%d DIFF=%d OFF=%d DLT=%d SMTC_END\n",
	ph, main_use, main_raw, ref_a_name, ref_a_use, ref_b_name, ref_b_use, ref_c_name, ref_c_use,
	state, avg, diff, off, dlt_var);
}

//=================================================================================================
static void smtc_log_raw_data(Self self)
{
    int ph, state;
    u16 offset;
    s32 useful, average, diff;
    u32 reg_val, dbg_ph;
    u32 use_hex, avg_hex, dif_hex, dlt_hex;
    PROX_STATUS ph_prox_state[NUM_PHASES];

    int ref_ph_a, ref_ph_b, ref_ph_c;
    s32 ref_a_use=0, ref_b_use=0, ref_c_use=0;
    char ref_a_name[] = "na";
    char ref_b_name[] = "na";
    char ref_c_name[] = "na";

    ref_ph_a = self->ref_phase_a;
    ref_ph_b = self->ref_phase_b;
    ref_ph_c = self->ref_phase_c;

    smtc_i2c_read(self, REG_PROX_STATUS, &reg_val);
    SMTC_LOG_DBG("prox_state= 0x%X", reg_val);
    smtc_get_ph_prox_state(reg_val, ph_prox_state);

    smtc_i2c_read(self, REG_DBG_SEL, &dbg_ph);
    dbg_ph = (dbg_ph >> 3) & 0x7;
    smtc_i2c_read(self, REG_DLT_VAR, &dlt_hex);

    if(ref_ph_a != -1)
    {
        smtc_i2c_read(self, REG_PH0_USEFUL + ref_ph_a*4, &reg_val);
        ref_a_use = (s32)reg_val >> 10;
        sprintf(ref_a_name, "%d", ref_ph_a);
    }
    if(ref_ph_b != -1)
    {
        smtc_i2c_read(self, REG_PH0_USEFUL + ref_ph_b*4, &reg_val);
        ref_b_use = (s32)reg_val >> 10;
        sprintf(ref_b_name, "%d", ref_ph_b);
    }
    if(ref_ph_c != -1)
    {
        smtc_i2c_read(self, REG_PH0_USEFUL + ref_ph_c*4, &reg_val);
        ref_c_use = (s32)reg_val >> 10;
        sprintf(ref_c_name, "%d", ref_ph_c);
    }

    for(ph =0; ph<NUM_PHASES; ph++)
    {
        int shift = ph*4;
        smtc_i2c_read(self, REG_PH0_USEFUL  + shift,    &use_hex);
        useful = (s32)use_hex>>10;
        smtc_i2c_read(self, REG_PH0_AVERAGE + shift,    &avg_hex);
        average = (s32)avg_hex>>10;
        smtc_i2c_read(self, REG_PH0_DIFF    + shift,    &dif_hex);
        diff = (s32)dif_hex>>10;
        smtc_i2c_read(self, REG_PH0_OFFSET + shift*OFFSET_PH_REG_SHIFT,  &reg_val);
        offset = (u16)(reg_val & OFFSET_VAL_MASK);
        state = ph_prox_state[ph];

        pr_info(
        "SMTC_DAT PH= %d DIFF= %d USE= %d PH%s_USE= %d PH%s_USE= %d PH%s_USE= %d STATE= %d OFF= %d AVG= %d SMTC_END\n",
        ph, diff, useful, ref_a_name, ref_a_use, ref_b_name, ref_b_use, ref_c_name, ref_c_use, state, offset, average);

        if (self->dbg_flag.log_hex_data){
            pr_info(
            "SMTC_HEX PH= %d USE= 0x%X AVG= 0x%X DIF= 0x%X PH%d_DLT= 0x%X SMTC_END\n",
            ph, use_hex, avg_hex, dif_hex, dbg_ph, dlt_hex);
        }
    }

    if (self->dbg_flag.log_dbg_data){
        smtc_log_dbg_data(self);
    }
}

//=================================================================================================
static void smtc_process_touch_status(Self self)
{
    int ph;
	u32 prox_state = 0;
    bool need_sync = false, status_changed=false;
    phase_p phase;
    struct input_dev *input;
    char msg_updated_status[128];
    char msg_prox_status[128];
    int msg_len = 0;

	smtc_i2c_read(self, REG_PROX_STATUS, &prox_state);
	SMTC_LOG_DBG("prox_state= 0x%X", prox_state);
    if (self->dbg_flag.log_raw_data){
        smtc_log_raw_data(self);
    }

    for (ph = 0; ph < NUM_PHASES; ph++)
    {
        phase = &smtc_phase_table[ph];
        if (phase->usage != MAIN)
            continue;

        input = phase->input;
        need_sync = false;

    	if (prox_state & phase->prox4_mask)
    	{
            if (phase->state == PROX4){
                SMTC_LOG_DBG("%s is PROX4 already", phase->name);
            }
            else{
                SMTC_LOG_DBG("%s reports PROX4", phase->name);
                msg_len += sprintf(msg_updated_status+msg_len, " PH%d=4", ph);
                phase->state = PROX4;
                input_report_abs(input, ABS_DISTANCE, (int)PROX4);
                need_sync = true;
            }
    	}
        else if (prox_state & phase->prox3_mask)
    	{
            if (phase->state == PROX3){
                SMTC_LOG_DBG("%s is PROX3 already", phase->name);
            }
            else{
                SMTC_LOG_DBG("%s reports PROX3", phase->name);
                msg_len += sprintf(msg_updated_status+msg_len, " PH%d=3", ph);
                phase->state = PROX3;
                input_report_abs(input, ABS_DISTANCE, (int)PROX3);
                need_sync = true;
            }
    	}
        else if (prox_state & phase->prox2_mask)
    	{
            if (phase->state == PROX2){
                SMTC_LOG_DBG("%s is PROX2 already", phase->name);
            }
            else{
                SMTC_LOG_DBG("%s reports PROX2", phase->name);
                msg_len += sprintf(msg_updated_status+msg_len, " PH%d=2", ph);
                phase->state = PROX2;
                input_report_abs(input, ABS_DISTANCE, (int)PROX2);
                need_sync = true;
            }
    	}
        else if (prox_state & phase->prox1_mask)
    	{
            if (phase->state == PROX1){
                SMTC_LOG_DBG("%s is PROX1 already", phase->name);
            }
            else{
                SMTC_LOG_DBG("%s reports PROX1", phase->name);
                msg_len += sprintf(msg_updated_status+msg_len, " PH%d=1", ph);
                phase->state = PROX1;
                input_report_abs(input, ABS_DISTANCE, (int)PROX1);
                need_sync = true;
            }
    	}
        else
        {
            if (phase->state == RELEASED){
                SMTC_LOG_DBG("%s is RELEASED already", phase->name);
            }
            else
            {
                SMTC_LOG_DBG("%s reports RELEASED", phase->name);
                msg_len += sprintf(msg_updated_status+msg_len, " PH%d=0", ph);
                phase->state = RELEASED;
                input_report_abs(input, ABS_DISTANCE, (int)RELEASED);
                need_sync = true;
            }
        }

        if (need_sync){
            input_sync(input);
            status_changed = true;
        }
    }

    if (status_changed)
    {
        msg_len = 0;
        for (ph = 0; ph < NUM_PHASES; ph++)
        {
            phase = &smtc_phase_table[ph];
            if (phase->usage == MAIN){
                msg_len += sprintf(msg_prox_status+msg_len, " PH%d=%d", ph, phase->state);
            }
        }

        SMTC_LOG_INF("Prox status= 0x%08X %s Updated status:%s", prox_state, msg_prox_status, msg_updated_status);

    }else{
        SMTC_LOG_INF("No proximity state is updated");
    }
}

//#################################################################################################
static void smtc_update_status(Self self)
{
    int ret=0, irq_bit=0, irq_src=0;

    ret = smtc_read_and_clear_irq(self, &irq_src);
    if (ret){
        SMTC_LOG_ERR("Failed to read irq source. ret=%d.", ret);
        goto SUB_OUT;
    }
    SMTC_LOG_INF("irq_src= 0x%X", irq_src);

    for (irq_bit=0; irq_bit<NUM_IRQ_BITS; irq_bit++)
    {
        if (irq_src >> irq_bit & 0x1)
        {
            if(self->irq_handler[irq_bit])
            {
                //call to smtc_process_touch_status() or smtc_log_raw_data()
                self->irq_handler[irq_bit](self);
            }else{
                SMTC_LOG_ERR("No handler to IRQ bit= %d", irq_bit);
            }
        }
    }

    if (irq_src == 0)
    {
        SMTC_LOG_INF("Force to update touch status");
        smtc_process_touch_status(self);
    }

SUB_OUT:
#if defined(ENABLE_STAY_AWAKE) && !WAKE_TMOUT
    SMTC_LOG_DBG("Release wake lock");
    __pm_relax(self->wake_lock);
#endif
    SMTC_LOG_DBG("Exit");
    return;
}

#ifndef USE_THREAD_IRQ
static void smtc_worker_func(struct work_struct *work)
{
    Self self = container_of(work, self_t, worker.work);
    SMTC_LOG_DBG("Enter");
    smtc_update_status(self);
    SMTC_LOG_DBG("Exit");
}
#endif

//=================================================================================================
//call flow: smtc_irq_handler--> smtc_worker_func--> smtc_process_touch_status
static irqreturn_t smtc_irq_isr(int irq, void *pvoid)
{
    Self self = (Self)pvoid;

#ifdef USE_THREAD_IRQ
    SMTC_LOG_DBG("IRQ= %d is received", irq);
#else
    SMTC_LOG_INF("IRQ= %d is received", irq);
#endif

    if (!smtc_is_irq_low(self))
    {
        SMTC_LOG_ERR("GPIO=%d must be low when an IRQ is received.", self->gpio);
        return IRQ_HANDLED;
    }

#ifdef ENABLE_STAY_AWAKE
#if WAKE_TMOUT
    SMTC_LOG_DBG("Stay awake for %d seconds", WAKE_TMOUT);
    __pm_wakeup_event(self->wake_lock, WAKE_TMOUT);
#else
    SMTC_LOG_DBG("Stay awake");
    __pm_stay_awake(self->wake_lock);
#endif
#endif

#ifdef USE_THREAD_IRQ
    SMTC_LOG_DBG("Update status with thread IRQ");
    smtc_update_status(self);
#else
    SMTC_LOG_DBG("Update status with worker");
    cancel_delayed_work(&self->worker);
    schedule_delayed_work(&self->worker, 0);
#endif

    return IRQ_HANDLED;
}

#ifdef CONFIG_CAPSENSE_USB_CAL
static void ps_notify_callback_work(struct work_struct *work)
{
	struct self_s *self = container_of(work, struct self_s, ps_notify_work);
	smtc_calibrate(self);
}

static int ps_get_state(struct power_supply *psy, bool *present)
{
	union power_supply_propval pval = {0};
	int retval;

#ifdef CONFIG_USE_POWER_SUPPLY_ONLINE
	retval = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE,
			&pval);
#else
	retval = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT,
			&pval);
#endif

	if (retval) {
		SMTC_LOG_ERR("%s psy get property failed\n", psy->desc->name);
		return retval;
	}
	*present = (pval.intval) ? true : false;

#ifdef CONFIG_USE_POWER_SUPPLY_ONLINE
	SMTC_LOG_ERR("%s is %s\n", psy->desc->name,
			(*present) ? "online" : "not online");
#else
	SMTC_LOG_ERR("%s is %s\n", psy->desc->name,
			(*present) ? "present" : "not present");
#endif

	return 0;
}

static int ps_notify_callback(struct notifier_block *self,
		unsigned long event, void *p)
{
	struct self_s *data =
		container_of(self, struct self_s, ps_notif);
	struct power_supply *psy = p;
	bool present;
	int retval;

//#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)
	if (event == PSY_EVENT_PROP_CHANGED
//#else
//	if ((event == PSY_EVENT_PROP_ADDED || event == PSY_EVENT_PROP_CHANGED)
//#endif
			&& psy && psy->desc->get_property && psy->desc->name &&
			!strncmp(psy->desc->name, USB_POWER_SUPPLY_NAME, sizeof(USB_POWER_SUPPLY_NAME)) && data) {
		SMTC_LOG_ERR("ps notification: event = %lu\n", event);
		retval = ps_get_state(psy, &present);
		if (retval) {
			return retval;
		}

		if (event == PSY_EVENT_PROP_CHANGED) {
			if (data->ps_is_present == present) {
				SMTC_LOG_ERR("ps present state not change\n");
				return 0;
			}
		}
		data->ps_is_present = present;
		schedule_work(&data->ps_notify_work);
	}

	return 0;
}
#endif
#ifdef HEADSET_CALI
static void hs_notify_callback_work(struct work_struct *work)
{
    struct self_s *self = container_of(work, struct self_s, hs_notify_work);
    smtc_calibrate(self);
}

static int hs_notify_callback(struct notifier_block *self,
                unsigned long event, void *p)
{
        struct self_s *data = container_of(self, struct self_s, hs_notif);

        if (data->hs_is_plug == event) {
                //AWLOGE(p_sar->dev, "hs plug state not change");
                return 0;
        }
        data->hs_is_plug = event;
        schedule_work(&data->hs_notify_work);

        return 0;
}
#endif
//#################################################################################################
static int smtc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0, i;
    Self self = NULL;
    struct input_dev *input = NULL;
    struct i2c_adapter *adapter = NULL;
#ifdef CONFIG_CAPSENSE_USB_CAL
	struct power_supply *psy = NULL;
#endif
    SMTC_LOG_INF("Start to initialize Semtech SAR sensor %s driver.", SMTC_CHIP_NAME);

    adapter = to_i2c_adapter(client->dev.parent);
    ret = i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_WORD_DATA);
    if (!ret)
    {
        SMTC_LOG_ERR("Failed to check i2c functionality.");
        return ret;
    }

    //kernel will automatically free the memory allocated by the devm_kzalloc
    //when probe is failed and driver is removed.
    self = devm_kzalloc(&client->dev, sizeof(self_t), GFP_KERNEL);
    if (!self)
    {
        SMTC_LOG_ERR("Failed to create self. memory size=%d bytes.", sizeof(self_t));
        return -ENOMEM;
    }

    smtc_init_variables(self);
    client->dev.platform_data = self;
    i2c_set_clientdata(client, self);
    self->client = client;

    ret = smtc_parse_dts(self);
    if (ret) {return ret;}

    ret = smtc_check_hardware(self);
    if (ret) {return ret;}

    ret = smtc_init_irq_gpio(self);
    if (ret) {return ret;}

    ret = smtc_reset_and_init_chip(self);
    if (ret) {goto FREE_GPIO;}

    //refer to register 0x4000
    self->irq_handler[0] = 0; /* UNUSED */
    self->irq_handler[1] = 0; /* UNUSED */
    self->irq_handler[2] = 0; /* UNUSED */
    self->irq_handler[3] = smtc_log_raw_data;       //CONVEDONEIRQ
    self->irq_handler[4] = smtc_process_touch_status; //COMPDONEIRQ
    self->irq_handler[5] = smtc_process_touch_status; //FARANYIRQ
    self->irq_handler[6] = smtc_process_touch_status; //CLOSEANYIRQ
    self->irq_handler[7] = 0; /* RESET_STAT */
#ifdef SMTC_SX933X
    self->irq_handler[2] = smtc_process_touch_status; //PROX2(table) and PROX3(body) irq
#endif

    ret = sysfs_create_group(&client->dev.kobj, &smtc_sysfs_nodes);
    if (ret)
    {
        SMTC_LOG_ERR("Failed to create sysfs node.");
        goto FREE_GPIO;
    }

    //.............................................................................................
    //Create input nodes for each main phase
    self->phases = smtc_phase_table;
    for (i=0; i<NUM_PHASES; i++)
    {
        phase_p phase = &smtc_phase_table[i];
        SMTC_LOG_DBG("name=%s input=0x%p enabled=%d usage=%d state=%d",
            phase->name, phase->input, phase->enabled, phase->usage, phase->usage);

        //main phases were specified in the dts
        if ((self->main_phases & 1<<i) == 0)
        {
            if (self->ref_phases & 1<<i){
                phase->usage = REF;
            }
            continue;
        }

        input = input_allocate_device();
        if (!input)
        {
            SMTC_LOG_ERR("Failed to create input device %s.", phase->name);
            ret = -ENOMEM;
            goto FREE_INPUTS;
        }

        input->name = phase->name;
        input->id.bustype = BUS_I2C;
        input->dev.parent = &client->dev;
	    input_set_abs_params(input, ABS_DISTANCE, 0, 4, 0, 0);
        ret = input_register_device(input);
        if (ret)
        {
            SMTC_LOG_ERR("Failed to register input device %s. ret= %d", phase->name, ret);
            goto FREE_INPUTS;
        }

        phase->input = input;
        phase->enabled = false;
        phase->usage = MAIN;
        phase->state = RELEASED;
    }

    //.............................................................................................
 #ifdef USE_THREAD_IRQ
    SMTC_LOG_INF("Use thread IRQ");
    ret = request_threaded_irq(self->irq_id, NULL, smtc_irq_isr,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			client->dev.driver->name, self);

    if (ret){
        SMTC_LOG_ERR("Failed to request irq= %d.", self->irq_id);
        goto FREE_INPUTS;
    }
 #else
    SMTC_LOG_INF("Use hard IRQ");

    spin_lock_init(&self->lock);
    mutex_init(&self->reg_wr_lock);
    INIT_DELAYED_WORK(&self->worker, smtc_worker_func);

    ret = request_irq(self->irq_id, smtc_irq_isr, IRQF_TRIGGER_FALLING,
                      client->dev.driver->name, self);
    if (ret)
    {
        SMTC_LOG_ERR("Failed to request irq= %d.", self->irq_id);
        goto FREE_INPUTS;
    }
 #endif

    SMTC_LOG_INF("registered irq= %d.", self->irq_id);

#ifdef ENABLE_STAY_AWAKE
    SMTC_LOG_INF("Enable wake lock");
    self->wake_lock = wakeup_source_register(SMTC_DRIVER_NAME);
    if (!self->wake_lock){
        SMTC_LOG_ERR("Failed to create wake lock.");
        ret = -ENOMEM;
        goto FREE_IRQ;
    }
#endif
#ifdef CONFIG_CAPSENSE_USB_CAL
                /*notify usb state*/
                INIT_WORK(&self->ps_notify_work, ps_notify_callback_work);
                self->ps_notif.notifier_call = ps_notify_callback;
                ret = power_supply_reg_notifier(&self->ps_notif);
                if (ret)
                {
                        SMTC_LOG_ERR("Unable to register ps_notifier: %d\n", ret);
                        goto FREE_WAKEUP_SOURCE;
                }

                psy = power_supply_get_by_name(USB_POWER_SUPPLY_NAME);
                if (psy) {
                        ret = ps_get_state(psy, &self->ps_is_present);
                        if (ret) {
                                SMTC_LOG_ERR("psy get property failed rc=%d\n", ret);
                                goto FREE_PS_NOTIFIER;
                                //power_supply_unreg_notifier(&self->ps_notif);
                        }
                }
#endif
#ifdef HEADSET_CALI
    INIT_WORK(&self->hs_notify_work, hs_notify_callback_work);
    self->hs_notif.notifier_call = (notifier_fn_t)hs_notify_callback;
    ret = headset_register_notifier(&self->hs_notif);
    if (ret) {
        SMTC_LOG_ERR("Unable to register hs_notifier: %d", ret);
        goto FREE_PS_NOTIFIER;
    }
    self->hs_is_plug = -1;
#endif
    SMTC_LOG_INF("Done.");
    return 0;

    //.............................................................................................
#ifdef CONFIG_CAPSENSE_USB_CAL
FREE_PS_NOTIFIER:
    cancel_work_sync(&self->ps_notify_work);
    power_supply_unreg_notifier(&self->ps_notif);
FREE_WAKEUP_SOURCE:
#ifdef ENABLE_STAY_AWAKE
    wakeup_source_unregister(self->wake_lock);
#endif
#endif
#ifdef ENABLE_STAY_AWAKE
FREE_IRQ:
    free_irq(self->irq_id, self);
#endif

FREE_INPUTS:
    for (i=0; i<NUM_PHASES; i++)
    {
        if (self->phases[i].input){
            input_unregister_device(self->phases[i].input);
        }
    }

    sysfs_remove_group(&client->dev.kobj, &smtc_sysfs_nodes);

FREE_GPIO:
    gpio_free(self->gpio);
    SMTC_LOG_INF("Failed to probe ret=%d.", ret);
    return ret;
}

//#################################################################################################
static int smtc_remove(struct i2c_client *client)
{
    int ret, i;
    Self self = i2c_get_clientdata(client);
    SMTC_LOG_INF("Remove driver.");

#ifdef HEADSET_CALI
    headset_unregister_notifier(&self->hs_notif);
#endif
#ifdef CONFIG_CAPSENSE_USB_CAL
    cancel_work_sync(&self->ps_notify_work);
    power_supply_unreg_notifier(&self->ps_notif);
#endif
    //enter pause mode to save power
    ret = smtc_send_cmd(self, CMD_PAUSE);
    if (ret){
        SMTC_LOG_ERR("Failed to enter pause mode. ret= %d", ret);
    }
    smtc_read_and_clear_irq(self, NULL);
    free_irq(self->irq_id, self);

#ifndef USE_THREAD_IRQ
    cancel_delayed_work_sync(&self->worker);
#endif

    for (i=0; i<NUM_PHASES; i++)
    {
        if (self->phases[i].input){
            input_unregister_device(self->phases[i].input);
        }
    }
    sysfs_remove_group(&client->dev.kobj, &smtc_sysfs_nodes);
    gpio_free(self->gpio);

#ifdef ENABLE_STAY_AWAKE
    wakeup_source_unregister(self->wake_lock);
#endif

    return 0;
}

//===================================================`==============================================
static int smtc_suspend(struct device *dev)
{
//.................................................................................................
#if DISABLE_DURING_SUSPEND
    Self self = dev_get_drvdata(dev);
    SMTC_LOG_INF("Disable SAR sensor");

    //turn off all phases
    SMTC_LOG_DBG("enabled_phases=0x%X", self->variables.enabled_phases);
    smtc_i2c_write(self, REG_PHEN, self->variables.enabled_phases & ~0xFF);
    disable_irq(self->irq_id);
    //release IRQ pin to avoid leakage of current
    smtc_read_and_clear_irq(self, NULL);

//.................................................................................................
#elif PAUSE_DURING_SUSPEND
    Self self = dev_get_drvdata(dev);
    SMTC_LOG_INF("Suspend SAR sensor");
    smtc_send_cmd(self, 0xD);
    disable_irq(self->irq_id);
    //release IRQ pin to avoid leakage of currents
    smtc_read_and_clear_irq(self, NULL);

//.................................................................................................
#elif WAKE_BY_IRQ
    //wake up function is enabled in the probe
    SMTC_LOG_INF("wakeup by IRQ is enabled");

//.................................................................................................
#else
    Self self = dev_get_drvdata(dev);
    SMTC_LOG_INF("SAR sensor will be still running during suspend but with wakup disabled");
    disable_irq(self->irq_id);
    //don't raise IRQ during suspend
    smtc_i2c_write(self, REG_IRQ_MASK, 0x0);
    //release IRQ pin to avoid leakage of current
    smtc_read_and_clear_irq(self, NULL);
#endif

    return 0;
}

static int smtc_resume(struct device *dev)
{
//.................................................................................................
#if DISABLE_DURING_SUSPEND
    self_t *self = dev_get_drvdata(dev);
    SMTC_LOG_INF("Enable SAR sensor");
    enable_irq(self->irq_id);

    if (mutex_trylock(&self->phen_lock))
    {
        SMTC_LOG_DBG("enabled_phases=0x%X", self->variables.enabled_phases);
        smtc_i2c_write(self, REG_PHEN, self->variables.enabled_phases);
        mutex_unlock(&self->phen_lock);
        smtc_send_cmd(self, CMD_COMPENSATE);
    }else{
        SMTC_LOG_INF("Up layer is enabling phases");
    }

//.................................................................................................
#elif PAUSE_DURING_SUSPEND
    Self self = dev_get_drvdata(dev);
    SMTC_LOG_INF("Resume SAR sensor");
    enable_irq(self->irq_id);
    smtc_send_cmd(self, CMD_RESUME);

//.................................................................................................
#elif WAKE_BY_IRQ
    SMTC_LOG_INF("Enter");

//.................................................................................................
#else //chip is enabled but with IRQ is disabled
    Self self = dev_get_drvdata(dev);
    SMTC_LOG_INF("Enable IRQ");
    enable_irq(self->irq_id);
    smtc_i2c_write(self, REG_IRQ_MASK, self->variables.irq_mask);

#ifdef ENABLE_STAY_AWAKE
    //__pm_wakeup_event(self->wake_lock, WAKE_TMOUT);
    __pm_stay_awake(self->wake_lock);
#endif

//in case touch status updated during system suspend
#ifdef USE_THREAD_IRQ
    SMTC_LOG_DBG("Force to update status");
    smtc_update_status(self);
#else
    SMTC_LOG_DBG("Update status with worker");
    schedule_delayed_work(&self->worker, 0);
#endif //USE_THREAD_IRQ

#ifdef ENABLE_STAY_AWAKE
#if WAKE_TMOUT
    SMTC_LOG_INF("Relax stay awake");
    __pm_relax(self->wake_lock);
#else
    SMTC_LOG_INF("smtc_update_status calls __pm_relax when there is no WAKE_TMOUT");
#endif
#endif //ENABLE_STAY_AWAKE

#endif

    return 0;
}

//#################################################################################################
static struct i2c_device_id smtc_idtable[] =
{
    {SMTC_DRIVER_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, smtc_idtable);

static struct of_device_id smtc_match_table[] =
{
    { .compatible = SMTC_COMPATIBLE_NAME,},
    { }
};

static const struct dev_pm_ops smtc_pm_ops =
{
    .suspend = smtc_suspend,
    .resume = smtc_resume,
};
static struct i2c_driver smtc_driver =
{
    .driver = {
        .owner			= THIS_MODULE,
        .name			= SMTC_DRIVER_NAME,
        .of_match_table	= smtc_match_table,
        .pm				= &smtc_pm_ops,
    },

    .id_table   = smtc_idtable,
    .probe      = smtc_probe,
    .remove     = smtc_remove,
};

static int __init smtc_i2c_init(void)
{
    SMTC_LOG_INF("Enter");
    return i2c_add_driver(&smtc_driver);
}
static void __exit smtc_i2c_exit(void)
{
    i2c_del_driver(&smtc_driver);
}

late_initcall(smtc_i2c_init);
module_exit(smtc_i2c_exit);

MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX93_3x7x Capacitive SAR and Touch Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("3.0");
