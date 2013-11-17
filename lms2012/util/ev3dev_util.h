/*
 * ev3dev_util.h - helper routines to allow multiple ev3dev compatible
 *                 modules to create additional nodes under
 *
 *                 /sysfs/devices/platform/ev3dev
 *
 * Copyright (c) 2013-2013 - Ralph Hempel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/* -----------------------------------------------------------------------
 * Every ev3dev_xx module has its own static copy of the ev3_device
 * structure. If this is the first ev3dev compatible module to load then
 * this module is responsible for creating the device node.
 *
 * When an ev3dev_xx module loads a check is made for the "ev3dev" platform
 * device node. If that node does not exist, then the "ev3dev" platform
 * device is registered.
 *
 * The ev3dev module must register the xx device as a child of the ev3dev
 * device, and then register any additional instances of subdevices or
 * device attributes.
 *
 * The result is a tree that looks like this, for example:
 *
 * sys/devices/platform/ev3dev/pwm/motorA
 *                                /motorB 
 *
 * Key functions that make this work: device_find_child()
 *                                    device foreach child()
 *
 * The ev3dev_release function is responsible for cleaning up the 
 * ev3dev entry when the last module is unloaded.
 */

#include <linux/device.h>

struct ev3dev_minmax_attribute {
        struct device_attribute attr;
        void *var;
        int   min;
        int   max; 
};

extern ssize_t ev3dev_store_minmax(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t size);

extern ssize_t ev3dev_show_minmax(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf);

#define EV3DEV_MINMAX_ATTR(_name, _mode, _var, _min, _max ) \
        struct ev3dev_minmax_attribute dev_attr_##_name =   \
                { __ATTR(_name, _mode, ev3dev_show_minmax, ev3dev_store_minmax), &(_var), _min, _max }

extern int  ev3dev_register(  struct platform_device ** );
extern void ev3dev_unregister(struct platform_device ** );


