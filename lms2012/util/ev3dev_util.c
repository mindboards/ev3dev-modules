/*
 * ev3dev_register.c - helper routines to allow multiple ev3dev compatible
 *                     modules to create additional nodes under
 *
 *                     /sysfs/devices/platform/ev3dev
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

#include <asm/page.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include "ev3dev_util.h"

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

static void ev3dev_release(struct device *dev)
{
}

static struct platform_device ev3dev_device = {
        .name        = "ev3dev",
        .dev.release = ev3dev_release,
        .id          = -1,
};

/* -----------------------------------------------------------------------
 * A simple string comparison routine that is good enough for comparing
 * device names
 */
 
static int match_child(struct device *dev, void *data)
{
       if (!dev_name(dev))
               return 0;
       return !strcmp(dev_name(dev), (char *)data);
}

/* -----------------------------------------------------------------------
 * A simple counter increment that counts the children for a
 * particular device node
 */

static int count_child(struct device *dev, void *data)
{
       (*(int *)data)++;
       return 0;
}

int ev3dev_register( struct platform_device **pdev )
{
        int ret = 0;
 
        struct device *dev;

        // Check to see if the "ev3dev" device is already registered...
        //
        dev = device_find_child( &platform_bus, "ev3dev", match_child );

        printk( "ev3dev_register search for ev3dev platform child %08x\n", (unsigned int)(dev) );

        if( NULL != dev ) {
            printk( "ev3dev_register already registered %08x\n", (unsigned int)(dev) );
        } else {
            ret = platform_device_register( &ev3dev_device );
        } 

//        printk( "ev3dev_registered device at %08x\n", (unsigned int)( ev3dev_device.dev) );
        printk( "ev3dev_registered device in %08x\n", (unsigned int)(&ev3dev_device    ) );

        *pdev = &ev3dev_device;

        return ret;
}

void ev3dev_unregister(struct platform_device **pdev)
{
        int children = 0;

        struct device *dev;

        // Check to see if the "ev3dev" device is still registered...
        //
        dev = device_find_child( &platform_bus, "ev3dev", match_child );

        printk( "ev3dev_unregister search for ev3dev platform child %08x\n", (unsigned int)dev);

        printk( "ev3dev_unregister local   ev3dev_device is %08x\n", (unsigned int)&ev3dev_device );
        printk( "ev3dev_unregister passed  ev3dev_device is %08x\n", (unsigned int)*pdev          );

        if ( NULL != dev ) {
            device_for_each_child( dev, &children, count_child );

            printk( "ev3dev_unregister %08x still has %d children\n", (unsigned int)dev, children );
        
            if( 0 == children ) {
                platform_device_unregister(*pdev);

//              printk( "ev3dev_unregister result  %08x\n", (unsigned int)(ev3dev_device.dev) );

                if( NULL != pdev ) {
                    *pdev  = NULL;
                }
            }
        }
}

/* -----------------------------------------------------------------------
 * Custom store/show functions for ev3dev attribute types
 */
#define to_ev3dev_minmax_attr(x) container_of(x, struct ev3dev_minmax_attribute, attr)

ssize_t ev3dev_store_minmax(struct device *dev,
                            struct device_attribute *attr,
                            const char *buf, size_t size)
{
        struct ev3dev_minmax_attribute *ea = to_ev3dev_minmax_attr(attr);
        char *end;
        long new = simple_strtol(buf, &end, 0);

        if (end == buf || new > ea->max || new < ea->min )
                return -EINVAL;

        *(int *)(ea->var) = new;

        /* Always return full write size even if we didn't consume all */
        return size;
}

ssize_t ev3dev_show_minmax(struct device *dev,
                           struct device_attribute *attr,
                           char *buf)
{
        struct ev3dev_minmax_attribute *ea = to_ev3dev_minmax_attr(attr);

        return snprintf(buf, PAGE_SIZE, "%d\n", *(int *)(ea->var) );
}
