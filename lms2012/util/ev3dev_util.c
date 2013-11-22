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
 */

int ev3dev_register( struct platform_device **pdev )
{
        int ret = 0;
 
        struct device *dev;

        // Check to see if the "ev3dev" device is already registered...
        //
        dev = device_find_child( &platform_bus, "ev3dev", match_child );

        if( NULL != dev ) {
            *pdev = to_platform_device(dev);
        } else {
            ret = -ENODEV;
            *pdev = NULL;
        } 

        return ret;
}

/* -----------------------------------------------------------------------
 * Custom store/show functions for ev3dev attribute types
 */
#define to_ev3dev_minmax_attr(x) container_of(x, struct ev3dev_minmax_attribute, dev_attr)

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
