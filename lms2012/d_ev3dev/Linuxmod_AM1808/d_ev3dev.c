/*
 * Copyright (c) 2013-2013 - Ralph Hempel - based on original code attributed
 *                                          to the copyright holder(s) below:
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
 * Every ev3dev_xx module needs access to the ev3dev device node, which
 * is created by this module.
 *
 * When an ev3dev_xx module loads a check is made for the "ev3dev" platform
 * device node. If that node does not exist, then the ev3dev_xx module will
 * fail to load.
 *
 * The ev3dev_xx module must register the xx device as a child of the
 * ev3dev device, and then register any additional instances of subdevices
 * or device attributes.
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
 *
 * When the ev3dev module that holds the ev3dev_device structure is
 * unloaded, a check is made for modules that still are registered
 * under the ev3dev node. If there are still device children
 * registered the module will not unload.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#include "ev3dev_util.h"

static void ev3dev_release(struct device *dev)
{
       printk( "ev3dev_release\n");
}

static struct platform_device ev3dev_device = {
        .name        = "ev3dev",
        .id          = -1,
        .dev.release = ev3dev_release,
};

struct platform_device *ev3dev = NULL;

EXPORT_SYMBOL_GPL(ev3dev);

static int ev3dev_init(void)
{
       int ret;

       ret = platform_device_register( &ev3dev_device );

       ev3dev = &ev3dev_device;

       printk( "ev3dev_init registers ev3dev at %08x\n", (unsigned int)&ev3dev_device);

       return ret;
}

static void ev3dev_exit(void)
{
      //  int children = 0;

       // struct device *dev;

       // Check to see if the "ev3dev" device is still registered...
       //
       // dev = device_find_child( &platform_bus, "ev3dev", match_child );

       // printk( "ev3dev_unregister search for ev3dev platform child %08x\n", (unsigned int)dev);

       // printk( "ev3dev_unregister local   ev3dev_device is %08x\n", (unsigned int)&ev3dev_device );
       // printk( "ev3dev_unregister passed  ev3dev_device is %08x\n", (unsigned int)*pdev          );

       // if ( NULL != dev ) {

       //     device_for_each_child( dev, &children, count_child );

        //    printk( "ev3dev_unregister %08x still has %d children\n", (unsigned int)dev, children );
        
         //   if( 0 == children ) {
        //      platform_device_unregister( to_platform_device(dev) );
       ev3dev = NULL;

       platform_device_unregister( &ev3dev_device );

//              printk( "ev3dev_unregister result  %08x\n", (unsigned int)(ev3dev_device.dev) );

      //          if( NULL != pdev ) {
       //            *pdev  = NULL;
        //        }
       //     }
     //   }

}

module_init(ev3dev_init);
module_exit(ev3dev_exit)

MODULE_AUTHOR("Ralph Hempel <rhempel@hempeldesigngroup.com>");
MODULE_DESCRIPTION("Driver node for LEGO MINDSTORMS EV3");
MODULE_LICENSE("GPL");

