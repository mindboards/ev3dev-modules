/*
 * Copyright (c) 2013-2013 - Ralph Hempel - based on original code attributed
 *                                          to the copyright holder(s) below:
 *
 * LEGO® MINDSTORMS EV3
 *
 * Copyright (C) 2010-2013 The LEGO Group
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


/* Manages button and LEDs
 *
 *  The standard LEGO UI driver accepts a single character between '0' and '9'
 *  which is used to specify one of 10 standard LED falsh combinations. This 
 *  is somewhat limiting, so the ev3dev ui driver is a lot more flexible.
 *
 *  Like all the ev3dev drivers, the ev3dev_ui module is controlled and
 *  monitored using standard fle access in the sysfs tree. The location of
 *  the ev3dev_ui platform device is:
 *
 *  /sys/devices/platform/ev3dev/ui
 *
 *  LED DRIVER OPERATION
 *  -------------------
 *
 *  The following files control the operation of the ev3dev_ui LED driver
 *
 *  /sys/devices/platform/ev3dev/ui/ledleft0
 *  /sys/devices/platform/ev3dev/ui/ledleft1
 *  /sys/devices/platform/ev3dev/ui/ledright0
 *  /sys/devices/platform/ev3dev/ui/ledright1
 *  /sys/devices/platform/ev3dev/ui/pattern
 *
 *  Let's take a step back and go over the LED operation on the EV3 hardware.
 *
 *  There are two LEDs on either side of the front panel button cluster, we'll
 *  call them ledleft and ledright. Each of these LEDs can take on one of the
 *  four following colours:
 *
 *  0 - black
 *  1 - green
 *  2 - red
 *  3 - orange
 *
 *  The flashing pattern of the LEDs is controlled by a pattern of up
 *  to 20 1's and 0's and each step in the pattern takes 50 msec, or 20 times
 *  per second. Therefore, you can specify an arbitrary flash pattern up
 *  to one second in length.
 * 
 *  You can probably figure out where this is headed. When the pattern bit
 *  is a '0' character, the left and right LEDs take on the colour in the
 *  leftled0 or rightled0 file respectively. Similarly, when the pattern bit
 *  is a '1' character, the left and right LEDs take on the colour in the
 *  rightled1 or righttled1 file.
 * 
 *  So, for an example, if you want to flash the left and right LEDs
 *  alternately red and green for 100 msec each time, that looks like this
 *  in the shell - it's trivial to do this in a real programming language.
 *
 *  echo "0" > /sys/devices/platform/ev3dev/ui/ledleft0
 *  echo "2" > /sys/devices/platform/ev3dev/ui/ledleft1
 *  echo "1" > /sys/devices/platform/ev3dev/ui/ledright0
 *  echo "0" > /sys/devices/platform/ev3dev/ui/ledright1
 *
 *  echo "0011" > /sys/devices/platform/ev3dev/ui/ledright1
 *  
 *  That's all there is to the ev3dev_ui LED driver!
 *  
 *  BUTTON DRIVER OPERATION
 *  -----------------------
 *
 *  The following files control the operation of the ev3dev_ui Button driver
 *
 *  /sys/devices/platform/ev3dev/ui/buttonesc
 *  /sys/devices/platform/ev3dev/ui/buttonleft
 *  /sys/devices/platform/ev3dev/ui/buttonright
 *  /sys/devices/platform/ev3dev/ui/buttonup
 *  /sys/devices/platform/ev3dev/ui/buttondown
 *  /sys/devices/platform/ev3dev/ui/buttonenter
 *
 *  These files hold the current state of the individual buttons. Simply read
 *  the file to get the button state. A '1' means the button is pressed, while
 *  a '0' means the button is not pressed.
 *
 *  /sys/devices/platform/ev3dev/ui/buttons
 *
 *  This file holds the current state of all the buttons at one. Read the file
 *  to return a string of '1' and '0' characters that represent the switch
 *  state  in this order
 *
 *  0  UP
 *  1  ENTER
 *  2  DOWN
 *  3  RIGHT
 *  4  LEFT
 *  5  BACK
 *
 *  Reading the files from the shell looks like this, it's trivial to do in a
 *  real programming language.
 *
 *  cat /sys/devices/platform/ev3dev/ui/buttons
 *
 *  If the result is "001010" it means the DOWN and LEFT buttons are pressed
 */

#include  "lms2012.h"
#include  "am1808.h"

#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <asm/gpio.h>

#include "ev3dev_util.h"

#define DEVICE_NAME "ev3dev_ui"

#define EV3DEV_UI_OFFSET_VER_MAJOR	0x00
#define EV3DEV_UI_OFFSET_VER_MINOR	0x01

// enum      UiLedPins
// {
//   DIODE0,
//   DIODE1,
//   DIODE2,
//   DIODE3,
//   DIODE4,
//   DIODE5,
//   LED_PINS
// };
// 
// enum      UiButPins
// {
//   BUT0,     // UP
//   BUT1,     // ENTER
//   BUT2,     // DOWN
//   BUT3,     // RIGHT
//   BUT4,     // LEFT
//   BUT5,     // BACK
//   BUT_PINS
// };

#define   NO_OF_LEDS                    LEDS
#define   NO_OF_BUTTONS                 BUTTONS

#define EV3_DIODE_0_PIN		GPIO_TO_PIN(6, 12)
#define EV3_DIODE_1_PIN		GPIO_TO_PIN(6, 14)
#define EV3_DIODE_2_PIN		GPIO_TO_PIN(6, 13)
#define EV3_DIODE_3_PIN		GPIO_TO_PIN(6,  7)

static const int legoev3_led_gpio[]  = {
	EV3_DIODE_0_PIN,
        EV3_DIODE_1_PIN,
        EV3_DIODE_2_PIN,
        EV3_DIODE_3_PIN,
};

#define EV3_BUTTON_0_PIN		GPIO_TO_PIN(7, 15)
#define EV3_BUTTON_1_PIN		GPIO_TO_PIN(1, 13)
#define EV3_BUTTON_2_PIN		GPIO_TO_PIN(7, 14)
#define EV3_BUTTON_3_PIN		GPIO_TO_PIN(7, 12)
#define EV3_BUTTON_4_PIN		GPIO_TO_PIN(6, 6 )
#define EV3_BUTTON_5_PIN		GPIO_TO_PIN(6, 10)

static const int legoev3_button_gpio[]  = {
	EV3_BUTTON_0_PIN,
        EV3_BUTTON_1_PIN,
        EV3_BUTTON_2_PIN,
        EV3_BUTTON_3_PIN,
        EV3_BUTTON_4_PIN,
        EV3_BUTTON_5_PIN
};

void      InitGpio(void)
{
  int     Pin;

    // This is CRITICAL code to making the LEFT button work - it disables the
    // internal pullup on pin group 25 which is where the GPIO6_6 lives. That's
    // what was keeping the LEFT button input in a permanent high state!

    {
        void    __iomem *Reg;
     
        Reg   =  da8xx_syscfg0_base - 0x14000 + 0x22C000 + 0x10;
        *(u32*)Reg &=  0xFDFFFFFF;
    }

    for (Pin = 0;Pin < NO_OF_LEDS;Pin++) {
        gpio_request(          legoev3_led_gpio[Pin], "ev3_diode");
        gpio_direction_output( legoev3_led_gpio[Pin], 0          );
    }

    for (Pin = 0;Pin < NO_OF_BUTTONS;Pin++) {
        gpio_request(          legoev3_button_gpio[Pin], "ev3_button");
        gpio_direction_input(  legoev3_button_gpio[Pin]              );
    }
}

// DEVICE1 ********************************************************************

static    struct hrtimer Device1Timer;
static    ktime_t        Device1Time;
static    struct hrtimer Device2Timer;
static    ktime_t        Device2Time;


#define   BUTRead(B)                    (gpio_get_value( legoev3_button_gpio[B] ))


#define   DIODEOn(D)                    {\
	                                  gpio_set_value( legoev3_led_gpio[D], 1);\
                                        }

#define   DIODEOff(D)                   {\
	                                  gpio_set_value( legoev3_led_gpio[D], 0);\
                                        }

#define EV3DEV_UI_LED_BLACK  (0)
#define EV3DEV_UI_LED_GREEN  (1)
#define EV3DEV_UI_LED_RED    (2)
#define EV3DEV_UI_LED_ORANGE (3)

#define EV3DEV_LEFT_LED_RED     (0)
#define EV3DEV_LEFT_LED_GREEN   (1)
#define EV3DEV_RIGHT_LED_RED    (2)
#define EV3DEV_RIGHT_LED_GREEN  (3)

#define EV3DEV_LEFT_LED         (0)
#define EV3DEV_RIGHT_LED        (1)

#define EV3DEV_PATTERN_0        (0)
#define EV3DEV_PATTERN_1        (1)

static unsigned int ledleft[ 2];
static unsigned int ledright[2];

#define EV3DEV_UI_MAX_PATTERN (20)

static unsigned char pattern[EV3DEV_UI_MAX_PATTERN + 1];

static unsigned int  patternlength;

static enum hrtimer_restart Device1TimerInterrupt1(struct hrtimer *pTimer)
{
    static unsigned int patternIdx = 0;

    static unsigned int prevPatternState = EV3DEV_PATTERN_1 + 1;

    static unsigned int thisPatternState = EV3DEV_PATTERN_1 + 1;

    // Calculate the current pattern state, then check to see if the LEDs
    // need to be updated - default the state to 0!

    if( '1' ==  pattern[patternIdx] ) {
       thisPatternState = EV3DEV_PATTERN_1;
    } else {
       thisPatternState = EV3DEV_PATTERN_0;
    }
 
    if ( thisPatternState != prevPatternState ) {

        prevPatternState = thisPatternState;
   
        // Update the left LED state
 
        switch( ledleft[thisPatternState] ) {
     
        case EV3DEV_UI_LED_BLACK  : DIODEOff( EV3DEV_LEFT_LED_RED  );
                                    DIODEOff( EV3DEV_LEFT_LED_GREEN);
                                    break;
        case EV3DEV_UI_LED_GREEN  : DIODEOff( EV3DEV_LEFT_LED_RED  );
                                    DIODEOn(  EV3DEV_LEFT_LED_GREEN);
                                    break;
        case EV3DEV_UI_LED_RED    : DIODEOn(  EV3DEV_LEFT_LED_RED  );
                                    DIODEOff( EV3DEV_LEFT_LED_GREEN);
                                    break;
        case EV3DEV_UI_LED_ORANGE : DIODEOn(  EV3DEV_LEFT_LED_RED  );
                                    DIODEOn(  EV3DEV_LEFT_LED_GREEN);
                                    break;
        default:                    break;
        }

        // Update the right LED state

        switch( ledright[thisPatternState] ) {
     
        case EV3DEV_UI_LED_BLACK  : DIODEOff( EV3DEV_RIGHT_LED_RED  );
                                    DIODEOff( EV3DEV_RIGHT_LED_GREEN);
                                    break;
        case EV3DEV_UI_LED_GREEN  : DIODEOff( EV3DEV_RIGHT_LED_RED  );
                                    DIODEOn(  EV3DEV_RIGHT_LED_GREEN);
                                    break;
        case EV3DEV_UI_LED_RED    : DIODEOn(  EV3DEV_RIGHT_LED_RED  );
                                    DIODEOff( EV3DEV_RIGHT_LED_GREEN);
                                    break;
        case EV3DEV_UI_LED_ORANGE : DIODEOn(  EV3DEV_RIGHT_LED_RED  );
                                    DIODEOn(  EV3DEV_RIGHT_LED_GREEN);
                                    break;
        default:                    break;
        }

    }

    // Now increment the patternIdx, and make sure it wraps around based on pattern length
 
    if( 0 == patternlength ) {
        patternIdx = 0;
    } else {
        patternIdx = (( patternIdx + 1 ) % patternlength);
    }

  // restart timer
  hrtimer_forward_now(pTimer,ktime_set(0,50000000));

  return (HRTIMER_RESTART);
}
 
#define EV3DEV_BUTTON_UP    (0)
#define EV3DEV_BUTTON_ENTER (1)
#define EV3DEV_BUTTON_DOWN  (2)
#define EV3DEV_BUTTON_RIGHT (3)
#define EV3DEV_BUTTON_LEFT  (4)
#define EV3DEV_BUTTON_BACK  (5)

static unsigned int ButtonState[NO_OF_BUTTONS];

static enum hrtimer_restart Device2TimerInterrupt1(struct hrtimer *pTimer)
{
    UWORD   Tmp;

    for(Tmp = 0;Tmp < NO_OF_BUTTONS;Tmp++)
    {
        if (BUTRead(Tmp)) { 
            ButtonState[Tmp]  =  1;
        } else {
            ButtonState[Tmp]  =  0;
        }
    }

    // restart timer
    hrtimer_forward_now(pTimer,ktime_set(0,10000000));

    return (HRTIMER_RESTART);
}

static int Device1Init(void)
{
  int     Result = 0;

      // setup ui update timer interrupt
      Device2Time  =  ktime_set(0,10000000);
      hrtimer_init(&Device2Timer,CLOCK_MONOTONIC,HRTIMER_MODE_REL);
      Device2Timer.function  =  Device2TimerInterrupt1;
      hrtimer_start(&Device2Timer,Device2Time,HRTIMER_MODE_REL);

      // setup ui update timer interrupt
      Device1Time  =  ktime_set(0,50000000);
      hrtimer_init(&Device1Timer,CLOCK_MONOTONIC,HRTIMER_MODE_REL);
      Device1Timer.function  =  Device1TimerInterrupt1;
      hrtimer_start(&Device1Timer,Device1Time,HRTIMER_MODE_REL);

      printk("  %s device register success\n",DEVICE_NAME);
 
   return (Result);
   }


static void Device1Exit(void)
{
  int     Tmp;

  hrtimer_cancel(&Device1Timer);
  hrtimer_cancel(&Device2Timer);
 
   for (Tmp = 0;Tmp < NO_OF_LEDS;Tmp++)
   {
     DIODEOff(Tmp);
   }
}

static int ModuleInit(void)
{
      InitGpio();

      Device1Init();

  return (0);
}


static void ModuleExit(void)
{
    Device1Exit();

}

// -----------------------------------------------------------------------

static ssize_t show_pattern(struct device *dev,
        struct device_attribute *attr, char *buf)
{
        return snprintf( buf, PAGE_SIZE, "%s\n", pattern );
}

static ssize_t store_pattern(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
        int i;

        memset( pattern, '\0', sizeof( pattern ) );

        for( i=0; i<EV3DEV_UI_MAX_PATTERN; ++i ) {
            if      (buf[i] == '1') { pattern[i] = '1'; }
            else if (buf[i] == '0') { pattern[i] = '0'; }
            else    break;
        }

        printk("  pattern length is %08x\n", patternlength);
        patternlength = i;
        printk("  pattern length is %08x\n", patternlength);

        return( count );
}

static EV3DEV_MINMAX_ATTR(ledleft0,  ledleft0,  0666, ledleft[ 0], 0, 3);
static EV3DEV_MINMAX_ATTR(ledleft1,  ledleft1,  0666, ledleft[ 1], 0, 3);
static EV3DEV_MINMAX_ATTR(ledright0, ledright0, 0666, ledright[0], 0, 3);
static EV3DEV_MINMAX_ATTR(ledright1, ledright1, 0666, ledright[1], 0, 3);

static DEVICE_ATTR(pattern, 0666, show_pattern, store_pattern);

static ssize_t show_buttons(struct device *dev,
        struct device_attribute *attr, char *buf)
{
        unsigned int i;

        for( i=0; i<NO_OF_BUTTONS; ++i ) {
            buf[i] = ( 0 == ButtonState[i] ) ? '0' : '1'; 
        }
        buf[i++] = '\n';
        buf[i++] = '\0';

        return i;
}

static DEVICE_INT_ATTR(buttonup,    0444, ButtonState[EV3DEV_BUTTON_UP   ]);
static DEVICE_INT_ATTR(buttonenter, 0444, ButtonState[EV3DEV_BUTTON_ENTER]);
static DEVICE_INT_ATTR(buttondown,  0444, ButtonState[EV3DEV_BUTTON_DOWN ]);
static DEVICE_INT_ATTR(buttonright, 0444, ButtonState[EV3DEV_BUTTON_RIGHT]);
static DEVICE_INT_ATTR(buttonleft,  0444, ButtonState[EV3DEV_BUTTON_LEFT ]);
static DEVICE_INT_ATTR(buttonback,  0444, ButtonState[EV3DEV_BUTTON_BACK ]);

static DEVICE_ATTR(buttons, 0444, show_buttons, NULL );

static struct attribute *ev3dev_ui_attrs[] = {
    &dev_attr_ledleft0.dev_attr.attr
  , &dev_attr_ledleft1.dev_attr.attr
  , &dev_attr_ledright0.dev_attr.attr
  , &dev_attr_ledright1.dev_attr.attr
  , &dev_attr_pattern.attr
  , &dev_attr_buttonback.attr.attr
  , &dev_attr_buttonup.attr.attr
  , &dev_attr_buttonenter.attr.attr
  , &dev_attr_buttondown.attr.attr
  , &dev_attr_buttonright.attr.attr
  , &dev_attr_buttonleft.attr.attr
  , &dev_attr_buttons.attr
  , NULL
};

static struct attribute_group ev3dev_ui_attr_group = {
        .attrs = ev3dev_ui_attrs,
};

// -----------------------------------------------------------------------

extern struct platform_device *ev3dev;

static struct platform_device *ui;

static int ev3dev_ui_init(void)
{
        int ret = -ENXIO;

        if ( !ev3dev ) {
            printk( "ev3dev node is not yet registered, load the ev3dev_ev3dev module\n");

            goto err0; 
        }

        ret = -ENOMEM;

        ui = platform_device_alloc( "ui", -1 );         
        if (!ui) {
                printk( KERN_CRIT "failed to allocate ev3dev\\ui device\n");
                goto err1;
        }

        printk("  ui_init ui allocated  %08x\n", (unsigned int)(ui));

        ui->dev.parent     = &ev3dev->dev;

        ret = platform_device_add(ui);
        if (ret) {
                printk( "failed to register ui device\n");
                goto err4;
        }

        ret = sysfs_create_group(&ui->dev.kobj,
                                  &ev3dev_ui_attr_group);

        ModuleInit();
        
        return 0;

err4: 
        platform_device_put( ui );

err1: 

err0: 
        return ret;
}

static void ev3dev_ui_exit(void)
{
        ModuleExit();

        sysfs_remove_group(&ui->dev.kobj, &ev3dev_ui_attr_group);

        platform_device_del( ui );
        platform_device_put( ui );
}

module_init(ev3dev_ui_init);
module_exit(ev3dev_ui_exit)

MODULE_AUTHOR("Ralph Hempel <rhempel@hempeldesigngroup.com>");
MODULE_DESCRIPTION("Driver for LEGO MINDSTORMS EV3 ui Device");
MODULE_LICENSE("GPL");

