/*
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


/*! \page UiModule Ui Button/Led Module
 *
 *  Manages button and LEDs\n
 *
 *-  \subpage UiDriver
 *-  \subpage UiModuleMemory
 *-  \subpage UiModuleResources
 *
 */


#ifndef PCASM
#include  <asm/types.h>
#endif

#define   HW_ID_SUPPORT

#include  "../../lms2012/source/lms2012.h"
#include  "../../lms2012/source/am1808.h"


#define   MODULE_NAME                   "ui_module"
#define   DEVICE1_NAME                  UI_DEVICE

static    int  ModuleInit(void);
static    void ModuleExit(void);

#include  <linux/kernel.h>
#include  <linux/fs.h>

#include  <linux/sched.h>

#ifndef   PCASM
#include  <linux/mm.h>
#include  <linux/hrtimer.h>

#include  <linux/init.h>
#include  <linux/uaccess.h>
#include  <linux/debugfs.h>

#include  <linux/ioport.h>
#include  <asm/gpio.h>
#include  <asm/io.h>
#include  <linux/module.h>
#include  <linux/miscdevice.h>
#include  <asm/uaccess.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <linux/proc_fs.h>

#include <mach/mux.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("The LEGO Group");
MODULE_DESCRIPTION(MODULE_NAME);
MODULE_SUPPORTED_DEVICE(DEVICE1_NAME);

module_init(ModuleInit);
module_exit(ModuleExit);

#else
// Keep Eclipse happy
#endif

// #ifdef    DEBUG_D_UI
#define   DEBUG
// #endif

int       Hw = 0;
// 
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
// 
// 
// INPIN     UiLedPin[LED_PINS];
// 
// INPIN     UiButPin[BUT_PINS];
// 
// 
#define   NO_OF_LEDS                    LEDS
#define   NO_OF_BUTTONS                 BUTTONS

// /*! \page UiModuleResources Gpios and Resources used for Module
//  *
//  *  Describes use of gpio and resources\n
//  *
//  *  \verbatim
//  */
// 
// INPIN     EP2_UiLedPin[LED_PINS] =
// {
//   { GP6_12,  NULL, 0 },  // DIODE1
//   { GP6_14,  NULL, 0 },  // DIODE2
//   { GP6_13,  NULL, 0 },  // DIODE3
//   { GP6_7 ,  NULL, 0 },  // DIODE0
// };
// 
// INPIN     EP2_UiButPin[BUT_PINS] =
// {
//   { GP7_15,  NULL, 0 },  // BUT0
//   { GP1_13,  NULL, 0 },  // BUT1
//   { GP7_14,  NULL, 0 },  // BUT2
//   { GP7_12,  NULL, 0 },  // BUT3
//   { GP6_6 ,  NULL, 0 },  // BUT4
//   { GP6_10,  NULL, 0 },  // BUT5
// };
// 
// 
// INPIN     FINALB_UiLedPin[LED_PINS] =
// {
//   { GP6_12,  NULL, 0 },  // DIODE1
//   { GP2_1 ,  NULL, 0 },  // DIODE2
//   { GP6_13,  NULL, 0 },  // DIODE3
//   { GP6_7 ,  NULL, 0 },  // DIODE0
// };
// 
// INPIN     FINALB_UiButPin[BUT_PINS] =
// {
//   { GP7_15,  NULL, 0 },  // BUT0
//   { GP0_1 ,  NULL, 0 },  // BUT1
//   { GP7_14,  NULL, 0 },  // BUT2
//   { GP7_12,  NULL, 0 },  // BUT3
//   { GP6_6 ,  NULL, 0 },  // BUT4
//   { GP6_10,  NULL, 0 },  // BUT5
// };
// 
// 
// INPIN     FINAL_UiLedPin[LED_PINS] =
// {
//   { GP6_7 ,  NULL, 0 },  // DIODE0
//   { GP6_13,  NULL, 0 },  // DIODE1
//   { GP2_1 ,  NULL, 0 },  // DIODE2
//   { GP5_7 ,  NULL, 0 },  // DIODE3
// };
// 
// INPIN     FINAL_UiButPin[BUT_PINS] =
// {
//   { GP7_15,  NULL, 0 },  // BUT0
//   { GP0_1 ,  NULL, 0 },  // BUT1
//   { GP7_14,  NULL, 0 },  // BUT2
//   { GP7_12,  NULL, 0 },  // BUT3
//   { GP6_6 ,  NULL, 0 },  // BUT4
//   { GP6_10,  NULL, 0 },  // BUT5
// };

/*  \endverbatim
 *  \n
 */


// INPIN     *pUiLedPin[] =
// {
//   [FINAL]     =   FINAL_UiLedPin,       //  FINAL   platform
//   [FINALB]    =   FINALB_UiLedPin,      //  FINALB  platform
//   [EP2]       =   EP2_UiLedPin,         //  EP2     platform
// };
// 
// 
// INPIN     *pUiButPin[] =
// {
//   [FINAL]     =   FINAL_UiButPin,       //  FINAL   platform
//   [FINALB]    =   FINALB_UiButPin,      //  FINALB  platform
//   [EP2]       =   EP2_UiButPin,         //  EP2     platform
// };

#define EV3_DIODE_0_PIN		GPIO_TO_PIN(6, 7 )
#define EV3_DIODE_1_PIN		GPIO_TO_PIN(6, 12)
#define EV3_DIODE_2_PIN		GPIO_TO_PIN(6, 13)
#define EV3_DIODE_3_PIN		GPIO_TO_PIN(6, 14)

static const int legoev3_led_pins[]  = {
	EV3_DIODE_0_PIN,
        EV3_DIODE_1_PIN,
        EV3_DIODE_2_PIN,
        EV3_DIODE_3_PIN,
};

#define EV3_BUTTON_0_PIN		GPIO_TO_PIN(7, 15)
#define EV3_BUTTON_1_PIN		GPIO_TO_PIN(0, 1 )
#define EV3_BUTTON_2_PIN		GPIO_TO_PIN(7, 14)
#define EV3_BUTTON_3_PIN		GPIO_TO_PIN(7, 12)
#define EV3_BUTTON_4_PIN		GPIO_TO_PIN(6, 6 )
#define EV3_BUTTON_5_PIN		GPIO_TO_PIN(6, 10)

static const int legoev3_button_pins[]  = {
	EV3_BUTTON_0_PIN,
        EV3_BUTTON_1_PIN,
        EV3_BUTTON_2_PIN,
        EV3_BUTTON_3_PIN,
        EV3_BUTTON_4_PIN,
        EV3_BUTTON_5_PIN,
};

#define PROCFS_MAX_SIZE		512
#define PROCFS_NAME 		"ev3dev_ui"

static char          procfs_read_buffer[PROCFS_MAX_SIZE];
static unsigned long procfs_read_buffer_size = 0;

static char          procfs_write_buffer[PROCFS_MAX_SIZE];
static unsigned long procfs_write_buffer_size = 0;

static struct proc_dir_entry *procfs_file;

//*****************************************************************************


// static    void      __iomem *GpioBase;
// 
// void      SetGpio(int Pin)
// {
//   int     Tmp = 0;
//   void    __iomem *Reg;
// 
//   if (Pin >= 0)
//   {
//     while ((MuxRegMap[Tmp].Pin != -1) && (MuxRegMap[Tmp].Pin != Pin))
//     {
//       Tmp++;
//     }
//     if (MuxRegMap[Tmp].Pin == Pin)
//     {
//       Reg   =  da8xx_syscfg0_base + 0x120 + (MuxRegMap[Tmp].MuxReg << 2);
// 
//       *(u32*)Reg &=  MuxRegMap[Tmp].Mask;
//       *(u32*)Reg |=  MuxRegMap[Tmp].Mode;
// 
//   #ifdef DEBUG
//       printk("    GP%d_%-2d  0x%08X and 0x%08X or 0x%08X\n",(Pin >> 4),(Pin & 0x0F),(u32)Reg, MuxRegMap[Tmp].Mask, MuxRegMap[Tmp].Mode);
//   #endif
// 
//     }
//     else
//     {
//       printk("*   GP%d_%-2d  ********* ERROR not found *********\n",(Pin >> 4),(Pin & 0x0F));
//     }
//   }
// }


void      InitGpio(void)
{
  int     Pin;
  int     ret;

  // unlock
//   REGUnlock;

#ifdef DEBUG
  printk("  Ui leds\n");
#endif


//   memcpy(UiLedPin,pUiLedPin[Hw],sizeof(EP2_UiLedPin));
//   if (memcmp((const void*)UiLedPin,(const void*)pUiLedPin[Hw],sizeof(EP2_UiLedPin)) != 0)
//   {
//     printk("%s UiLedPin tabel broken!\n",MODULE_NAME);
//   }
// 
//   for (Pin = 0;Pin < NO_OF_LEDS;Pin++)
//   {
//     if (UiLedPin[Pin].Pin >= 0)
//     {
//       UiLedPin[Pin].pGpio  =  (struct gpio_controller *__iomem)(GpioBase + ((UiLedPin[Pin].Pin >> 5) * 0x28) + 0x10);
//       UiLedPin[Pin].Mask   =  (1 << (UiLedPin[Pin].Pin & 0x1F));
// 
//       SetGpio(UiLedPin[Pin].Pin);
//     }
//   }



// #ifdef DEBUG
//   printk("  Ui buttons\n");
// #endif
//   memcpy(UiButPin,pUiButPin[Hw],sizeof(EP2_UiButPin));
//   if (memcmp((const void*)UiButPin,(const void*)pUiButPin[Hw],sizeof(EP2_UiButPin)) != 0)
//   {
//     printk("%s UiButPin tabel broken!\n",MODULE_NAME);
//   }
// 
//   for (Pin = 0;Pin < NO_OF_BUTTONS;Pin++)
//   {
//     if (UiButPin[Pin].Pin >= 0)
//     {
//       UiButPin[Pin].pGpio  =  (GPIOC)(GpioBase + ((UiButPin[Pin].Pin >> 5) * 0x28) + 0x10);
//       UiButPin[Pin].Mask   =  (1 << (UiButPin[Pin].Pin & 0x1F));
// 
//       SetGpio(UiButPin[Pin].Pin);
//     }
//   }

  for (Pin = 0;Pin < NO_OF_LEDS;Pin++) {
    gpio_request(          legoev3_led_pins[Pin], "ev3_diode");
    gpio_set_value(        legoev3_led_pins[Pin], 0          );
    gpio_direction_output( legoev3_led_pins[Pin], 0          );
  }

  for (Pin = 0;Pin < NO_OF_BUTTONS;Pin++) {
    gpio_request(          legoev3_button_pins[Pin], "ev3_button");
    gpio_set_value(        legoev3_button_pins[Pin], 0           );
    gpio_direction_output( legoev3_button_pins[Pin], 0           );
  }
// 
//   // lock
//   REGLock;
}

// DEVICE1 ********************************************************************

static    struct hrtimer Device1Timer;
static    ktime_t        Device1Time;
static    struct hrtimer Device2Timer;
static    ktime_t        Device2Time;

// static    UI UiDefault;
// static    UI *pUi = &UiDefault;

#define   BUTFloat(B)                   {\
	                                  gpio_direction_input(legoev3_led_pins[B]);\
                                        }


#define   BUTRead(B)                    (gpio_get_value( legoev3_button_pins[B] ))


#define   DIODEInit(D)                  {\
	                                  gpio_direction_output(legoev3_led_pins[D], 0);\
                                        }

#define   DIODEOn(D)                    {\
	                                  gpio_set_value( legoev3_led_pins[D], 1);\
                                        }

#define   DIODEOff(D)                   {\
	                                  gpio_set_value( legoev3_led_pins[D], 0);\
                                        }


ULONG     LEDPATTERNDATA[NO_OF_LEDS + 1][LEDPATTERNS] =
{ //  LED_BLACK   LED_GREEN   LED_RED    LED_ORANGE           LED_GREEN_FLASH                     LED_RED_FLASH                     LED_ORANGE_FLASH                      LED_GREEN_PULSE                       LED_RED_PULSE                      LED_ORANGE_PULSE
  {  0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0b00000000000000000000000000000000, 0b00000000000000000111110000011111, 0b00000000000000000111110000011111, 0b00000000000000000000000000000000, 0b00000000000000000000000001110111, 0b00000000000000000000000001110111 }, // RR
  {  0x00000000, 0xFFFFFFFF, 0x00000000, 0xFFFFFFFF, 0b00000000000000000111110000011111, 0b00000000000000000000000000000000, 0b00000000000000000111110000011111, 0b00000000000000000000000001110111, 0b00000000000000000000000000000000, 0b00000000000000000000000001110111 }, // RG
  {  0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0b00000000000000000000000000000000, 0b00000000000000000111110000011111, 0b00000000000000000111110000011111, 0b00000000000000000000000000000000, 0b00000000000000000000000001110111, 0b00000000000000000000000001110111 }, // LR
  {  0x00000000, 0xFFFFFFFF, 0x00000000, 0xFFFFFFFF, 0b00000000000000000111110000011111, 0b00000000000000000000000000000000, 0b00000000000000000111110000011111, 0b00000000000000000000000001110111, 0b00000000000000000000000000000000, 0b00000000000000000000000001110111 }, // LG
  { 0 }
};


UBYTE     PatternBlock    = 0;          // Block pattern update
UBYTE     PatternBits     = 20;         // Pattern bits
UBYTE     PatternBit      = 0;          // Pattern bit pointer
ULONG     ActPattern[NO_OF_LEDS];
ULONG     TmpPattern[NO_OF_LEDS];

static enum hrtimer_restart Device1TimerInterrupt1(struct hrtimer *pTimer)
{
  UBYTE   Tmp;

  if (PatternBlock)
  {
    PatternBlock  =  0;
  }
  else
  {
    for (Tmp = 0;Tmp < NO_OF_LEDS;Tmp++)
    {
      if (PatternBit == 0)
      {
        TmpPattern[Tmp]  =  ActPattern[Tmp];
      }
      if ((TmpPattern[Tmp] & 0x00000001))
      {
//        DIODEOn(Tmp);
      }
      else
      {
//        DIODEOff(Tmp);
      }
      TmpPattern[Tmp] >>=  1;
    }

    if (++PatternBit >= PatternBits)
    {
      PatternBit  =  0;
    }
  }

  // restart timer
  hrtimer_forward_now(pTimer,ktime_set(0,50000000));

  return (HRTIMER_RESTART);
}

static enum hrtimer_restart Device2TimerInterrupt1(struct hrtimer *pTimer)
{
  UWORD   Tmp;

  for(Tmp = 0;Tmp < NO_OF_BUTTONS;Tmp++)
  {
    if (BUTRead(Tmp))
    { // Button active

      procfs_read_buffer[Tmp]  =  "1";

    }
    else
    { // Button inactive

      procfs_read_buffer[Tmp]  =  "0";

    }
  }

  // restart timer
  hrtimer_forward_now(pTimer,ktime_set(0,10000000));

  return (HRTIMER_RESTART);
}


/*! \page UiDriver
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 *
 *  CP        C = Color, P = Pattern
 *
 *  C = 0     Off
 *  C = 1     Green
 *  C = 2     Red
 *  C = 3     Orange
 *
 *  P = 0     Off
 *  P = 1     On
 *  P = 2     50% (250mS on, 250mS off)
 *  P = 3     30% (150mS on, 50mS off, 150mS on, 650mS off)
 *
 *- 0pct      Set all LED = pct ["0".."100"]
 *- 1pct      Set LED 1 = pct
 *- 2pct      Set LED 2 = pct
 *- 3pct      Set LED 3 = pct
 *- 4pct      Set LED 4 = pct
 */
/*! \brief    Device1Write
 *
 *
 *
 */
// static ssize_t Device1Write(struct file *File,const char *Buffer,size_t Count,loff_t *Data)
// {
//   char    Buf[2];
//   UBYTE   No;
//   UBYTE   Tmp;
//   int     Lng     = 0;
// 
//   if (Count >= 2)
//   {
//     Lng     =  Count;
//     copy_from_user(Buf,Buffer,2);
//     No      =  Buf[0] - '0';
//     if ((No >= 0) && (No < LEDPATTERNS))
//     {
//       PatternBlock  =  1;
// 
//       PatternBits   =  20;
//       PatternBit    =  0;
// 
//       for (Tmp = 0;Tmp < NO_OF_LEDS;Tmp++)
//       {
//         ActPattern[Tmp]  =  LEDPATTERNDATA[Tmp][No];
//       }
// 
//       PatternBlock  =  0;
//     }
//   }
// 
//   return (Lng);
// }
// 
// 
// static ssize_t Device1Read(struct file *File,char *Buffer,size_t Count,loff_t *Offset)
// {
//   int     Lng = 0;
// 
//   Lng  =  snprintf(Buffer,Count,"V%c.%c0",HwId[0],HwId[1]);
// 
//   return (Lng);
// }
// 
// 
// #define     SHM_LENGTH    (sizeof(UiDefault))
// #define     NPAGES        ((SHM_LENGTH + PAGE_SIZE - 1) / PAGE_SIZE)
// static void *kmalloc_ptr;
// 
// static int Device1Mmap(struct file *filp, struct vm_area_struct *vma)
// {
//    int ret;
// 
//    ret = remap_pfn_range(vma,vma->vm_start,virt_to_phys((void*)((unsigned long)pUi)) >> PAGE_SHIFT,vma->vm_end-vma->vm_start,PAGE_SHARED);
// 
//    if (ret != 0)
//    {
//      ret  =  -EAGAIN;
//    }
// 
//    return (ret);
// }
// 
// 
// static    const struct file_operations Device1Entries =
// {
//   .owner        = THIS_MODULE,
//   .read         = Device1Read,
//   .write        = Device1Write,
//   .mmap         = Device1Mmap
// };
// 
// 
// static    struct miscdevice Device1 =
// {
//   MISC_DYNAMIC_MINOR,
//   DEVICE1_NAME,
//   &Device1Entries
// };
 
static int Device1Init(void)
{
  int     Result = -1;
  int     Tmp;
  int     i;
  UWORD   *pTmp;
// 
//   Result  =  misc_register(&Device1);
//   if (Result)
//   {
//     printk("  %s device register failed\n",DEVICE1_NAME);
//   }
//   else
//   {
//     // allocate kernel shared memory for ui button states (pUi)
//     if ((kmalloc_ptr = kmalloc((NPAGES + 2) * PAGE_SIZE, GFP_KERNEL)) != NULL)
//     {
//       pTmp = (UWORD*)((((unsigned long)kmalloc_ptr) + PAGE_SIZE - 1) & PAGE_MASK);
//       for (i = 0; i < NPAGES * PAGE_SIZE; i += PAGE_SIZE)
//       {
//         SetPageReserved(virt_to_page(((unsigned long)pTmp) + i));
//       }
//       pUi      =  (UI*)pTmp;
//       memset(pUi,0,sizeof(UI));
// 
//       for (Tmp = 0;Tmp < NO_OF_LEDS;Tmp++)
//       {
//         DIODEInit(Tmp);
//       }
//       for (Tmp = 0;Tmp < NO_OF_BUTTONS;Tmp++)
//       {
//         BUTFloat(Tmp);
//       }

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

// #ifdef DEBUG
//       printk("  %s device register succes\n",DEVICE1_NAME);
// #endif
//     }
//   }

  return (Result);
}


static void Device1Exit(void)
{
  int     Tmp;
  int     i;
  UWORD   *pTmp;

  hrtimer_cancel(&Device1Timer);
  hrtimer_cancel(&Device2Timer);

  for (Tmp = 0;Tmp < NO_OF_LEDS;Tmp++)
  {
    DIODEOff(Tmp);
  }

//   // free shared memory
//   pTmp   =  (UWORD*)pUi;
//   pUi    =  &UiDefault;
// 
//   for (i = 0; i < NPAGES * PAGE_SIZE; i+= PAGE_SIZE)
//   {
//     ClearPageReserved(virt_to_page(((unsigned long)pTmp) + i));
// #ifdef DEBUG
//     printk("  %s memory page %d unmapped\n",DEVICE1_NAME,i);
// #endif
//   }
//   kfree(kmalloc_ptr);
// 
//   misc_deregister(&Device1);
// #ifdef DEBUG
//   printk("  %s device unregistered\n",DEVICE1_NAME);
// #endif
}

// MODULE *********************************************************************
int 
procfile_read(char *buffer,
	      char **buffer_location,
	      off_t offset, int buffer_length, int *eof, void *data)
{
	int ret;
	
	printk(KERN_INFO "procfile_read (/proc/%s) called\n", PROCFS_NAME);
	
	if (offset > 0) {
		/* we have finished to read, return 0 */
		ret  = 0;
	} else {
		/* fill the buffer, return the buffer size */
                memcpy( procfs_read_buffer, "it's me, silly!", 16 );
                procfs_read_buffer_size = 16; 
		memcpy(buffer, procfs_read_buffer, procfs_read_buffer_size);
		ret = procfs_read_buffer_size;
	}

	return ret;
}

int
procfile_write(struct file *file, const char *buffer, unsigned long count,
		   void *data)
{
        unsigned int idx;

	/* get buffer size */
	procfs_write_buffer_size = count;
	if (procfs_write_buffer_size > PROCFS_MAX_SIZE ) {
		procfs_write_buffer_size = PROCFS_MAX_SIZE;
	}
	
	/* write data to the buffer */
	if ( copy_from_user(procfs_write_buffer, buffer, procfs_write_buffer_size) ) {
		return -EFAULT;
	}

        for( idx=0; idx<NO_OF_LEDS; ++idx ) {
            if( procfs_write_buffer[idx] == '0' ) {
                DIODEOn(idx);
            } else {
                DIODEOff(idx);
            } 
        }
 
	return procfs_write_buffer_size;
}

int
procfile_init( void )
{
	/* create the /proc file */
	procfs_file = create_proc_entry(PROCFS_NAME, 0, NULL);
	
	if (procfs_file == NULL) {
		remove_proc_entry(PROCFS_NAME, NULL);
		printk(KERN_ALERT "Error: Could not initialize /proc/%s\n",
			PROCFS_NAME);
		return -ENOMEM;
	}

	procfs_file->read_proc  = procfile_read;
	procfs_file->write_proc = procfile_write;
	procfs_file->mode 	  = S_IFREG | S_IRUGO | S_IWUGO;
	procfs_file->uid 	  = 0;
	procfs_file->gid 	  = 0;
	procfs_file->size 	  = PROCFS_MAX_SIZE;

	printk(KERN_INFO "/proc/%s created\n", PROCFS_NAME);	
	return 0;	/* everything is ok */
}

void
procfile_exit( void )
{
	remove_proc_entry(PROCFS_NAME, NULL);
	printk(KERN_INFO "/proc/%s removed\n", PROCFS_NAME);
}


#ifndef PCASM
module_param (HwId, charp, 0);
#endif

static int ModuleInit(void)
{
  Hw  =  HWID;

  if (Hw < PLATFORM_START)
  {
    Hw  =  PLATFORM_START;
  }
  if (Hw > PLATFORM_END)
  {
    Hw  =  PLATFORM_END;
  }

#ifdef DEBUG
  printk("%s init started\n",MODULE_NAME);
#endif

//  if (request_mem_region(DA8XX_GPIO_BASE,0xD8,MODULE_NAME) >= 0)
//  {
//    GpioBase  =  (void*)ioremap(DA8XX_GPIO_BASE,0xD8);
//    if (GpioBase != NULL)
//    {
#ifdef DEBUG
//      printk("%s gpio address mapped\n",MODULE_NAME);
#endif

      InitGpio();

      procfile_init();

      Device1Init();
//    }
//  }

  return (0);
}


static void ModuleExit(void)
{
#ifdef DEBUG
  printk("%s exit started\n",MODULE_NAME);
#endif

    Device1Exit();
    procfile_exit();
//  iounmap(GpioBase);

}
