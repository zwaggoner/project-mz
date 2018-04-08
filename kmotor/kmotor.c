#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/kfifo.h>
#include <linux/mutex.h>
#include "../fixed/fixed.h"
#include "kmotor.h"

#define MODULE_NAME "mz-motor"

static dev_t motor_devno;
static unsigned long pwm_period;
static struct class* motor_class;
static struct cdev motor_dev; 
static fixed32_t voltage;
static u8 running;

static long motor_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
  switch(cmd)
  {
    case MOTOR_TEST:
      printk(KERN_INFO "cmd is: %u\n", cmd);
      printk(KERN_INFO "arg is: %lu\n", arg);
      return 0;
    case MOTOR_SET_VOLTAGE:
    {
      float voltage_temp = *((float*)&arg);
      fixed32_t voltage_temp_fx;
      FixedPointError err;

      voltage_temp_fx = float_to_fixed(voltage_temp, &err);

      if(err != NONE)
      {
        return -EINVAL;
      }

      voltage = voltage_temp_fx; 

      printk(KERN_INFO "Voltage is: %x\n", voltage);
      
      return 0;
    }
    case MOTOR_GET_VOLTAGE:
    {
      float voltage_temp = fixed_to_float(voltage);

      return *((u32*)&voltage_temp);    
    } 
    default:
      printk(KERN_INFO "Unknown ioctl command: %u\n", cmd);
      return -1; 
  }

  return -1;
}

struct file_operations motor_fops = {
  .owner = THIS_MODULE,
  .unlocked_ioctl = motor_ioctl,
};

static int __init motor_init(void)
{
  int err;
  struct device *dev = NULL;

  //INIT_KFIFO(print_fifo);
  // This function call requests a dynamically allocated device number from the kernel
  err = alloc_chrdev_region(&motor_devno, 0, 1, MODULE_NAME);
  if (err)
    {
      return err;
    }

  cdev_init(&motor_dev, &motor_fops);
  err = cdev_add(&motor_dev, motor_devno, 1);
  if (err)
    {
      unregister_chrdev_region(motor_devno, 1);
      return err;
    }

  motor_class = class_create(THIS_MODULE, MODULE_NAME);
  if (IS_ERR(motor_class))
    {
      return -ENOENT;
    }
  
  // Creates the device file for this in the /dev tree
  //
  //
  dev = device_create(motor_class,
                NULL,
                motor_devno,
                NULL,
                "mz-motor0");

  // TODO add tracing for successful char def creation
  if(IS_ERR(dev))
  {
    cdev_del(&motor_dev);
    device_destroy(motor_class, motor_devno);

    // delete class
    class_destroy(motor_class);

    // unregister chrdev region
    unregister_chrdev_region(motor_devno, 1); 
  }
  else
  {
    printk(KERN_INFO "Motor device driver Major:%i, Minor:%i\n", MAJOR(motor_devno), MINOR(motor_devno)); }
  
  return 0;
}


static void __exit motor_exit(void)
{
  cdev_del(&motor_dev);
  // delete char device and driver
  device_destroy(motor_class, motor_devno);

  // delete class
  class_destroy(motor_class);

  // unregister chrdev region
  unregister_chrdev_region(motor_devno, 1);

  //kfifo_free(&print_fifo);
}

module_init(motor_init);
module_exit(motor_exit);

MODULE_DESCRIPTION("Motor Driver");
MODULE_LICENSE("GPL");
