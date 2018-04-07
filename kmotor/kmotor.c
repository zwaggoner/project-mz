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

#define MODULE_NAME "mz-motor"

static dev_t motor_devno;
static struct cdev motor_dev;

static ssize_t motor_sample_time_store(struct device* dev,
                                          struct device_attribute* attr,
                                          const char* buf, size_t count)
{
  return count;
}

static ssize_t motor_sample_time_show(struct device* dev,
                                       struct device_attribute* attr,
                                       char* buf)
{
  return sprintf(buf, "%u\n", 0); 
}

static struct device_attribute motor_sample_time = {
  .attr = {
    .name = "sample_time",
    .mode = S_IWUSR | S_IRUGO,
  },
  .store = motor_sample_time_store,
  .show = motor_sample_time_show,
};

static struct attribute* motor_attrs[] = {
  &motor_sample_time.attr,
  NULL
};

// Group all attributes to "control" folder, 
// this creates /sys/class/axitimer/axitimerX/control
static struct attribute_group motor_attr_group = {
  .name = "control", // control folder
  .attrs = motor_attrs,
};

static const struct attribute_group* motor_attr_groups[] = {
  &motor_attr_group, // the "control" folder
  NULL
};

// array of class-wide attributes (none)
static struct class_attribute motor_class_attrs[] = {
  __ATTR_NULL, // no class-wide attributes
};

static struct class motor_class = {
  .name = "mz-motor",
  .owner = THIS_MODULE,
  .class_attrs = motor_class_attrs,
};

static ssize_t motor_read(struct file* f,
                         char __user *buf,
                         size_t count,
                         loff_t* offset)
{
	return count;
}

static ssize_t motor_write(struct file* f,
                          const char __user *buf,
                          size_t count,
                          loff_t* offset)
{
  return count;
}

struct file_operations motor_fops = {
  .read = motor_read,
  .write = motor_write,
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

  // Initializes both the cdev and file_operations structure for our character device 
  cdev_init(&motor_dev, &motor_fops);

  // Let the kernel know about the device, given the major device number and number of devices to set up 
  err = cdev_add(&motor_dev, motor_devno, 1);
  if (err)
    {
      unregister_chrdev_region(motor_devno, 1);
      return err;
    }

 class_register(&motor_class);

 // Creates the device file for this in the /dev tree
 dev = device_create_with_groups(&motor_class, NULL,
                                  motor_devno, NULL, motor_attr_groups,
                                  "mz-motor0");

  // TODO add tracing for successful char def creation
  if(IS_ERR(dev))
  {
    cdev_del(&motor_dev);
    device_destroy(&motor_class, motor_devno);

    // delete class
    class_destroy(&motor_class);

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
  // delete char device and driver
  cdev_del(&motor_dev);
  device_destroy(&motor_class, motor_devno);

  // delete class
  class_destroy(&motor_class);

  // unregister chrdev region
  unregister_chrdev_region(motor_devno, 1);

  //kfifo_free(&print_fifo);
}

module_init(motor_init);
module_exit(motor_exit);

MODULE_DESCRIPTION("Motor Driver");
MODULE_LICENSE("GPL");
