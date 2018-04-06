/* LAB 3 PWM Controller kernel module reference implementation */
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/of_device.h>
#include <asm/io.h>
#include <linux/hrtimer.h>

// define driver/module name
#define DRIVER_NAME "axitimer"

// INSTANCE flags
#define INSTANCE_FLAG_RUNNING 0x01

// maximum number of instances
#define MAX_INSTANCE_NUMBER 16
#define TIMER_MAX_COUNT 0xFFFFFFFF

#define PWM_US_MULTIPLIER 100 

#define TCSR0_REG 0x00 
#define PERIOD_REG 0x04
#define TCSR1_REG 0x10
#define DUTY_REG 0x14

#define CTRL_REG_VAL 0x206

#define ALL_ON_MASK 0x400
#define OFF_MASK ~(0x80)


// define structure for device instance data
struct mz_motor_instance
{
  void __iomem *timer_regs; // memory-mapped registers
  void __iomem *gpio_regs;

  unsigned int flags; // instance flags
  struct cdev chr_dev;
  dev_t devno; // device number
  struct list_head inst_list;
  unsigned long period;
  unsigned long duty;
  struct hrtimer hrtimer_inst;
  ktime_t hrtimer_interval;
  wait_queue_head_t waitq;
};

struct mz_motor_driver
{
  u32 dev_major;
  dev_t first_devno;
  struct class* class;
  unsigned int instance_number;
  struct list_head instance_list;
};

// define structure for class data (common across all instances)
/**struct esl_axitimer_driver
{
  u32 dev_major; //device major number
  dev_t devt;

  unsigned int instance_number; // ever-increasing number for instances
};**/

// Instantiate class data 
static struct mz_motor_driver driver_data = {
  .instance_count = 0,
  .instance_list = LIST_HEAD_INIT(driver_data.instance_list),
};

// Utility method to read registers
static inline u32 reg_read(struct mz_motor_instance* inst, u32 reg)
{
  return ioread32(inst->timer_regs + reg);
}

// Utility method to write registers
static inline void reg_write(struct mz_motor_instance* inst, u32 reg,
                             u32 value)
{
  iowrite32(value, inst->timer_regs + reg);
}

static struct mz_motor_instance* inode_to_instance(struct inode* i)
{
  struct mz_motor_instance *inst_iter;
  unsigned int minor = iminor(i);

  list_for_each_entry(inst_iter, &driver_data.instance_list, inst_list)
    {
      if (MINOR(inst_iter->devno) == minor)
        {
          // found our corresponding instance
          return inst_iter;
        }
    }

  // not found
  return NULL;
}

static struct mz_motor_instance* file_to_instance(struct file* f)
{
  return inode_to_instance(f->f_path.dentry->d_inode);
}

static ssize_t mz_motor_write(struct file* f,
                               const char __user *buf, size_t len,
                               loff_t* offset)
{
  struct mz_motor_instance *inst = file_to_instance(f);
  unsigned int written = 0;
  unsigned int curr_int; 
  unsigned int i;


  if (!inst)
    {
      // instance not found
      return -ENOENT;
    }

    /**for(i = 0; i < (len / sizeof(unsigned int)); i++)
    {
      wait_event_interruptible(inst->waitq, !fifo_full(inst));

      written += copy_from_user((void*)&curr_int, buf + i*sizeof(unsigned int), sizeof(unsigned int));

      if((written % sizeof(unsigned int)) != 0)
      {
        return -EINVAL;
      }

      iowrite32(curr_int, inst->regs + WRITE_REG);
    }**/

    return written;
}

static ssize_t mz_motor_read(struct file* f,
                              const char __user *buf, size_t len,
                              loff_t* offset)
{
  struct mz_motor_instance *inst = file_to_instance(f);
  unsigned int read  0;

  return read;
}

struct file_operations mz_motor_fops = {
  .write = mz_motor_write,
  .read = mz_motor_read
};

static irqreturn_t mz_motor_encodera_gpio_irq_handler(int irq, void* dev_id)
{
  struct esl_audio_instance* inst = dev_id;

  // TODO handle interrupts
 
  return IRQ_HANDLED;
}

static irqreturn_t mz_motor_encoderb_gpio_irq_handler(int irq, void* dev_id)
{
  struct esl_audio_instance* inst = dev_id;

  // TODO handle interrupts
 
  return IRQ_HANDLED;
}

static enum hrtimer_restart esl_axitimer_animate(struct hrtimer* timer)
{
  // retrieve esl_axitimer_instance object from timer argument
  struct esl_axitimer_instance* inst = container_of(timer,
                                                    struct esl_axitimer_instance,
                                                    hrtimer_inst);
  printk("Timer Callback\n");
  
  reg_write(inst, DUTY_REG, ((inst->duty / 100) * inst->period * PWM_US_MULTIPLIER) - 2);

  // restart the timer!
  hrtimer_forward_now(timer, inst->hrtimer_interval);
  
  return HRTIMER_RESTART;
}

// initialize device
static void axitimer_initialize(struct esl_axitimer_instance* inst)
{
  inst->period = 0;
  inst->duty = 0;
  
  inst->hrtimer_interval = ktime_set(1, 0); // sec, ns

  reg_write(inst, TCSR0_REG, CTRL_REG_VAL);
  reg_write(inst, TCSR1_REG, CTRL_REG_VAL); 
}

// This function would be called whenever the status file is read from the sysfs directory structure corresponding to this kernel module the string idle gets returned when the pwm controller is idle
static ssize_t esl_axitimer_state_show(struct device* dev,
                                       struct device_attribute* attr,
                                       char* buf) {
  struct esl_axitimer_instance* inst = dev_get_drvdata(dev);

  if (inst->flags & INSTANCE_FLAG_RUNNING)
    {
      return sprintf(buf, "running\n");
    }

  return sprintf(buf, "idle\n");
}


// This function is included as a function pointer in the device_attribute struct esl_axitimer_state, which is then placed in the array of attributes called esl_axitimer_attrs this is then included in the attribute_group struct esl_axitimer_attr_group which is then placed in an array esl_axitimer_attr_groups. This array is finally passed to the call that instantiates the sysfs filesystem for this kernel module    
static ssize_t esl_axitimer_control_store(struct device* dev,
                                          struct device_attribute* attr,
                                          const char* buf, size_t count)
{
  struct esl_axitimer_instance* inst = dev_get_drvdata(dev);

  unsigned long duty = 0, period = 0;

  if (!strncmp(buf, "on", count))
    {
      hrtimer_cancel(&inst->hrtimer_inst);
      period = inst->period * PWM_US_MULTIPLIER - 2; 
      printk(KERN_INFO "Value written to register:%lu\n", period);
      reg_write(inst, PERIOD_REG, period);

      duty = ((inst->duty * inst->period * PWM_US_MULTIPLIER ) / 100) - 2; 
      printk(KERN_INFO "Value written to register:%lu\n", duty);
      reg_write(inst, DUTY_REG, duty);

      reg_write(inst, TCSR0_REG, reg_read(inst, TCSR0_REG) | ALL_ON_MASK); 
      inst->flags |= INSTANCE_FLAG_RUNNING;
    }
  else if (!strncmp(buf, "off", count))
    {
      hrtimer_cancel(&inst->hrtimer_inst);
      reg_write(inst, DUTY_REG, 0);
      reg_write(inst, PERIOD_REG, 0);
      reg_write(inst, TCSR0_REG, reg_read(inst, TCSR0_REG) & OFF_MASK);
      reg_write(inst, TCSR1_REG, reg_read(inst, TCSR1_REG) & OFF_MASK);
      inst->flags &= ~(INSTANCE_FLAG_RUNNING);
    }
  else
    {
      return -EINVAL;
    }

  return count;
}

 
 
//set_reg_mask(timer, 0x00, 0x400);

//  clear_reg_mask(timer, 0x00, 0x80);
//  clear_reg_mask(timer, 0x10, 0x80);

// Change into corresponding sysfs directory and test with echo 1000 > period
static ssize_t esl_axitimer_period_store(struct device* dev,
                                          struct device_attribute* attr,
                                          const char* buf, size_t count)
{
  struct esl_axitimer_instance* inst = dev_get_drvdata(dev);
  unsigned long period;
  int status = kstrtoul(buf, 10, &period);	
  
  if(status)
  {
    return -EINVAL;
  }

  printk(KERN_INFO "Desired period in microseconds:%lu\n", period);
  inst->period = period;
  

  return count;
}

// In corresponding sysfs directory test with cat period 
static ssize_t esl_axitimer_period_show(struct device* dev,
                                       struct device_attribute* attr,
                                       char* buf)
{
  struct esl_axitimer_instance* inst = dev_get_drvdata(dev);

  return sprintf(buf, "%lu\n", inst->period); 
}

// Change into corresponding sysfs directory and test with echo 1000 > period
static ssize_t esl_axitimer_duty_store(struct device* dev,
                                          struct device_attribute* attr,
                                          const char* buf, size_t count)
{
  struct esl_axitimer_instance* inst = dev_get_drvdata(dev);
  unsigned long duty;
  int status = kstrtoul(buf, 10, &duty);	
  
  if(status)
  {
    return -EINVAL;
  }

  printk(KERN_INFO "Desired duty cycle in microseconds:%lu\n", duty);
  inst->duty = duty;
  

  return count;
}

// In corresponding sysfs directory test with cat period 
static ssize_t esl_axitimer_duty_show(struct device* dev,
                                       struct device_attribute* attr,
                                       char* buf)
{
  struct esl_axitimer_instance* inst = dev_get_drvdata(dev);

  return sprintf(buf, "%lu\n", inst->duty); 
}

static ssize_t esl_axitimer_an_duration_store(struct device* dev,
                                              struct device_attribute* attr,
                                              const char* buf, size_t count)
{
  //struct esl_axitimer_instance* inst = dev_get_drvdata(dev);
  unsigned long an_duration;
  int status = kstrtoul(buf, 10, &an_duration);	
  
  if(status)
  {
    return -EINVAL;
  }

  printk(KERN_INFO "Desired animation duration in microseconds:%lu\n", an_duration);

  return count;
}

static ssize_t esl_axitimer_an_duration_show(struct device* dev,
                                       struct device_attribute* attr,
                                       char* buf)
{
  //struct esl_axitimer_instance* inst = dev_get_drvdata(dev);

  return sprintf(buf, "%u\n", 0); 
}

// define our own class for proper and automatic sysfs file group creation


// start / stop control
static struct device_attribute esl_axitimer_control = {
  .attr = {
    .name = "control",
    .mode = S_IWUSR, // write only
  },
  .store = esl_axitimer_control_store,
};

// current state
static struct device_attribute esl_axitimer_state = {
  .attr = {
    .name = "status",
    .mode = S_IRUGO, 
  },
  .show = esl_axitimer_state_show,
};

static struct device_attribute esl_axitimer_period = {
  .attr = {
    .name = "period",
    .mode = S_IWUSR | S_IRUGO,
  },
  .store = esl_axitimer_period_store,
  .show = esl_axitimer_period_show,
};

static struct device_attribute esl_axitimer_duty = {
  .attr = {
    .name = "duty",
    .mode = S_IWUSR | S_IRUGO,
  },
  .store = esl_axitimer_duty_store,
  .show = esl_axitimer_duty_show,
};

static struct device_attribute esl_axitimer_an_duration = {
  .attr = {
    .name = "an_duration",
    .mode = S_IWUSR | S_IRUGO,
  },
  .store = esl_axitimer_an_duration_store,
  .show = esl_axitimer_an_duration_show,
};

// array of attributes
static struct attribute* esl_axitimer_attrs[] = {
  &esl_axitimer_control.attr,
  &esl_axitimer_state.attr,
  &esl_axitimer_period.attr,
  &esl_axitimer_duty.attr,
  &esl_axitimer_an_duration.attr,
  NULL
};

// Group all attributes to "control" folder, 
// this creates /sys/class/axitimer/axitimerX/control
static struct attribute_group esl_axitimer_attr_group = {
  .name = "control", // control folder
  .attrs = esl_axitimer_attrs,
};

// array of attribute groups (sysfs files and folders)
static const struct attribute_group* esl_axitimer_attr_groups[] = {
  &esl_axitimer_attr_group, // the "control" folder
  NULL
};

// array of class-wide attributes (none)
static struct class_attribute esl_axitimer_class_attrs[] = {
  __ATTR_NULL, // no class-wide attributes
};

// define the class, creates /sys/class/axitimer
static struct class esl_axitimer_class = {
  .name = "axitimer",
  .owner = THIS_MODULE,
  .class_attrs = esl_axitimer_class_attrs,
};


// probe, or add an instance
static int esl_axitimer_probe(struct platform_device* pdev)
{
  struct esl_axitimer_instance* inst = NULL; // new instance
  struct resource* res; // for reading resources from device tree
  struct device* dev;

  const void* property;

  // make new device; we need device a number (major, minor)
  // use instance count as the minor, which we increase each time
  // major has already been allocated in init function
  // to take a look at majors, do cat /proc/devices
  dev_t devno = MKDEV(driver_data.dev_major, driver_data.instance_number);

  // verify if timer is marked as PWM timer, get resource from device tree

  property = of_get_property(pdev->dev.of_node, "esl,pwm-timer", NULL);
  if (!property)
    {
      // timer is not marked for PWM, fail probe
      printk(KERN_INFO "kPWM: %s not marked as PWM timer.\n",
             pdev->name);
      return -EPERM;
    }

  //allocate new instance. explanation for this:
  // 1. we use devm_kzalloc instead of kzalloc because it has the nice property
  //    of freeing unused resources automatically when the platform driver is
  //    removed (devm = dev managed and attached to pdev->dev so when it is
  //    destroyed, data is garbage collected)
  // 2. GFP_KERNEL means we are allocating KERNEL memory, otherwise it should be
  //    GFP_USER
  inst = devm_kzalloc(&pdev->dev, sizeof(struct esl_axitimer_instance),
                      GFP_KERNEL);

  if (!inst)
    {
      // ran out of memory
      return -ENOMEM;
    }

  //This function returns a struct with information about a given resource including addresses and size of the memory space associated with that resource here what we're looking for the memory/registers associated with the timer peripheral that we're trying to instantiate  
  res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (IS_ERR(res))
    {
      return PTR_ERR(res);
    }

  //This function checks that the memory we're trying to get is correct and mappable and configures it appropriately based upon whether it can be cached or not 
  inst->regs = devm_ioremap_resource(&pdev->dev, res);
  if (IS_ERR(inst->regs))
    {
      // error mapping
      return PTR_ERR(inst->regs);
    }

  // set platform driver data (our instance struct)
  platform_set_drvdata(pdev, inst);

  // create the character device, this will create:
  // 1. /dev/axitimerX
  // 2. all the sysfs stuff
  dev = device_create_with_groups(&esl_axitimer_class, &pdev->dev,
                                  devno, inst, esl_axitimer_attr_groups,
                                  "axitimer%d", driver_data.instance_number);

  if (IS_ERR(dev))
    {
      return PTR_ERR(dev);
    }

  // set dev number
  inst->devno = devno;

  // increment instance counter
  driver_data.instance_number++;

  // initialize timer
  axitimer_initialize(inst);

  printk(KERN_INFO "probed kPWM with timer %s\n", pdev->name);

  hrtimer_init(&inst->hrtimer_inst, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  inst->hrtimer_inst.function = &esl_axitimer_animate;

  return 0;
}

// remove an instance
static int esl_axitimer_remove(struct platform_device* pdev)
{
  struct esl_axitimer_instance* inst = platform_get_drvdata(pdev);

  // cleanup and remove
  device_destroy(&esl_axitimer_class, inst->devno);

  return 0;
}

// matching table
// list of strings that indicate what platform
// device is compatible with
static struct of_device_id esl_axitimer_of_ids[] = {
  { .compatible = "xlnx,xps-timer-1.00.a" },
  { },
};

// platform driver definition
static struct platform_driver esl_axitimer_driver = {
  .probe = esl_axitimer_probe,
  .remove = esl_axitimer_remove,
  .driver = {
    .name = DRIVER_NAME,
    .of_match_table = of_match_ptr(esl_axitimer_of_ids),
  },
};

// module initialization
static int __init esl_axitimer_init(void)
{
  int err;
  // allocate character device region, /dev/axitimerX
  err = alloc_chrdev_region(&driver_data.devt, 0, MAX_INSTANCE_NUMBER,
                            DRIVER_NAME);
  if (err)
    {
      return err;
    }

  // initialize our data
  driver_data.instance_number = 0;
  driver_data.dev_major = MAJOR(driver_data.devt);

  //register class
  class_register(&esl_axitimer_class);

  //register platform driver
  platform_driver_register(&esl_axitimer_driver);

  return 0;
}

// module removal
static void __exit esl_axitimer_exit(void)
{
  //cleanup
  platform_driver_unregister(&esl_axitimer_driver);
  class_unregister(&esl_axitimer_class);
  unregister_chrdev_region(driver_data.devt, MAX_INSTANCE_NUMBER);
}

module_init(esl_axitimer_init);
module_exit(esl_axitimer_exit);

MODULE_DESCRIPTION("AXI Timer driver");
MODULE_LICENSE("GPL");
