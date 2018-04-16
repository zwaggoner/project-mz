#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/kfifo.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include "../fixed/fixed.h"
#include "kmotor.h"

#define MODULE_NAME "mz-motor"

// INSTANCE flags
#define INSTANCE_FLAG_RUNNING 0x01
#define ANIMATION_FLAG_RUNNING 0x02

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

#define GPIO_BASE 865
#define GPIO_BASE_ADDR 0x411F0000
#define GLOBAL_INT_OFFSET (u32)0x011C
#define IP_INT_OFFSET (u32)0x0128
#define IP_ISR_OFFSET (u32)0x0120

#define CTRL_1_GPIO GPIO_BASE
#define CTRL_2_GPIO (GPIO_BASE + 1)
#define ENCODER_A_GPIO (GPIO_BASE + 2)
#define ENCODER_B_GPIO (GPIO_BASE + 3)

static dev_t motor_devno;
static struct class* motor_class;
static struct cdev motor_dev; 

static fixed32_t voltage;
static fixed32_t max_voltage;
static unsigned long pwm_period;
static u8 running;

static u8 last_encoder_state;
static fixed32_t angle;
static fixed32_t angle_increment;

static int gpio_irqnum;
static void __iomem *gpio_regs;

static void __iomem *timer_regs;

static inline u32 reg_read(void __iomem* regs, u32 reg)
{
  return ioread32(regs + reg);
}

// Utility method to write registers
static inline void reg_write(void __iomem* regs, u32 reg,
                             u32 value)
{
  iowrite32(value, regs + reg);
}

static long get_fixed_from_arg(fixed32_t* dest, unsigned long* arg)
{
  float temp = *((float*)arg);
  fixed32_t temp_fx;
  FixedPointError err;
  long ret = 0;
 
  temp_fx = float_to_fixed(temp, &err);

  if(err != NONE)
  {
    ret =  -EINVAL;
  }

  *dest = temp_fx;

  return ret;
}

static long return_float_from_fixed(fixed32_t* src)
{
  float temp = fixed_to_float(*src);

  return *((long*)&temp);
}

static long motor_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
  long ret = 0;

  switch(cmd)
  {
    case MOTOR_TEST:
      printk(KERN_INFO "cmd is: %u\n", cmd);
      printk(KERN_INFO "arg is: %lu\n", arg);
      break;
    case MOTOR_SET_VOLTAGE:
      if(running)
      {
        u32 duty;
        FixedPointError err;
        s8 prev_sign;
        s8 curr_sign;
        
        prev_sign = (voltage) ? voltage / abs(voltage) : 1; 

        ret = get_fixed_from_arg(&voltage, &arg);

        curr_sign = (voltage) ? voltage / abs(voltage) : 1;

        if(prev_sign != curr_sign)
        {
          reg_write(timer_regs, DUTY_REG, 0);

          switch(curr_sign)
          {
            case 1:
              gpio_direction_output(CTRL_1_GPIO, 1);
              gpio_direction_output(CTRL_2_GPIO, 0);
              break;
            case -1:
              gpio_direction_output(CTRL_1_GPIO, 0);
              gpio_direction_output(CTRL_2_GPIO, 1);
              break;
          }
        }
        
        duty = (voltage) ? (fixed_divide(curr_sign * voltage, max_voltage, &err) * pwm_period) >> POINT : 0;
        reg_write(timer_regs, DUTY_REG, (duty) ? (duty * PWM_US_MULTIPLIER - 2) : 0);
      }
      else
      {
        ret = get_fixed_from_arg(&voltage, &arg);
      }

      break;
    case MOTOR_GET_VOLTAGE:
      ret  = return_float_from_fixed(&voltage);    
      break;
    case MOTOR_SET_MAX_VOLTAGE:
      if(running)
      {
        printk(KERN_WARNING "Cannot set new Motor max voltage while running\n");
        ret = -EINVAL;
      }
      else
      {
        ret = get_fixed_from_arg(&max_voltage, &arg);
      }
      break;
    case MOTOR_GET_MAX_VOLTAGE:
      ret = return_float_from_fixed(&max_voltage);
      break;
    case MOTOR_SET_PWM_PERIOD:
      if(running)
      {
        printk(KERN_WARNING "Cannot set new Motor PWM Period while running\n");
        ret = -EINVAL; 
      }
      else
      {
        pwm_period = arg;
      }
      break;
    case MOTOR_GET_PWM_PERIOD:
      ret = pwm_period;
      break;
    case MOTOR_SET_STATE:
      running = (arg) ? 1 : 0;

      if(running)
      {
        u32 duty; 
        FixedPointError err;
        s8 curr_sign;

        curr_sign = (voltage) ? (voltage / abs(voltage)) : 1;
      
        switch(curr_sign)
        {
          case 1:
            gpio_direction_output(CTRL_1_GPIO, 1);
            gpio_direction_output(CTRL_2_GPIO, 0);
            break;
          case -1:
            gpio_direction_output(CTRL_1_GPIO, 0);
            gpio_direction_output(CTRL_2_GPIO, 1);
            break;
        } 

        reg_write(timer_regs, PERIOD_REG, (pwm_period) ? (pwm_period * PWM_US_MULTIPLIER - 2) : 0);

        duty = (voltage) ? (fixed_divide(curr_sign * voltage, max_voltage, &err) * pwm_period) >> POINT : 0;
        reg_write(timer_regs, DUTY_REG, (duty) ? (duty * PWM_US_MULTIPLIER - 2) : 0);       

        reg_write(timer_regs, TCSR0_REG, reg_read(timer_regs, TCSR0_REG) | ALL_ON_MASK);
      }
      else
      {
        gpio_direction_output(CTRL_1_GPIO, 0);
        gpio_direction_output(CTRL_2_GPIO, 0);
        reg_write(timer_regs, DUTY_REG, 0);
        reg_write(timer_regs, PERIOD_REG, 0);
        reg_write(timer_regs, TCSR0_REG, reg_read(timer_regs, TCSR0_REG) & OFF_MASK);
        reg_write(timer_regs, TCSR1_REG, reg_read(timer_regs, TCSR1_REG) & OFF_MASK);
      }
      break;
    case MOTOR_GET_STATE:
      ret = running;
      break;
    case MOTOR_GET_POSITION:
      ret =  return_float_from_fixed(&angle);
      break;
    default:
      printk(KERN_WARNING "Unknown ioctl command in Motor: %u\n", cmd);
      ret = -1;
      break; 
  }

  return ret;
}

static irqreturn_t encoder_irq_handler(int irq, void* dev_id)
{
  u8 curr_encoder_state = 0;

  curr_encoder_state |= gpio_get_value(ENCODER_A_GPIO);
  curr_encoder_state |= gpio_get_value(ENCODER_B_GPIO) << 1;

  switch(last_encoder_state)
  {
    case 0b00:
      switch(curr_encoder_state)
      {
        case 0b10:
          angle -= angle_increment;
          break;
        case 0b01:
          angle += angle_increment;
          break;
      } 
      break;
    case 0b01:
      switch(curr_encoder_state)
      {
        case 0b00:
          angle -= angle_increment;
          break;
        case 0b11:
          angle += angle_increment;
          break;
      }
      break;
    case 0b11:
      switch(curr_encoder_state)
      {
        case 0b01:
          angle -= angle_increment;
          break;
        case 0b10:
          angle += angle_increment;
          break;
      }
      break;
    case 0b10:
      switch(curr_encoder_state)
      {
        case 0b00:
          angle += angle_increment;
          break;
        case 0b11:
          angle -= angle_increment;
          break;
      }
      break;
  }
  
  last_encoder_state = curr_encoder_state;
   
  reg_write(gpio_regs, IP_ISR_OFFSET, 1);
  
  return IRQ_HANDLED;
}

struct file_operations motor_fops = {
  .owner = THIS_MODULE,
  .unlocked_ioctl = motor_ioctl,
};

static int __init motor_init(void)
{
  FixedPointError fx_err;
  int err;
  struct device *dev = NULL;
  struct device_node *dev_node = NULL;
  const void* property;
  struct device* gpio_dev = NULL;
  struct gpio_desc* gdesc;
  struct gpio_chip* chip;
  struct platform_device* gpio_pdev;
  struct resource* gpio_res;

  voltage = 0;
  max_voltage = float_to_fixed(12.0f, &fx_err);
  pwm_period = 1000;
  running = 0;
  last_encoder_state = 0;
  angle = 0;
  angle_increment = float_to_fixed(1.40845070423f, &fx_err);

  dev_node = of_find_compatible_node(NULL, NULL, "xlnx,xps-timer-1.00.a");

  while(dev_node)
  {
    property = of_get_property(dev_node, "esl,pwm-timer", NULL);

    if(property)
    { 
      printk(KERN_INFO "Compatible driver %s found\n", dev_node->full_name);
      break;
    }
    else
    {
      printk(KERN_INFO "Incompatible driver %s found\n", dev_node->full_name);
    }

    dev_node = of_find_compatible_node(dev_node, NULL, "xlnx,xps-timer-1.00.a");
  }
 
  if(dev_node)
  {
    timer_regs = of_iomap(dev_node, 0);

    if(!timer_regs)
    {
      printk(KERN_INFO "Could not secure registers\n");

      return -ENOENT;
    }
  }
  else
  {
    printk(KERN_INFO "No Compatible driver found\n");

    return -ENOENT;
  }

  gdesc = gpio_to_desc(GPIO_BASE);
  
  if(!gdesc)
  {
    printk(KERN_WARNING "Error getting GPIO descriptor\n");
    
    return -ENODEV; 
  }

  chip = gpiod_to_chip(gdesc); 

  if(!chip)
  {
    printk(KERN_WARNING "Error getting GPIO chip\n");
    
    return -ENODEV;
  }

  gpio_dev = chip->parent;
  gpio_pdev = to_platform_device(gpio_dev);

  gpio_res =  platform_get_resource(gpio_pdev, IORESOURCE_IRQ, 0);

  gpio_irqnum = gpio_res->start;
  request_threaded_irq(gpio_irqnum, encoder_irq_handler, NULL, IRQF_TRIGGER_RISING, "Encoder Interrupt", NULL);

  gpio_regs = ioremap(GPIO_BASE_ADDR, IP_INT_OFFSET);

  reg_write(timer_regs, TCSR0_REG, CTRL_REG_VAL);
  reg_write(timer_regs, TCSR1_REG, CTRL_REG_VAL);

  reg_write(gpio_regs, GLOBAL_INT_OFFSET, (u32)(1 << 31));
  reg_write(gpio_regs, IP_INT_OFFSET, (u32)1);

  gpio_direction_output(CTRL_1_GPIO, 0);
  gpio_direction_output(CTRL_2_GPIO, 0);

  gpio_direction_input(ENCODER_A_GPIO);
  gpio_direction_input(ENCODER_B_GPIO);

  last_encoder_state |= gpio_get_value(ENCODER_A_GPIO);
  last_encoder_state |= gpio_get_value(ENCODER_B_GPIO) << 1;

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
    return -ENOENT;
  }
  
  printk(KERN_INFO "Motor device driver Major:%i, Minor:%i\n", MAJOR(motor_devno), MINOR(motor_devno));

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
