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

#define CTRL_1_GPIO (GPIO_BASE + 10)
#define CTRL_2_GPIO (GPIO_BASE + 11)
#define ENCODER_A_GPIO (GPIO_BASE + 0)
#define ENCODER_B_GPIO (GPIO_BASE + 1)


//Motor device device file related data
static dev_t motor_devno;
static struct class* motor_class;
static struct cdev motor_dev; 


// Motor control parameters
static fixed32_t voltage;
static fixed32_t max_voltage;
static unsigned long pwm_period;
static u8 running;


// Encoder related parameters
static u8 last_encoder_state;
static fixed32_t angle;
static fixed32_t angle_increment;


// GPIO interrupt and other resources
static int gpio_irqnum;
static void __iomem *gpio_regs;
static struct resource* gpio_res;


// Timer resources
static struct resource timer_res;
static void __iomem *timer_regs;

/**
  * Utility function for reading from a memory location  
  * @param regs The base address from where to read from
  * @param reg  The offset from the base to read from
  * @return The value in the memory location 
  */
static inline u32 reg_read(void __iomem* regs, u32 reg)
{
  return ioread32(regs + reg);
}


/**
 * Utility function to write to a memory location 
 * @param regs The base address from where to write
 * @param reg The offset from the base addres to write to 
 * @param value The value to write into the memory location
 */
static inline void reg_write(void __iomem* regs, u32 reg,
                             u32 value)
{
  iowrite32(value, regs + reg);
}

/**
 * Utility function to get the fixed point representation of the floating point number passed
 * into the ioctl function. 
 * @param dest Pointer to the fixed point number to store the result in
 * @param arg Pointer to the input float as an unsigned long
 * @return The status of the operation
 */ 
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

/**
 * Utility function to return the floating point number representation of a fixed point number
 * from the ioctl function.
 * @param src Pointer to the source fixed point number
 * @return The status of the operation
 */
static long return_float_from_fixed(fixed32_t* src)
{
  float temp = fixed_to_float(*src);

  return *((long*)&temp);
}

/**
 * ioctl function for controlling the motor driver
 */
static long motor_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
  long ret = 0;

  switch(cmd)
  {
    case MOTOR_TEST:
      // Test case for ioctl commands
      printk(KERN_INFO "cmd is: %u\n", cmd);
      printk(KERN_INFO "arg is: %lu\n", arg);
      break;
    case MOTOR_SET_VOLTAGE:
      // Set the motor voltage
      if(running)
      {
        // If actually running set the PWM output
        u32 duty;
        FixedPointError err;
        s8 prev_sign;
        s8 curr_sign;
       
        // Check if there is a sign change, necessitating a direction change 
        prev_sign = (voltage) ? voltage / abs(voltage) : 1; 

        ret = get_fixed_from_arg(&voltage, &arg);

        curr_sign = (voltage) ? voltage / abs(voltage) : 1;

        if(prev_sign != curr_sign)
        {
          // If there is a direction change, temporarily stop the motor instead of instantaneousl commanding the right voltage in the wrong direction 
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

	// Calculate the duty 	
        duty = (voltage) ? (fixed_divide(curr_sign * voltage, max_voltage, &err) * pwm_period) >> POINT : 0;
	// Write the duty into the register
        reg_write(timer_regs, DUTY_REG, (duty) ? (duty * PWM_US_MULTIPLIER - 2 - 1) : 0);
      }
      else
      {
	// If not running just store for later use
        ret = get_fixed_from_arg(&voltage, &arg);
      }

      break;
    case MOTOR_GET_VOLTAGE:
      // Return the set voltage
      ret  = return_float_from_fixed(&voltage);    
      break;
    case MOTOR_SET_MAX_VOLTAGE:
      // Set new maximum voltage as long as motor isn't already running 
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
      // Inform user of maximum voltage
      ret = return_float_from_fixed(&max_voltage);
      break;
    case MOTOR_SET_PWM_PERIOD:
      // Set new PWM period so long as motor isn't currently running (units in microseconds)
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
      // Return the PWM period
      ret = pwm_period;
      break;
    case MOTOR_SET_STATE:
      //Set the state of the motor
      running = (arg) ? 1 : 0;

      if(running)
      {
	// If state now running, initialize like the set voltage and enable the timer
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
        reg_write(timer_regs, DUTY_REG, (duty) ? (duty * PWM_US_MULTIPLIER - 2 - 1) : 0);       

        reg_write(timer_regs, TCSR0_REG, reg_read(timer_regs, TCSR0_REG) | ALL_ON_MASK);
      }
      else
      {
        // If turned off, shut off the motor and turn off the timer
        gpio_direction_output(CTRL_1_GPIO, 0);
        gpio_direction_output(CTRL_2_GPIO, 0);
        reg_write(timer_regs, DUTY_REG, 0);
        reg_write(timer_regs, PERIOD_REG, 0);
        reg_write(timer_regs, TCSR0_REG, reg_read(timer_regs, TCSR0_REG) & OFF_MASK);
        reg_write(timer_regs, TCSR1_REG, reg_read(timer_regs, TCSR1_REG) & OFF_MASK);
      }
      break;
    case MOTOR_GET_STATE:
      // Return current motor state
      ret = running;
      break;
    case MOTOR_GET_POSITION:
      // Return current morot position
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

  // Encode current motor grey code
  curr_encoder_state |= gpio_get_value(ENCODER_A_GPIO);
  curr_encoder_state |= gpio_get_value(ENCODER_B_GPIO) << 1;

  // Check for actual change in encoder state
  if(curr_encoder_state != last_encoder_state)
  {
    // Look at current encoder state, and check for valid next states, and increment angle accordingly 
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

/** Removed roll wrap
    if(angle > (360 << POINT))
    {
      angle -= (360 << POINT);
    }

    if(angle < 0)
    {
      angle += (360 << POINT);
    }
**/

    // Record last encoder state
    last_encoder_state = curr_encoder_state;
  }
  
  // Acnkowledge interrupt 
  reg_write(gpio_regs, IP_ISR_OFFSET, 1);//reg_read(gpio_regs, IP_ISR_OFFSET));
  
  return IRQ_HANDLED;
}

// Register ioctl function with character device file operations
struct file_operations motor_fops = {
  .owner = THIS_MODULE,
  .unlocked_ioctl = motor_ioctl,
};

/**
 * Motor driver device initialization funciton
 */
static int __init motor_init(void)
{
  FixedPointError fx_err;
  int err;
  int rc = 0;
  struct device *dev = NULL;
  struct device_node *dev_node = NULL;
  const void* property;
  struct device* gpio_dev = NULL;
  struct gpio_desc* gdesc;
  struct gpio_chip* chip;
  struct platform_device* gpio_pdev;

  // Variable initialization 
  voltage = 0;
  max_voltage = float_to_fixed(12.0f, &fx_err);
  pwm_period = 1000;
  running = 0;
  last_encoder_state = 0;
  angle = 0;
  angle_increment = float_to_fixed(0.35211267605f, &fx_err);

  // Find first compatible timer
  dev_node = of_find_compatible_node(NULL, NULL, "xlnx,xps-timer-1.00.a");

  while(dev_node)
  {
    // Cycle through the nodes until we find the esl pwm timer
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

  // If we found a compatible device, configure 
  if(dev_node)
  {
    // Get resources of the timer device found, and map the registers into kernel 
    rc = of_address_to_resource(dev_node, 0, &timer_res);
    
    if(rc)
    {
      printk(KERN_WARNING "Failed to get timer resource\n");
      return -ENOENT;
    }
  
    if(!request_mem_region(timer_res.start, resource_size(&timer_res), timer_res.name))
    {
      printk(KERN_WARNING "Failed to request timer mem region\n");
      return -ENOENT;
    }
 
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

  // GPIO driver initialization 
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


  // After all GPIO driver information is found, retrieve the interrupt resource and configure
  gpio_res =  platform_get_resource(gpio_pdev, IORESOURCE_IRQ, 0);

  gpio_irqnum = gpio_res->start;
  request_threaded_irq(gpio_irqnum, encoder_irq_handler, NULL, IRQF_TRIGGER_RISING, "Encoder Interrupt", NULL);

  // Now secure the GPIO registers
  gpio_res = platform_get_resource(gpio_pdev, IORESOURCE_MEM, 0);

  if(!request_mem_region(gpio_res->start, resource_size(gpio_res), gpio_res->name))
  {
    printk(KERN_WARNING "Error requesting GPIO mem region\n");
    return -ENODEV;
  }

  gpio_regs = ioremap(gpio_res->start, resource_size(gpio_res));


  // Initialize timer registers and GPIO interrupt registers
  reg_write(timer_regs, TCSR0_REG, CTRL_REG_VAL);
  reg_write(timer_regs, TCSR1_REG, CTRL_REG_VAL);

  reg_write(gpio_regs, GLOBAL_INT_OFFSET, (u32)(1 << 31));
  reg_write(gpio_regs, IP_INT_OFFSET, (u32)1);

  // Configure GPIOs
  gpio_direction_output(CTRL_1_GPIO, 0);
  gpio_direction_output(CTRL_2_GPIO, 0);

  gpio_direction_input(ENCODER_A_GPIO);
  gpio_direction_input(ENCODER_B_GPIO);

  // Record initial encoder state
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

  // Check for successful char def creation
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
  // Reset all peripherals
  gpio_direction_output(CTRL_1_GPIO, 0);
  gpio_direction_output(CTRL_2_GPIO, 0);
  reg_write(timer_regs, DUTY_REG, 0);
  reg_write(timer_regs, PERIOD_REG, 0);
  reg_write(timer_regs, TCSR0_REG, reg_read(timer_regs, TCSR0_REG) & OFF_MASK);
  reg_write(timer_regs, TCSR1_REG, reg_read(timer_regs, TCSR1_REG) & OFF_MASK);

  // Unmap all the registers
  iounmap(timer_regs);
  iounmap(gpio_regs);

  // Release the resources
  release_mem_region(gpio_res->start, resource_size(gpio_res));
  release_mem_region(timer_res.start, resource_size(&timer_res));

  // Free the GPIO interrupt
  free_irq(gpio_irqnum, NULL);

  cdev_del(&motor_dev);
  // delete char device and driver
  device_destroy(motor_class, motor_devno);

  // delete class
  class_destroy(motor_class);

  // unregister chrdev region
  unregister_chrdev_region(motor_devno, 1);
}

module_init(motor_init);
module_exit(motor_exit);

MODULE_DESCRIPTION("Motor Driver");
MODULE_LICENSE("GPL");
