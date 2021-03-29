
/**
    @file

    @brief
        aDIO driver source code

//----------------------------------------------------------------------------
//  COPYRIGHT (C) RTD EMBEDDED TECHNOLOGIES, INC.  ALL RIGHTS RESERVED.
//
//  This software package is dual-licensed.  Source code that is compiled for
//  kernel mode execution is licensed under the GNU General Public License
//  version 2.  For a copy of this license, refer to the file
//  LICENSE_GPLv2.TXT (which should be included with this software) or contact
//  the Free Software Foundation.  Source code that is compiled for user mode
//  execution is licensed under the RTD End-User Software License Agreement.
//  For a copy of this license, refer to LICENSE.TXT or contact RTD Embedded
//  Technologies, Inc.  Using this software indicates agreement with the
//  license terms listed above.
//----------------------------------------------------------------------------

    $Id: rtd-aDIO.c 98012 2016-03-17 19:59:03Z rgroner $
*/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>

#include <linux/init.h>
#include <linux/stat.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/time.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>

#include <aDIO_driver.h>
#include <aDIO_types.h>
#include <aDIO_version.h>

/*=============================================================================
Driver documentation
 =============================================================================*/

#define DRIVER_DESCRIPTION "aDIO device driver"

#define DRIVER_NAME "rtd-aDIO"

MODULE_AUTHOR(RTD_COPYRIGHT_STRING);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_LICENSE("GPL");

#ifdef DEBUG

static int debug = 0;

module_param(debug, int, 0444);

MODULE_PARM_DESC(debug, "Debug level, bits field");

#endif

#ifndef DEBUG

#define r_outb_p(x, y)      (outb_p(y, x))
#define r_inb_p(x)      (inb_p(x))

#else

unsigned char r_inb_p(int x)
{
	unsigned char a;
	a = inb_p(x);
	if (debug & DBG_INOUTB)
		printk(KERN_INFO "%X = inb_p(%X);\n", a, x);
	return a;
}

void r_outb_p(int x, int y)
{
	if (debug & DBG_INOUTB)
		printk(KERN_INFO "outb_p(%X,%X);\n", x, y);
	outb_p(y, x);
}

#endif

static board_t boards[ADIO_MAX_BOARDS];

static ulong io[ADIO_MAX_BOARDS] = { 0 };

static ulong irq[ADIO_MAX_BOARDS] = { 0 };

static int minor_num_to_auto_detect = ADIO_MINOR_NUM_DEFAULT;

/**
 * Character device descriptor
 */
static struct cdev adio_cdev;

/**
 * Character device major number; dynamically assigned
 */
static int adio_major;

/**
 * @BOARDNAME@ device descriptors
 */

static struct class *dev_class = NULL;

static const char name[] = DRIVER_NAME;
static int aDIO_major = 0;
static int force = 1;

#define	NUM_IRQ_ELEM	8

static const int irq_decode[NUM_IRQ_ELEM] = { 0, 5, 7, 10, 11, 12,
#if defined(__INTEL_MONTEVINA_CHIPSET) || defined (__AMD_GSERIES_CHIPSET) || \
	defined(__INTEL_CHIEF_RIVER_CHIPSET) || \
	defined(__INTEL_BAYTRAIL_CHIPSET) || \
	defined(__INTEL_855GME_CHIPSET)	|| \
	defined(__GEODELX_CHIPSET)
	3, 6
#else
	0, 0
#endif
};

#define NUM_ADDR_ELEM	8

static const unsigned int addr_decode[NUM_ADDR_ELEM] = {
#if defined(__INTEL_MONTEVINA_CHIPSET)
	0x9C0,
#elif defined(__AMD_GSERIES_CHIPSET) || defined (__INTEL_CHIEF_RIVER_CHIPSET) || defined (__INTEL_BAYTRAIL_CHIPSET)
	0xEC0,
#else
	0x450,
#endif
	0x440, 0x410, 0x400, 0x350, 0x340, 0x310, 0x300
};

module_param_array(io, ulong, NULL, 0444);
module_param_array(irq, ulong, NULL, 0444);
module_param(minor_num_to_auto_detect, int, 0444);

MODULE_PARM_DESC(io, "I/O port base address");
MODULE_PARM_DESC(irq, "IRQ line numbers");
MODULE_PARM_DESC(minor_num_to_auto_detect,
		 "Device minor number to utilize for auto detecting ADIO");

static irqreturn_t adio_interrupt(int irq, board_t * dev);

static int adio_register_device(board_t * dev);
static void adio_unregister_device(board_t * dev);
void aDIO_cleanup_module(void);
/*========================================================================
* fileops relative functions
*=======================================================================*/

/*************************************************************************
Disable IRQs
*************************************************************************/
inline void aDIODisableIrq(board_t * dev)
{
	unsigned char controlReg;

	controlReg = r_inb_p(dev->io + r_MODE_DIO);
	controlReg &= 0xE7;
	r_outb_p(dev->io + r_MODE_DIO, controlReg);
}

/*************************************************************************
Clear interrupt
*************************************************************************/
inline void aDIOClearIrq(board_t * dev)
{
	unsigned char controlReg;

	controlReg = r_inb_p(dev->io + r_MODE_DIO);
	controlReg &= 0xFC;	// to set clear register
	r_outb_p(dev->io + r_MODE_DIO, controlReg);
	r_inb_p(r_CLEAR_DIO);	//dummy read from clear register to clear interrupts
}

/*========================================================================
* ioctl relative functions
*=======================================================================*/

/******************************************************************************
Read Byte
*****************************************************************************/
static int ADIO_INB_Handler(ulong arg, board_t * dev)
{
	struct DEVICE_IO_Data wb;
#ifdef DEBUG
	if (debug & DBG_IOCTLS)
		printk(KERN_INFO "INB() ");
#endif

	if (copy_from_user(&wb, (struct DEVICE_IO_Data *)arg, sizeof(wb)))
		return -EFAULT;
	if (dev->irq)
		disable_irq(dev->irq);
	spin_lock(&dev->lock);

	wb.Data = r_inb_p(dev->io + wb.Port);
	spin_unlock(&dev->lock);
	if (dev->irq)
		enable_irq(dev->irq);

	if (copy_to_user((struct DEVICE_IO_Data *)arg, &wb, sizeof(wb)))
		return -EFAULT;
#ifdef DEBUG
	if (debug & DBG_IOCTLS)
		printk(KERN_INFO "0x%02x from 0x%02x\n", wb.Data, wb.Port);
#endif
	return 0;
}

/******************************************************************************
Write Byte
*****************************************************************************/
static int ADIO_OUTB_Handler(ulong arg, board_t * dev)
{
	struct DEVICE_IO_Data wb;
#ifdef DEBUG

	if (debug & DBG_IOCTLS)
		printk(KERN_INFO "OUTB() ");
#endif

	if (copy_from_user(&wb, (struct DEVICE_IO_Data *)arg, sizeof(wb)))
		return -EFAULT;
#ifdef DEBUG
	if (debug & DBG_IOCTLS)
		printk(KERN_INFO "0x%02x to 0x%02x\n", wb.Data, wb.Port);
#endif
	if (dev->irq)
		disable_irq(dev->irq);
	spin_lock(&dev->lock);
	r_outb_p(dev->io + wb.Port, wb.Data);
	spin_unlock(&dev->lock);
	if (dev->irq)
		enable_irq(dev->irq);

	return 0;
}

/******************************************************************************
Get interrupt struct from int queue
*******************************************************************************/
static int ADIO_GET_INT_Handler(ulong arg, board_t * dev)
{
	int_status_info_t int_status;
	int interrupts_in_queue;
#ifdef DEBUG
	if (debug & DBG_IOCTLS)
		printk(KERN_INFO "GET_INT() ");
#endif

	if (dev->irq)
		disable_irq(dev->irq);
	spin_lock(&dev->lock);

/*
 * Check if there are any interrupts in the queue
 */
	if (dev->queue_out <= dev->queue_in)
		interrupts_in_queue = dev->queue_in - dev->queue_out;
	else
		interrupts_in_queue =
		    ADIO_INT_QUEUE_SIZE - (dev->queue_out - dev->queue_in);

/*
 * If there is an interrupt in the queue then retrieve the data from the queue
 */
	if (interrupts_in_queue > 0) {

		/*
		 * Make local copies of interrupt int_count, port1, port2, compare,
		 * & control
		 */
		int_status.interrupt_status.int_count =
		    dev->int_status_queue[dev->queue_out].int_count;
		int_status.interrupt_status.port0 =
		    dev->int_status_queue[dev->queue_out].port0;
		int_status.interrupt_status.port1 =
		    dev->int_status_queue[dev->queue_out].port1;
		int_status.interrupt_status.compare =
		    dev->int_status_queue[dev->queue_out].compare;
		int_status.interrupt_status.control =
		    dev->int_status_queue[dev->queue_out].control;

		/*
		 * Make copy of the calculated number of interrupts in the queue and
		 * return this value - 1 to signify how many more interrupts the
		 * reading device needs to recieve
		 */

		int_status.remaining_interrupts = interrupts_in_queue - 1;

		/*
		 * Remove cached IRQ Status Register value so that adio_poll knows when
		 * there is status available.  This must be done in the critical
		 * section.  (wrap around if necessary)
		 */
		dev->queue_out++;
		if (dev->queue_out == ADIO_INT_QUEUE_SIZE)
			dev->queue_out = 0;
	} else {

		/*
		 * -1 indicates that there were no interrupts in the queue
		 */
		int_status.remaining_interrupts = -1;

	}

/*
 * Pass back the missed interupts regardless number of interrupts
 */
	int_status.missed_interrupts = dev->queues_missed_ints;

	spin_unlock(&dev->lock);
	if (dev->irq)
		enable_irq(dev->irq);

	if (copy_to_user
	    ((int_status_info_t *) arg, &int_status, sizeof(int_status_info_t)))
		return -EFAULT;
	return 0;
}

/******************************************************************************
Get current total interrupt count
*****************************************************************************/
static int ADIO_GET_INT_COUNT_Handler(ulong arg, board_t * dev)
{
	unsigned int interrupt_count;
#ifdef DEBUG
	if (debug & DBG_IOCTLS)
		printk(KERN_INFO "ADIO_GET_INT_COUNT() ");
#endif

	if (dev->irq)
		disable_irq(dev->irq);
	spin_lock(&dev->lock);

	interrupt_count = dev->int_count;

	spin_unlock(&dev->lock);
	if (dev->irq)
		enable_irq(dev->irq);

	if (copy_to_user
	    ((unsigned int *)arg, &interrupt_count, sizeof(interrupt_count)))
		return -EFAULT;
	return 0;
}

/******************************************************************************
Get number of missed interrupts
*****************************************************************************/
static int ADIO_GET_MISSED_INTS_Handler(ulong arg, board_t * dev)
{
	unsigned int missed_interrupts;
#ifdef DEBUG
	if (debug & DBG_IOCTLS)
		printk(KERN_INFO "ADIO_GET_MISSED_INTS() ");
#endif

	if (dev->irq)
		disable_irq(dev->irq);
	spin_lock(&dev->lock);

	missed_interrupts = dev->queues_missed_ints;

	spin_unlock(&dev->lock);
	if (dev->irq)
		enable_irq(dev->irq);

	if (copy_to_user
	    ((unsigned int *)arg, &missed_interrupts,
	     sizeof(missed_interrupts)))
		return -EFAULT;
	return 0;
}

/*========================================================================
* fileops functions
*=======================================================================*/

/*************************************************************************
Prepares an aDIO to be opened and used.
*************************************************************************/
static int adio_open(struct inode *inode, struct file *fild)
{
	int rc;
	uint32_t minor_number;
	static board_t *dev;

#ifdef DEBUG
	if (debug & DBG_FILEOPS)
		printk(KERN_INFO "adio_open()\n");
#endif

	minor_number = MINOR(inode->i_rdev);

	if (minor_number >= ADIO_MAX_BOARDS) {
		return -ENODEV;
	}

	dev = &(boards[minor_number]);

	if (!dev->io) {
		printk(KERN_INFO
		       "adio_open() boards[%u] had io:0x%x region assigned\n",
		       minor_number, dev->io);
		return -ENXIO;
	}
	if (dev->counter) {
		return -EBUSY;
	}
	if (!(dev->flags & INITIALIZED) && (rc = adio_register_device(dev))) {
		return rc;
	}
	if (dev->irq)
		disable_irq(dev->irq);
	spin_lock(&(dev->lock));
	fild->private_data = dev;
	dev->counter++;
	dev->int_count = 0;
	dev->queues_missed_ints = 0;
	dev->queue_out = 0;
	dev->queue_in = 0;

	spin_unlock(&(dev->lock));
	if (dev->irq)
		enable_irq(dev->irq);

#ifdef DEBUG
	if (debug & DBG_FILEOPS)
		printk(KERN_INFO "adio_open() returning sucessfully\n");
#endif

	return 0;

}

/*************************************************************************
Do all processing necessary after the last reference to an aDIO
device file is released in the kernel
*************************************************************************/
static int adio_release(struct inode *inode, struct file *fild)
{
	board_t *dev = fild->private_data;

#ifdef DEBUG
	if (debug & DBG_FILEOPS)
		printk(KERN_INFO "adio_release()\n");
#endif

	if (dev->irq)
		disable_irq(dev->irq);
	spin_lock(&dev->lock);

	aDIODisableIrq(dev);
	dev->counter--;
	spin_unlock(&(dev->lock));
	if (dev->irq)
		enable_irq(dev->irq);

	if ((dev->counter <= 0) && !force) {
		adio_unregister_device(dev);
	}

	if (dev->irq)
		disable_irq(dev->irq);
	spin_lock(&(dev->lock));
	fild->private_data = 0;

	spin_unlock(&dev->lock);
	if (dev->irq)
		enable_irq(dev->irq);

	return 0;
}

/******************************************************************************
Determine whether or not a aDIO device is readable.  This function
supports the poll(2) and select(2) system calls.
 ******************************************************************************/
static unsigned int
adio_poll(struct file *file_p, struct poll_table_struct *poll_table_p)
{

	board_t *device_p = file_p->private_data;
	unsigned int status_mask = 0;
	unsigned int interrupts_in_queue;

#ifdef DEBUG
	if (debug & DBG_FILEOPS)
		printk(KERN_INFO "adio_poll()\n");
#endif

	/* If no IRQ line was allocated to the device when the driver was loaded,
	 * no status is available
	 */
	if (device_p->irq == 0) {
		/*
		 * This value causes select(2) to indicate that a file descriptor is
		 * present in its file descriptor sets but it will be in the exception
		 * set rather than in the input set.  The user library will look for
		 * this exception and return EIO.
		 */
		return POLLPRI;
	}
	/*
	 * Register with the file system layer so that it can wait on and check
	 * for adio events
	 */
	poll_wait(file_p, &(device_p->int_wait_queue), poll_table_p);

    /*=========================================================================
     Waiting is done interruptibly, which means that a signal could have been
     delivered.  Thus we might have been woken up by a signal before an
     interrupt occurred.  Therefore, the process needs to examine the device's
     cached IRQ Status Register value.
     =========================================================================*/

	/*
	 * Disable interrupts on this DM6814 device because the interrupt routine
	 * modifies this information and we want to avoid a race condition.  The
	 * critical section is small, which minimizes interrupt interference.  This
	 * particular call waits for any active interrupt to complete which means
	 * we can't be holding any resource the handler needs or we deadlock.
	 * Therefore disable the interrupt then grab the spin lock.
	 */

	if (device_p->irq)
		disable_irq(device_p->irq);

	/*
	 * Multiprocessor protection
	 */
	spin_lock(&(device_p->lock));

	if (device_p->queue_out <= device_p->queue_in)
		interrupts_in_queue = device_p->queue_in - device_p->queue_out;
	else
		interrupts_in_queue =
		    ADIO_INT_QUEUE_SIZE - (device_p->queue_out -
					   device_p->queue_in);

	/*
	 * We must check to see if the user is trying to remove the ISR
	 * and need to rejoin the thread for waiting on ISR.  To test for this case
	 */
	if (device_p->flags & REMOVE_ISR) {
		status_mask = (POLLIN | POLLRDNORM);
		device_p->flags &= ~REMOVE_ISR;
	}

	/*
	 * Unlock and enable the device's interrupts once again
	 */
	spin_unlock(&(device_p->lock));
	if (device_p->irq)
		enable_irq(device_p->irq);
	/*
	 * If there is an interrupt in the queue then change status_mask
	 * to reflect this
	 */
	if (interrupts_in_queue > 0)
		status_mask = (POLLIN | POLLRDNORM);

#ifdef DEBUG
	if (debug & DBG_FILEOPS)
		printk(KERN_INFO "adio_poll() returning status_mask: %d\n",
		       status_mask);
#endif

	return status_mask;
}

/******************************************************************************
Read data from a aDIO device.
 ******************************************************************************/
static ssize_t adio_read(struct file *file_p,
			 char *buffer_p, size_t byte_count, loff_t * offset_p)
{
	int_status_info_t int_status;
	ssize_t return_value = 0;
	board_t *device_p = file_p->private_data;
	uint8_t status_available;
	//wait_queue_t status_wait;
	wait_queue_entry_t status_wait;
	unsigned int interrupts_in_queue;

#ifdef DEBUG
	if (debug & DBG_FILEOPS)
		printk(KERN_INFO "adio_read()\n");
#endif
	/*
	 * If no IRQ line was allocated to the device when the driver was loaded
	 * no interrupt status is available
	 */
	if (device_p->irq == 0)
		return -EIO;

	/*
	 * Make sure byte count to read agrees with size of information to be
	 * returned
	 */
	if (byte_count != sizeof(int_status_info_t))
		return -EINVAL;

	/*
	 * Initialize status wait queue entry and add it to the device's interrupt
	 * wait queue.  This prepares the process for going to sleep waiting for
	 * an interrupt.  This is done whether or not the process will ultimately
	 * sleep to avoid a race condition where an interrupt may be missed if it
	 * occurs after reading the cached IRQ Status Register value and before
	 * the process inserts itself on the interrupt wait queue.
	 */
	init_waitqueue_entry(&status_wait, current);
	add_wait_queue(&(device_p->int_wait_queue), &status_wait);

#ifdef DEBUG
	if (debug & DBG_FILEOPS)
		printk(KERN_INFO
		       "adio_read() initialized wait queue entry for wait\n");
#endif

	while (1) {

		/*
		 * Set process' state to indicate to rest of system that is is asleep;
		 * more preparation for putting the process to sleep.  This is done
		 * whether or not the process will ultimately sleep to avoid a race
		 * condition where an interrupt may be missed if it occurs after reading
		 * the cached IRQ Status Register value and before the process inserts
		 * itself on the interrupt wait queue.
		 */
		set_current_state(TASK_INTERRUPTIBLE);
     /*==================================================================
      Grab the driver's interrupt count, port1, port2, compare, & control
      ==================================================================*/

		/*
		 * Disable interrupts on this DM6814 device because the interrupt
		 * routine modifies this information and we want to avoid a race
		 * condition.  The critical section is small, which minimizes interrupt
		 * interference.  This particular call waits for any active interrupt
		 * to complete which means we can't be holding any resource the handler
		 * needs or we deadlock.  Therefore disable the interrupt then grab the
		 * spin lock
		 */
		if (device_p->irq)
			disable_irq(device_p->irq);

		/*
		 * Multiprocessor protection
		 */
		spin_lock(&(device_p->lock));

		/*
		 * Check if there are any interrupts in the queue
		 */
		if (device_p->queue_out <= device_p->queue_in)
			interrupts_in_queue =
			    device_p->queue_in - device_p->queue_out;
		else
			interrupts_in_queue =
			    ADIO_INT_QUEUE_SIZE - (device_p->queue_out -
						   device_p->queue_in);

		/*
		 * If there is an interrupt in the queue then
		 * retrieve the data from the queue
		 */
		if (interrupts_in_queue > 0) {

			/*
			 * Make local copies of interrupt int_count, port1, port2, compare,
			 * & control
			 */
			int_status.interrupt_status.int_count =
			    device_p->int_status_queue[device_p->queue_out].
			    int_count;
			int_status.interrupt_status.port0 =
			    device_p->int_status_queue[device_p->queue_out].
			    port0;
			int_status.interrupt_status.port1 =
			    device_p->int_status_queue[device_p->queue_out].
			    port1;
			int_status.interrupt_status.compare =
			    device_p->int_status_queue[device_p->queue_out].
			    compare;
			int_status.interrupt_status.control =
			    device_p->int_status_queue[device_p->queue_out].
			    control;

			/*
			 * Make copy of the calculated number of interrupts in the queue and
			 * return this value - 1 to signify how many more interrupts the
			 * reading device needs to recieve
			 */

			int_status.remaining_interrupts =
			    interrupts_in_queue - 1;

			/*
			 * Remove cached IRQ Status Register value so that adio_poll knows when
			 * there is status available.  This must be done in the critical
			 * section.  (wrap around if necessary)
			 */
			device_p->queue_out++;
			if (device_p->queue_out == ADIO_INT_QUEUE_SIZE)
				device_p->queue_out = 0;
		} else {

			/*
			 * -1 indicates that there were no interrupts in the queue
			 */
			int_status.remaining_interrupts = -1;

		}

		/*
		 * Pass back the queue overflow flag regardless number of interrupts
		 */
		int_status.missed_interrupts = device_p->queues_missed_ints;

		/*
		 * Unlock and enable this device's interrupts once again
		 */
		spin_unlock(&(device_p->lock));
		if (device_p->irq)
			enable_irq(device_p->irq);

    /*=======================================================================
     Local copies f interrupt status obtained, so examine the status
    ========================================================================*/

		/*
		 * Is IRQ Status Register value available?
		 */
		if (interrupts_in_queue > 0) {
			status_available = 0xFF;
			break;
		}

		/*
		 * An interrupt has not occurred since the last time interrupt status
		 * was read.  Theoretically, no data is available to read.
		 */

		/*
		 * Normally, using non-blocking I/O would mean that we return -EAGAIN
		 * here.  However, there is no other way to get interrupt status and
		 * therefore we must return the driver's interrupt count and cached IRQ
		 * Status Register value whatever they may be when using non-blocking
		 * I/O.  Although we could look at this another way, namely that the
		 * driver's interrupt count is always available.  Non-blocking I/O is
		 * used when: 1) busy-waiting for interrupts to occur, 2) when getting
		 * an initial interrupt status before enabling interrupts and waiting
		 * for them to occur, or 3) when getting a final interrupt status after
		 * disabling interrupts and all processing is complete.
		 */
		if (file_p->f_flags & O_NONBLOCK) {
			status_available = 0xFF;
			break;
		}
		/*
		 * At this point no status is available and the process wants blocking
		 * I/O
		 */
		status_available = 0x00;
		/*
		 * Is there a signal pending for the process?
		 */
		if (signal_pending(current)) {
			/*
			 * The process has a signal pending, inform the file system layer that
			 * a signal is pending and letit decide what to do about it
			 */
			return_value = -ERESTARTSYS;
			break;
		}
		/*
		 * Switch this process away from the CPU, thus finally putting it to
		 * sleep
		 */
		schedule();
	}
	/*
	 * Was there status available?
	 */

	if (status_available) {

		/*
		 * Interrupt status available, copy information back to user space
		 */
		if (copy_to_user
		    (buffer_p, &int_status, sizeof(int_status_info_t)
		    ) == 0) {
			return_value = sizeof(int_status_info_t);
		} else {
			return_value = -EFAULT;
		}
	}
	/*
	 * Mark process as runnable again which undoes previously setting the
	 * state to sleep
	 */
	set_current_state(TASK_RUNNING);

	/*
	 * Remove entry from the device's interupt wait queue since the process
	 * is either not going to sleep or has already slept
	 */
	remove_wait_queue(&(device_p->int_wait_queue), &status_wait);

#ifdef DEBUG
	if (debug & DBG_FILEOPS)
		printk(KERN_INFO
		       "adio_read() removed from wait_queue and returning:%d\n",
		       return_value);
#endif

	return return_value;

}

static inline void ADIO_Initdev(board_t * dev)
{
#ifdef DEBUG
	if (debug & DBG_DEV)
		printk(KERN_INFO "Initdev()\n");
#endif
	// disable IRQ sharing, set IRQ positive edge, disable IRQ
	dev->m_RegIrq = 0x04;
	aDIODisableIrq(dev);

}

/******************************************************************************
Process ioctl(2) system calls directed toward an aDIO device file.
 ******************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static int
adio_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
#else
static long adio_ioctl(struct file *file, uint cmd, ulong arg)
#endif
{
	int rc = 0;
	board_t *dev = file->private_data;

#ifdef DEBUG
	if (debug & DBG_FILEOPS)
		printk(KERN_INFO "adio_ioctl()\n");
#endif

	if (!dev)
		return -EBADFD;

	switch (cmd) {

	case ADIO_IOCTL_OUTB:
		rc = ADIO_OUTB_Handler(arg, dev);
		break;

	case ADIO_IOCTL_INB:
		rc = ADIO_INB_Handler(arg, dev);
		break;

	case ADIO_IOCTL_GET_INT:
		rc = ADIO_GET_INT_Handler(arg, dev);
		break;

	case ADIO_IOCTL_GET_GLOB_INT_COUNT:
		rc = ADIO_GET_INT_COUNT_Handler(arg, dev);
		break;

	case ADIO_IOCTL_GET_GLOB_MISSED_INTS:
		rc = ADIO_GET_MISSED_INTS_Handler(arg, dev);
		break;

	case ADIO_IOCTL_WAKEUP:
#ifdef DEBUG
		if (debug & DBG_FILEOPS)
			printk(KERN_INFO "waking up thread\n");
#endif
		if (dev->irq)
			disable_irq(dev->irq);
		spin_lock(&dev->lock);
		//this must be set for poll to return readable
		dev->flags |= REMOVE_ISR;
		wake_up_interruptible(&(dev->int_wait_queue));
		spin_unlock(&dev->lock);
		if (dev->irq)
			enable_irq(dev->irq);
		rc = 0;
		break;

	case ADIO_IOCTL_FLUSH_INT_QUEUE:
#ifdef DEBUG
		if (debug & DBG_FILEOPS)
			printk(KERN_INFO
			       "flushing(emptying) out interrupt queue\n");
#endif
		if (dev->irq)
			disable_irq(dev->irq);
		spin_lock(&dev->lock);
		dev->queue_in = 0;
		dev->queue_out = 0;
		spin_unlock(&dev->lock);
		if (dev->irq)
			enable_irq(dev->irq);
		rc = 0;
		break;

	default:
		rc = -EINVAL;
	}
	return rc;
}

// + file_operations switch
/*****************************************************************************/

static struct file_operations driver_fops = {
	.owner = THIS_MODULE,
	.open = adio_open,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
	.ioctl = adio_ioctl,
#else
	.unlocked_ioctl = adio_ioctl,
#endif
	.release = adio_release,
	.read = adio_read,
	.poll = adio_poll
};

/******************************************************************************
Interrupt handler for aDIO devices.
 ******************************************************************************/
static irqreturn_t adio_interrupt(int irq, board_t * dev)
{

	static int previously_handled[ADIO_MAX_BOARDS] = { 0 };
	//{[0...ADIO_MAX_BOARDS - 1] = 0 };
	unsigned char contReg, original_ctrl_reg;
	int minor;
	unsigned int interrupts_in_queue;

#ifdef DEBUG
	if (debug & DBG_INT)
		printk(KERN_INFO "adio_interrupt()\n");
#endif

	if (dev == NULL) {
		printk(KERN_ERR
		       "%s> ERROR: Interrupt handler has NULL device pointer\n",
		       DRIVER_NAME);
		return IRQ_NONE;
	}

	spin_lock(&dev->lock);

	//  check if interrupt caused by  aDIO
	contReg = r_inb_p(dev->io + rCONTROL);
	original_ctrl_reg = contReg;
	if (!(contReg & 0x40)) {	// Spurious or not aDIO Interrupt
		spin_unlock(&dev->lock);

		// check if previously handled interrupt
		if (previously_handled[dev->minor]) {
			previously_handled[dev->minor] = 0;
#ifdef DEBUG
			if (debug & DBG_INT)
				printk(KERN_INFO
				       "adio_interrupt previously handle just waking up and exiting()\n");
#endif

			/* wake up any waiting interrupts for this process */
			wake_up_interruptible(&(dev->int_wait_queue));

			return IRQ_HANDLED;
		} else {
#ifdef DEBUG
			if (debug & DBG_INT)
				printk(KERN_INFO
				       "adio_interrupt spurious and exiting()\n");
#endif

			return IRQ_NONE;
		}
	}
	// increment interrupt counter
	dev->int_count++;
#ifdef DEBUG
	if (debug & DBG_INT)
		printk(KERN_INFO "triggered aDIO interrupt minor:%d\n",
		       dev->minor);
#endif
	/* check for room in the interrupt queue */
	if (dev->queue_out <= dev->queue_in)
		interrupts_in_queue = dev->queue_in - dev->queue_out;
	else
		interrupts_in_queue =
		    ADIO_INT_QUEUE_SIZE - (dev->queue_out - dev->queue_in);

	/* if interrupt queue isn't full */
	if (interrupts_in_queue < ADIO_INT_QUEUE_SIZE - 1) {
		/* collect interrupt data and store in device structure */
		dev->int_status_queue[dev->queue_in].control = contReg;
		dev->int_status_queue[dev->queue_in].port0 =
		    r_inb_p(dev->io + rPORT0DATA);
		dev->int_status_queue[dev->queue_in].port1 =
		    r_inb_p(dev->io + rPORT1DATA);
		contReg = r_inb_p(dev->io + rCONTROL);
		contReg |= 0x3;	// set to read compare register
		r_outb_p(dev->io + rCONTROL, contReg);	//load new value
		dev->int_status_queue[dev->queue_in].compare =
		    r_inb_p(dev->io + rMULTIFUNCTION);
		dev->int_status_queue[dev->queue_in].int_count = dev->int_count;

		/* increment queue in ptr */
		dev->queue_in++;
		if (dev->queue_in == ADIO_INT_QUEUE_SIZE)
			dev->queue_in = 0;	/* wrap around */

	} else {		/* indicate an overflow and ignore register values */
		printk(KERN_WARNING
		       "skipped interrupt info because interrupt queue is full\n");
		dev->queues_missed_ints++;
	}

	/* clear interrups */
	contReg = r_inb_p(dev->io + rCONTROL);
	contReg &= 0xFC;	// set rMultifunction to clear
	r_outb_p(dev->io + rCONTROL, contReg);	//load new value
	r_inb_p(dev->io + r_CLEAR_DIO);	// clear interrupt

	/**
	 * Restore the control reg to its previous state prior to
	 * the interrupt subroutine.
	 */
	r_outb_p(dev->io + rCONTROL, original_ctrl_reg);
	spin_unlock(&dev->lock);

	/* check for all other interrupts */
	for (minor = 0; minor < ADIO_MAX_BOARDS; minor++) {
		if (boards[minor].irq) {
			spin_lock(&boards[minor].lock);

			// check if interrupt caused by  aDIO
			contReg = r_inb_p(boards[minor].io + rCONTROL);
			original_ctrl_reg = contReg;
			if (contReg & 0x40) {	// aDIO Interrupt
				boards[minor].int_count++;
#ifdef DEBUG
				if (debug & DBG_INT)
					printk(KERN_INFO
					       "found aDIO interrupt on minor:%d\n",
					       minor);
#endif
				/* check for room in the interrupt queue */
				if (boards[minor].queue_out <=
				    boards[minor].queue_in)
					interrupts_in_queue =
					    boards[minor].queue_in -
					    boards[minor].queue_out;
				else
					interrupts_in_queue =
					    ADIO_INT_QUEUE_SIZE -
					    (boards[minor].queue_out -
					     boards[minor].queue_in);

				/* if interrupt queue isn't full */
				if (interrupts_in_queue <
				    ADIO_INT_QUEUE_SIZE - 1) {
					/* collect interrupt data and store in device structure */
					boards[minor].int_status_queue[boards
								       [minor].
								       queue_in].control
					    = contReg;
					boards[minor].
					    int_status_queue[boards[minor].
							     queue_in].port0 =
					    r_inb_p(boards[minor].io +
						    rPORT0DATA);
					boards[minor].
					    int_status_queue[boards[minor].
							     queue_in].port1 =
					    r_inb_p(boards[minor].io +
						    rPORT1DATA);
					contReg =
					    r_inb_p(boards[minor].io +
						    rCONTROL);
					contReg |= 0x3;	// set to read compare register
					r_outb_p(boards[minor].io + rCONTROL, contReg);	//load new value
					boards[minor].int_status_queue[boards
								       [minor].
								       queue_in].compare
					    =
					    r_inb_p(boards[minor].io +
						    rMULTIFUNCTION);
					boards[minor].
					    int_status_queue[boards[minor].
							     queue_in].
					    int_count = boards[minor].int_count;

					/* increment queue in ptr */
					dev->queue_in++;
					if (dev->queue_in ==
					    ADIO_INT_QUEUE_SIZE)
						dev->queue_in = 0;	/* wrap around */

				} else {	/* indicate an overflow and ignor register values */
					printk(KERN_WARNING
					       "skipped interrupt info because interrupt queue is full\n");
					dev->queues_missed_ints++;
				}

				/* mark as previously handled interrupt */
				previously_handled[boards[minor].minor] = 1;

				/* clear interrups */
				contReg = r_inb_p(boards[minor].io + rCONTROL);
				contReg &= 0xFC;	// set rMultifunction to clear
				r_outb_p(boards[minor].io + rCONTROL, contReg);	//load new value
				r_inb_p(boards[minor].io + r_CLEAR_DIO);	// clear interrupt

				/**
				 * Restore the control reg to its previous state prior to
				 * the interrupt subroutine.
				 */
				r_outb_p(boards[minor].io + rCONTROL,
					 original_ctrl_reg);

			}
			spin_unlock(&boards[minor].lock);
		}
	}
#ifdef DEBUG
	if (debug & DBG_INT)
		printk(KERN_INFO
		       "waking up and returning from handled interrupts\n");
#endif
	/* wake up any waiting interrupts for this process */
	wake_up_interruptible(&(dev->int_wait_queue));

	return IRQ_HANDLED;
}

/******************************************************************************
Frees the irq and io region requested for the device
 ******************************************************************************/
static void adio_unregister_device(board_t * dev)
{

#ifdef DEBUG

	if (debug & DBG_REG_DEVS)
		printk(KERN_INFO "adio_unregister_device()\n");

#endif

	if (dev && (dev->flags & INITIALIZED)) {
		if (dev->irq)
			disable_irq(dev->irq);
		spin_lock(&dev->lock);
		aDIODisableIrq(dev);

		if (dev->io != 0) {
#ifdef DEBUG
			if (debug & DBG_REG_DEVS)
				printk(KERN_INFO
				       "calling release_region(%x,%d);\n",
				       dev->io, ADIO_IO_EXTENT);
#endif
			release_region(dev->io, ADIO_IO_EXTENT);
		}

		if (dev->irq)
			free_irq(dev->irq, dev);

		dev->counter--;

		dev->flags &= ~INITIALIZED;
		spin_unlock(&dev->lock);
		if (dev->irq)
			enable_irq(dev->irq);
	}
}

/******************************************************************************
Request the necessary irq and io region for the device
 ******************************************************************************/
static int adio_register_device(board_t * dev)
{
	int rc = 0;

#ifdef DEBUG

	if (debug & DBG_REG_DEVS)
		printk(KERN_INFO "adio_register_device(io=%#x, irq=%d)\n",
		       dev->io, dev->irq);

#endif

	if (dev->flags & INITIALIZED) {
		return 0;
	}

	if (!dev->io) {
		return -ENXIO;
	}

	if (dev->irq)
		disable_irq(dev->irq);

	if (request_region(dev->io, ADIO_IO_EXTENT, name) == NULL) {
		printk(KERN_ERR
		       "Unable get IO port range %#x-%#x: resource busy\n",
		       dev->io, (dev->io + ADIO_IO_EXTENT - 1)
		    );

		if (dev->irq)
			enable_irq(dev->irq);
		return -EBUSY;
	}

	if (dev->irq) {
		rc = request_irq(dev->irq,
				 (adio_handler_t) adio_interrupt, 0, name, dev);
		if (rc != 0) {
			printk(KERN_ERR "ERROR: Unable to allocate IRQ %d\n",
			       dev->irq);
			release_region(dev->io, ADIO_IO_EXTENT);
			return rc;
		}
	}

	printk(KERN_INFO
	       "%s device successfully registered at io=%#x, irq=%d\n", name,
	       dev->io, dev->irq);

	ADIO_Initdev(dev);
	dev->flags |= INITIALIZED;

#ifdef DEBUG
	if (debug & DBG_REG_DEVS)
		printk(KERN_INFO "adio_register() done\n");
#endif
	return 0;
}

/******************************************************************************
Initialize an aDIO device
******************************************************************************/
int aDIO_init_module(void)
{
	int rc;
	int minor;
	unsigned char adio_addr;
	unsigned char adio_irq;
	unsigned char adio_setup;
	int status;

	dev_t device, devno;
	char dev_file_name[30];
	struct device *dev = NULL;

#ifdef DEBUG

	if (debug & DBG_LOAD)
		printk(KERN_INFO "init_module()\n");

#endif
#if defined(__INTEL_BAYTRAIL_CHIPSET)
	printk(KERN_INFO "INTEL BAYTRAIL CHIPSET was detected\n");
#endif
#if defined(__INTEL_MONTEVINA_CHIPSET)
	printk(KERN_INFO "INTEL MONTEVINA CHIPSET was detected\n");
#endif
#if defined(__INTEL_855GME_CHIPSET)
	printk(KERN_INFO "INTEL 855GME CHIPSET was detected\n");
#endif
#if defined(__VIA_CHIPSET)
	printk(KERN_INFO "VIA CHIPSET was detected\n");
#endif
#if defined(__GEODE_CHIPSET)
	printk(KERN_INFO "GEODE CHIPSET was detected\n");
#endif
#if defined(__GEODELX_CHIPSET)
	printk(KERN_INFO "GEODE LX CHIPSET was detected\n");
#endif
#if defined(__AMD_GSERIES_CHIPSET)
	printk(KERN_INFO "AMD G-SERIES CHIPSET was detected\n");
#endif
#if defined(__INTEL_CHIEF_RIVER_CHIPSET)
	printk(KERN_INFO "INTEL CHIEF RIVER (i7) CHIPSET was detected\n");
#endif
#if defined(__WARNING_UNKNOWN_RTD_CHIPSET)
	printk(KERN_INFO "WARNING NON RTD CHIPSET was detected\n");
#endif

	/*
	 * Read a byte from aDIO setup port to get BIOS aDIO setup.  On Geode
	 * processors, only bits 0 through 2 have meaning (the aDIO IRQ) and all
	 * other bits are pulled low by the EPLD.  On VIA Eden processors, bits 0
	 * through 2 are the aDIO IRQ, bit 3 is the multiPort mode, bits 4 through
	 * 6 are the aDIO base I/O address, and bit 7 is reserved.
	 */

/* Check that minor number that the onboard aDIO will be assigned is in range*/
	if (minor_num_to_auto_detect < ADIO_MAX_BOARDS) {

		adio_setup = inb_p(ADIO_SETUP_PORT);

#if defined(__INTEL_855GME_CHIPSET)
		if ((adio_setup & MULTIPORT_MODE_MASK_855GME) ==
		    MULTIPORT_MODE_855GME) {
#else
#if defined(__GEODELX_CHIPSET)
		if ((adio_setup & MULTIPORT_MODE_MASK_GEODELX) ==
		    MULTIPORT_MODE_GEODELX) {
#else
#if defined(__VIA_CHIPSET)
		if ((adio_setup & MULTIPORT_MODE_MASK_VIA) ==
		    MULTIPORT_MODE_VIA) {
#else
#if defined(__GEODE_CHIPSET)
		if ((inb_p(0x19) & MULTIPORT_MODE_MASK_GEODE) ==
		    MULTIPORT_MODE_GEODE) {
#else
		if (1) {
#endif
#endif
#endif
#endif
			/*
			 * Extract bits 4 through 6, which are DIO_ADDR[2:0].  On a Geode
			 * processor, these are guaranteed by the EPLD to be zero.
			 */

			adio_addr = ((adio_setup & ADIO_ADDR_MASK) >> 4);

			/*
			 * Extract bits 0 through 2, which are DIO_IRQ[2:0]
			 */

			adio_irq = (adio_setup & ADIO_IRQ_MASK);

			/*
			 * Decode the DIO_IRQ[2:0] bits to determine which IRQ to use, if any
			 */
			boards[minor_num_to_auto_detect].irq = 0;
			if (adio_irq < NUM_IRQ_ELEM) {
				boards[minor_num_to_auto_detect].irq =
				    irq_decode[adio_irq];
			}

			/*
			 * If the aDIO IRQ is disabled, print a warning message
			 */

			if (boards[minor_num_to_auto_detect].irq == 0) {
				printk(KERN_INFO
				       "aDIO: WARNING: Interrupts disabled.\n");
			}

			/*
			 * Decode the DIO_ADDR[2:0] bits to determine base I/O address
			 */
			if (adio_addr >= NUM_ADDR_ELEM) {
				return -EFAULT;
			}
			boards[minor_num_to_auto_detect].io =
			    addr_decode[adio_addr];

			/* save minor number in each device */
			boards[minor_num_to_auto_detect].minor =
			    minor_num_to_auto_detect;

			printk(KERN_INFO "%s[%d] base address: 0x%x\n",
			       DRIVER_NAME,
			       minor_num_to_auto_detect,
			       boards[minor_num_to_auto_detect].io);
			printk(KERN_INFO "%s[%d] IRQ: %d\n",
			       DRIVER_NAME,
			       minor_num_to_auto_detect,
			       boards[minor_num_to_auto_detect].irq);
		} else {
			printk(KERN_INFO
			       "%s: WARNING: CPU's onboard aDIO is disabled in BIOS\n",
			       DRIVER_NAME);
			/*
			 * This is done to prevent the next section of code from
			 * reserving a board structure (at location 0 by default)
			 * for this aDIO which is disabled
			 */
			minor_num_to_auto_detect = ADIO_MAX_BOARDS;
		}

	} else {
		printk("WARNING: minor number to assign auto detected aDIO "
		       "is out of range\n");
	}

/*
 * Assign the base addresses and irq's from the makefile.
 * IE.  io=0xAA,0xBB,0xCC irq=D,E,F means the first available minor number
 * is asigned to the device with base address AA and irq D. The next available
 * minor number is assigned to the device with base address BB and irq E and
 * so on and so forth.  The num_to_auto_detect is the minor number that the
 * auto detected base address and Irq will be assigned to(which means this
 * minor number no longer available for use in this section and must be
 * skipped
 */
	for (minor = 0; ((minor < ADIO_MAX_BOARDS - 1) ||
			 ((minor < minor_num_to_auto_detect)
			  && (minor < ADIO_MAX_BOARDS))); minor++) {
		if (io[minor] != ADIO_IO_UNUSED) {
			if (minor < minor_num_to_auto_detect) {
				boards[minor].io = io[minor];
				boards[minor].irq = irq[minor];
				boards[minor].minor = minor;
				printk(KERN_INFO
				       "%s[%d] : base address: 0x%x IRQ: %d\n",
				       DRIVER_NAME, minor, boards[minor].io,
				       boards[minor].irq);
			} else {
				boards[minor + 1].io = io[minor];
				boards[minor + 1].irq = irq[minor];
				boards[minor + 1].minor = minor + 1;
				printk(KERN_INFO
				       "%s[%d] : base address: 0x%x IRQ: %d\n",
				       DRIVER_NAME, minor + 1,
				       boards[minor + 1].io,
				       boards[minor + 1].irq);
			}
		}
	}

	status = alloc_chrdev_region(&device, 0, ADIO_MAX_BOARDS, DRIVER_NAME);

	if (status < 0) {
		return status;
	}

	cdev_init(&adio_cdev, &driver_fops);
	adio_cdev.owner = THIS_MODULE;

	status = cdev_add(&adio_cdev, device, ADIO_MAX_BOARDS);

	if (status < 0) {
		unregister_chrdev_region(device, ADIO_MAX_BOARDS);
		return status;
	}

	adio_major = MAJOR(device);

	dev_class = class_create(THIS_MODULE, DRIVER_NAME);

	if (dev_class == NULL) {
		unregister_chrdev_region(device, ADIO_MAX_BOARDS);
		return -ENODEV;
	}

	for (minor = 0; minor < ADIO_MAX_BOARDS; minor++) {

		devno = MKDEV(adio_major, minor);
		sprintf(dev_file_name, "rtd-aDIO-%d", minor);
		dev =
		    device_create(dev_class, NULL, devno, NULL, dev_file_name,
				  0);

		if (dev == NULL) {
			return -ENODEV;
		}

	}

	printk(KERN_INFO "%s registered using character major number %d\n",
	       name, aDIO_major);
	/*
	 * initialize all variable of installed aDIO devices
	 */
	for (minor = 0; minor < ADIO_MAX_BOARDS; minor++) {

		if (boards[minor].io) {
			spin_lock_init(&(boards[minor].lock));
			spin_lock(&(boards[minor].lock));
			boards[minor].queue_in = 0;
			boards[minor].queue_out = 0;
			boards[minor].queues_missed_ints = 0;
			boards[minor].counter = 0;
			boards[minor].flags = 0;
			init_waitqueue_head(&(boards[minor].int_wait_queue));
#ifdef DEBUG

			if (debug & DBG_LOAD)
				printk(KERN_INFO
				       "init_module() init_waitqueue_head for minor:%d\n",
				       minor);

#endif
			spin_unlock(&(boards[minor].lock));

			/*
			 * if force then don't wait to register the device till the
			 * file is open.  Do it now.
			 */
			if (force) {
				rc = adio_register_device(&boards[minor]);
				if (rc != 0) {
					printk(KERN_ERR
					       "aDIO device registration failed.\n");
					aDIO_cleanup_module();
					return rc;
				}
			}
		}
	}

#ifdef DEBUG

	if (debug & DBG_LOAD)
		printk(KERN_INFO "init_module() done\n");

#endif

	return 0;
}

/******************************************************************************
Remove an aDIO module
******************************************************************************/
void aDIO_cleanup_module(void)
{

	unsigned int minor = 0;

#ifdef DEBUG

	if (debug & DBG_LOAD)
		printk(KERN_INFO "cleanup_module()\n");

#endif

	cdev_del(&adio_cdev);

	for (minor = 0; minor < ADIO_MAX_BOARDS; minor++) {
		device_destroy(dev_class, MKDEV(adio_major, minor));

	}

	class_unregister(dev_class);

	class_destroy(dev_class);

	unregister_chrdev_region(MKDEV(adio_major, 0), ADIO_MAX_BOARDS);

/*

	unregister_chrdev(aDIO_major, name);
*/
	for (minor = 0; minor < ADIO_MAX_BOARDS; minor++)
		if (boards[minor].io) {
			adio_unregister_device(&boards[minor]);
		}
#ifdef DEBUG

	if (debug & DBG_LOAD)
		printk(KERN_INFO "cleanup_module() done\n");

#endif

}

module_init(aDIO_init_module);
module_exit(aDIO_cleanup_module);
