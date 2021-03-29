/**
    @file

    @brief
        Definitions for the aDIO driver

    @verbatim
    --------------------------------------------------------------------------
    This file and its contents are copyright (C) RTD Embedded Technologies,
    Inc.  All Rights Reserved.

    This software is licensed as described in the RTD End-User Software License
    Agreement.  For a copy of this agreement, refer to the file LICENSE.TXT
    (which should be included with this software) or contact RTD Embedded
    Technologies, Inc.
    --------------------------------------------------------------------------
    @endverbatim

    $Id: aDIO_driver.h 97214 2016-02-17 19:02:26Z rgroner $
*/

#ifndef __aDIO_driver_h__
#define __aDIO_driver_h__

#include <linux/spinlock.h>
#include <asm/byteorder.h>
#include <linux/types.h>

#include <aDIO_ioctl.h>
#include <aDIO_types.h>

typedef irqreturn_t(*adio_handler_t) (int, void *);


//the register location below was moved in the pentium board
/**
 * @defgroup aDIO_Driver_Chipset aDIO Chipset identification information
 * @{
 */
/**
 * @def ADIO_SETUP_PORT
 * @brief Address of the aDIO setup byte
 */
#if defined(__INTEL_MONTEVINA_CHIPSET)
    #define ADIO_SETUP_PORT 0x9CF
#elif defined(__INTEL_855GME_CHIPSET)
    #define ADIO_SETUP_PORT 0x45F
#elif defined(__GEODELX_CHIPSET)
    #define ADIO_SETUP_PORT 0x45F
#elif (defined(__AMD_GSERIES_CHIPSET) || defined(__INTEL_CHIEF_RIVER_CHIPSET) || defined(__INTEL_BAYTRAIL_CHIPSET))
    #define ADIO_SETUP_PORT 0xECF
#else
    #define ADIO_SETUP_PORT 0x1F
#endif

#define MULTIPORT_MODE_MASK_MONTEVINA 0x88
#define MULTIPORT_MODE_MONTEVINA 0x80
#define MULTIPORT_MODE_MASK_855GME 0x88
#define MULTIPORT_MODE_855GME 0x80
#define MULTIPORT_MODE_MASK_GEODELX 0x88
#define MULTIPORT_MODE_GEODELX 0x80
#define MULTIPORT_MODE_MASK_VIA 0x08
#define MULTIPORT_MODE_VIA 0x08
#define MULTIPORT_MODE_MASK_GEODE 0x0C
#define MULTIPORT_MODE_GEODE 0x04
#define MULTIPORT_MODE_MASK_GSERIES 0x88
#define MULTIPORT_MODE_GSERIES 0x80
#define MULTIPORT_MODE_MASK_CHIEFRIVER 0x88
#define MULTIPORT_MODE_CHIEFRIVER 0x80

/**
 * @}
 */
/**
 * @defgroup aDIO_Driver_Header aDIO driver definitions
 * @{
 */
#define ADIO_ADDR_MASK 0x70
#define ADIO_IRQ_MASK 0x07
#define ADIO_IO_EXTENT 4

#define ADIO_IO_UNUSED 0
#define ADIO_AUTO_DETECT_OFF ~0UL

#define ADIO_IO_DEFAULT     ADIO_IO_UNUSED
#define ADIO_IRQ_DEFAULT 0
#define ADIO_MINOR_NUM_DEFAULT 0

#define DRIVER_NAME     "rtd-aDIO"
#define ADIO_MAX_BOARDS 2
#define ADIO_MAX_QUEUED_INTS 256
#define ADIO_INT_QUEUE_SIZE (ADIO_MAX_QUEUED_INTS + 1)

/**
 * @brief
 *      Driver level debug flags
 */
enum DBG_FLAGS {
    DBG_LOAD        =   0x00001,
    DBG_REG_DEVS    =   0x00002,
    DBG_FILEOPS     =   0x00004,
    DBG_IOCTLS      =   0x00008,
    DBG_DEV     =   0x00010,
    DBG_INT     =   0x00020,
    DBG_INOUTB  =   0x00040
};

/**
 * @brief
 *      Driver status flags
 */

enum FLAGS {
    INITIALIZED = 0x01,
    REMOVE_ISR = 0x02
};

/**
 * @brief
 *      Driver level device descriptor
 */

struct aDIODevice
{
    /**
     * Interrupt status queue
     */
    int_status_t int_status_queue[ADIO_INT_QUEUE_SIZE];
    /**
     * Number of entries in the interrupt status queue
     */
    unsigned int queue_in;
    /**
     * Number of entries that have left the interrupt status queue
     */
    unsigned int queue_out;
    /**
     * Number of missed interrupts due to full status queue
     */
    unsigned int queues_missed_ints;
    /**
     * Total interupt count
     */
    unsigned int    int_count;
    /**
     * Interrupt wait queue structure
     */
    wait_queue_head_t   int_wait_queue;
    /**
     * Device minor number
     */
    unsigned int    minor;
    /**
     * Cached copy of IRQ register
     */
    unsigned char   m_RegIrq;
    /**
     * Base address of device
     */
    unsigned int    io;
    /**
     * IRQ assigned to device
     */
    unsigned int    irq;
    /**
     * Variable to maintain single-open semantics
     */
    unsigned int    counter;
    /**
     * Spin lock value
     */
    spinlock_t  lock;
    /**
     * Status flags
     */
    unsigned int    flags;
};

/**
 * @brief
 *      Driver device descriptor type
 */

typedef struct aDIODevice board_t;
/**
 * @}
 */

/**
 * @defgroup aDIO_Driver_Chipset aDIO driver function prototypes
 * @{
 */
/*************************************************************************
@brief
    Disable interrupts for aDIO device.

@param
    dev

    Pointer to a device to disable irq on.

@note
    This function does not acquires the aDIO device's spin lock.
    The spin lock is a presquite for calling this function.

*************************************************************************/
inline void aDIODisableIrq(board_t * dev);

/*************************************************************************
@brief
    Clears interrupt for aDIO device.

@param
    dev

    Pointer to a device to clear the irq on.

@note
    This function does not acquires the aDIO device's spin lock.
    The spin lock is a presquite for calling this function.

*************************************************************************/
inline void aDIOClearIrq(board_t * dev);

/******************************************************************************
@brief
    Read an unsigned 8-bit value from a device's I/O port space.

@param
    dev

    Address of device descriptor for device.

@param
    arg

    Third parameter given on ioctl() call.  This is the user space
    address of the structure used to pass in the arguments.

@retval
    0

    Success.

@retval
    < 0

    Failure with value returned as follows:@n@n
        @arg \c
        -EFAULT

        ioctl_param is not a valid user address.

*****************************************************************************/
static int ADIO_INB_Handler(ulong arg, board_t * dev);

/******************************************************************************
@brief
    Write an unsigned 8-bit value to a device's I/O port space.

@param
    dev

    Address of device descriptor for device.

@param
    arg

    Third parameter given on ioctl() call.  This is the user space
    address of the structure used to pass in the arguments.

@retval
    0

    Success.

@retval
    < 0

    Failure with value returned as follows:@n@n
        @arg \c
        -EFAULT

        ioctl_param is not a valid user address.

*****************************************************************************/
static int ADIO_OUTB_Handler(ulong arg, board_t * dev);

/******************************************************************************
@brief
    Gets interrupt status info structure from an interrupt int the
    queue if one is available.

@param
    dev

    Address of device descriptor for device.

@param
    arg

    Third parameter given on ioctl() call.  This is the user space
    address of the structure used to recieve the interrupt status
    info structure

@retval
    0

    Success.

@retval
    < 0

    Failure with value returned as follows:@n@n
        @arg \c
        -EFAULT

        ioctl_param is not a valid user address.

@note
    The data available to be read from a aDIO device is the interrupt
    status information kept by the interrupt queue.  The returned
    structure contains one sub structure(interrupt_status), one
    integer(remaining_interrupts), and an unsigned int(missed_interrupts).
    The substructure is only valid if remaining_interrupts is a positive
    number.  If there were no interrupts available remaining_interrupts
    will equal -1, otherwise it will be the remaining interrupts in the
    interrupt queue.  Last the overflow flag is set to a one if at any
    point the interrupt handler skipped grabbing intterrupt information
    because the interrupt queue was filled to capacity, which is
    ADIO_MAX_QUEUED_INTS which can be changed in aDIO_driver.h file.

@note
    This function disables interrupts for the given device and then
    acquires the device's spin lock to read interrupt status maintained by
    the interrupt handler.  Once that information is obtained, the spin
    lock is released and interrupts are enabled for the device.  This time
    interval is short, on the order of a few instructions.

*******************************************************************************/
static int ADIO_GET_INT_Handler(ulong arg, board_t * dev);

/******************************************************************************
@brief
    Retrieve the current global interrupt count from the driver

@param
    dev

    Address of device descriptor for device.

@param
    arg

    Third parameter given on ioctl() call.  This is the user space
    address of the structure used to pass in the arguments.

@retval
    0

    Success.

@retval
    < 0

    Failure with value returned as follows:@n@n
        @arg \c
        -EFAULT
            ioctl_param is not a valid user address.

*****************************************************************************/
static int ADIO_GET_INT_COUNT_Handler(ulong arg, board_t * dev);

/******************************************************************************
@brief
    Retrieve the current number of skipped interrupts from the driver

@param
    dev

    Address of device descriptor for device.

@param
    arg

    Third parameter given on ioctl() call.  This is the user space
    address of the structure used to pass in the arguments.

@retval
    0

    Success.

@retval
    < 0

    Failure with value returned as follows:@n@n
        @arg \c
        -EFAULT
            ioctl_param is not a valid user address.

*****************************************************************************/
static int ADIO_GET_MISSED_INTS_Handler(ulong arg, board_t * dev);

/*************************************************************************
@brief
    Prepares an aDIO to be opened and used.

@param
    inode

    Address of kernel's inode descriptor for the device file.

@param
    fild

    Address of kernel's file descriptor for the device file.

@retval
    0

    Success.

@retval
    < 0

    Failures with values returned as follows:@n@n
        @arg \c
        -ENODEV

        The device minor number is not valid.

        @arg \c
        -ENXIO

        The devices io address space is 0. (not valid)
        This should be correctly configured in the Makefile.

        @arg \c
        -EBUSY

        The device is already opened.

        @arg \c
        -EBUSY

        Unable to get device io range

        @arg \c
        -EBUSY

        The interrupt line requested is being used by another
        device; returned by request_irq().

        @arg \c
        -EINVAL

        The interrupt line requested is not valid; returned by
        request_irq().

        @arg \c
        -EINVAL

        No interrupt handler is to be associated with the requested
        interrupt line; returned by request_irq().

        @arg \c
        -ENOMEM

        Memory for interrupt action descriptor could not be
        allocated; returned by request_irq().

@note
    This function acquires and releases the aDIO device's spin lock.

*************************************************************************/
static int adio_open(struct inode *inode, struct file *fild);

/*************************************************************************
@brief
    Do all processing necessary after the last reference to an aDIO
    device file is released in the kernel

@param
    inode

    Address of kernel's inode descriptor for the device file.

@param
    fild

    Address of kernel's file descriptor for the device file.

@retval
    0

    Success.

@note
    This function is not necessarily invoked each time a user process calls
    close() on a device file.  When a file structure is shared, for example
    after a call to fork() or dup(), the release method is not called until
    the last reference is released.

@note
    This function acquires and releases the aDIO device's spin lock.

*************************************************************************/
static int adio_release(struct inode *inode, struct file *fild);

/******************************************************************************
@brief
    Determine whether or not a aDIO device is readable.  This function
    supports the poll(2) and select(2) system calls.

@param
    file_p

    Address of kernel's file descriptor for the device file.

@param
    poll_table_p

    Address of kernel's poll table descriptor.  This keeps track of all event
    queues on which the process can wait.

@retval
    Bit mask describing the status of the device.  The mask assumes one of
    of the following states:
        * the POLLPRI bit will be set if the device has no allocated IRQ
        * the POLLIN and POLLRDNORM bits will be set if a cached Interrupt
          Status Register is available
        * no bits are set if none of the above two conditions are met

@note
    A aDIO device is readable if and only if an interrupt has occurred
    on the device and there are still interrupt_status's remaining in
    the interrup queue.

@note
    This function is used in the process of waiting until an interrupt
    occurs on a device.

@note
    This function can be executed before an interrupt occurs, which happens
    if something sends a signal to the process.

@note
    This function disables interrupts for the given device and then
    acquires the device's spin lock to read interrupt status maintained by
    the interrupt handler.  Once that information is obtained, the spin
    lock is released and interrupts are enabled for the device.  This time
    interval is short, on the order of a few instructions.
 ******************************************************************************/
static unsigned int
adio_poll(struct file *file_p, struct poll_table_struct *poll_table_p);

/******************************************************************************
@brief
    Read data from a aDIO device.

@param
    file_p

    Address of kernel's file descriptor for the device file.

@param
    buffer_p

    Address of user space buffer in which data will be stored.

@param
    byte_count

    Number of bytes to read.  This MUST be equal to sizeof(int_status_info_t).

@param
    offset_p

    Address of memory containing the offset within the file being accessed.
    Most read operations would update this offset, e.g. when reading from
    a normal file.  However, this offset has no meaning for a aDIO device
    and it is not updated.

@retval
    sizeof(int_status_t)

    Success.

@retval
    < 0

    Failure with value returned as follows:@n@n
        @arg \c
        -EFAULT

        buffer_p is not a valid user address.

        @arg \c
        -EINVAL

        byte_count is not equal to sizeof(int_status_t).

        @arg \c
        -EIO

        No IRQ line was allocated to the device when the driver was loaded.

        @arg \c
        -ERESTARTSYS

        A signal was delivered to the process while it slept.

@note
    The data available to be read from a aDIO device is the interrupt
    status information kept by the interrupt queue.  The returned structure
    contains one sub structure(interrupt_status),
    one integer(remaining_interrupts), and
    an unsigned int(missed_interrupts).  The substructure is only valid if
    remaining_interrupts is a positive number.  If there were no interrupts
    available remaining_interrupts will equal -1, otherwise it will be the
    remaining interrupts in the interrupt queue.  Last the overflow flag is
    set to a one if at any point the interrupt handler skipped grabbing
    intterrupt information because the interrupt queue was filled to
    capacity, which is ADIO_MAX_QUEUED_INTS which can be changed in
    aDIO_driver.h file.

@note
    This function disables interrupts for the given device and then
    acquires the device's spin lock to read interrupt status maintained by
    the interrupt handler.  Once that information is obtained, the spin
    lock is released and interrupts are enabled for the device.  This time
    interval is short, on the order of a few instructions.

@note
    If the device file is set to be accessed using blocking I/O, i.e.
    O_NONBLOCK is not set for the file, this function allows the calling
    process to wait for interrupt occurrence and return status in an atomic
    operation.

@note
    A aDIO device file is readable in the following situations: 1) the
    cached IRQ Status Register value is non-zero regardless of using
    blocking or non-blocking I/O, or 2) non-blocking I/O is used regardless
    of the cached IRQ Status Register value.  The latter represents a
    departure from usual read(2) behavior in that read(2) should return
    EAGAIN when using non-blocking I/O and no data is available.  Since
    read(2) is the only way to retrieve interrupt status information,
    EAGAIN cannot be returned from here.  An application may want to get
    status before interrupts are enabled, usually when busy-waiting for an
    interrupt to occur.  A aDIO device file is not readable if the cached
    IRQ Status Register value is zero and blocking I/O is used.
 ******************************************************************************/
static ssize_t adio_read(struct file *file_p,
			 char *buffer_p, size_t byte_count, loff_t * offset_p);

/******************************************************************************
@brief
    Process ioctl(2) system calls directed toward an aDIO device file.

@param
    inode

    Address of kernel's inode descriptor for the device file.  Unused.

@param
    file

    Address of kernel's file descriptor for the device file.

@param
    cmd

    The service being requested.

@param
    arg

    Third parameter given on ioctl() call is a pointer to a struct
    DEVICE_IO_Data

@retval
    0

    Success.

@retval
    < 0

    Failure with value returned as follows:@n@n
        @arg \c
        -EBADFD

        File structure's private data pointer is NULL but it should
        be the address of the device descriptor for the device

        @arg \c
        -EINVAL

        request_code is not valid.

        @arg \c
        -EFAULT

        arg is not a valid user address.

@note
    This function indirectly acquire and release the aDIO device's spin lock.

 ******************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static int adio_ioctl(struct inode *inode, struct file *file, uint cmd,
		      ulong arg);
#else
static long adio_ioctl(struct file *file, uint cmd, ulong arg);
#endif


/******************************************************************************
@brief
    Interrupt handler for aDIO devices.

@param
    irq_number

    Interrupt number.

@param
    dev_id

    Address of device's aDIO device descriptor.  This is set on
    request_irq() call.

@param
    registers_p

    Address of processor context at time of interrupt. Unused.

@retval
    IRQ_HANDLED

    Interrupt successfully processed.

@retval
    IRQ_NONE

    Interrupt could not be processed.
 ******************************************************************************/
static irqreturn_t adio_interrupt(int irq, board_t * dev);


/******************************************************************************
@brief
    Frees the irq and io region requested for the device

@param
    dev

    a pointer to the device to remove to release requested resources

 ******************************************************************************/
static void adio_unregister_device(board_t * dev);

/******************************************************************************
@brief
    Request the necessary irq and io region for the device

@param
    dev

    a pointer to the device to remove to release requested resources

@retval
    0

    Success.

@retval
    < 0

    Failures with values returned as follows:@n@n
        @arg \c
        -ENXIO

        The devices io address space is 0. (not valid)
        This should be correctly configured in the Makefile.

        @arg \c
        -EBUSY

        The device is already opened.

        @arg \c
        -EBUSY

        Unable to get device io range

        @arg \c
        -EBUSY

        The interrupt line requested is being used by another
        device; returned by request_irq().

        @arg \c
        -EINVAL

        The interrupt line requested is not valid; returned by
        request_irq().

        @arg \c
        -EINVAL

        No interrupt handler is to be associated with the requested
        interrupt line; returned by request_irq().

        @arg \c
        -ENOMEM

        Memory for interrupt action descriptor could not be
        allocated; returned by request_irq().
@note
    This function acquires and releases the aDIO device's spin lock.

 ******************************************************************************/
static int adio_register_device(board_t * dev);


/******************************************************************************
aDIO_init_module()

    Purpose:
        This function is called at insmod time to insert the kernel module
        into the kernel and begin providing services.
        1)  It reads in the io array and irq array from the insmod command line
        and gives these the remaining minor numbers in order.
        2)  Registers the character device with the kernel
        3)  initializes lock, counter, flags, and wait_que for each device and
        calls adio_register() on each device if force = 1

@retval
    0

    Success.

@retval
    < 0

    Failures with values returned as follows:@n@n
        @arg \c
        -ENOMEM

        Unable to create proc file entry for the device

        @arg \c
        -EINVAL

        Invalid major number was requested for device

        @arg \c
        -EBUSY

        major number requested for device was busy

        These failures below are from adio_register_device()
        which will be called if force = 1
        @arg \c
        -ENXIO

        The devices io address space is 0. (not valid)
        This should be correctly configured in the Makefile.

        @arg \c
        -EBUSY

        The device is already opened.

        @arg \c
        -EBUSY

        Unable to get device io range

        @arg \c
        -EBUSY

        The interrupt line requested is being used by another
        device; returned by request_irq().

        @arg \c
        -EINVAL


        The interrupt line requested is not valid; returned by
        request_irq().

        @arg \c
        -EINVAL

        No interrupt handler is to be associated with the requested
        interrupt line; returned by request_irq().

        @arg \c
        -ENOMEM

        Memory for interrupt action descriptor could not be
        allocated; returned by request_irq().
@note
    This function acquires and releases the aDIO device's spin lock.
******************************************************************************/
int aDIO_init_module(void);

/******************************************************************************
@brief
    This function is called when the dev module is removed from the kernel.@n
    1)  This function unregisters the devices major number with the kernel.@n
    2)  Calls the device adio_unregister_device() which releases the
    resources for each device

@note
    This function acquires and releases the aDIO device's spin lock.
******************************************************************************/
void aDIO_cleanup_module(void);
/**
 * @}
 */
#endif /* __aDIO_driver_h__ */
