/**
    @file

    @brief
        This file contains function prototypes for the aDIO user level
        library.

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

    $Id: aDIO_library.h 37457 2009-05-08 14:49:43Z wtate $
*/

#ifndef __aDIO_library_h__
#define __aDIO_library_h__

#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <aDIO_ioctl.h>
#include <pthread.h>
#include <sched.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <linux/unistd.h>

#include <aDIO_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *      Interrupts disabled.
 */
#define DISABLE_INT_MODE    0
/**
 * @brief
 *      Event mode interrupts.
 */
#define EVENT_INT_MODE      1
/**
 * @brief
 *      Strobe mode interrupts
 */
#define STROBE_INT_MODE     2
/**
 * @brief
 *      Match mode interrupts
 */
#define MATCH_INT_MODE      3

/**
 * @brief
 *      Status Flags
 */
enum FLAGS {
    /**
     * Flag indicating the user space ISR should be uninstalled
     */
    EXIT_FLAG       = 0x01,
    /**
     * Flag indicating that a user space ISR has been installed
     */
    ISR_INSTALLED   = 0x02
};

/**
 * @brief
 *      aDIO library device descriptor
 */
struct aDIODeviceDescriptor
{
    /**
     * file handle
     */
    int         hDevice;
    /**
     * minor number
     */
    uint32_t        minor;
    /**
     * status flags
     */
    unsigned int    flags;
    /**
     * process ID of the user space ISR
     */
    pthread_t       pid;
    /**
     * function prototype for user space ISR
     */
    void (*isr)(isr_info_t);
};
/**
 * @brief
 *      aDIO library device descriptor type
 */
typedef struct aDIODeviceDescriptor * DeviceHandle;

/**
 * @defgroup aDIO_Library_Header aDIO library API
 * @{
 */

/******************************************************************************
@brief
    Clear the aDIO circuitry.  This performs the following actions:@n
    1) sets all aDIO Compare Register bits to zero.@n
    2) sets all aDIO Mask Register bits to zero. @n
    3) sets all aDIO Control Register bits to zero. @n
    4) sets all aDIO Port 0 Data Register bits to zero. @n
    5) sets all aDIO Port 1 Data Register bits to zero.

@param
    Device  
    
    Pointer to a device structure

@retval
    0
    
    Success.

@retval
    -1
    
    Failure.@n@n
    
    Please see the descriptions of the internal functions InPort() and OutPort()
    for information on possible values errno may have in this case.
******************************************************************************/

int ClearDio_aDIO(DeviceHandle Device);


/******************************************************************************
@brief
    Close the device file associated with the aDIO device.

@brief
    Device 
    
    Pointer to a device structure

@retval
    0
    
    Success.

@retval
    -1
       
    Failure.@n@n  
    
    Please see the fclose(3) man page for information on possible values errno 
    may have in this case.

@note
    Be sure to call this function after you are done with the aDIO device.
******************************************************************************/

int CloseDIO_aDIO(DeviceHandle Device);


/******************************************************************************
@brief
    Enable or disable aDIO digital interrupts.  If interrupts are enabled,
    this also will set the interrupt mode.

@brief
    Device 
    
    Pointer to a device structure
    
@brief
    Mode 
    
    Interrupt mode selector.  Valid values are:@n@n    
    DISABLE_INT_MODE (0) - Disable digital interrupts@n
    EVENT_INT_MODE (1) --- Enable event mode interrupts@n
    STROBE_INT_MODE (2) -- Enable strobe mode interrupts@n
    MATCH_INT_MODE (3) --- Enable match mode interrupts

@retval
    0
    
    Success.

    
@retval
    -1
    
    Failure with errno set as follows:@n
            @arg \c
            EINVAL      Mode is not valid.

    Please see the descriptions of the internal functions
    ClearIrq_aDIO(), InPort(), and OutPort() and the description of
    LoadPort0BitDir_aDIO() for information on other possible values
    errno may have in this case.

@note
    Before enabling interrupts, an interrupt callback function should be 
    installed via InstallISR_aDIO().

    Whenever any interrupt mode is enabled, this function will set the
    direction for all Port 0 bits to input.

    After match mode is enabled, you should load the desired bit mask into
    the Compare Register with LoadComp_aDIO().
    
******************************************************************************/

int EnableInterrupts_aDIO(DeviceHandle Device, unsigned char Mode);


/******************************************************************************
@brief
    Return the current aDIO digital interrupt mode.

@param
    Device
    
    Pointer to a device structure
    
@param
    mode
    
    Address where interrupt mode will be stored.  DISABLE_INT_MODE
    (0) will be stored here if digital interrupts are disabled.
    EVENT_INT_MODE (1) will be stored here if digital interrupts
    are in event mode.  STROBE_INT_MODE (2) will be stored here if
    digital interrupts are in strobe mode.  MATCH_INT_MODE (3) will
    be stored here if digital interrupts are in match mode.
    Nothing is written to this memory location if the function
    fails.

@retval
    0
    
    Success.

@retval
    -1
    
    Failure.@n@n
    
    Please see the description of the internal function
    InPort() for information on possible values errno may have in this
    case.
******************************************************************************/

int GetInterruptMode_aDIO(DeviceHandle Device, unsigned char *mode);


/******************************************************************************
@brief
    Write an 8-bit value into the aDIO Compare Register.

@param
    Device
    
    Pointer to a device structure
    
@param
    Value
    
    The value to write into Compare Register.

@retval
    0
    
    Success.

@retval
    -1
    
    Failure.@n@n
    
    Please see the descriptions of the internal functions
    InPort() and OutPort() for information on possible values errno may
    have in this case.
******************************************************************************/

int LoadComp_aDIO(DeviceHandle Device, unsigned char Value);


/******************************************************************************
@brief
    Write an 8-bit value into the aDIO Mask Register.

@param
    Device

    Pointer to a device structure

@param
    Bit7
    
    Flag to signify whether bit 7 should be set or cleared.
    
@param
    Bit6
    
    Flag to signify whether bit 6 should be set or cleared.
    
@param
    Bit5
    
    Flag to signify whether bit 5 should be set or cleared.
    
@param
    Bit4
    
    Flag to signify whether bit 4 should be set or cleared.
    
@param
    Bit3
    
    Flag to signify whether bit 3 should be set or cleared.
    
@param
    Bit2
    
    Flag to signify whether bit 2 should be set or cleared.

@param
    Bit1
    
    Flag to signify whether bit 1 should be set or cleared.
    
@param
    Bit0
    
    Flag to signify whether bit 0 should be set or cleared.

@retval
    0
    
    Success.

@retval
    -1
    
    Failure.@n@n
    
    Please see the descriptions of the internal functions
    InPort() and OutPort() for information on possible values errno may
    have in this case.

@note
    A zero in a bit position means that bit in the Compare Register is
    masked off and ignored in Event Mode and Match Mode.
    
@note
    A value of false for any of the above bit flags indicates that the
    corresponding bit in the Mask Register should be cleared.  A value
    of true for any of the above bit flags indicates that the
    corresponding bit in the Mask Register should be set.
******************************************************************************/

int LoadMask_aDIO(
    DeviceHandle Device,
    unsigned char Bit7,
    unsigned char Bit6,
    unsigned char Bit5,
    unsigned char Bit4,
    unsigned char Bit3,
    unsigned char Bit2,
    unsigned char Bit1,
    unsigned char Bit0
);


/******************************************************************************
@brief
    Set the direction (input or output) of each port 0 bit by writing an
    8-bit value into the aDIO Port 0 Direction Register.

@param
    Device 
    
    Pointer to a device structure
    
@param
    Bit7
    
    Flag to signify whether bit 7 should be input or output.
    
@param
    Bit6
    
    Flag to signify whether bit 6 should be input or output.
    
@param
    Bit5
    
    Flag to signify whether bit 5 should be input or output.
    
@param
    Bit4
    
    Flag to signify whether bit 4 should be input or output.

@param
    Bit3
    
    Flag to signify whether bit 3 should be input or output.
    
@param
    Bit2
    
    Flag to signify whether bit 2 should be input or output.
    
@param
    Bit1
    
    Flag to signify whether bit 1 should be input or output.
    
@param
    Bit0
    
    Flag to signify whether bit 0 should be input or output.

@retval
    0
    
    Success.

@retval
    -1
    
    Failure.@n@n
    
    Please see the descriptions of the internal functions
    InPort() and OutPort() for information on possible values errno may
    have in this case.
    
@note
    A value of false for any of the above bit flags indicates that the
    corresponding port 0 bit should be set to input.  A value of true
    for any of the above bit flags indicates that the corresponding
    port 0 bit should be set to output.
******************************************************************************/

int LoadPort0BitDir_aDIO(
    DeviceHandle Device,
    unsigned char Bit7,
    unsigned char Bit6,
    unsigned char Bit5,
    unsigned char Bit4,
    unsigned char Bit3,
    unsigned char Bit2,
    unsigned char Bit1,
    unsigned char Bit0
);


/******************************************************************************
@brief
    Set the direction (input or output) of all port 1 bits.

@param
    Device
    
    Pointer to a device structure
    
@param
    Dir
    
    Bit direction for port 1 bits.  A value of false means that all
    port 1 bits should be set to input.  A value of true indicates
    that all port 1 bits should be set to output.

@retval
    0
    
    Success.

@retval
    -1
      
    Failure.@n@n
    
    Please see the descriptions of the internal functions
    InPort() and OutPort() for information on possible values errno may
    have in this case.
 *****************************************************************************/

int LoadPort1PortDir_aDIO(DeviceHandle Device, unsigned char Dir);


/******************************************************************************
@brief
    Enable or disableblocking I/O on the device file.  This determines
    whether or not GetIntStatus_aDIO() can block inside the kernel.

@param
    Device
    
    Pointer to a device structure
    
@param
    Enable
    
    Flag which controls whether or not blocking I/O is enabled.
    A value of false means blocking I/Ois disabled and GetIntStatus_aDIO()
    will never block in the kernel waiting for an interrupt to occur.  A
    value of true means blocking I/O is enabled and GetIntStatus_aDIO()
    can block in the kernel waiting for an interrupt to occur.

@retval
    0
    
    Success
    
@retval
    -1
    
    Failure.@n@n
    Please see fcntl(2) man page for information on possible values errno may 
    have in this case.

@note
    The default behaviour when a device file is opened is to disable 
    blocking I/O.

******************************************************************************/

int EnableGetIntStatusWait_aDIO(DeviceHandle Device, unsigned char Enable);


/******************************************************************************
@brief
    Retrieves the next queued up interrupt from the interrupt cache.
    This queued interrupt contains the state of:  the interrupt count,
    port0 data register, port1 data register, compare data register, and
    the control data register.  All of the above variable values returned
    refer to the values at the time that the interrupt occurred.  The last
    two values returned, remaining interrupt and missed interrupts, values
    are retrieved at the time of this function call.  For example, the
    first time this function is called the int_count value will equal 1.
    This is true because at the time the first interrupt occured this
    would be the interrupt count.  On the same token, remaining_interrupts
    may be a number larger than 1, like 10.  This would indicate that 10
    interrupts have occurred, and you are currently retrieving the first
    queued interrupt.

@param
    Device
    
    Pointer to a device structure.
    
@param
    int_count
    
    copy of the global interrupt count at the time of the interrupt
    
@param
    port0
    
    copy of the port 0 data register at the time of the interrupt
    
@param
    port1
    
    copy of the port 1 data register at the time of the interrupt
    
@param
    compare
    
    copy of the compare data register at the time of the interrupt
    
@param    
    control
    
    copy of the control data register at the time of the interrupt
    
@param
    remaining_interrupts
    
    number of interrupt remaining in the interrupt queue
    
@param
    missed_interrupts
    
    number of interrupts ignored due to a full interrupt queue

@retval
    0
    
    Success
    
@retval
    -1
    
    Failures with errno set as follows:@n@n
        @arg \c
        -EBADMSG
        
        Driver read back incorrect number of bytes.  Internally
        somethng is broken in the driver.

@note
    Please see read(2) man page for possible return values, since
    the file operation is utilized to retrieve the data from the
    device.

******************************************************************************/

int GetIntStatus_aDIO(
    DeviceHandle Device,
    unsigned int * int_count,
        unsigned char * port0,
    unsigned char * port1,
    unsigned char * compare,
    unsigned char * control,
    int * remaining_interrupts,
    unsigned int * missed_interrupts );


/******************************************************************************
@brief
    Open the device file associated with the aDIO device.  You must call
    this function before using any of the other library routines.

@param
    Device_ptr
    
    Pointer to a pointer of a device structure.
    
@param
    nDevice
    
    Minor number associated with the device to open.  This
    minor number assignment is determined by the parameters passed into
    the driver at the insmod command in the Makefile for the driver.

@retval
    0

    Success.

@retval
    -1
    
    Failure with errno set as follows:@n@n
        @arg \c
            ENOMEM
            
            Unable to allocate memory for the device structure.
                
        @arg \c
            ENODEV
            
            The device minor number is not valid.  Check the insmod
            line in the Makefile for compatibility of the minor number
            
        @arg \c
            ENXIO
            
            The device io address space is 0.  (not valid)
            This should be correctly configured in the Makefile
            
        @arg \c
            EBUSY
            
            The aDIO device file is already open.
            
        @arg \c
            EBUSY
            
            Allocation of aDIO I/O ports failed.
            
        @arg \c
            EBUSY
            
            The interrupt line requested is being used by another
            device.
            
        @arg \c
            EINVAL
            
            The interrupt line requested is invalid, or no interrupt
            handler is to be associated with interrupt line.
            
        @arg \c
            ENOMEM
            
            Memory for the interrupt action descriptor could not be
            allocated

        Please see the fopen(3) man page and the descriptions of
        EnableInterrupts_aDIO() and ClearDio_aDIO() for information on
        other possible values errno may have in this case.

@note
    Calling this function will also disable digital interrupts and clear
    the aDIO circuitry.
******************************************************************************/

int OpenDIO_aDIO(DeviceHandle * Device, uint32_t nDevice);


/******************************************************************************
@brief
    Device_ptr => Pointer to a pointer of a device structure.
    Read the 8-bit value in the aDIO Compare Register.

@param
    val
    
    Address where Compare Register value will be stored.  Nothing
    is written to this memory location if the function fails.

@retval
    0
    
    Success.

@retval
    -1
        
    Failure.  Please see the descriptions of the internal functions
    InPort() and OutPort() for information on possible values errno may
    have in this case.
******************************************************************************/

int ReadComp_aDIO(DeviceHandle Device, unsigned char *val);


/******************************************************************************
@brief
    Read the 8-bit value in the aDIO Control Register.

@param
    Device_ptr
    
    Pointer to a pointer of a device structure.
    
@param
    val
    
    Address where Control Register value will be stored.  Please see
    the appropriate cpuModule hardware manual for the interpretation
    of the individual bits in the Control Register.  Nothing is
    written to this memory location if the function fails.

@retval
    0
    
    Success.

@retval
    -1
    
    Failure.  Please see the description of the internal function
    InPort() for information on possible values errno may have in this
    case.
******************************************************************************/

int ReadControlRegister_aDIO(DeviceHandle Device, unsigned char *val);


/******************************************************************************
@brief
    Read an 8-bit value from the specified digital port.

@param
    Device_ptr
    
    Pointer to a pointer of a device structure.
    
@param
    PortNum
    
    Digital I/O port to read.  Valid values are 0 and 1.
    
@param
    val
    
    Address where port value will be stored.  Nothing is written to this 
    memory location if the function fails.

@retval
    0

    Success.

@retval
    -1
    
    Failure with errno set as follows:@n@n
        @arg \c
        EINVAL
        
        PortNum is not valid.

@note
    Please see the description of the internal function InPort() for
    information on other possible values errno may have in this case.
******************************************************************************/

int ReadPort_aDIO(DeviceHandle Device, int PortNum, unsigned char *val);


/******************************************************************************
@brief
    Determine whether or not data was strobed into digital input port 0.

@param
    Device_ptr
    
    Pointer to a pointer of a device structure.
    
@param
    val
    
    Address where port 0 strobe flag will be stored.  False will be
    stored here if data was not strobed into port 0.  True will be
    stored here if data was strobed into port 0.  Nothing is written
    to this memory location if the function fails.

@retval
    0
    
    Success.

@retval
    -1
    
    Failure.  Please see the description of ReadControlRegister_aDIO()
    for information on possible values errno may have in this case.
******************************************************************************/

int ReadStrobe0_aDIO(DeviceHandle Device, unsigned char *val);


/******************************************************************************
@brief
    Determine whether or not data was strobed into digital input port 1.

@param
    Device_ptr
    
    Pointer to a pointer of a device structure.
    
@param
    val
    
    Address where port 1 strobe flag will be stored.  False will be
    stored here if data was not strobed into port 1.  True will be
    stored here if data was strobed into port 1.  Nothing is written
    to this memory location if the function fails.

@retval
    0
    
    Success.

@retval
    -1
    
    Failure.  Please see the description of ReadControlRegister_aDIO()
    for information on possible values errno may have in this case.
******************************************************************************/

int ReadStrobe1_aDIO(DeviceHandle Device, unsigned char *val);


/******************************************************************************
@brief
    This function writes a value to port PortNum.

@param
    Device_ptr
    
    Pointer to a pointer of a device structure.
    
@param
    PortNum
    
    This is the port where to write the data.
    
@param
    Data
    
    The data to be written to PortNum.

@retval
    0
    
    Success.

@retval
    < 0
    
    Failure with value returned as follows:@n@n
        @arg \c
        -EINVAL
        The given port PortNum is not valid
    
******************************************************************************/

int WritePort_aDIO(DeviceHandle Device, int PortNum, unsigned char Data);


/******************************************************************************
@brief
    Install a function that will be called in the event of an interupt.
    This function must not return anything and must except an integer.
    The integer value passed to it is actually an error code from the
    WaitForInterrupt_aDIO() function.  The comment header to this function
    described the possible error codes passed to the ISR function.

@param
    Device_ptr
    
    Pointer to a pointer of a device structure.
    
@param
    IsrIrq
    
    function pointer that points to the function that accepts an
    accepts an integer parameter (for the error code) and that
    will be called after interrupts.
    
@param
    policy
    
    Process scheduling policy, the default value is SCHED_OTHER.  
    Other possible values are SCHED_RR (realtime round-robin) or
    SCHED_FIFO (realtime first-in first-out).

@param
    priority
    
    Process priority of the new thread 0 (lowest) to 99 (highest).

@retval
    0
    
    Success
    
@retval
    < 0
    
    Failures with errnos as follows:@n@n
        @arg \c
        -EBUSY
            Another ISR is currently installed and must be removed.
            
        @arg \c
        -EFAULT
            pthread_create has failed.
        
        Also see pthread_create(3) man page for possible error values.

@note
    Any previously installed ISR must be removed before installing a
    new ISR
    
@note
    This function creates another thread that runs WaitForInterrupt_aDIO()
    function.  This thread is joined back in the RemoveISR_aDIO() function.

******************************************************************************/

int InstallISR_aDIO(DeviceHandle Device, void (*IsrIrq)(isr_info_t), int, int );


/******************************************************************************
@brief
    Remove a previously installed ISR function.

@param
    Device_ptr
    
    Pointer to a pointer of a device structure.

@retval
    0

    Success
    
@retval
    < 0
    
    Failures with errnos as follows:@n@n
        @arg \c
        -EBUSY
        
        No previously installed ISR
        
        @arg \c
        -EBUSY
        
        
        Unable to disable interrupts.

        Also see pthread_join(3) man page, EnableInterrupts_aDIO, and
        GetInterruptMode_aDIO for possible error values.

@note
    This function joins the thread that was created in InstallISR_aDIO().

******************************************************************************/

int RemoveISR_aDIO(DeviceHandle Device);


/******************************************************************************
@brief
    All interrupts are queued up by the kernel into an interrupt queue.
    The size of this interrupt queue is defined in aDIO_driver.h.  If you
    are generating interrupts, but did not install an ISR and are not
    making periodical calls to GetIntStatus_aDIO, this function
    can be utilized to prevent the buffer from overflowing and thus
    causing the driver to write warnings to your log file.

@param
    Device_ptr
    
    Pointer to a pointer of a device structure.

******************************************************************************/

void FlushIntQueue_aDIO(DeviceHandle Device);


/******************************************************************************
@brief
    This function returns the global interrupt count since the driver was
    last opened.

@param
    Device_ptr
    
    Pointer to a pointer of a device structure.
    
@param
    interrupt_count
    
    pointer to an unsigned int where the count will be stored.

@retval
    0

    Success.

@retval
    < 0
    
    Failure with value returned as follows:@n@n
        @arg \c
        -EFAULT
        
        interrupt_count pointer is not valid argument.

@note
    The global interrupt count is a 32 bit unsigned number that will count
    up from the time the driver was opened.  This number could eventually
    rollover after 4,294,967,296 interrupts.

******************************************************************************/

int GetIntCount_aDIO(DeviceHandle Device, unsigned int * interrupt_count);


/******************************************************************************
@brief
    This function returns the global interrupt missed count since the driver
    was last opened.

@param
    Device_ptr
    
    Pointer to a pointer of a device structure.
    
@param
    missed_interrupt_count
    
    pointer to an unsigned int where the count will be stored.

@retval
    0
    
    Success.

@retval
    < 0
    
    Failure with value returned as follows:
        @arg \c
        -EFAULT
        
        missed_interrupt_count pointer is not valid argument.

@note
    The global missed interrupt count is a 32 bit unsigned number that will
    count up from the time the driver was opened.  This number could
    eventually rollover after 4,294,967,296 interrupts.

******************************************************************************/
int GetMissedIntCount_aDIO(
        DeviceHandle Device, unsigned int * missed_interrupt_count);

/******************************************************************************
@brief
    Read one byte from the specified offset within aDIO I/O space.

@param
    port_offset
    
    Offset from base I/O address where read should occur.  Valid values are 0 
    through 3.
    
@param
    data_p
    
    Address where data read will be stored.  Nothing is written to this memory 
    location if the function fails.

@retval
    0
    
    Success.
    
@retval
    -1
    
    Failure with errno set as follows:@n@n
        @arg \c
        EFAULT
            
        Address of third ioctl() argument is not valid.

        @arg \c
        EINVAL
        
        Driver's pointer to device structure is NULL.

        @arg \c
        EINVAL
        
        ioctl() request code is not valid.
        
@note
    This function should not be directly called by the user.
 ******************************************************************************/

int
InPort(int hDevice, unsigned char port_offset, unsigned char *data_p);
 
/******************************************************************************
@brief
    Write one byte to the specified offset within aDIO I/O space.

@param
    port_offset
    
    Offset from base I/O address where write should occur.  Valid values are 0 
    through 3.
    
@param
    data
    
    Data to write.

@retval
    0

    Success.

@retval
    -1
    
    Failure with errno set as follows:@n@n
        @arg \c
        EFAULT
            
        Address of third ioctl() argument is not valid.

        @arg \c
        EINVAL
            
        Driver's pointer to device structure is NULL.

        @arg \c
        EINVAL
        
        ioctl() request code is not valid.
        
@note
    This function should not be directly called by the user.
    
 ******************************************************************************/

int
OutPort(int hDevice, unsigned char port_offset, unsigned char data);

/******************************************************************************
@brief
    Clear the digital interrupt status flag in the aDIO Control Register.

@param
    hDevice
    
    Pointer to a device structure

@retval
    0
     
    Success.

@retval
    -1

    Failure.  Please see the descriptions of the internal functions
    InPort() and OutPort() for information on possible values errno may
    have in this case.

@note
    Calling this function acknowledges a digital interrupt.  The aDIO
    circuitry will not generate any more interrupts until the interrupt
    status flag is cleared.
    
@note
    This function should not be directly called by the user.
    
 ******************************************************************************/

int
ClearIrq_aDIO(int hDevice);

/******************************************************************************
@brief
    This function is called with a new thread that does nothing but wait
    for an interrupt to occur, and when it does occur it calls the
    installed ISR function passing it an appropriate error code.

@param
    ptr
    
    null pointer that will be casted to type DeviceHandle

@note
    Error codes passed into ISR
@retval
    0
    
    Successful interrupt detection
    
@retval
    < 0

    Failure with errno set as folllows:@n@n
        @arg \c
        -EBADF
            
        An invalid file descriptor was passed into select.
        This could happen if an invalid DeviceHandle was
        passed into WaitForInterrupt_aDIO().
        
        @arg \c
        -EINTR
            
        A non block signal was caught by the select function.
        This signal may have been intended for the process
        utilizing this library and ISR will be left to sort
        out this error.
        
        @arg \c
        -ENOMEM
            
        Unable to allocate memory for file tables in select
        function
        
        @arg \c
        -ENODATA
            
        Indicates something is broken in the driver
        
        @arg \c
        -EIO
            
        No irq was allocated.
        
@note
    This function should not be directly called by the user.

 ******************************************************************************/

void *WaitForInterrupt_aDIO(void *ptr);
/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif  /* __aDIO_library_h__ */
