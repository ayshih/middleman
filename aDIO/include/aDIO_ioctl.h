/**
    @file

    @brief
        aDIO register, structure, and low level ioctl() request code
        definitions.

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

    $Id: aDIO_ioctl.h 37447 2009-05-07 21:13:07Z wtate $
*/

#ifndef __aDIO_ioctl_h__
#define __aDIO_ioctl_h__

#include <aDIO_version.h>
#include <linux/ioctl.h>

#ifdef __cplusplus

extern "C" {

#endif

/**
 * @brief
 *      Structure for ADIO_IOCTL_INB an ADIO_IOCTL_OUTB transfers
 */
struct DEVICE_IO_Data
{
    /**
     * Port to access
     */
    unsigned char Port;
    /**
     * Data used during transaction
     */
    unsigned char Data;
};

/**
 * @def r_PORT_0_DIO
 * @brief Digital I/O Port 0 Register
 */
#define r_PORT_0_DIO            0
/**
 * @def r_PORT_1_DIO
 * @brief Digital I/O Port 1 Register
 */
#define r_PORT_1_DIO            1
/**
 * @def r_PORT_DIR_DIO
 * @brief Digital I/O Port Direction Register
 */
#define r_PORT_DIR_DIO          2
/**
 * @def r_IRQ_SOURCE_DIO
 * @brief Digital I/O IRQ Source Register
 */
#define r_IRQ_SOURCE_DIO        2
/**
 * @def r_CLEAR_DIO
 * @brief Digital I/O Clear Register
 */
#define r_CLEAR_DIO             2
/**
 * @def r_STATUS_DIO
 * @brief Digital I/O Status Register
 */
#define r_STATUS_DIO            3
/**
 * @def r_MODE_DIO
 * @brief Digital I/O Mode Register
 */
#define r_MODE_DIO              3
/**
 * @def rPORT0DATA
 * @brief Digital I/O Port 0 Register
 */
#define rPORT0DATA              0x0
/**
 * @def rPORT1DATA
 * @brief Digital I/O Port 1 Register
 */
#define rPORT1DATA              0x1
/**
 * @def rMULTIFUNCTION
 * @brief Digital I/O Configuration Register
 */
#define rMULTIFUNCTION          0x2
/**
 * @def rCONTROL
 * @brief Digital I/O Control Register
 */
#define rCONTROL                0x3


/* ioctl codes definition */
#define  __DEVICE_IOCTL_ID_LETTER   UNIQUE_IOCTL_NUM


/**
 * @brief
 *      This ioctl inputs a byte from an aDIO register. 
 */
 
#define ADIO_IOCTL_INB \
    _IOWR(__DEVICE_IOCTL_ID_LETTER, 0x810, struct DEVICE_IO_Data)

/**
 * @brief
 *      This ioctl outputs a byte to an aDIO register. 
 */
#define ADIO_IOCTL_OUTB \
    _IOW(__DEVICE_IOCTL_ID_LETTER, 0x811, struct DEVICE_IO_Data)

/**
 * @brief 
 *      This ioctl retrieves next interrupt to be removed from the interrupt 
 *      queue in the driver. 
 */
#define ADIO_IOCTL_GET_INT \
    _IO(__DEVICE_IOCTL_ID_LETTER, 0x812)

/**
 * @brief
 *      This ioctl retrieves the global interrupt count from the driver. 
 */
#define ADIO_IOCTL_GET_GLOB_INT_COUNT \
    _IO(__DEVICE_IOCTL_ID_LETTER, 0x813)

/**
 * @brief 
 *      This ioctl retrieves the global missed interrupt count from the driver. 
 */
#define ADIO_IOCTL_GET_GLOB_MISSED_INTS \
    _IO(__DEVICE_IOCTL_ID_LETTER, 0x814)

/**
 * @brief
 *      This ioctl wakes up the ISR process.  The EXIT_FLAG is set to terminate 
 *      the ISR process then this ioctl is called to wake it up so that it can 
 *      check the EXIT_FLAG at which point the ISR process will exit.  
 */
#define ADIO_IOCTL_WAKEUP \
    _IO(__DEVICE_IOCTL_ID_LETTER, 0x815)
    
/**
 * @brief
 *      This ioctl flushes all interrupts out of the interrupt queue in the 
 *      driver. 
 */
#define ADIO_IOCTL_FLUSH_INT_QUEUE \
    _IO(__DEVICE_IOCTL_ID_LETTER, 0x816)

#ifdef __cplusplus

} /* extern "C" */

#endif

#endif /* __aDIO_ioctl_h__ */
