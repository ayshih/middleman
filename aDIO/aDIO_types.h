/**
    @file

    @brief
        Type definitions used both in the kernel driver and in the user
        library

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

    $Id: aDIO_types.h 37447 2009-05-07 21:13:07Z wtate $
*/


#ifndef __adio_types_h__
#define __adio_types_h__



/*=============================================================================
Structure which defines the interrupt status information returned on a read(2)
system call
 =============================================================================*/



/**
 * @defgroup aDIO_Types_Header aDIO specific type definitions
 * @{
 */
/**
 * @brief
 *      Interrupt status information
 */
struct int_status
{
    /**
     * Number of interrupts that occurred
     */
    unsigned int    int_count;
    /**
     * The state of port 0 during interrupt
     */
    unsigned char   port0;
    /**
     * The state of port 1 during interrupt
     */
    unsigned char   port1;
    /**
     * Value of compare during interrupt
     */
    unsigned char   compare;
    /**
     * The state of control byte during interrupt
     */
    unsigned char   control;
};
/**
 * @brief
 *      Interrupt status information type
 */
typedef struct int_status int_status_t;

/**
 * @brief
 *      aDIO status information
 */
struct int_status_info
{
    /**
     * Interrupt status information
     */
    int_status_t interrupt_status;
    /**
     * Number of interrupts whose information we haven't read yet.
     */
    int remaining_interrupts;
    /**
     * Number of interrupt missed due to full queue in the driver
     */
    unsigned int missed_interrupts;
};
/**
 * @brief
 *      aDIO status information type
 */
typedef struct int_status_info int_status_info_t;
/**
 * @brief
 *      User space ISR parameter
 */
struct isr_info
{
    /**
     * Return code
     */
    int status;
    /**
     * Interrupt status
     */
    int_status_t * interrupt_status;
    /**
     * Remaining interrupts
     */
    int remaining_interrupts;
    /**
     * Missed interrupts
     */
    unsigned int missed_interrupts;
};
/**
 * @brief
 *      User space ISR parameter type
 */
typedef struct isr_info isr_info_t;
/**
 * @}
 */
#endif  /* #ifndef __adio_types_h__ */
