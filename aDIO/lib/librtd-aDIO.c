/**
    @file

    @brief
        aDIO user library source code.

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

    $Id: librtd-aDIO.c 37457 2009-05-08 14:49:43Z wtate $
*/

#define RETERR  if (ret<0){ return(ret);}

#include <stdlib.h>		//for exit

#include <aDIO_library.h>
#include <aDIO_types.h>

#ifdef __cplusplus

extern "C" {

#endif				// __cplusplus

/*=============================================================================
Non-exported library functions
 =============================================================================*/

	int
	 InPort(int hDevice, unsigned char port_offset, unsigned char *data_p) {
		int ret;
		struct DEVICE_IO_Data wb;

		 wb.Port = port_offset;
		 ret = ioctl(hDevice, ADIO_IOCTL_INB, &wb);
		if (ret == 0) {
			*data_p = wb.Data;
		}
		return ret;
	}

	int
	 OutPort(int hDevice, unsigned char port_offset, unsigned char data) {
		struct DEVICE_IO_Data wb;

		wb.Port = port_offset;
		wb.Data = data;
		return ioctl(hDevice, ADIO_IOCTL_OUTB, &wb);
	}

	int
	 ClearIrq_aDIO(int hDevice) {
		int ret;
		unsigned char dummy = 0;
		unsigned char ControlReg = 0;

		/*
		 * Read aDIO Control Register because we do not want to disturb the
		 * settings of bits 2 through 7
		 */

		ret = InPort(hDevice, rCONTROL, &ControlReg);
		RETERR;

		/*
		 * Clear bits 0 and 1 in aDIO Control Register to select Clear Register
		 */

		ControlReg &= 0xFC;
		ret = OutPort(hDevice, rCONTROL, ControlReg);
		RETERR;

		/*
		 * Read aDIO Multifunction Register to clear digital interrupt
		 */

		return InPort(hDevice, rMULTIFUNCTION, &dummy);
	}

	void *WaitForInterrupt_aDIO(void *ptr) {

		fd_set exception_fds;
		fd_set read_fds;
		int status;
		int_status_info_t int_status;
		DeviceHandle Device;
		Device = (DeviceHandle) ptr;
		isr_info_t info = { 0, NULL, 0, 0 };

		while (1) {
			/*
			 * Set up the set of file descriptors that will be watched for input
			 * activity.  Only the aDIO device file descriptor is of interest.
			 */

			FD_ZERO(&read_fds);
			FD_SET(Device->hDevice, &read_fds);

			/*
			 * Set up the set of file descriptors that will be watched for 
			 * exception activity.  Ony the aDIO file descriptor is of interest.
			 */
			FD_ZERO(&exception_fds);
			FD_SET(Device->hDevice, &exception_fds);

			/*
			 * Wait for the interrupt to happen.  No timeout is given, which 
			 * means the process will not be woken up until either an interrupt 
			 * occurs or a signal is delivered
			 */

			status =
			    select((Device->hDevice + 1), &read_fds, NULL,
				   &exception_fds, NULL);

			/*
			 * Check if this wake up was caused by the wake up ioctl call used
			 * to wake up this process and terminate this thread.  If this flag
			 * is set the interrupts should have already been turned off before
			 * this flag set
			 */
			if (Device->flags & EXIT_FLAG) {
				Device->flags &= ~EXIT_FLAG;
				break;
			}
			/*
			 * Check select() error status
			 */

			if (status == -1) {

				/*
				 * Some error occurred
				 */
				info.status = status;
				(*(Device->isr)) (info);
				break;
			}

			if (status == 0) {

				/*
				 * No file descriptors have data available.  This means 
				 * something is broken in the driver
				 */
				info.status = -ENODATA;
				(*(Device->isr)) (info);
				break;
			}

			if (FD_ISSET(Device->hDevice, &exception_fds)) {

				/*
				 * An exception occurred.  This means that no IRQ line was 
				 * allocated to the device when the driver was loaded.
				 */
				info.status = -EIO;
				(*(Device->isr)) (info);
				break;
			}

			/*
			 * At least one file descriptor has data available and no exception
			 * occurred.  Check the device file descriptor to see if it is 
			 * readable.
			 */

			if (!FD_ISSET(Device->hDevice, &read_fds)) {

				/*
				 * The device file is not readable.  This means something is 
				 * broken in the driver
				 */
				info.status = -ENODATA;
				(*(Device->isr)) (info);
				break;
			}

			/*
			 * An interrupt occurred
			 */
			/* get first interrupt from the driver */
			status =
			    ioctl(Device->hDevice, ADIO_IOCTL_GET_INT,
				  &int_status);

			/* loop through and get all remaining interrupts */

			while (int_status.remaining_interrupts != -1) {

				info.status = status;
				info.interrupt_status =
				    &(int_status.interrupt_status);
				info.remaining_interrupts =
				    int_status.remaining_interrupts;
				info.missed_interrupts =
				    int_status.missed_interrupts;

				/* call isr */
				(*(Device->isr)) (info);
				/* get next interrupt from the driver */
				status =
				    ioctl(Device->hDevice, ADIO_IOCTL_GET_INT,
					  &int_status);
			}	/* while valid interrupts remain in the queue */

		}

		/*
		 * Terminating waiting thread
		 */
		return NULL;
	}

/*=============================================================================
End of non-exported library functions
 =============================================================================*/

/*=============================================================================
Exported library functions
 =============================================================================*/

	int
	 ClearDio_aDIO(DeviceHandle Device) {
		int ret;
		unsigned char ControlReg = 0;

		/*
		 * Read aDIO Control Register because we do not want to disturb the
		 * settings of bits 2 through 7
		 */

		ret = InPort(Device->hDevice, rCONTROL, &ControlReg);
		RETERR;

		/*
		 * Clear bits 0 and 1 in aDIO Control Register to select Clear Register
		 */

		ControlReg &= 0xFC;
		ret = OutPort(Device->hDevice, rCONTROL, ControlReg);
		RETERR;

		/*
		 * Write a dummy value to aDIO Multifunction Register to clear aDIO
		 * circuitry
		 */

		return OutPort(Device->hDevice, rMULTIFUNCTION, 0xFF);
	}

	int
	 CloseDIO_aDIO(DeviceHandle Device) {
		int ret;

		ret = close(Device->hDevice);
		RETERR;

		free(Device);

		return 0;
	}

	int
	 EnableInterrupts_aDIO(DeviceHandle Device, unsigned char Mode) {
		unsigned char Digital_IRQ_Enable = 0;
		unsigned char Digital_IRQ_Mode = 0;
		int ret = -1;
		unsigned char ControlReg = 0;
		unsigned char CompareReg;

		switch (Mode) {
		case DISABLE_INT_MODE:
			Digital_IRQ_Mode = 0;
			Digital_IRQ_Enable = 0;
			break;

		case EVENT_INT_MODE:
			Digital_IRQ_Mode = 0;
			Digital_IRQ_Enable = 1;
			ret =
			    LoadPort0BitDir_aDIO(Device, 0, 0, 0, 0, 0, 0, 0,
						 0);
			RETERR;
			break;

		case STROBE_INT_MODE:
			Digital_IRQ_Mode = 1;
			Digital_IRQ_Enable = 0;
			ret =
			    LoadPort0BitDir_aDIO(Device, 0, 0, 0, 0, 0, 0, 0,
						 0);
			RETERR;

			/*
			 * Compare Register must be read to enable strobe mode
			 */

			ret = ReadComp_aDIO(Device, &CompareReg);
			RETERR;
			break;

		case MATCH_INT_MODE:
			Digital_IRQ_Mode = 1;
			Digital_IRQ_Enable = 1;
			ret =
			    LoadPort0BitDir_aDIO(Device, 0, 0, 0, 0, 0, 0, 0,
						 0);
			RETERR;
			break;

		default:
			errno = EINVAL;
			return -1;
			break;
		}

		/*
		 * Read aDIO Control Register because we do not want to disturb the
		 * settings of bits 0 through 2 and bits 5 through 7
		 */

		ret = InPort(Device->hDevice, rCONTROL, &ControlReg);
		RETERR;

		/*
		 * Clear bits 3 and 4 to set Digital IRQ Mode and Digital IRQ Enable 
		 * bits in Control Register to a known state
		 */

		ControlReg &= 0xE7;
		ret = OutPort(Device->hDevice, rCONTROL, ControlReg);
		RETERR;

		/*
		 * Set Digital IRQ Mode bit in Control Register if strobe or match mode
		 * is enabled
		 */

		if (Digital_IRQ_Mode != 0) {
			ControlReg |= (Digital_IRQ_Mode << 3);
			ret = OutPort(Device->hDevice, rCONTROL, ControlReg);
			RETERR;
		}

		/*
		 * Set Digital IRQ Enable bit in Control Register if event or match mode
		 * is enabled
		 */

		if (Digital_IRQ_Enable != 0) {
			ControlReg |= (Digital_IRQ_Enable << 4);
			ret = OutPort(Device->hDevice, rCONTROL, ControlReg);
			RETERR;
		}

		/*
		 * Make sure Digital IRQ Status bit in Control Register is cleared, thus
		 * acknowledging and clearing any pending interrupt
		 */

		return ClearIrq_aDIO(Device->hDevice);
	}

	int
	 GetInterruptMode_aDIO(DeviceHandle Device, unsigned char *mode) {
		int ret;
		unsigned char ControlReg = 0;
		unsigned char IrqBits = 0;

		ret = InPort(Device->hDevice, rCONTROL, &ControlReg);
		RETERR;

		/*
		 * Mask off all Control Register bits except 3 and 4 and then shift them
		 * so that they are now the least significant bits
		 */

		IrqBits = ControlReg >> 3;
		IrqBits &= 0x3;

		switch (IrqBits) {
		case 0:
			*mode = DISABLE_INT_MODE;
			break;

		case 1:
			*mode = STROBE_INT_MODE;
			break;

		case 2:
			*mode = EVENT_INT_MODE;
			break;

		case 3:
			*mode = MATCH_INT_MODE;
			break;
		}

		return 0;
	}

	int LoadComp_aDIO(DeviceHandle Device, unsigned char Value) {
		int ret;
		unsigned char ControlReg = 0;

		/*
		 * Read aDIO Control Register because we do not want to disturb the
		 * settings of bits 2 through 7
		 */

		ret = InPort(Device->hDevice, rCONTROL, &ControlReg);

		/*
		 * Set bits 0 and 1 in aDIO Control Register to select Compare Register
		 */

		ControlReg &= 0xFC;
		ControlReg |= 3;

		ret = OutPort(Device->hDevice, rCONTROL, ControlReg);
		RETERR;

		/*
		 * Perform actual load of Compare Register
		 */

		return OutPort(Device->hDevice, rMULTIFUNCTION, Value);
	}

	int
	 LoadMask_aDIO(DeviceHandle Device,
		       unsigned char Bit7,
		       unsigned char Bit6,
		       unsigned char Bit5,
		       unsigned char Bit4,
		       unsigned char Bit3,
		       unsigned char Bit2,
		       unsigned char Bit1, unsigned char Bit0) {
		int ret;
		unsigned char ControlReg = 0;
		unsigned char maskRegister;
		/*
		 * Read aDIO Control Register because we do not want to disturb the
		 * settings of bits 2 through 7
		 */

		ret = InPort(Device->hDevice, rCONTROL, &ControlReg);
		RETERR;

		/*
		 * Set bit 1 in aDIO Control Register to select Mask Register
		 */

		ControlReg &= 0xFC;
		ControlReg |= 0x2;

		ret = OutPort(Device->hDevice, rCONTROL, ControlReg);
		RETERR;

		maskRegister = ((Bit7 * 128)
				+ (Bit6 * 64)
				+ (Bit5 * 32)
				+ (Bit4 * 16)
				+ (Bit3 * 8)
				+ (Bit2 * 4)
				+ (Bit1 * 2)
				+ (Bit0 * 1)
		    );

		/*
		 * Perform actual load of Mask Register
		 */

		return OutPort(Device->hDevice, rMULTIFUNCTION, maskRegister);
	}

	int
	 LoadPort0BitDir_aDIO(DeviceHandle Device,
			      unsigned char Bit7,
			      unsigned char Bit6,
			      unsigned char Bit5,
			      unsigned char Bit4,
			      unsigned char Bit3,
			      unsigned char Bit2,
			      unsigned char Bit1, unsigned char Bit0) {
		int ret;
		unsigned char ControlReg = 0;
		unsigned char port0_direction = 0;

		/*
		 * Read aDIO Control Register because we do not want to disturb the
		 * settings of bits 2 through 7
		 */

		ret = InPort(Device->hDevice, rCONTROL, &ControlReg);
		RETERR;

		/*
		 * Set bit 0 in aDIO Control Register to select Port 0 Direction 
		 * Register
		 */

		ControlReg &= 0xFC;
		ControlReg |= 0x1;

		ret = OutPort(Device->hDevice, rCONTROL, ControlReg);
		RETERR;

		port0_direction = ((Bit7 * 128)
				   + (Bit6 * 64)
				   + (Bit5 * 32)
				   + (Bit4 * 16)
				   + (Bit3 * 8)
				   + (Bit2 * 4)
				   + (Bit1 * 2)
				   + (Bit0 * 1)
		    );

		/*
		 * Perform actual load of Port 0 Direction Register
		 */

		return OutPort(Device->hDevice, rMULTIFUNCTION,
			       port0_direction);
	}

	int
	 LoadPort1PortDir_aDIO(DeviceHandle Device, unsigned char Dir) {
		int ret;
		unsigned char ControlReg = 0;

		/*
		 * Read aDIO Control Register because we do not want to disturb the
		 * settings of bits 0, 1, and 3 through 7
		 */

		ret = InPort(Device->hDevice, rCONTROL, &ControlReg);
		RETERR;

		/*
		 * Set bit 2 in aDIO Control Register according to caller's bit 
		 * direction
		 */

		ControlReg &= 0xFB;
		ControlReg |= (Dir * 4);

		return OutPort(Device->hDevice, rCONTROL, ControlReg);
	}

	int EnableGetIntStatusWait_aDIO(DeviceHandle Device,
					unsigned char Enable) {
		int ret, file_flags;

		file_flags = fcntl(Device->hDevice, F_GETFL);
		ret = file_flags;
		RETERR;

		if (Enable) {
			file_flags &= ~O_NONBLOCK;
		} else {
			file_flags |= O_NONBLOCK;
		}

		ret = fcntl(Device->hDevice, F_SETFL, file_flags);
		RETERR;

		return 0;

	}

	int
	 GetIntStatus_aDIO(DeviceHandle Device,
			   unsigned int *int_count,
			   unsigned char *port0,
			   unsigned char *port1,
			   unsigned char *compare,
			   unsigned char *control,
			   int *remaining_interrupts,
			   unsigned int *missed_interrupts) {

		int_status_info_t int_status;
		ssize_t bytes_read;

		bytes_read =
		    read(Device->hDevice, &int_status,
			 sizeof(int_status_info_t));

		/*
		 * Check read() error status
		 */

		if (bytes_read == -1) {

			/*
			 * some errors occured
			 */
			return -1;
		}

		/*
		 * No erros occurred but check the number of bytes read
		 */

		if (bytes_read != sizeof(int_status_info_t)) {

			/*
			 * We were expecting number of bytes equal to the interrupt status
			 * structure size but did not.  Something is wrong with driver.
			 */

			errno = EBADMSG;
			return -EBADMSG;
		}

		*int_count = int_status.interrupt_status.int_count;
		*port0 = int_status.interrupt_status.port0;
		*port1 = int_status.interrupt_status.port1;
		*compare = int_status.interrupt_status.compare;
		*control = int_status.interrupt_status.control;
		*remaining_interrupts = int_status.remaining_interrupts;
		*missed_interrupts = int_status.missed_interrupts;

		return 0;

	}

	int
	 OpenDIO_aDIO(DeviceHandle * Device_ptr, uint32_t nDevice) {
		int ret;
		char devName[1024];

		/*
		 * Upon opening a device size allocate memory to store device
		 * information as pertains to the library
		 */
		if ((*Device_ptr =
		     malloc(sizeof(struct aDIODeviceDescriptor))) == NULL) {
			errno = ENOMEM;
			return -ENOMEM;
		}
		/*
		 * Open the actual device and return an error if unsuccesful
		 */
		snprintf(devName, sizeof(devName), "/dev/rtd-aDIO-%u", nDevice);
		(*Device_ptr)->hDevice = open(devName, (O_RDWR | O_NONBLOCK));
		if ((*Device_ptr)->hDevice == -1) {
			return (*Device_ptr)->hDevice;
		}

		/*
		 * Save the minor number for this device, set all flags to 0,
		 * and intialize isr function pointer to NULL
		 */
		(*Device_ptr)->minor = nDevice;
		(*Device_ptr)->flags = 0;
		(*Device_ptr)->isr = NULL;

		ret = EnableInterrupts_aDIO((*Device_ptr), DISABLE_INT_MODE);
		RETERR;

		ret = ClearDio_aDIO((*Device_ptr));
		RETERR;

		return 0;
	}

	int
	 ReadComp_aDIO(DeviceHandle Device, unsigned char *val) {
		int ret = -1;
		unsigned char ControlReg = 0;

		/*
		 * Read aDIO Control Register because we do not want to disturb the
		 * settings of bits 2 through 7
		 */

		ret = InPort(Device->hDevice, rCONTROL, &ControlReg);
		RETERR;

		/*
		 * Set bits 0 and 1 in aDIO Control Register to select Compare Register
		 */

		ControlReg &= 0xFC;
		ControlReg |= 3;

		ret = OutPort(Device->hDevice, rCONTROL, ControlReg);
		RETERR;

		/*
		 * Perform actual read of Compare Register
		 */

		return InPort(Device->hDevice, rMULTIFUNCTION, val);
	}

	int
	 ReadControlRegister_aDIO(DeviceHandle Device, unsigned char *val) {
		return InPort(Device->hDevice, rCONTROL, val);
	}

	int
	 ReadPort_aDIO(DeviceHandle Device, int PortNum, unsigned char *val) {
		switch (PortNum) {
		case 0:
			return InPort(Device->hDevice, rPORT0DATA, val);
			break;

		case 1:
			return InPort(Device->hDevice, rPORT1DATA, val);
			break;

		default:
			errno = EINVAL;
			return -1;
			break;
		}

		return 0;
	}

	int
	 ReadStrobe0_aDIO(DeviceHandle Device, unsigned char *val) {
		int ret;
		unsigned char control_register;

		ret = ReadControlRegister_aDIO(Device, &control_register);
		if (ret == 0) {

			/*
			 * Mask off all Control Register bits except 7 and shift them so 
			 * that bit 7 is least significant bit
			 */

			*val = (((control_register & 0x80) >> 7) == 1);
		}
		return ret;
	}

	int
	 ReadStrobe1_aDIO(DeviceHandle Device, unsigned char *val) {
		int ret;
		unsigned char control_register;

		ret = ReadControlRegister_aDIO(Device, &control_register);
		if (ret == 0) {

			/*
			 * Mask off all Control Register bits except 5 and shift them so 
			 * that bit 5 is least significant bit
			 */

			*val = (((control_register & 0x20) >> 5) == 1);
		}
		return ret;
	}

	int
	 WritePort_aDIO(DeviceHandle Device, int PortNum, unsigned char Data) {
		switch (PortNum) {
		case 0:
			return OutPort(Device->hDevice, rPORT0DATA, Data);
			break;

		case 1:
			return OutPort(Device->hDevice, rPORT1DATA, Data);
			break;

		default:
			errno = EINVAL;
			return -EINVAL;
			break;
		}

		return 0;
	}

	int InstallISR_aDIO(DeviceHandle Device,
			    void (*IsrIrq) (isr_info_t), int policy,
			    int priority) {

		struct sched_param param;
		int status;

		param.sched_priority = priority;

		/*
		 * Check if the device needs an interrupt removed before installing one
		 */

		if (Device->flags & (ISR_INSTALLED | EXIT_FLAG)) {
			errno = EBUSY;
			return -EBUSY;
		}

		/*
		 *  Set device's function pointer to the new IsrIrq parameter entered
		 */

		Device->isr = IsrIrq;

		status = pthread_create(&(Device->pid), NULL,
					WaitForInterrupt_aDIO, Device);

		/*
		 * Verify that clone process is sucessfully running by checking return 
		 * value
		 */

		if (status != 0) {
			errno = -EFAULT;
			return -1;
		}

		if (getuid() == 0) {
			status =
			    pthread_setschedparam(Device->pid, policy, &param);

			if (status != 0) {
				errno = -EFAULT;
				return -1;
			}
		}

		/*
		 * Mark flag as an ISR installed
		 */

		Device->flags |= ISR_INSTALLED;

		return status;
	}

	int RemoveISR_aDIO(DeviceHandle Device) {

		int ret;
		unsigned char mode = 0;

		/*
		 * Make sure ISR is installed first
		 */

		if (!(Device->flags & (ISR_INSTALLED))) {
			errno = EBUSY;
			return -EBUSY;
		}

		ret = EnableInterrupts_aDIO(Device, DISABLE_INT_MODE);
		RETERR;

		ret = GetInterruptMode_aDIO(Device, &mode);
		RETERR;

		if (mode != DISABLE_INT_MODE) {
			errno = EBUSY;
			return -EBUSY;
		}

		/*
		 * Signal any waiting ISR threads to exit
		 */
		Device->flags |= EXIT_FLAG;

		/*
		 * Wakeup any waiting ISR threads
		 */

		ioctl(Device->hDevice, ADIO_IOCTL_WAKEUP);

		/*
		 * Join back up with ISR thread
		 */
		ret = pthread_join(Device->pid, NULL);
		RETERR;

		/*
		 * mark flags as ISR removed
		 */
		Device->flags &= ~ISR_INSTALLED;

		/*
		 * Make ISR pointer null terminated
		 */

		Device->isr = NULL;

		return 0;
	}
	void FlushIntQueue_aDIO(DeviceHandle Device) {

		ioctl(Device->hDevice, ADIO_IOCTL_FLUSH_INT_QUEUE);

	}
	int GetIntCount_aDIO(DeviceHandle Device, unsigned int *interrupt_count) {

		return ioctl(Device->hDevice,
			     ADIO_IOCTL_GET_GLOB_INT_COUNT, interrupt_count);
	}

	int GetMissedIntCount_aDIO(DeviceHandle Device,
				   unsigned int *missed_interrupt_count) {

		int ret;

		ret = ioctl(Device->hDevice,
			    ADIO_IOCTL_GET_GLOB_MISSED_INTS,
			    missed_interrupt_count);
		RETERR;

		return 0;
	}

/*=============================================================================
End of non-exported library functions
 =============================================================================*/

#ifdef __cplusplus

}

#endif
