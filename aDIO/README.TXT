Linux Software (aDIO)

SWP-700010042 rev J

Version 03.02.00

Copyright (C), RTD Embedded Technologies, Inc.  All Rights Reserved.

Using this software indicates acceptance of the RTD End-User Software 
License Agreement.  See the file LICENSE.TXT.



------------------
Supported Hardware
------------------

This software supports any RTD cpuModule having aDIO capability.  The also 
includes cpuModules with an external PCI-to-ISA bridge board.


-------------------------
Supported Kernel Versions
-------------------------

This software supports all Linux kernels after 2.6.19.  It has been tested with 
the following Linux distributions and kernel versions:

    * openSUSE 11.4 (unmodified kernel 2.6.37)
    * openSUSE 13.2 (unmodified kernel 3.16)
    * openSUSE 13.2 (with kernel 4.2 from kernel.org)
    
Note that due to kernel API differences between kernel versions, RTD cannot
guarantee compatibility with kernels and distributions not listed above.  It may
be necessary to modify the kernel code and/or Makefiles for your specific Linux
environment.

During the compile process /bin/bash is utilized to determine the specific
RTD CPU board that it is currently compiling for.  It is required to have
a version of bash in your distribution's /bin directory.  


------------------
Software Structure
------------------

The diagram below illustrates the general structure of the software.

        +------------------+        +----------------------------+
        | Example Programs |   OR   | User-Developed Application |
        | Provided by RTD  |        |                            |
        +------------------+        +----------------------------+
                 |                                |
                 |                                |
                 +--------------------------------+
                               |
                               | High-level C Functions
                               |
                          +---------+
                          | Library |
                          +---------+
                               |
                               | Low-level IOCTLs
    User Mode                  |
    ==================================================================
    Kernel Mode                |
                           +--------+
                           | Driver |
                           +--------+
                               |
                               | Kernel Services
                               |
    ==================================================================
                            Hardware


------
Driver
------

The directory driver/ contains source code related to the driver.


In order to use the driver, one must first compile it, load it into the kernel,
and create device files for the board(s).  To do this, issue the following
commands while sitting in the driver/ directory:

    * make
    * make insmod


The driver object is named rtd-aDIO.ko.  


The following arguments to insmod are accepted when loading the driver:

    * debug
    Bit mask which controls what debug information is printed about driver
    operation.  This argument has meaning only if the driver was compiled
    with -DDEBUG.  Valid bit mask values can be found in
    include/aDIO_driver.h.  If this option is not given and the driver
    was compiled with -DDEBUG, no debug information is printed.

    * io/irq
        Assigns the base addresses and irq's to available minor numbers
    IE.  "io=0xAA,0xBB,0xCC irq=D,E,F" means the first available minor
    number (0) is assigned to the device with base address AA and irq D.
    The next available minor number is assigned to the device with base
    address BB and irq E and so on and so forth.  


Here are some example insmod command lines along with a description of what
each does (each example assumes that your current directory is driver/):

    - /sbin/insmod ./rtd-aDIO.ko io=0x450 irq=6
        Load the driver into the kernel.  The driver produces no debug
        output.  An aDIO board with base address of 450 will be given
        irq 6 and minor number 0.  


It is recommended that you not build the driver module with the -DDEBUG switch.
When the driver is built with DEBUG defined, extra code which prints debugging
information is compiled into the driver  This code causes a driver performance
penalty and extra system load in the form of more work for syslogd (the system
log daemon).  Also, a small delay is added to each access of an address in the
board's I/O space.



-----------------
Library Interface
-----------------

The directory lib/ contains source code related to the user library.


The aDIO library is created with a file name of librtd-aDIO.a and is a
statically linked library.


These functions are prototyped in the file include/aDIO_library.h; this header
file must be included in any code which wishes to call library functions.  
For usage details please read the header file's brief descriptions of its
public library functions.

To build the library, issue the command "make" within lib/.



----------------
Example Programs
----------------

The directory examples/ contains source code related to the example programs.
This code demonstrates key features of the aDIO facility.


The following example programs are provided:

    * ints
    Demonstrates the different interrupt modes.
    
    * ports
    Demonstrates digital input and output.

To build the example programs, issue the command "make" within examples/.



------------
Header Files
------------

The directory include/ contains all header files needed by the driver,
libraries, example programs, and user applications.



-----------------
Known Limitations
-----------------

 1. The major argument on the insmod command is no longer supported by the
    driver.  Instead, the driver now asks the kernel to dynamically assign a
    character major number when the kernel module is loaded.

 2. On Geode, VIA Eden, and Pentium M processors if you disable the aDIO's IRQ
    in the BIOS, aDIO functionality is still enabled.  In this case, the
    driver will load and prints a warning message to the system log that
    interrupts are disabled.  You will not be able to use any aDIO feature
    relating to interrupts.  You still can use the digital input and output
    features though.  Make sure that you have configured the aDIO feature
    appropriately in the BIOS for how you wish to use it.

 3. On Geode, VIA Eden, and Pentium M processors if you disable the aDIO in
    the BIOS or set the multiport to function as a parallel port or floppy
    drive port, aDIO functionality is disabled.  In this case, the driver will
    load and prints a warning message to the system log that the onboard aDIO
    is disabled in BIOS.  You will not be able to use any aDIO features unless
    you have an aDIO in your system external to your CPU board, such as an ISA
    Bridge card.  In this case the io address and irq of this external aDIO
    will need to be specified in the insmod command in the Makefile, in order
    to utilize this device with the driver.  Make sure that you have
    configured the aDIO feature appropriately in the BIOS for how you wish to
    use it.

 4. In version 2.00.00 the interrupt scheme was changed.  Interrupts are no
    longer handled by signals from the kernel to user-space.  We now utilize
    a poll/select methodology treating the driver as a readable file instead
    of hardware.  This adds reliability to the code by removing the
    possibility of conflicting signals in the system disrupting the aDIO
    driver.  Now when an ISR is inserted into the driver a new process is
    cloned off that has the single purpose of calling the ISR after an
    interrupt.  To reduce latency in the ISR call back this process should be
    given as high as a priority as possible.


-------------------------
Getting Technical Support
-------------------------

If you require additional support with this product, or any other products
from RTD Embedded Technologies, contact us using the information below:

    RTD Embedded Technologies, Inc.
    103 Innovation Blvd.
    State College, PA 16803 USA

    Telephone: (814) 234-8087
    Fax: (814) 234-5218
    Sales Information and Quotes: sales@rtd.com
    Technical Assistance: techsupport@rtd.com
    Web Site: http://www.rtd.com
