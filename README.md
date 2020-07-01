# BOOMS Interface Computer

This repository contains the main executable program and various test programs for the Interface Computer on the Balloon Observations Of Microburst Scales (BOOMS) ballon project.
The main executable program is based heavily on the analogous program for the Aspect Computer on the GRIPS balloon, which in turn was based on the analogous program for the Solar Aspect System (SAS) computer on the HEROES balloon.
The network code re-uses the GRIPS network code: https://github.com/GRIPS/network.

## Executables
* `main`: The main executable program.
* `simulator`: Simulates the serial packets from one of the imagers using a user-specified event rate.
* `listen_pps`: Listens on the CTS serial pin of the user-specified device for the PPS signal.
* `checker`: Checks the stream of serial packets from `simulator`. 
