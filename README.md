# BOOMS Interface Computer

This repository contains the main executable program and various test programs for the Interface Computer on the Balloon Observations Of Microburst Scales (BOOMS) ballon project.
The main executable program is based heavily on the analogous program for the Aspect Computer on the GRIPS balloon, which in turn was based on the analogous program for the Solar Aspect System (SAS) computer on the HEROES balloon.
The network code re-uses the GRIPS network code: https://github.com/GRIPS/network.

## Executables
* `main`: The main executable program.
* `simulator`: Simulates the serial packets from one of the imagers using a user-specified event rate.
* `listen_pps`: Listens on the CTS serial pin of the user-specified device for the PPS signal.
* `checker`: Checks the stream of serial packets from `simulator`. 
* `route_telemetry`: Tells the Interface Computer (specified by IP) the IP of the flight computer.
* `turn_off`: Turn off the Interface Computer (specified by IP)
* `inspect`: Inspect the contents of a telemetry log file, and write out extracted/filtered data.
* `log_telemetry`: Saves all received telemetry packets.
  Be aware that the `network` submodule also contains useful utilities (`listen_telemetry`, `fake_telemetry`, etc.).
* `quick`: A simple commander for testing locally

## Installation
The main program runs as a service.
Modify the path in `middleman.service` appropriately and then copy it to `/etc/systemd/system/`.
To enable the service, run `sudo systemctl enable middleman`.
Here are commands for operating the service:
* `sudo systemctl start middleman`
* `sudo systemctl stop middleman`
* `systemctl status middleman`
* `journalctl -u middleman`
