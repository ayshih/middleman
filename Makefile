
CC = g++
EXEC = simulator checker main listen_pps route_telemetry turn_off inspect log_telemetry quick playback

CXXFLAGS = -Inetwork -IaDIO/include -Wall -pthread

all: $(EXEC)

install: main driver
	systemctl enable $(shell pwd)/middleman.service
	ln -sf $(shell pwd)/rtd-aDIO.conf /etc/modules-load.d/
	ln -sf $(shell pwd)/aDIO/driver/rtd-aDIO.ko /lib/modules/$(shell uname -r)/kernel/drivers/
	depmod

uninstall:
	systemctl stop middleman
	systemctl disable middleman
	rm -f /etc/modules-load.d/rtd-aDIO.conf
	rm -f /lib/modules/$(shell uname -r)/kernel/drivers/rtd-aDIO.ko
	depmod

clean:
	make -C network clean
	make -C aDIO/lib clean
	make -C aDIO/driver clean
	rm *.o $(EXEC)

simulator: simulator.c serial.o
	$(CC) -o $@ $^

sim_spectrometer: sim_spectrometer.c serial.o
	$(CC) -o $@ $^

fake_sip: fake_sip.c serial.o
	$(CC) -o $@ $^

playserial: playserial.c serial.o
	$(CC) -o $@ $^

checker: checker.cpp serial.o ring.o
	$(CC) -o $@ $^

main: main.o ring.o serial.o
	make -C network all
	make -C aDIO/lib all
	$(CC) -o $@ $^ network/*.o -pthread -LaDIO/lib -lrtd-aDIO

driver:
	make -C aDIO/driver driver

listen_pps: listen_pps.c serial.o
	$(CC) -o $@ $^

route_telemetry: route_telemetry.o
	make -C network all
	$(CC) -o $@ $^ network/*.o -pthread

turn_off: turn_off.o
	make -C network all
	$(CC) -o $@ $^ network/*.o -pthread

inspect: inspect.o
	make -C network all
	$(CC) -o $@ $^ network/*.o -pthread

log_telemetry: log_telemetry.o
	make -C network all
	$(CC) -o $@ $^ network/*.o -pthread

quick: quick.o
	make -C network all
	$(CC) -o $@ $^ network/*.o -pthread

playback: playback.o
	make -C network all
	$(CC) -o $@ $^ network/*.o -pthread -static

%.o: %.c %.h
	$(CC) -c $< -o $@
