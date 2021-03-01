
CC = g++
EXEC = simulator checker main listen_pps route_telemetry turn_off inspect log_telemetry quick

CXXFLAGS = -Inetwork -IaDIO -Wall -pthread

all: $(EXEC)

clean:
	make -C network clean
	make -C aDIO clean
	rm *.o $(EXEC)

simulator: simulator.c serial.o
	$(CC) -o $@ $^

checker: checker.cpp serial.o ring.o
	$(CC) -o $@ $^

main: main.o ring.o serial.o
	make -C network all
	make -C aDIO all
	$(CC) -o $@ $^ network/*.o -pthread -LaDIO -lrtd-aDIO

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

%.o: %.c %.h
	$(CC) -c $< -o $@
