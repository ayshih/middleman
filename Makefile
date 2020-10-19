
CC = g++
EXEC = simulator checker main listen_pps log_telemetry quick

CXXFLAGS = -Inetwork -Wall -pthread

all: $(EXEC)

clean:
	make -C network clean
	rm *.o $(EXEC)

simulator: simulator.c serial.o
	$(CC) -o $@ $^

checker: checker.cpp serial.o ring.o
	$(CC) -o $@ $^

main: main.o ring.o serial.o
	make -C network all
	$(CC) -o $@ $^ network/*.o -pthread

listen_pps: listen_pps.c serial.o
	$(CC) -o $@ $^

log_telemetry: log_telemetry.o
	make -C network all
	$(CC) -o $@ $^ network/*.o -pthread

quick: quick.o
	make -C network all
	$(CC) -o $@ $^ network/*.o -pthread

%.o: %.c %.h
	$(CC) -c $< -o $@
