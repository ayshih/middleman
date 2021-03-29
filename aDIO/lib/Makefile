#
#       FILE NAME: Makefile
#
#       FILE DESCRIPTION: Make description file for building aDIO library
#
#       PROJECT NAME: Linux Software (aDIO)
#
#       PROJECT VERSION: (Defined in README.TXT)
#
#       Copyright 2004 RTD Embedded Technologies, Inc.  All Rights Reserved.
#

CC=gcc
DEBUG_FLAGS=
INCLUDE_FLAGS=-I../include
OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall
CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)

all:	librtd-aDIO.a

librtd-aDIO.a:	librtd-aDIO.o
	ar -cru librtd-aDIO.a librtd-aDIO.o

librtd-aDIO.o:	librtd-aDIO.c
	$(CC) $(CFLAGS) -c librtd-aDIO.c

clean:
	rm -f *.o *.a *~
