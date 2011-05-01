# Makefile for serialsnoop.

CFLAGS = -Wall -g

all: serialsnoop

serialsnoop: serialsnoop.o
serialsnoop.o: serialsnoop.c version.h
version.h: version-template.h VERSION
	./make-version-header.sh


clean:
	rm -f serialsnoop.o serialsnoop version.h

