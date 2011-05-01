# Makefile for serialsnoop.

CFLAGS = -Wall -g

all: serialsnoop man

serialsnoop: serialsnoop.o
serialsnoop.o: serialsnoop.c version.h
version.h: version-template.h VERSION
	./make-version-header.sh

.PHONY: man
man: man/serialsnoop.1

man/serialsnoop.1: man/serialsnoop.pod
	pod2man --center="User Commands" \
		--release="User Commands" \
		man/serialsnoop.pod \
		man/serialsnoop.1


clean:
	rm -f serialsnoop.o serialsnoop version.h man/serialsnoop.1

