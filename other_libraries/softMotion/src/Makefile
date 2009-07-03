#Makefile par Xavier Broquere 04 juin 2007

CC = gcc

all:SoftMotion

SoftMotion: softMotion.o main.o 
	$(CC) -g -Wall -pipe -W -lm -o SoftMotion main.o  softMotion.o

main.o :  main.c softMotionStruct.h softMotion.h
	gcc -g -Wall -pipe -W -c main.c

softMotion.o : softMotion.c softMotionStruct.h softMotionConst.h 
	gcc -g -Wall -pipe -W -c softMotion.c 




clean:
	rm SoftMotion *.o
