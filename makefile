all: clean ass3

ass3: ass3.o drone.o scheduler.o printer.o target.o
	gcc -m32 -Wall -g ass3.o drone.o scheduler.o printer.o target.o -o ass3
	rm -f *.o
	
ass3.o: ass3.s
	nasm -f elf ass3.s -g -o ass3.o

drone.o: drone.s
	nasm -f elf drone.s -g -o drone.o
	
scheduler.o: scheduler.s
	nasm -f elf scheduler.s -g -o scheduler.o

printer.o: printer.s
	nasm -f elf printer.s -g -o printer.o

target.o: target.s
	nasm -f elf target.s -g -o target.o

.PHONY: clean
clean:
	rm -f *.o ass3
	
	
