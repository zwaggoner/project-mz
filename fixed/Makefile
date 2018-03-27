ifeq ($(subst coe.neu.edu,,$(shell hostname --domain)),$(shell hostname --domain))
# not COE must be local install o
INCLUDE = /home/build/work/linux/include
CROSS_COMPILE = /usr/bin/arm-linux-gnueabihf-
else
INCLUDE=/ECEnet/Apps1/esl/linux-4.9-zed/include/
CROSS_COMPILE=/ECEnet/Apps1/esl/gcc-7.2.1-arm/bin/arm-linux-gnueabihf-
endif

FLAGS=-g -c

fixed.a : fixed.o
	ar rcs fixed.a fixed.o

fixed.o : fixed.c
	$(CROSS_COMPILE)gcc $(FLAGS) -I $(INCLUDE) -o fixed.o fixed.c 

clean: 
	rm -f *.o 

clean-all: 
	rm -f *.o *.a
