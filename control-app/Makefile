SRC := ./src
INC := ./inc
OBJ := ./obj
FLAGS := -g

ifeq ($(subst coe.neu.edu,,$(shell hostname --domain)),$(shell hostname --domain))
# not COE must be local install o
CROSS_COMPILE ?= /usr/bin/arm-linux-gnueabihf-
ZEDBOARD_IP?=1.2.2.1
else 
# COE
CROSS_COMPILE ?= /ECEnet/Apps1/esl/gcc-7.2.1-arm/bin/arm-linux-gnueabihf-
ZEDBOARD_IP?=qemu
endif
UPLOAD_PATH ?= /home/root/

$(shell mkdir -p obj)

control-app : $(OBJ)/control-app.o $(OBJ)/motor.o $(OBJ)/potentiometer.o $(OBJ)/controllers.o
	$(CROSS_COMPILE)gcc $(FLAGS) -I$(INC) $^ -o $@ -lm

upload : control-app
	scp $^ root@$(ZEDBOARD_IP):$(UPLOAD_PATH)

$(OBJ)/control-app.o : $(SRC)/control-app.c 
	$(CROSS_COMPILE)gcc $(FLAGS) -c -I $(INC) $< -o $@

$(OBJ)/controllers.o : $(SRC)/controllers.c
	$(CROSS_COMPILE)gcc $(FLAGS) -c -I $(INC) $< -o $@

$(OBJ)/motor.o : $(SRC)/motor.c
	$(CROSS_COMPILE)gcc $(FLAGS) -c -I $(INC) -I../kmotor/ $< -o $@

$(OBJ)/potentiometer.o : $(SRC)/potentiometer.c
	$(CROSS_COMPILE)gcc $(FLAGS) -c -I $(INC) $< -o $@ 

.PHONY: clean
clean : 
	rm $(OBJ)/*.o control-app
