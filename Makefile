SRC=capture.c
OBJ:=$(SRC:%.c=%.o)
OUT:=$(SRC:%.c=%)

CC=gcc
CFLAGS=-O3
INC=leptonSDKEmb32PUB/
LINK=leptonSDKEmb32PUB/Debug/
LIB=LEPTON_SDK
LINKERFLAG=


.PHONY: all clean

all:
	$(MAKE) -C$(INC)
	$(CC) $(CFLAGS) -c $(SRC) -o$(OBJ) -I$(INC)
	$(CC) $(OBJ) -o$(OUT) -L$(LINK) -l$(LIB) $(LINKERFLAG)
	
clean:
	rm -f *.o $(OUT) *.pgm
	$(MAKE) clean -C$(INC)
