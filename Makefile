SRC=capture.c
OBJ:=$(SRC:%.c=%.o)
OUT:=$(SRC:%.c=%)

CC=gcc
CFLAGS=
INC=leptonSDKEmb32PUB/
LINK=leptonSDKEmb32PUB/Debug
LIB=LEPTON_SDK
LINKERFLAG=-static


.PHONY: all clean

all:
	$(MAKE) -C$(INC)
	$(CC) $(CFLAGS) -c $(SRC) -o$(OBJ) -I$(INC)
	$(CC) $(OBJ) -o$(OUT) -L$(LINK) -l$(LIB) $(LINKERFLAG)
	
clean:
	rm -f *.o $(OUT)
	$(MAKE) clean -C$(INC)
