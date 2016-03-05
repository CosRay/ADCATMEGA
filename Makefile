ARCHFLAGS=-mthumb -mcpu=cortex-m0plus
CFLAGS=
LDFLAGS=--specs=nano.specs -Wl,--gc-sections,-Map,$(TARGET).map,-Tlink.ld

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
SIZE=arm-none-eabi-size
RM=rm -f

TARGET=main
SRC=$(wildcard *.c)
OBJ=$(patsubst %.c, %.o, $(SRC))

all: build size
build: elf srec
elf: $(TARGET).elf
srec: $(TARGET).srec

clean:
	$(RM) $(TARGET).srec $(TARGET).elf $(TARGET).map $(OBJ)

%.o: %.c
	$(CC) -c $(ARCHFLAGS) $(CFLAGS) -o $@ $<

$(TARGET).elf: $(OBJ)
	$(LD) $(LDFLAGS) -o $@ $(OBJ)

%.srec: %.elf
	$(OBJCOPY) -O srec $< $@

size:
	$(SIZE) $(TARGET).elf
