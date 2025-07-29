SRCDIR = src
BUILDDIR = build
PROJECT_NAME = projeto

MCU = atmega328p
F_CPU = 16000000UL
AVRDUDE_PROGRAMMER = arduino
AVRDUDE_PORT = /dev/ttyUSB0
FUSES = -U lfuse:w:0xFF:m -U hfuse:w:0xDE:m -U efuse:w:0xFD:m

CC = avr-gcc
OBJCOPY = avr-objcopy
AVRDUDE = avrdude

# Compiler flags
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall -std=c99 -g
# Linker flags
LDFLAGS = -mmcu=$(MCU)

# Source files
SRC_FILES = main.c usart.c
SRCS = $(addprefix $(SRCDIR)/, $(SRC_FILES))
# Object files (automatically generated from SRCS)
OBJS = $(patsubst $(SRCDIR)/%.c, $(BUILDDIR)/%.o, $(SRCS))
# Executable name
TARGET = $(BUILDDIR)/$(PROJECT_NAME)

all: $(TARGET).hex

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(TARGET).elf: $(OBJS) | $(BUILDDIR)
	$(CC) $(LDFLAGS) -o $@ $(OBJS)

$(BUILDDIR)/%.o: $(SRCDIR)/%.c | $(BUILDDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

flash: $(TARGET).hex
	$(AVRDUDE) -p $(MCU) -c $(AVRDUDE_PROGRAMMER) -P $(AVRDUDE_PORT) -U flash:w:$<:i

fuses:
	# Set fuse bits for external 16MHz crystal, long startup time
	# LFUSE = 0xFF (external crystal, high freq, long startup)
	# HFUSE = 0xDE (default for Arduino Uno, disable brown-out, enable SPIEN)
	# EFUSE = 0xFD (default, no special options)
	# $(AVRDUDE) -p $(MCU) -c $(AVRDUDE_PROGRAMMER) -P $(AVRDUDE_PORT) $(FUSES)

clean:
	rm -rf $(BUILDDIR)
