SRCDIR = src
BUILDDIR = build
PROJECT_NAME = projeto

MCU = atmega328p
F_CPU = 20000000UL # PROVAVELMENTE VAI MUDAR PRO SEU CLOCK
AVRDUDE_PROGRAMMER = arduino
AVRDUDE_PORT = /dev/ttyUSB0 #PROVAVELMENTE VAI PRECISAR ALTERAR PRA SUA PORTA
FUSES = -U lfuse:w:0xFF:m -U hfuse:w:0xDE:m -U efuse:w:0xFD:m

CC = avr-gcc
OBJCOPY = avr-objcopy
AVRDUDE = avrdude

# Ferramentas WinAVR (assumindo PATH configurado ou caminhos completos)
# Se você não adicionou WinAVR ao seu PATH, use os caminhos completos aqui:
# CC = "C:\WinAVR-20100110\bin\avr-gcc.exe"
# OBJCOPY = "C:\WinAVR-20100110\bin\avr-objcopy.exe"
# AVRDUDE = "C:\WinAVR-20100110\bin\avrdude.exe"
# AVRDUDE_CONF = "C:\WinAVR-20100110\bin\avrdude.conf"

# Compiler flags
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall -std=c99 -g
# Linker flags
LDFLAGS = -mmcu=$(MCU)

# Source files
# SRC_FILES = main.c usart.c
SRC_FILES = agatha2.c # AQUI É O NOME DO ARQUIVO QUE VC QUER COMPILAR, EU USEI AGATHA2, mas pode ser qlqr outro
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
	# Set fuse bits for external crystal, long startup time
	# LFUSE = 0xFF (external crystal, high freq, long startup)
	# HFUSE = 0xDE (default for Arduino Uno, disable brown-out, enable SPIEN)
	# EFUSE = 0xFD (default, no special options)
	# $(AVRDUDE) -p $(MCU) -c $(AVRDUDE_PROGRAMMER) -P $(AVRDUDE_PORT) $(FUSES)

clean:
	rm -rf $(BUILDDIR)
