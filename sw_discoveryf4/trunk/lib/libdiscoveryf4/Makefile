#******************************************************************************
#* File Name          : Makefile
#* Date First Issued  : 02/26/2012
#* Description        : Build library for STM32 Discovery board hardware: libdiscoveryf4.a
#*******************************************************************************/ 
LIBNAME		= libdiscoveryf4

# Prefix the name for the particular toolchain
PREFIX		= arm-none-eabi
# The path for the commands to find the compiler, linker, et al.
TOOLDIR 	= $(HOME)/CodeSourcery/Sourcery_G++_Lite/bin
CC		= $(TOOLDIR)/$(PREFIX)-gcc
LD		= $(TOOLDIR)/$(PREFIX)-ld
OBJCOPY		= $(TOOLDIR)/$(PREFIX)-objcopy
OBJDUMP		= $(TOOLDIR)/$(PREFIX)-objdump
AR		= $(TOOLDIR)/$(PREFIX)-ar

# Current directory
CURDIR=.

# Navigation to libraries in 'svn_discoveryf4'
LIBF4D		= $(CURDIR)/..

# Directory path--includes for .h and .c files common to all routines
LIBCOMMONALL      = $(CURDIR)/../../../../common_all/trunk

# Library directory path
LGCC		= $(HOME)/CodeSourcery/Sourcery_G++_Lite/lib/gcc/arm-none-eabi/4.5.2/thumb2
LOTHER		= $(HOME)/CodeSourcery/Sourcery_G++_Lite/arm-none-eabi/lib/thumb2

# Compiler flags -Os = no optimization, -Os = minimum space, -O3 = fastest
CFLAGS		= -Os -g -Wall -Wextra -fno-common -mcpu=cortex-m4 -mthumb -Wstrict-prototypes
CFLAGS += -I$(LIBCOMMONALL)
CFLAGS += -I$(LIBF4D)
CFLAGS += -I$(LIBF4D)/libusartstm32f4
CFLAGS += -I$(LIBF4D)/libopencm3/cm3
CFLAGS += -I$(LIBF4D)/libopencm3/stm32
CFLAGS += -I$(LIBF4D)/libopencm3/stm32/f4/

AFLAGS  = -mcpu=cortex-m4 -mthumb

ARFLAGS	= rcsv
#ARFLAGS		= rcs

#OBJS	+= default_irq_handler.o
OBJS	+= 4x20lcd.o
OBJS	+= DISCpinconfig.o
OBJS	+= canwinch_ldr.o
OBJS	+= panic_leds.o
OBJS	+= CAN_gateway.o
OBJS	+= adc_mc.o
OBJS	+= cic_filter_l_N2_M3.o
OBJS	+= countdowntimer.o
OBJS	+= spi2rw.o
OBJS	+= switchdebounce.o
OBJS	+= switchdebounce2.o



# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
endif

all: $(LIBNAME).a

$(LIBNAME).a: $(OBJS)
	@printf "  AR      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(AR) $(ARFLAGS) $@ $^

%.o: %.c
	@printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

clean:
	@printf "  CLEAN   $(subst $(shell pwd)/,,$(OBJS))\n"
	$(Q)rm -f *.o
	@printf "  CLEAN   $(LIBNAME).a\n"
	$(Q)rm -f $(LIBNAME).a

.PHONY: clean

