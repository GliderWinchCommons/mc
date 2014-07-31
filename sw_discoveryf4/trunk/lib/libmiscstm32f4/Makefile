#*******************************************************************************
#* File Name          : Makefile
#* Date First Issued  : 02/29/2012 deh
#* Description        : Build library: libmiscstm32f4.a
#*******************************************************************************/ 
LIBNAME		= libmiscstm32f4

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
LIBF4D		=$(CURDIR)../

# Library directory path
LGCC		= $(HOME)/CodeSourcery/Sourcery_G++_Lite/lib/gcc/arm-none-eabi/4.5.2/thumb2
LOTHER		= $(HOME)/CodeSourcery/Sourcery_G++_Lite/arm-none-eabi/lib/thumb2

# Navigation to libraries in 'svn_discoveryf4'
LIBF4D		= $(CURDIR)/..

# Directory path--includes for .h and .c files common to all routines
LIBCOMMONALL      = $(CURDIR)/../../../../common_all/trunk

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

OBJS =
OBJS	+= busfreq.o
OBJS	+= clockspecifysetup.o
OBJS	+= newlib_support.o
OBJS	+= printf.o
OBJS	+= systick1.o
OBJS	+= SYSTICK_24bitdiff.o
OBJS	+= SYSTICK_getcount32.o
OBJS	+= SYSTICK_getcount64.o
OBJS	+= SYSTICK_getcountdiv256.o
OBJS	+= SYSTICK_systickduration.o
OBJS	+= reset.o 
#OBJS	+= USB_PC_gateway.o
OBJS	+= DTW_counter.o



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

