# ==============================================================================#
#Copyright (c) 2007 Amrita Research Labs, Amrita Vishwa Vidyapeetham,			#
#Amritapuri, India. All rights reserved.										#
#This document contains confidential and proprietary information				#
#of Amrita Research Labs. No part of this document may be reproduced			#
#in any form, published, broadcasted or transmitted or have an adaptation		#
#made of it. Any violation will be pursued and enforced to the maximum extent	#
#punishable by law.																#
#===============================================================================#

export RUN_FROM = RAM
export BSP = lpc2148_module_test
#export BSP = lpc2148_edu_board
# Compiler Flags.
export CC = arm-elf-gcc
export LD = arm-elf-ld
export CFLAGS = -Wall -g -mlittle-endian -mcpu=arm7tdmi -DRUN_FROM_$(RUN_FROM)

# Platform Flags.
export PLATFORM = lpc21xx
export UT_PLATFORM = lpc21xx

# Build Directory Flags.
BUILD_DIR = build/$(PLATFORM)
PLATFORM_DIR = platform/$(PLATFORM)
UT_PLATFORM_DIR = platform/$(PLATFORM)/ut

# Platform Independant Sources and Objects Files.
SRC = ./*.c
OBJS = $(patsubst %.c,%.o,$(wildcard $(SRC)))

# Include Header Directory Flags.
INC = -I. -Iinclude -I../../eventq/src
PLATFORM_INC = -I$(PLATFORM_DIR)
UT_PLATFORM_INC = -I$(UT_PLATFORM_DIR) 

# Default Module Make.
all: default

# Compiling and generating  Test Programs.
test: default
	$(MAKE) -C $(PLATFORM_DIR) $@
	$(BUILD_DIR)/$(PLATFORM)_ut.exe 3   

# Compiling & LInking Module Source.
default: $(OBJS)
	$(MAKE) -C $(PLATFORM_DIR) $(PLATFORM)	# Compile Platform Specific Code.
	$(MAKE) -C uip default	# Compile uip code	
	
# Generate Documentations.	
doc:
	doxygen ../doxygen-config
	
# Display Help File.
help:
	cat Readme.txt	

#Source File Compilations.	
%.o: %.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(CC) $(CFLAGS) $(INC) $(PLATFORM_INC) -c -o$@ $<
	@$(CC) $(CFLAGS) $(INC) $(PLATFORM_INC) -c -o$@ $<
	@echo 'Finished building: $<'
	@echo ' '

# Cleanup all Directories.	
clean: 
	$(RM) *.o
	$(RM) $(UT_PLATFORM_DIR)/*.o
	$(RM) -r build/doc/*
	$(MAKE) -C $(PLATFORM_DIR) $@
	$(MAKE) -C uip $@	

sync:
	cp ../../linux_eventq/src/*.c .
	cp ../../linux_eventq/src/*.h .			