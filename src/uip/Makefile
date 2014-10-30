# ==============================================================================#
#Copyright (c) 2007 Amrita Research Labs, Amrita Vishwa Vidyapeetham,			#
#Amritapuri, India. All rights reserved.										#
#This document contains confidential and proprietary information				#
#of Amrita Research Labs. No part of this document may be reproduced			#
#in any form, published, broadcasted or transmitted or have an adaptation		#
#made of it. Any violation will be pursued and enforced to the maximum extent	#
#punishable by law.																#
#===============================================================================#


# Include Header Paths.
INC = -I../ -I../include -I. -I../../../eventq/src -I../platform/$(PLATFORM) -I../platform/$(PLATFORM)/drivers -I-I../platform/$(PLATFORM)/bsp/$(BSP)

# Source and Object Files.
SRC1 = ./*.c
OBJ1 = $(patsubst %.c,%.o,$(wildcard $(SRC1)))

# Build Directory.
BUILD_DIR = ../build/$(PLATFORM)

# Make default
default: $(BUILD_DIR)/lib$(BSP).a

# Make test
test: 

# Platform Dependant Source Compilation.
$(BUILD_DIR)/lib$(BSP).a: $(OBJ1)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC Archiver'
	@echo ar -r $@ $(OBJ1) 
	@ar -r $@ $(OBJ1) 
	@echo 'Finished building target: $@'
	@echo ' '
	

#Source File Compilations.	
%.o: %.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(CC) $(CFLAGS) $(INC) -c -o$@ $<
	@$(CC) $(CFLAGS) $(INC) -c -o$@ $<
	@echo 'Finished building: $<'
	@echo ' '

# Cleanup
clean:
	$(RM) *.o

