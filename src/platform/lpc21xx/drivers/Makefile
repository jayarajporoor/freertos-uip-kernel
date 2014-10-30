

# Include Header Paths.
SRC_DIR=../../..
BASE_DIR=../../../../..
INC = -I.. -I$(SRC_DIR) -I$(SRC_DIR)/include -I$(SRC_DIR)/uip -I. -I../bsp/$(BSP) -I$(BASE_DIR)/eventq/src

# Source and Object Files.
SRC1 = ./*.c
OBJ1 = $(patsubst %.c,%.o,$(wildcard $(SRC1)))

# Build Directory.
BUILD_DIR = ../../../build/lpc21xx

# Make (lpc21xx Specific Source)Flag.
lpc21xx: $(BUILD_DIR)/lib$(BSP).a 

# Make (lpc21xx Specific Source & Unit Test)Flag.
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

