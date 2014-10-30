
MEMMAN = heap_1

# Include Header Paths.
INC = -I../../ -I../../include -I. -Ibsp/$(BSP) -I../../../../eventq/src
UT_INC = -I../../ -I. -I./ut

# Source and Object Files.
SRC1 = ./*.c
OBJ1 = $(patsubst %.c,%.o,$(wildcard $(SRC1)))
SRC2 = ../../*.c
OBJ2 = $(patsubst %.c,%.o,$(wildcard $(SRC2)))

MSRC = mem/$(MEMMAN).c
MOBJ = mem/$(MEMMAN).o

BSP_SRC = bsp/$(BSP)/board_config.c
BSP_OBJ = bsp/$(BSP)/board_config.o
# Build Directory.
BUILD_DIR = ../../build/$(PLATFORM)

# Make (lpc21xx Specific Source)Flag.
lpc21xx: $(BUILD_DIR)/lib$(BSP).a $(BUILD_DIR)/start.o
	$(MAKE) -C drivers $@

# Make (lpc21xx Specific Source & Unit Test)Flag.
test: 

# Platform Dependant Source Compilation.
$(BUILD_DIR)/lib$(BSP).a: $(OBJ1) $(OBJ2) $(MOBJ) $(BSP_OBJ)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC Archiver'
	@echo ar -r $@ $(OBJ1) $(OBJ2) $(MOBJ) $(BSP_OBJ)
	@ar -r $@ $(OBJ1) $(OBJ2) $(MOBJ) $(BSP_OBJ)
	@echo 'Finished building target: $@'
	@echo ' '
	

#Source File Compilations.	
$(MOBJ): $(MSRC)
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(CC) $(CFLAGS) $(INC) -c -o$@ $<
	@$(CC) $(CFLAGS) $(INC) -c -o$@ $<
	@echo 'Finished building: $<'
	@echo ' '

$(BSP_OBJ): $(BSP_SRC)
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(CC) $(CFLAGS) $(INC) -c -o$@ $<
	@$(CC) $(CFLAGS) $(INC) -c -o$@ $<
	@echo 'Finished building: $<'
	@echo ' '

#Source File Compilations.	
$(BUILD_DIR)/start.o: start.s
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(CC) $(CFLAGS) $(INC) -c -o$@ $<
	@$(CC) $(CFLAGS) $(INC) -c -o$@ $<
	@echo 'Finished building: $<'
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
	$(RM) $(BUILD_DIR)/*.exe
	$(RM) $(BUILD_DIR)/*.o	
	$(RM) $(BUILD_DIR)/*.a
	$(RM) $(BSP_OBJ)
	$(RM) *.o
	$(RM) mem/*.o
	$(MAKE) -C drivers $@