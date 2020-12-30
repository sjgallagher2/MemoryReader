# REFRESHER: In a Makefile, a "rule" has the format 'target: dependencies', where both are expected to
# be one or more files. If the target is not a file (e.g. "all") it will always run - it is called a
# phony target. 
# Special variables: $^ (list of dependencies), $@ (target name, or whichever caused rule to run), @< 
# (first prereq), $? (out of date dependencies)
# You can use wildcards (e.g. file*.c)
# Functions have the format '$(func arg0,arg1,...)' e.g $(wildcard *.c */*.c)
# Functions include wildcard, subst, patsubst, filter, dir, notdir, and join.
#------------------------------------

#NOTE: If you get Undefined reference to _init, see basic-arm.txt. Same for Undefined reference
# to _sbrk.


# TARGET NAME
TARGET = MemoryReader# <-- Set target name

# SOURCE FILES
SRCDIR = Src/
HALDIR = Drivers/STM32F4xx_HAL_Driver/Src/
OBJDIR = obj/
SRCS = $(wildcard $(SRCDIR)*.c) $(wildcard $(HALDIR)*.c)


# OBJECT FILES
OBJ = $(addprefix $(OBJDIR),$(notdir ${SRCS:c=o}))

# INCLUDE DIRECTORIES
LOCAL_INCLUDES = -ISrc/ -IInc/ -IDrivers/CMSIS/Device/ST/STM32F4xx/Include/ -IDrivers/STM32F4xx_HAL_Driver/Inc/
STM32_INCLUDES = @stm32_include.inc 

# LIBRARY DIRECTORIES
LDIR = -L/home/sam/Documents/Electronics/ARM/STM32F4xx/Libraries/CMSIS/Lib/

# LIBRARIES
LIBS = -lm

# HEADER DEPENDENCIES
DEPS = 

# COMPILER SETTINGS
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
GDB = arm-none-eabi-gdb
# FLAGS
# g3 = debu with extra symbols, softfp=hard FPU with soft support, VFP version 4 with single precision and 16bit 
# data
# Define the GCC, the CPU, use of HAL driver, little-endian data (ARM core default), architecture
# (technically not needed, because setting mcpu sets this automatically), don't know omit-frame-pointer,
# use thumb instruction set (as opposed to the larger ARM), optimize data and function settings, optimize
# for size is -Os, no standard library, use minimal C library (newlib nano).
CFLAGS = -g3 $(LOCAL_INCLUDES) $(STM32_INCLUDES) -mfloat-abi=softfp -mfpu=fpv4-sp-d16 \
		 -fno-strict-aliasing -fno-dwarf2-cfi-asm \
		 -DGCC_ARMCM4F -mcpu=cortex-m4 \
		 -DUSE_HAL_DRIVER \
		 -mlittle-endian -march=armv7e-m \
		 -fomit-frame-pointer \
		 -mthumb -fdata-sections -ffunction-sections -Os \
		 -nostdlib -specs=nano.specs -specs=nosys.specs \
		 -nostartfiles

LDFLAGS = -T STM32F401VCTx_FLASH.ld -mthumb -u _printf_float -Xlinker -M -Xlinker -M=$(TARGET).map $(LDIR)


# MAKEFILE ENTRY POINT
all : $(TARGET).bin

# COMPILE
$(OBJ) : $(SRCS) $(DEPS)
	@if ! [ -d $(OBJDIR) ]; then mkdir $(OBJDIR); fi
	$(CC) -c $(CFLAGS) $(SRCS) startup/startup_stm32f401xc.s
	mv *.o $(OBJDIR)
	
# LINK
$(TARGET).elf : $(OBJ) 
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJDIR)/startup_stm32f401xc.o $^ $(LIBS)

# CONVERT TO HEX AND BINARY
$(TARGET).hex : $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(TARGET).bin : $(TARGET).elf
	$(OBJCOPY) -O binary $< $@
	$(OBJDUMP) -h -S -d $(TARGET).elf > $(TARGET).list

program : $(TARGET).bin
	st-flash write $(TARGET).bin 0x08000000

debug : 
	openocd -f stlink-v2.cfg -f stm32f4x.cfg &
	$(GDB) $(TARGET).elf -ex 'set logging file gdb.log' \
		-ex 'set logging on' \
		-ex 'target remote localhost:3333' \
		-ex 'monitor reset halt'

# CLEANUP
.PHONY: clean

clean:
	rm -f $(OBJDIR)/*.o
	rm -f $(TARGET).elf $(TARGET).bin $(TARGET).hex $(TARGET).list $(TARGET).map
