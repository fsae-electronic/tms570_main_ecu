TARGET = tms570_main_ecu

# Toolchain -- custom big-endian arm-none-eabi-gcc (with C++ support),
# built from the toolchain repo's Dockerfile:
#
#   git clone git@github.com:fsae-electronic/tms570_debug_test.git
#   docker build -t tms570-toolchain tms570_debug_test/
#   docker run --rm -v "$(CURDIR):/project" -w /project tms570-toolchain make
#
TOOLCHAIN_BIN ?= /opt/arm-none-eabi-be/bin

ifneq ($(TOOLCHAIN_BIN),)
  _T := $(TOOLCHAIN_BIN)/arm-none-eabi-
else
  _T := arm-none-eabi-
endif

CC      = $(_T)gcc
AS      = $(_T)gcc
LD      = $(_T)gcc
OBJCOPY = $(_T)objcopy
SIZE    = $(_T)size

CPU_FLAGS = -mcpu=cortex-r4 -mfpu=vfpv3-d16 -mfloat-abi=hard -mbig-endian -mbe32 -marm

# Set BENCH_TEST_SIMULATE_CAN=1 to build with the synthetic CAN-data
# generator task instead of relying on real CAN hardware being connected.
BENCH_TEST_SIMULATE_CAN ?= 0
ifeq ($(BENCH_TEST_SIMULATE_CAN),1)
  EXTRA_DEFS += -DBENCH_TEST_SIMULATE_CAN
endif

CFLAGS  = $(CPU_FLAGS) -Os -Wall -Wextra -ffunction-sections -fdata-sections -g3 $(EXTRA_DEFS)
ASFLAGS = $(CPU_FLAGS) -x assembler-with-cpp

LDSCRIPT = linker/link.ld
LDFLAGS  = $(CPU_FLAGS) -nostartfiles -Wl,--gc-sections -Wl,-Map=$(TARGET).map -T $(LDSCRIPT)

INCLUDES = \
    -Iinclude \
    -Iapp \
    -Ifreertos/port

# ---- Source files -----------------------------------------------------------

SRCS_C  := $(wildcard app/*.c)
SRCS_C  += $(wildcard drivers/*.c)
SRCS_C  += $(wildcard startup/*.c)
SRCS_C  += $(wildcard freertos/kernel/*.c)
SRCS_C  += $(wildcard freertos/port/*.c)
SRCS_C  += $(wildcard port/*.c)

SRCS_S  := $(wildcard startup/*.s)
SRCS_S  += $(wildcard freertos/port/*.s)

OBJS := $(patsubst %.c, build/%.o, $(SRCS_C))
OBJS += $(patsubst %.s, build/%.o, $(SRCS_S))

# ---- Targets ----------------------------------------------------------------

.PHONY: all clean size docker-build

all: $(TARGET).elf

$(TARGET).elf: $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $^ -lgcc -lc -lm -lnosys
	$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin
	cp $(TARGET).elf $(TARGET).out

build/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

build/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) $(INCLUDES) -c $< -o $@

size: $(TARGET).elf
	$(SIZE) $<

DOCKER_IMAGE ?= tms570-toolchain
docker-build:
	docker run --rm -v "$(CURDIR):/project" -w /project $(DOCKER_IMAGE) make

clean:
	rm -rf build $(TARGET).elf $(TARGET).bin $(TARGET).out $(TARGET).map
