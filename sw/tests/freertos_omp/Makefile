# good defaults for many environment variables
include $(FREERTOS_PROJ_ROOT)/default_flags.mk

# manually set CFLAGS to disable some warnings (-Wconversion)
CFLAGS = \
	-ggdb  \
	-march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore \
	-fsigned-char -ffunction-sections -fdata-sections \
	-std=gnu11 \
	-Wall -Wextra -Wshadow -Wformat=2 -Wundef \
	-Wno-unused-parameter -Wno-unused-variable \
	-Os -mno-relax\
	-DFEATURE_CLUSTER=0 -D__PULP__=1 -DDEBUG \
    -Wstack-usage=1024

ASFLAGS = -march=rv32imac -mabi=ilp32 -mno-relax -mno-explicit-relocs

CONFIG_CLUSTER=n

# rtos, pulp
include $(FREERTOS_PROJ_ROOT)/default_srcs.mk

# application name
PROG = freertos_omp

# application/user specific code
USER_SRCS = main.c sw_mailbox.c

# FreeRTOS.h
CPPFLAGS += $(addprefix -I$(USER_DIR)/, ".")

CPPFLAGS += -DportasmHANDLE_INTERRUPT=vSystemIrqHandler
CPPFLAGS += -DUSE_STDIO
## number of error check polls before the tests is conclueded as being successful
CPPFLAGS += -DSTREAM_CHECK_ITERATIONS=2

# Uncomment to disable Additional reigsters (HW Loops)
#CPPFLAGS += -DportasmSKIP_ADDITIONAL_REGISTERS


# todo
VSIM_ARGS = -permit_unmatched_virtual_intf -gBAUDRATE=115200 +BINARY=$(shell pwd)/$(PROG)

# compile, simulation and analysis targets
include $(FREERTOS_PROJ_ROOT)/default_targets.mk

archive_app: $(OBJS)
	mkdir -p build
	riscv32-unknown-elf-ar rcs build/libomptarget_runtime.a $(OBJS)
