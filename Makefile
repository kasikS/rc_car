BINARY = rc_car
DEVICE = stm32f103c8
OBJS   = delay_timer.o servo.o serial.o adc.o buttons.o

LIBNAME = opencm3_stm32f1
DEFS    += -DSTM32F1

FP_FLAGS        ?= -msoft-float
ARCH_FLAGS      = -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd

OOCD            ?= openocd
OOCD_INTERFACE  ?= jlink
OOCD_TARGET     ?= stm32f1x

include rules.mk
