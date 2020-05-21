USE_BOOTLOADER := true

F0_TARGETS  := $(TARGET)

FLASH_SIZE  := 64
STM_CHIP    := stm32f051x8
TARGET_SRC  := \

ifneq ($(USE_BOOTLOADER),true)
TARGET_FLAGS := -D STM32F051x8
else
TARGET_FLAGS := -D STM32F051x8 -D USE_BOOTLOADER
endif
