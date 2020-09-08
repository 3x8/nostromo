
TARGET_DIR     = $(SRC_DIR)/target/$(TARGET)
TARGET_DIR_SRC = $(notdir $(wildcard $(TARGET_DIR)/*.c))

INCLUDE_DIRS  := $(SRC_DIR) \
                 $(SRC_DIR)/target \
                 $(TARGET_DIR) \
                 $(INCLUDE_DIRS)

VPATH         := $(VPATH):$(TARGET_DIR)

COMMON_SRC           = \
                    $(TARGET_DIR_SRC)

VPATH               := $(VPATH):$(SRC_DIR)

SPEED_OPTIMISED_SRC := $(SPEED_OPTIMISED_SRC) \
                    $(TARGET_DIR_SRC) \
                    main.c \
                    common/common.c \
                    drivers/adc.c \
                    drivers/led.c \
                    drivers/watchdog.c \
                    drivers/input.c \
                    drivers/motor.c \
                    drivers/uart.c \
                    drivers/telemetry.c

SIZE_OPTIMISED_SRC  := $(SIZE_OPTIMISED_SRC) \
                    $(TARGET_DIR_SRC) \
                    config/config.c \
                    drivers/eeprom.c \
                    drivers/system.c

# check if target.mk supplied
SRC                 := $(STARTUP_DIR)/$(STARTUP_SRC) $(COMMON_SRC) $(SPEED_OPTIMISED_SRC) $(SIZE_OPTIMISED_SRC) $(MCU_SRC) $(CMSIS_SRC) $(DRIVER_SRC)

#excludes
SRC                 := $(filter-out ${MCU_EXCLUDES}, $(SRC))
