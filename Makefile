GCC_ROOT    := C:/Progra~1/TI/MSP430-GCC
CC          := "$(GCC_ROOT)/bin/msp430-elf-gcc.exe"
FLASHER     := "C:/Progra~1/TI/UniFlash_9.3.0/dslite.bat"

MCU         := msp430g2553
LINKER     := $(GCC_ROOT)/msp430-elf/lib/ldscripts/$(MCU).ld

SRC_DIR     := src
BIN_DIR     := bin
OBJS        := $(patsubst $(SRC_DIR)/%.c,$(BIN_DIR)/%.o,$(wildcard $(SRC_DIR)/*.c))
OUT         := $(BIN_DIR)/firmware.elf

CFLAGS := \
    -mmcu=$(MCU) \
    -std=c11 \
    -g \
    -Os \
    -ffunction-sections \
    -fdata-sections \
    -Wall -Wextra -Wpedantic \
    -I$(SRC_DIR)

LDFLAGS := \
    -mmcu=$(MCU) \
    -Wl,-Map,$(BIN_DIR)/firmware.map \
    -Wl,--gc-sections \
    -T$(LINKER)

.PHONY: all flash clean
all: $(OUT)

$(BIN_DIR):
	if not exist "$(subst /,\,$(BIN_DIR))" mkdir "$(subst /,\,$(BIN_DIR))"

$(BIN_DIR)/%.o: $(SRC_DIR)/%.c | $(BIN_DIR)
	if not exist "$(subst /,\,$(dir $@))" mkdir "$(subst /,\,$(dir $@))"
	$(CC) $(CFLAGS) -c $< -o $@

$(OUT): $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $(OUT)

flash: $(OUT)
	$(FLASHER) --config=lib/$(MCU).ccxml --flash --verify $(OUT)

clean:
	-if exist "$(subst /,\,$(BIN_DIR))" rmdir /S /Q "$(subst /,\,$(BIN_DIR))"
