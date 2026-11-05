ENV ?= genericSTM32F411CE
BAUD ?= 115200

# Automatically detect STM32 serial port
PORT := $(shell pio device list | awk '/^\/dev\//{port=$$1} /GENERIC_F411CEUX/{print port; exit}')

build:
	@pio run -e $(ENV)

upload:
	@pio run -e $(ENV) --target upload --disable-auto-clean

monitor:
	@if [ -z "$(PORT)" ]; then echo "Error: STM32 not found."; exit 1; fi
	@pio device monitor -e $(ENV) --port $(PORT) --baud $(BAUD)

plot:
	@if [ -z "$(PORT)" ]; then echo "Error: STM32 not found."; exit 1; fi
	@echo "Using STM32 port: $(PORT)"
	@python3 tools/live_plot.py \
		--port $(PORT) \
		--baud $(BAUD) \
		--axes "1,2" \
		--group "pos,CmdPos" \
		--group "vel" \
		--group "acc"

run:
	@$(MAKE) upload
	@$(MAKE) monitor

run_plot:
	@$(MAKE) upload
	@sleep 1
	@$(MAKE) plot

clean:
	@pio run -e $(ENV) --target clean

erase:
	@pio run -e $(ENV) --target erase

debug:
	@pio debug -e $(ENV)

rebuild:
	@$(MAKE) clean
	@$(MAKE) build

.PHONY: build upload monitor plot run run_plot clean erase debug rebuild