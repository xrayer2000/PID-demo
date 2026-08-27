# Default PlatformIO environment
ENV ?= genericSTM32F411CE

build:
	@pio run -e $(ENV)

upload:
	@pio run -e $(ENV) --target upload --disable-auto-clean

monitor:
	@pio device monitor -e $(ENV)

plot:
	@python3 tools/live_plot.py \
		--port /dev/ttyACM0 \
		--baud 115200 \
		--axes "1,2" \
# 		--overlay

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