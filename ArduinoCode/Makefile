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


flash:
	@pio run -e $(ENV) --target upload --disable-auto-clean
	@pio device monitor -e $(ENV)

flash_plot: upload
	@sleep 1
	@$(MAKE) plot

clean:
	@pio run -e $(ENV) --target clean

erase:
	@pio run -e $(ENV) --target erase

debug:
	@pio debug -e $(ENV)

rebuild:
	@pio run -e $(ENV) --target clean
	@pio run -e $(ENV)

.PHONY: build upload monitor flash clean erase debug rebuild
