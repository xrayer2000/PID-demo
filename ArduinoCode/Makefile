ENV ?= bluepill_f103c8

build:
	@pio run -e $(ENV)

upload:
	@pio run -e $(ENV) --target upload --disable-auto-clean

monitor:
	@pio device monitor -e $(ENV)

clean:
	@pio run -e $(ENV) --target clean

flash:
	@pio run -e $(ENV) --target upload --disable-auto-clean
	@pio device monitor -e $(ENV)

erase:
	@pio run -e $(ENV) --target erase

debug:
	@pio debug -e $(ENV)

.PHONY: build upload monitor clean flash erase debug