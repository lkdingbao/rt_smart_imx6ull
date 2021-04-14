TARGET          ?= output

RM              := rm -f
RD              := rm -rf

.PHONY: clean

clean:
	@echo "clear ..."
	@$(shell $(RM) $(TARGET).bin $(TARGET).hex $(TARGET).elf $(TARGET).imx $(TARGET).map $(TARGET).DEBUG)
	@$(shell $(RD) build)
	@echo "clear all processing files success."
