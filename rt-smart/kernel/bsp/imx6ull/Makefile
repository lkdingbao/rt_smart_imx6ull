TARGET          ?= rtthread

RM              := rm -f
RD              := rm -rf

.PHONY: clean

clean:
	@echo "clear all processing files success."
	@$(shell $(RM) $(TARGET).bin $(TARGET).hex $(TARGET).elf $(TARGET).imx $(TARGET).map $(TARGET).DEBUG)
	@$(shell $(RD) build)