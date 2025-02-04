TOP=..

# Uncomment to display colour test bars for 1024x768
# DEFINES=-D VIDEO_TEST_BARS -D 1024x768
# Uncomment to display test text on a 1024x768 screen
# DEFINES=-D 1024x768
# Build as per video series
DEFINES=

include $(TOP)/Make.rules

LIB_UTILS=$(TOP)/lib
include $(LIB_UTILS)/Make.include


.PHONY: prog

FILES= \
	top.v \
	memory.v \
	$(LIB_UTILS_FILES) \
	$(VDP_LIBV)/video.v \
	$(VDP_LIBV)/vdp_table_test.v \
	$(VDP_LIBV)/vgasync.v \
	$(VDP_LIBV)/bars-1024x768.v \
	$(LIB_UTILS)/clock_gen/BHG_FP_clk_divider.sv \
	pll_25_130.v \
	pll_25_18432.v

all:: top.bin

top.json: $(FILES) rom.hex
	$(COMPILE.v) $(DEFINES) -p "$(SYNTH) -top top -json $@" $(FILES) $(COMPILE_OPT)

timing: top.asc
	icetime -tmd $(DEVICE) $^

prog: top.bin
	$(FLASH_PROG) $^

# extra dependancies
top.asc: $(PINMAP)

#
#DATE := $(shell date --rfc-3339=seconds)
#GIT_VERSION := $(shell git describe --long --dirty; git show -s --format='%ci')
#%.bin: %.asm
#	cat $< | sed -e "s/@@DATE@@/$(DATE)/g" -e "s/@@GIT_VERSION@@/$(GIT_VERSION)/g" | $(CROSS_AS) - -o $@ --list=$(basename $@).lst --label=$(basename $@).sym $(CROSS_AS_FLAGS)
#

rom.bin:
	cp ../../../2063-Z80-cpm/boot/firmware.bin rom.bin

%.hex: %.bin
	hexdump -v -e '/1 "%02x\n"' < $< > $@

clean::
	rm -f *.lst *.bin *.hex *.sym pll_25_18432.v pll_25_130.v


pll_25_18432.v:
	icepll -i 25 -o 18.432 -m -n pll_25_18432 > $@

pll_25_130.v:
	icepll -i 25 -o 130 -m -n pll_25_130 > $@


iorq_rd_fsm_tb.vvp : iorq_rd_fsm_tb.v iorq_rd_fsm.v
	iverilog -o $@ $^

iorqr: iorq_rd_fsm_tb.vcd
	gtkwave $^

iorq_wr_fsm_tb.vvp : iorq_wr_fsm_tb.v iorq_wr_fsm.v
	iverilog -o $@ $^

iorqw: iorq_wr_fsm_tb.vcd
	gtkwave $^
