TOP=..

include $(TOP)/Make.rules

.PHONY: prog

FILES= \
	top.v \
	pll.v \
	vdp_table_test.v \
	video.v \
	vgasync.v

DEPS=\
	../pinmap-2057.pcf \
	rom_color.hex \
	rom_name.hex \
	rom_pattern.hex


# FPGA oscillator speed 
OSC_CLK=25
PX_CLK=25

# FREQ is used for the timing nextpnr verification check
FREQ=$(PX_CLK)

all:: top.bin

top.json: $(FILES) $(DEPS)
	$(COMPILE.v) -p "$(SYNTH) -top top -json $@" $(FILES) $(COMPILE_OPT)

timing: top.asc
	icetime -tmd $(DEVICE) $^

vgasync_tb.vvp: vgasync_tb.v vgasync.v
	iverilog -o $@ $^

plot: vgasync_tb.vcd
	gtkwave $^

vdp_table_test_tb.vvp: vdp_table_test_tb.v vdp_table_test.v vgasync.v
	iverilog -o $@ $^

plotv: vdp_table_test_tb.vcd
	gtkwave $^

prog: top.bin
	$(FLASH_PROG) $^

pll.v:
	icepll -i $(OSC_CLK) -o $(PX_CLK) -m -f $@

# extra dependancies
top.asc: $(PINMAP)

clean::
	rm -f pll.v
