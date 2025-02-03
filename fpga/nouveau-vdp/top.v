//**************************************************************************
//
//    Copyright (C) 2024,2025  John Winans
//
//    This library is free software; you can redistribute it and/or
//    modify it under the terms of the GNU Lesser General Public
//    License as published by the Free Software Foundation; either
//    version 2.1 of the License, or (at your option) any later version.
//
//    This library is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//    Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public
//    License along with this library; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
//    USA
//
//**************************************************************************
// Don't allow inferred nets as that can cause debug issues.
`default_nettype none

module top #(
    parameter LOGIC_CLOCK_FREQ_HZ       = 25000000,
    parameter CPU_CLOCK_FREQ_HZ         = 18432000
) (
    input wire          hwclk,      // 25MHZ oscillator
    input wire          s1_n,
    output wire [7:0]   led,

    input wire [19:0]   a,
    inout wire [7:0]    d,          // bidirectional

    input wire          busack_n,
    output wire         busreq_n,

    output wire         ce_n,
    output wire         oe_n,
    output wire         we_n,

    output wire         dreq1_n,

    input wire          e,
    output wire         extal,
    input wire          phi,

    input wire          halt_n,

    output wire [2:0]   int_n,
    output wire         nmi_n,

    input wire          rd_n,
    input wire          wr_n,
    input wire          iorq_n,
    input wire          mreq_n,
    input wire          m1_n,

    output wire         reset_n,
    input wire          rfsh_n,
    input wire          st,
    input wire          tend1_n,
    output wire         wait_n,

    output wire         sd_mosi,
    output wire         sd_clk,
    output wire         sd_ssel_n,

    input wire          sd_miso,
    input wire          sd_det,

    output  wire        vga_red,
    output  wire        vga_grn,
    output  wire        vga_blu,
    output  wire        vga_hsync,
    output  wire        vga_vsync,

    output wire [15:0]  tp          // handy-dandy test-point outputs
    );

    localparam RAM_START = 20'h1000;

    // note that the test points here are different from the previous test proggies
    assign tp = { iorq_wr_tick, iorq_rd_tick, phi, e, iorq_n, we_n, oe_n, ce_n, wr_n, rd_n, mreq_n, m1_n };
    //            93            90            87   84 82      80    78    75    73    63    61      56

    // a boot ROM
    wire [7:0]  rom_data;           // ROM output data bus
    memory rom ( .rd_clk(phi), .addr(a[11:0]), .data(rom_data) );

    // consider debouncing s1_n using hwclk (no other clock possible)
    wire reset = ~s1_n || ~pll_locked;      // assert reset when PLL is starting up & unstable

    // Delay the CPU reset by 1ms.
    // This is always needed when extal is derived from the pixel clock pll.
    wire cpu_reset_n;
    wtm_resetSyncDelay #(1000, LOGIC_CLOCK_FREQ_HZ)
    sync_delay_reset_to_clock_cpu(
        .clock          (hwclk),
        .rst_n          (~reset),
        .rst_out_n      (cpu_reset_n)
    );
    assign reset_n = cpu_reset_n; // Have to do this because .pcf file shared between projects
    wire cpu_reset = ~cpu_reset_n;

    // When the CPU is reading from the FPGA drive the bus, else tri-state it.
    reg [7:0] dout;                 // what to write to data bus when requested
    reg dbus_out;                   // 1 if the FPGA shoudl drive the data bus
    assign d = dbus_out ? dout : 8'bz;  // a tri-state driver

    reg rom_sel;                    // true when the boot ROM is enabled
    always @(posedge phi)
        if ( cpu_reset )
            rom_sel <= 1;           // after a hard reset, the boot ROM is enabled...
        else if ( ioreq_rd_fe )     // until there is a read from IO port 0xfe
            rom_sel <= 0;

    // Determine if the FPGA will drive the data bus and with what
    // the CPU is reading from its data bus.
    always @(*) begin
        dbus_out = 1;
        dout = 8'bx;

        (* parallel_case *)     // no more than one case can match (one-hot)
        case (1)
        mreq_rom:       dout = rom_data;            // boot ROM memory
        ioreq_rd_f0:    dout = ioreq_rd_f0_data;    // gpio input
        default:        dbus_out = 0;
        endcase
    end

    wire        pll_locked;             // true when the PLL has locked to target freq
    wire        clock_px_x2;
    wire        clock_px;

`ifdef 1024x768

    pll_25_130 pll ( .clock_in(hwclk), .clock_out(clock_px_x2), .locked(pll_locked) );
    always@(posedge clock_px_x2)
        clock_px = ~clock_px;

    // PETER : 50MHz input, 130MHz output, 65MHz output, 25MHz output
    // pll_25_130 pll ( .clock_in(clock_50_sys_in), .clock_out(clock_px_x2), .clock_px(clock_px), .hwclk(hwclk), .locked(pll_locked) );

    // Clock source for Z8S180 at 18.432MHz derived from 130NHz clock
    // Source : https://github.com/BrianHGinc/Verilog-Floating-Point-Clock-Divider 
    BHG_FP_clk_divider #(
        .INPUT_CLK_HZ  (130000000),     // Source clk_in frequency.
        .OUTPUT_CLK_HZ ( 18432000)      // Target synthesized output frequency.
    ) clkdiv_z180 (
        .clk_in        (clock_px_x2),   // System source clock.
        .rst_in        (reset),         // Synchronous reset.
        .clk_out       (extal)          // Synthesized output clock, 50:50 duty cycle.
    );
    
`else 

    pll_25_18432 pll ( .clock_in(hwclk), .clock_out(extal), .locked(pll_locked) );
    // PETER : 50MHz input, 18.18MHz output, 25MHz output
    // pll_25_18432 pll ( .clock_in(clock_50_sys_in), .clock_out(extal), .hwclk(hwclk), .locked(pll_locked) );
    assign clock_px = hwclk;
    assign clock_px_x2 = 0;
    
`endif


    // for read cycle: latch value on first phi falling edge after iorq becomes true:
    // fsm counting falling phi when rd is true & enable when count = 0 && iorq is true
    wire    iorq_rd_tick;
    iorq_rd_fsm rd_fsm (.reset(cpu_reset), .phi(phi), .iorq(~iorq_n), .rd(~rd_n), .rd_tick(iorq_rd_tick) );

    // for a write cycle: latch value on second phi falling edge after iorq becomes true:
    // fsm counting falling phi when wr is true and enable when count = 1
    wire    iorq_wr_tick;
    iorq_wr_fsm wr_fsm (.reset(cpu_reset), .phi(phi), .iorq(~iorq_n), .wr(~wr_n), .wr_tick(iorq_wr_tick) );


    // qualified asynchronous bus enable signals
    wire iorq_rd = ~iorq_n && ~rd_n;
    wire iorq_wr = ~iorq_n && ~wr_n;
    wire mem_rd = ~mreq_n && ~rd_n;
    wire mem_wr = ~mreq_n && ~wr_n;

    // IO addres decoders (two variations):
    //  signal       = CPU asynchronous IO cycle
    //  signal_tick  = FPGA ff clock enable synchronized to phi

    // gpio input
    wire ioreq_rd_f0 = iorq_rd && (a[7:0] == 8'hf0);                // gpio input
    wire ioreq_rd_f0_tick = iorq_rd_tick && (a[7:0] == 8'hf0);

    wire ioreq_wr_f1 = iorq_wr && (a[7:0] == 8'hf1);                // gpio output
    wire ioreq_wr_f1_tick  = iorq_wr_tick && (a[7:0] == 8'hf1);

    wire ioreq_rd_fe = iorq_rd && (a[7:0] == 8'hfe);                // flash select disable access port
    wire ioreq_rd_fe_tick = iorq_rd_tick && (a[7:0] == 8'hfe);

    // The VDP regs are at address 0x80-0x81 (note LSB is not decoded)
    wire ioreq_rd_vdp = iorq_rd && (a[7:1] == 7'b1000000);
    wire ioreq_wr_vdp = iorq_wr && (a[7:1] == 7'b1000000);
    wire ioreq_rd_vdp_tick = iorq_rd_tick && (a[7:1] == 7'b1000000);
    wire ioreq_wr_vdp_tick = iorq_wr_tick && (a[7:1] == 7'b1000000);

    wire ioreq_rd_j3 = iorq_rd && (a[7:0] == 8'ha8);
    wire ioreq_rd_j3_tick = iorq_rd_tick && (a[7:0] == 8'ha8);         // joystick J3

    wire ioreq_rd_j4 = iorq_rd && (a[7:0] == 8'ha9);
    wire ioreq_rd_j4_tick = iorq_rd_tick && (a[7:0] == 8'ha9);         // joystick J4

    // ROM memory address decoder (address bus is 20 bits wide)
    // All following are suitable for our needs but can have very different number of LUTs!
    // a good example of optimizing for space/time
    //wire mreq_rom = rom_sel && mem_rd && a < RAM_START;       // if accessing low RAM during boot
    //wire mreq_rom = rom_sel && mem_rd && a[15:12] == 0;       // top 4 MSBs of bottom 4K are zero
    wire mreq_rom = rom_sel && mem_rd && a[19:12] == 0;         // all top MSBs of bottom 4K are zero

    // The GPIO output latch
    reg [7:0] gpio_out;
    always @(negedge phi) begin
        if ( ioreq_wr_f1_tick )
            gpio_out <= d;
    end

    // It is not really necessary to latch this because the SD signals will be stable during a read:
    reg [7:0] ioreq_rd_f0_data;     //  = {sd_miso,sd_det,6'bx};  // data value when reading port F0
    always @(negedge phi) begin
        if ( ioreq_rd_f0_tick )
            ioreq_rd_f0_data <= {sd_miso,sd_det,6'bx};
    end


    // VDP
`ifdef VIDEO_TEST_BARS
    `ifdef 1024x768
        video_bars vid (
    `endif
`else
    video vid (
`endif
        .pxclk(clock_px),       // Pixel clock frequency
        .reset(reset),
        .vga_red(vga_red),
        .vga_grn(vga_grn),
        .vga_blu(vga_blu),
        .vga_hsync(vga_hsync),
        .vga_vsync(vga_vsync)
    );


    assign sd_mosi = gpio_out[0];   // connect the GPIO output bits to the SD card pins
    assign sd_clk  = gpio_out[1];
    assign sd_ssel_n = gpio_out[2];

    assign busreq_n = 1'b1;     // de-assert /BUSREQ
    assign dreq1_n = 1'b1;      // de-assert /DREQ1
    assign int_n = 3'b111;      // de-assert /INT0 /INT1 /INT2
    assign nmi_n = 1'b1;        // de-assert /NMI
    assign wait_n = 1'b1;       // de-assert /WAIT

    // Enable the static RAM on memory cycles when the data bus is driven by the FPGA
    // The address range that is used to enable the SRAM varies depending on if/when
    // the shadow ROM is being enabled.
    assign ce_n = ~(~mreq_n && ~dbus_out );
    assign oe_n = mreq_n | rd_n;
    assign we_n = mreq_n | wr_n;

    // show some signals from the GPIO ports on the LEDs for reference
    assign led = {~sd_miso,sd_det,3'b111,~gpio_out[2:0]};

endmodule
