//**************************************************************************
//
//    Copyright (C) 2025  John Winans
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

`default_nettype none

module z80_vdp99 (
    input wire          reset,
    input wire          phi,            // the z80 PHI clock
    input wire          pxclk,          // the pixel clock

    input wire          cpu_mode,       // CPU address valid during cpu_rd/wr_tick
    input wire [7:0]    cpu_din,        // CPU data valid during cpu_wr_tick
    output wire [7:0]   cpu_dout,       // CPU data valid during cpu_rd_tick

    input wire          cpu_wr_tick,    // phi domain
    input wire          cpu_rd_tick,    // phi domain
    
    output wire [3:0]   color,
    output wire         hsync,
    output wire         vsync,
    output wire         irq             // Note: The IRQ is an async signal in the CPU domain
    );

    wire        vdp_wr_tick;            // pxclk domain
    wire        vdp_rd_tick;            // pxclk domain
    reg  [7:0]  vdp_din;                // pxclk domain
    reg         vdp_mode;               // pxclk domain

`ifdef WTM_CDC_MOD

    sync_stretch #(
        .STRETCH_BITS(1), .SYNC_LEN(2) 
    ) ss_wr_tValid (
        .reset(reset), .clk1(phi), .clk2(pxclk), .in(cpu_wr_tick), .out(vdp_wr_tick)
    );

    `ifdef SHOULD_NOT_NEED_THIS
        // cpu_wr_tick ALWAYS raises with falling edge T2.  CPU A and D busses are stable by then
        always @(posedge cpu_wr_tick) begin
            vdp_mode <= cpu_mode;
            vdp_din  <= cpu_din;
        end
    `else
        // D (transparent) latches used to ensure that the setup and hold times on 
        // vdp_mode and vdp_din are long enough for vdp_wr_tick.
        // nextpnr does not like this
        // get around it's loop failure with:
        //      NEXTPNR_OPT+=--ignore-loops 
        always @(*) begin
            if ( cpu_wr_tick ) begin
                vdp_mode <= cpu_mode;
                vdp_din  <= cpu_din;
            end
        end
    `endif
`else

    //Sync cpu_wr_tick, cpu_din, and cpu_mode into the pxclk domain 
    z80_wr_cdc #(
            .ADDR_WIDTH(1)      // address bus is only 1 bit wide
        ) wr_cdc (
            .reset(reset),
            .clk1(phi),
            .clk2(pxclk),
            .wr_tick1(cpu_wr_tick),
            .din1(cpu_din),
            .ain1(cpu_mode),
            .wr_tick2(vdp_wr_tick),
            .dout2(vdp_din),
            .aout2(vdp_mode)
        );
`endif

    // z80_rd_cdc ??
    assign cpu_dout = 0;     // XXX for now
    assign vdp_rd_tick = 0;

    // Connect the pxclk synchronized CPU bus to the VDP
    vdp99 vdp (
        .reset(reset),
        .pxclk(pxclk),

        .wr0_tick(vdp_wr_tick && vdp_mode==0),
        .wr1_tick(vdp_wr_tick && vdp_mode==1),
        .rd0_tick(vdp_rd_tick),                 // XXX need to derive independant from wr_mode
        .rd1_tick(vdp_rd_tick),                 // XXX

        .din(vdp_din),
        //.dout(),
        .irq(irq),
        .color(color),
        .hsync(hsync),
        .vsync(vsync)
    );

 
endmodule
