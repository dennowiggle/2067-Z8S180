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

module wtm_z80_vdp99 (
    input wire          reset,
    input wire          phi,            // the z80 PHI clock
    input wire          pxclk,          // the pixel clock

    // Read Port
    input wire          cpu_rd_tValid,  // phi domain
    input wire          cpu_rd_tMode,   // CPU address valid during cpu_rd_tValid
    output wire [7:0]   cpu_tDout,      // CPU data valid during cpu_rd_tValid

    // Write Port
    input wire          cpu_wr_tValid,  // phi domain
    input wire          cpu_wr_tMode,   // CPU address valid during cpu_wr_tValid
    input wire [7:0]    cpu_tDin,       // CPU data valid during cpu_wr_tValid
    
    output wire         irq,            // Note: The IRQ is an async signal in the CPU domain

    output wire [3:0]   color,
    output wire         hsync,
    output wire         vsync
    );

    wire        vdp_wr_tValid;          // pxclk domain
    wire        vdp_rd_tValid;          // pxclk domain
    wire [7:0]  vdp_tData;              // pxclk domain
    wire        vdp_tMode;               // pxclk domain

    sync_stretch #(
        .STRETCH_BITS(1), .SYNC_LEN(2) 
    ) ss_wr_tValid (
        .reset(reset), .clk1(phi), .clk2(pxclk), .in(cpu_wr_tValid), .out(vdp_wr_tValid)
    );

    sync_stretch #(
        .STRETCH_BITS(1), .SYNC_LEN(2) 
    ) ss_wr_tMode (
        .reset(reset), .clk1(phi), .clk2(pxclk), .in(cpu_wr_tMode), .out(vdp_tMode)
    );

    sync_stretch #(
        .STRETCH_BITS(1), .SYNC_LEN(2) 
    ) ss_wr_tData[7:0] (
        .reset(reset), .clk1(phi), .clk2(pxclk), .in(cpu_tDin), .out(vdp_tData)
    );

    // z80_rd_cdc ??
    assign cpu_tDout = 0;     // XXX for now
    assign vdp_rd_tValid = 0;

    // Connect the pxclk synchronized CPU bus to the VDP
    vdp99 vdp (
        .reset(reset),
        .pxclk(pxclk),

        .wr0_tick(vdp_wr_tValid && vdp_tMode==0),
        .wr1_tick(vdp_wr_tValid && vdp_tMode==1),
        .rd0_tick(vdp_rd_tValid),                 // XXX need to derive independant from wr_mode
        .rd1_tick(vdp_rd_tValid),                 // XXX

        .din(vdp_tData),
        //.dout(),
        .irq(irq),
        .color(color),
        .hsync(hsync),
        .vsync(vsync)
    );

 
endmodule
