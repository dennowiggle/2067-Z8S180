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

`timescale 1ns/1ns
`default_nettype none

// A gpio implementation.
// Registers :
//   xxxxx000 : gpio_din        Input data register.
//   xxxxx001 : gpio_dout       Output data register.
//   xxxxx010 : gpio_ddir       Data direction register.
//   xxxxx011 to xxxxx111       Reserved for future use e.g.
//                              special function(s), IRQ and IRQ mask.

module gpio #()
(
    input  wire         reset,
    input  wire         cpu_clock,      // the CPU clock

    input  wire [2:0]   reg_addr,
    input  wire [7:0]   cpu_din,        // valid during cpu_wr
    output wire [7:0]   cpu_dout,       // must be valid during cpu_rd

    input  wire         cpu_wr_tick,    // sync CPU write signal
    input  wire         cpu_rd_tick,    // sync CPU read signal

    input  wire [7:0]   gpio_in,
    output wire [7:0]   gpio_out,
    output wire [7:0]   gpio_oe
);

    localparam SYN_LEN = 3;             // too long for worst case write timing
    //localparam SYN_LEN = 2;           // not safe from metastability

    // Declare the GPIO registers
    reg [7:0] cpu_dout_reg;
    reg [7:0] gpio_reg_din;
    reg [7:0] gpio_reg_dout;
    reg [7:0] gpio_reg_ddir;

    // CPU read register logic
    always @(negedge cpu_clock) begin
        if (reset == 1'b1) begin
            cpu_dout_reg <= 8'hff;
        end else if (cpu_rd_tick == 1'b1) begin
            case(reg_addr[2:0])
                3'b000  : cpu_dout_reg <= gpio_reg_din;
                3'b001  : cpu_dout_reg <= gpio_reg_dout;
                3'b010  : cpu_dout_reg <= gpio_reg_ddir;
                default : cpu_dout_reg <= 8'hff;
            endcase
        end
    end

    assign cpu_dout = cpu_dout_reg;

    // CPU write register logic
    always @(negedge cpu_clock) begin
        if (reset == 1'b1) begin
            gpio_reg_dout <= 8'hff;
            gpio_reg_ddir <= 8'h00;
        end else if (cpu_wr_tick == 1'b1) begin
            case(reg_addr[2:0])
                3'b001  : gpio_reg_dout <= cpu_din;
                3'b010  : gpio_reg_ddir <= cpu_din;
            endcase
        end
    end

    assign gpio_out = gpio_reg_dout;
    assign gpio_oe  = gpio_reg_ddir;

    // Synchronize the gpio input data to the CPU clock domain
    sigSync #(SYN_LEN) gpio_in_sigSync[7:0](
        .clock      (cpu_clock),
        .reset      (reset),
        .sig_in     (gpio_in),
        .sig_out    (gpio_reg_din)
    );    

endmodule
