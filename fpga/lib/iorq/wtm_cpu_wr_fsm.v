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

// NOTE: This is NOT useful for a general purpose synchronizer because this
//      expects the input signal to meet the FPGA's setup and hold times with
//      the Z8S180 configured to operate in IOC=1 mode.
// Want:
// for a write cycle: latch value on second phi falling edge after iorq becomes true:
// fsm counting falling phi when wr is true and enable when count = 1

module wtm_cpu_wr_fsm (
    input  wire  clock,    // the CPU phi clock
    input  wire  reset,
    input  wire  sig_wr,
    input  wire  sig_in, 
    output wire  sig_out
    );
 
    reg [1:0]   state_reg;

    always @(negedge clock) begin
        if ( reset )
            state_reg <= 0;
        else begin
            state_reg <= 0;

            case ( state_reg )
            0:
                if ( sig_wr )
                    state_reg <= 1;
            1:
                if ( sig_wr )
                    state_reg <= 2;
            2:
                if ( sig_wr )
                    state_reg <= 2;
            endcase
        end
    end

    assign sig_out = ( state_reg==1 )? sig_in : 0;

endmodule
