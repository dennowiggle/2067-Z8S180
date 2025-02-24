// ****************************************************************************
// * 
// *    Code by Denno Wiggle aka WTM.
// *
// *    Copyright (C) 2024 Denno Wiggle
// *
// *    SNES Joysticks Module to read two SNES Controllers over one SNES 
// *    interface. Controllers have common clock and latch with separate
// *    data lines for each controller.
// * 
// * 
// ****************************************************************************
// * 
// *    This library is free software; you can redistribute it and/or
// *    modify it under the terms of the GNU Lesser General Public
// *    License as published by the Free Software Foundation; either
// *    version 2.1 of the License, or (at your option) any later version.
// *
// *    This library is distributed in the hope that it will be useful,
// *    but WITHOUT ANY WARRANTY; without even the implied warranty of
// *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// *    Lesser General Public License for more details.
// *
// *    You should have received a copy of the GNU Lesser General Public
// *    License along with this library; if not, write to the Free Software
// *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301
// *    USA
// *
// ****************************************************************************
// Don't allow inferred nets as that can cause debug issues
`default_nettype none

module wtm_snesJoysticks #(
    // Parameters determine :
    // (1) The polling interval for reading the controllers.
    // (2) The SNES clock frequency for shifting data out of the 
    //     controllers.
    parameter CLOCK_FREQ_HZ         = 25000000,
    parameter SNES_CLOCK_FREQ_HZ    =   100000,
    parameter SNES_POLLING_FREQ_HZ  =     1000
)(
    // Clock can be any clock but the frequency for it must be specified in
    // CLOCK_FREQ_HZ to maintain the desired SNES interface timing.
    input  wire         clock,
    // Module reset.
    input  wire         reset_n,

    // SNES Controller signals
    output wire         snes_clock,
    output reg          snes_latch,
    input  wire         snes1_data,
    input  wire         snes2_data,

    // Registers that are loaded with data we read from the two controllers.
    output reg  [15:0]   joy1_data,
    output reg  [15:0]   joy2_data
);
    // States for state machine that controls reading the controllers.
    localparam [1:0]
        IDLE_STATE = 0,
        LATCH_DATA = 1,
        SEND_CLOCK = 2;

    reg  [1:0] io_read_state;

    // Counter for timing the reading of the controllers.
    // The tick counter is used to time the SNES clock pulses.
    localparam TICK_COUNTER_MAX             = (CLOCK_FREQ_HZ / (SNES_CLOCK_FREQ_HZ * 2) ) - 1;
    localparam TICK_COUNTER_WIDTH           = $clog2(TICK_COUNTER_MAX + 1);

    reg  [TICK_COUNTER_WIDTH-1:0]           tick_counter;
    reg                                     tick_snes;

    // Interval conuter used to determing how often to poll the controllers.
    // the controllers for data
    localparam READ_INTERVAL_COUNT_MAX      = (SNES_CLOCK_FREQ_HZ / SNES_POLLING_FREQ_HZ) - 1;
    localparam READ_INTERVAL_COUNT_WIDTH    = $clog2(READ_INTERVAL_COUNT_MAX + 1);

    reg  [READ_INTERVAL_COUNT_WIDTH-1:0]    tick_read_counter;
    reg                                     tick_latch;
    
    // Data read from the SNES 16-bit shift registers
    reg  [15:0] joy1_read;
    reg  [15:0] joy2_read;

    // Internal clock that runs at the specified SNES clock frequency.
    // The final clock output to the controllers is a gated version of
    // this clock.
    reg snes_clock_i;

    // In order to clock serial data out of the SNES contoller a count
    // of 16 clock pulses is required.
    localparam  [4:0]   SHIFTREG_NUM_BITS = 5'b10000;
    reg         [4:0]   snes_clock_count;
    reg                 snes_clock_en;
    reg         [4:0]   shift_bit_index;


    // A tick counter that generates a tick pulse at a frequency
    // twice that specified in the SNES_CLOCK_FREQ_HZ input parameter.
    // This will be used to time the SNES clock.
    always @(posedge clock)
    begin
        if (reset_n == 1'b0)
        begin
            tick_counter      <= 0;
            tick_snes         <= 1'b0;
        end else
        begin
            if (tick_counter == TICK_COUNTER_MAX)
            begin
                tick_counter    <= 0;
                tick_snes       <= 1'b1;
            end else
            begin
                tick_counter    <= tick_counter + 1;
                tick_snes       <= 1'b0;
            end
        end
    end

    // Use the tick to generate the internal SNES clock.
    always @(posedge clock)
    begin
        if (reset_n == 1'b0)
        begin
            snes_clock_i  <=  1'b0;
        end else
        begin
            if (tick_snes == 1'b1)
                snes_clock_i <= ~snes_clock_i;
        end
    end

    // A tick counter that generates a tick pulse at a frequency
    // specified in the SNES_POLLING_FREQ_HZ input parameter.
    // This will be used to time the SNES data read interval.
    always @(posedge snes_clock_i, negedge reset_n)
    begin
        if (reset_n == 1'b0)
        begin
            tick_read_counter   <= 0;
            tick_latch          <= 1'b0;
        end else
        begin
            if (tick_read_counter == READ_INTERVAL_COUNT_MAX)
            begin
                tick_read_counter   <= 0;
                tick_latch          <= 1'b1;
            end else
            begin
                tick_read_counter   <= tick_read_counter + 1;
                tick_latch          <= 1'b0;
            end
        end
    end

    // Generate the SNES latch signal at the specified 
    // polling frequency
    always @(posedge snes_clock_i, negedge reset_n)
    begin
        if (reset_n == 1'b0)
        begin
            snes_latch    <=  1'b0;
        end else
        begin
            if (tick_latch == 1'b1)
                snes_latch <= 1'b1;
            else
                snes_latch <= 1'b0;
        end
    end

    // State macine for controlling when to send the SNES 
    // clock signals for reading data out of the SNES
    // shift registers.
    always @(posedge snes_clock_i, negedge reset_n)
    begin
        if (reset_n == 1'b0)
        begin
            io_read_state       <= IDLE_STATE;
            snes_clock_count    <= 5'b00000;
            snes_clock_en       <= 1'b0;
        end else
        begin

            // Assign default values to use if not assigned later.
            snes_clock_en       <= 1'b0;

            case(io_read_state)

            IDLE_STATE :
                if (snes_latch == 1'b1)
                    io_read_state       <= LATCH_DATA;

            LATCH_DATA :
                if (snes_latch == 1'b0)
                    io_read_state       <= SEND_CLOCK;

            SEND_CLOCK : // Send 16 clock pulses
                begin
                    snes_clock_en    <= 1'b1;

                    if (snes_clock_count == SHIFTREG_NUM_BITS)
                    begin
                        io_read_state       <= IDLE_STATE;
                        snes_clock_en       <= 1'b0;
                        snes_clock_count    <= 5'b00000;
                    end else
                    begin
                        io_read_state       <= SEND_CLOCK;
                        snes_clock_en       <= 1'b1;
                        snes_clock_count    <= snes_clock_count + 1;
                    end
                end

            default :
                begin
                    io_read_state       <= IDLE_STATE;
                end

            endcase
        end
    end

    // Function to map the SNES controller data format to Z80 Retro Joystick format;
    function [15:0] map_joystick_data;
    input [15:0] snes_data;
        begin
            map_joystick_data = 16'hffff;
            map_joystick_data[ 0] = snes_data[0 ];       // B = Button 1 = Fire
            map_joystick_data[11] = snes_data[1 ];       // Y = Button 4
            map_joystick_data[ 8] = snes_data[2 ];       // Select
            map_joystick_data[ 9] = snes_data[3 ];       // Start
            map_joystick_data[ 7] = snes_data[4 ];       // Up
            map_joystick_data[ 6] = snes_data[5 ];       // Down
            map_joystick_data[ 2] = snes_data[6 ];       // Left
            map_joystick_data[ 5] = snes_data[7 ];       // Right
            map_joystick_data[ 4] = snes_data[8 ];       // A = Button 2 
            map_joystick_data[10] = snes_data[9 ];       // X = Button 3
            map_joystick_data[12] = snes_data[10];       // L = Left front Button
            map_joystick_data[13] = snes_data[11];       // R = Right front button
        end
    endfunction

    // Latch in the SNES data when 'snes_clock_en' is asserted.
    // Note : Considered using 'snes_clock_count' delayed by one clock for the
    //        bit index but this code block is clocked on the negative edge 
    //        whereas 'snes_clock_count' is clocked on the positive edge.
    //        Having a separate counter makes the code easier to read.
    always @(negedge snes_clock_i, negedge reset_n)
    begin
        if (reset_n == 1'b0)
        begin
            joy1_read       <= 16'hFFFF;
            joy2_read       <= 16'hFFFF;
            joy1_data       <= 16'hFFFF;
            joy2_data       <= 16'hFFFF;
            shift_bit_index <=  0;
        end else
        begin
            // Set the default value that can be over-ridden
            shift_bit_index <= 0;
            
            // If the SNES clock is enabled read the serial data info from the 16-bit 
            // joystick / controller read registers in SNES 16 bit button order.
            if ( (snes_clock_en == 1'b1) && (shift_bit_index != SHIFTREG_NUM_BITS) )
            begin
                joy1_read[shift_bit_index]  <= snes1_data;
                joy2_read[shift_bit_index]  <= snes2_data;
                shift_bit_index             <= shift_bit_index + 1;
            end else
            begin
                // And the if the last SNES clock has been sent then
                // load the 8-bit joystick registers that the CPU will read.
                // Data is mapped to the Z80 Retro joystick config.
                if (shift_bit_index == SHIFTREG_NUM_BITS)
                begin
                    // Map the SNES data to Z80 Retro joystick config.
                    joy1_data <= map_joystick_data(joy1_read);
                    joy2_data <= map_joystick_data(joy2_read);
                end
            end
        end
    end

    // The SNES clock is enabled for 16 clock pulses when snes_clock_en is asserted.
    assign snes_clock = (snes_clock_en == 1'b1) ? snes_clock_i : 1'b1;


endmodule