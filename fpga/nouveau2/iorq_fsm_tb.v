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

// don't forget this!  Else can have trouble with 0-delay rise times after phi!
`timescale 1ns/1ps  // time units & precision 


module tb ();

    reg     phi = 1;
    reg     iorq = 0;
    wire    iorq_tick;
 
    iorq_fsm uut (.phi(phi), .iorq(iorq), .iorq_tick(iorq_tick) );

    always #25 phi = ~phi;

    reg t1 = 0;     // use for refernce when viewing waveform

    initial begin
        $dumpfile("iorq_fsm_tb.vcd");
        $dumpvars;
        
        #(6*25);         // skip a machine cycle

        // An external t1-t2-tw-t3 IO cycle
        // IOC=1 (default)
       
        // READ cycle IOC=1
        // earliest (worst) case IORQ edge is 0ns after t1 falling
        t1=1;
        #(25*2);
        t1=0;
        #1;         // worse than worse, we miss t2 rising setup on iorq
        iorq = 1;   // be careful of order of operations when this coinsides with phi rising edge!
        #(3*25*2-1);
        //#(3*25*2);
        iorq = 0; // latest (worst) case iorq falling edge is coincident with the next t1 rising

        t1=1;       // skip to make waveform easier to see
        #(2*25);
        t1=0;
        #(4*25);

        // READ cycle IOC=1
        // latest case IORQ edge is 25ns after t1 falling edge
        t1=1;
        #(25*2);
        t1=0;
        iorq = 1;   // be careful of order of operations when this coinsides with phi rising edge!
        #(2*25*2+25);
        iorq = 0;   // earliest case iorq edge is 0ns after t3 falling
        #25;

        t1=1;       // skip to make waveform easier to see
        #(2*25);
        t1=0;
        #(4*25);


        // WRITE cycle IOC=1
        // earliest IORQ edge is 0ns after t2 rise

        t1=1;
        #(2*25);
        t1=0;
        #1;
        iorq = 1;       // earliest IORQ edge immediately after t2 rise
        #(2*25*2+25-1);
        iorq = 0;       // latest IORQ edge 25ns after t3 fall
        #25;

        t1=1;       // skip to make waveform easier to see
        #(2*25);
        t1=0;
        #(4*25);

        // WRITE cycle IOC=1
        // latest IORQ edge is 25ns after t2 rise

        t1=1;
        #(2*25);
        t1=0;
        #25;
        iorq = 1;       // latest IORQ edge immediately after t2 fall
        #(2*25*2);
        iorq = 0;       // earliest IORQ edge 0ns after t3 fall
        #25;



        // What happens if we trigger on an internal t1-t2-t3 IO cycle?

        t1=1;       // skip to make waveform easier to see
        #(2*25);
        t1=0;
        #(4*25);

        // internal WRITE cycle (same as external without tw)
        // latest IORQ edge is 25ns after t2 rise.
        // This will demonstrate the iorq_tick rising with t3 and then ending early.

        // Adding an address restriction on iorq_sync.iorq to prevent it from starting
        // in the first place can clean this up.

        t1=1;
        #(2*25);
        t1=0;
        #1;
        iorq = 1;       // earliest IORQ edge immediately after t2 rise
        #(2*25*1-1);
        #25;            // t3 high
        iorq = 0;       // earliest IORQ edge 0ns after t3 fall  (coinside with t1 rise)
        #25;



        t1=1;       // skip to make waveform easier to see
        #(2*25);
        t1=0;
        #(4*25);


        $finish;
    end

endmodule
