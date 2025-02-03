`default_nettype none

module video_bars (
    input   wire        pxclk,
    input   wire        reset,
    output  wire        vga_red,
    output  wire        vga_grn,
    output  wire        vga_blu,
    output  wire        vga_hsync,
    output  wire        vga_vsync,
    );

    wire pxclk;
    wire vga_vid;
    wire vga_hsync;
    wire vga_vsync;
    wire [$clog2(1344)-1:0] vga_col;    // big enough to hold the counter value
    wire [$clog2(806)-1:0] vga_row;     // big enough to hold the counter value

    vgasync #(
        .HVID(1024),
        .HFP(24),
        .HS(136),
        .HBP(160),
        .VVID(768),
        .VFP(3),
        .VS(6),
        .VBP(29)
    ) vga (
        .clk(pxclk),
        .reset(reset),
        .hsync(vga_hsync),
        .vsync(vga_vsync),
        .col(vga_col),
        .row(vga_row),
        .vid_active(vga_vid)
    );


    // Note: Because the following logic is combinational, the way 
    // that the compiler choses to implement it can cause glitches 
    // and/or hesitations when the colors transition.


    // draw different patterns depending on the row
    assign {vga_red,vga_grn,vga_blu} = vga_vid ? (( vga_row > 300) ? vga_col[8:6] : ( vga_row > 290) ? 0 : ( vga_row > 200) ? vga_col[5:3] : ( vga_row > 190) ? 0 : vga_col[2:0] ) : 0;

    // draw same pattern on every row
//    assign {vga_red,vga_grn,vga_blu} = vga_vid ? vga_col[8:6] : 0;

    // draw full screen white
//    assign {vga_red,vga_grn,vga_blu} = vga_vid ? {3{vga_vid}} : 0;    // all three on at same time

    assign vga_hsync = ~vga_hsync;      // Polarity of horizontal sync pulse is negative.
    assign vga_vsync = ~vga_vsync;      // Polarity of vertical sync pulse is negative.

endmodule
