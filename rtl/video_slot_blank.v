// Keeps HDMI video timing valid while the four future video assets are empty.
// The active picture is pure black; there are no bars, labels, or test pattern.
module video_slot_blank #(
    parameter HTOTAL  = 800,
    parameter HACTIVE = 640,
    parameter HFP     = 16,
    parameter HSA     = 96,
    parameter HBP     = 48,
    parameter VTOTAL  = 525,
    parameter VACTIVE = 480,
    parameter VFP     = 10,
    parameter VSA     = 2,
    parameter VBP     = 33
) (
    input  wire        I_clk,
    input  wire        I_rst,
    output reg         O_video_user,
    output reg         O_video_valid,
    output reg         O_video_last,
    output reg  [23:0] O_video_data
);

    localparam HBLANK = HTOTAL - HACTIVE;
    localparam VBLANK = VTOTAL - VACTIVE;

    reg [13:0] col_count;
    reg [13:0] line_count;
    wire active_pixel = (line_count >= VBLANK) && (col_count >= HBLANK);

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            col_count     <= 14'd0;
            line_count    <= 14'd0;
            O_video_user  <= 1'b0;
            O_video_valid <= 1'b0;
            O_video_last  <= 1'b0;
            O_video_data  <= 24'd0;
        end else begin
            if (col_count == HTOTAL - 1) begin
                col_count <= 14'd0;
                if (line_count == VTOTAL - 1)
                    line_count <= 14'd0;
                else
                    line_count <= line_count + 1'b1;
            end else begin
                col_count <= col_count + 1'b1;
            end

            O_video_valid <= active_pixel;
            O_video_user  <= (line_count == VBLANK) && (col_count == HBLANK);
            O_video_last  <= (line_count >= VBLANK) && (col_count == HTOTAL - 1);
            O_video_data  <= 24'd0;
        end
    end

endmodule
