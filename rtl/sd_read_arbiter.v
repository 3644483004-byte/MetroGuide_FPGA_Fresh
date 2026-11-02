// A sector burst is indivisible. Audio has priority between bursts so video
// prefetch cannot starve the audio sample FIFO.
module sd_read_arbiter (
    input wire clk, rst, card_ready,
    input wire a_req,
    input wire [31:0] a_lba,
    input wire [5:0] a_blocks,
    output reg a_grant,
    input wire v_req,
    input wire [31:0] v_lba,
    input wire [5:0] v_blocks,
    output reg v_grant,
    output reg host_start,
    output reg [31:0] host_lba,
    output reg [5:0] host_blocks,
    input wire host_busy,
    input wire [7:0] host_data,
    input wire host_valid, host_block_end, host_done, host_error,
    output reg [7:0] data,
    output reg a_valid, a_block_end, a_done, a_error,
    output reg v_valid, v_block_end, v_done, v_error
);
    reg occupied, video_owner;
    // Register the complete response bundle, including coincident last byte
    // and done. This cuts arbitration out of the FAT parser's byte-counter path.
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            data<=0;a_valid<=0;a_block_end<=0;a_done<=0;a_error<=0;
            v_valid<=0;v_block_end<=0;v_done<=0;v_error<=0;
        end else begin
            data<=host_data;
            a_valid<=occupied && !video_owner && host_valid;
            a_block_end<=occupied && !video_owner && host_block_end;
            a_done<=occupied && !video_owner && host_done;
            a_error<=occupied && !video_owner && host_error;
            v_valid<=occupied && video_owner && host_valid;
            v_block_end<=occupied && video_owner && host_block_end;
            v_done<=occupied && video_owner && host_done;
            v_error<=occupied && video_owner && host_error;
        end
    end
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            occupied<=0; video_owner<=0; a_grant<=0; v_grant<=0;
            host_start<=0; host_lba<=0; host_blocks<=1;
        end else begin
            a_grant<=0; v_grant<=0; host_start<=0;
            if(occupied) begin
                if(host_done || host_error || !card_ready) occupied<=0;
            end else if(card_ready && !host_busy && !host_start) begin
                if(a_req) begin
                    occupied<=1; video_owner<=0; a_grant<=1;
                    host_start<=1; host_lba<=a_lba; host_blocks<=a_blocks;
                end else if(v_req) begin
                    occupied<=1; video_owner<=1; v_grant<=1;
                    host_start<=1; host_lba<=v_lba; host_blocks<=v_blocks;
                end
            end
        end
    end
endmodule
