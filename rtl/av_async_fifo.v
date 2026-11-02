// Dual-clock audio sample FIFO. Data RAM is not reset; pointers determine validity.
module av_async_fifo #(
    parameter DATA_BITS = 32,
    parameter ADDR_BITS = 11
) (
    input wire rst,
    input wire wr_clk,
    input wire wr_en,
    input wire [DATA_BITS-1:0] wr_data,
    output wire wr_full,
    output wire [ADDR_BITS:0] wr_count,
    input wire rd_clk,
    input wire rd_en,
    output reg [DATA_BITS-1:0] rd_data,
    output wire rd_empty
);
    reg [DATA_BITS-1:0] ram [0:(1<<ADDR_BITS)-1];
    reg [ADDR_BITS:0] wr_bin, wr_gray, rd_bin, rd_gray;
    (* ASYNC_REG="TRUE" *) reg [ADDR_BITS:0] rd_gray_w1, rd_gray_w2;
    (* ASYNC_REG="TRUE" *) reg [ADDR_BITS:0] wr_gray_r1, wr_gray_r2;
    function [ADDR_BITS:0] gray_to_bin;
        input [ADDR_BITS:0] value;
        integer n;
        begin
            gray_to_bin[ADDR_BITS] = value[ADDR_BITS];
            for(n=ADDR_BITS-1;n>=0;n=n-1)
                gray_to_bin[n] = gray_to_bin[n+1] ^ value[n];
        end
    endfunction
    wire [ADDR_BITS:0] wr_next = wr_bin + 1'b1;
    wire [ADDR_BITS:0] rd_next = rd_bin + 1'b1;
    assign wr_full = wr_gray == {~rd_gray_w2[ADDR_BITS:ADDR_BITS-1], rd_gray_w2[ADDR_BITS-2:0]};
    assign rd_empty = rd_gray == wr_gray_r2;
    assign wr_count = wr_bin - gray_to_bin(rd_gray_w2);
    always @(posedge wr_clk) begin
        if(wr_en && !wr_full) ram[wr_bin[ADDR_BITS-1:0]] <= wr_data;
    end
    always @(posedge rd_clk) begin
        if(rd_en && !rd_empty) rd_data <= ram[rd_bin[ADDR_BITS-1:0]];
    end
    always @(posedge wr_clk or posedge rst) begin
        if(rst) begin
            wr_bin<=0; wr_gray<=0; rd_gray_w1<=0; rd_gray_w2<=0;
        end else begin
            rd_gray_w1<=rd_gray; rd_gray_w2<=rd_gray_w1;
            if(wr_en && !wr_full) begin
                wr_bin<=wr_next; wr_gray<=(wr_next>>1)^wr_next;
            end
        end
    end
    always @(posedge rd_clk or posedge rst) begin
        if(rst) begin
            rd_bin<=0; rd_gray<=0; wr_gray_r1<=0; wr_gray_r2<=0;
        end else begin
            wr_gray_r1<=wr_gray; wr_gray_r2<=wr_gray_r1;
            if(rd_en && !rd_empty) begin
                rd_bin<=rd_next; rd_gray<=(rd_next>>1)^rd_next;
            end
        end
    end
endmodule
