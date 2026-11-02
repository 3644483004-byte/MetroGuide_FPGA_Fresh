// Pixel-clock side of the audio FIFO. The audio sample count is the scene clock.
module av_pcm_consumer #(
    parameter CLK_HZ=25_000_000,
    parameter AUDIO_HZ=48_000
) (
    input wire clk, rst, playing,
    input wire header_ready_async, video_ready_async,
    input wire [31:0] total_samples_async,
    input wire fifo_empty,
    input wire [31:0] fifo_data,
    output wire fifo_read,
    output reg audio_valid,
    output reg [23:0] audio_left, audio_right,
    output reg clip_end,
    output reg [31:0] total_samples
);
    (* ASYNC_REG="TRUE" *) reg [1:0] header_sync, video_sync;
    reg total_latched, pending, last_pending, finished;
    reg [31:0] issued;
    reg [24:0] phase;
    wire [25:0] phase_sum={1'b0,phase}+AUDIO_HZ;
    wire tick=phase_sum>=CLK_HZ;
    assign fifo_read=tick && playing && header_sync[1] && video_sync[1] &&
                     total_latched && !finished && issued<total_samples && !fifo_empty;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            header_sync<=0; video_sync<=0; total_latched<=0; total_samples<=0;
            pending<=0; last_pending<=0; finished<=0; issued<=0; phase<=0;
            audio_valid<=0; audio_left<=0; audio_right<=0; clip_end<=0;
        end else begin
            header_sync<={header_sync[0],header_ready_async};
            video_sync<={video_sync[0],video_ready_async};
            audio_valid<=0; audio_left<=0; audio_right<=0; clip_end<=0;
            if(header_sync[1] && !total_latched) begin
                total_samples<=total_samples_async; total_latched<=1;
            end
            if(tick) phase<=phase_sum-CLK_HZ;
            else phase<=phase_sum[24:0];
            pending<=fifo_read;
            last_pending<=fifo_read && issued==total_samples-1'b1;
            if(fifo_read) issued<=issued+1'b1;
            if(pending) begin
                audio_valid<=1;
                audio_left<={fifo_data[15:0],8'd0};
                audio_right<={fifo_data[31:16],8'd0};
                if(last_pending) begin clip_end<=1; finished<=1; end
            end
        end
    end
endmodule
