// Four-scene transport controller.  Clip completion comes from the real
// audio stream, so each scene ends at the actual end of its WAV file.
module playback_controller #(
    parameter integer AUTO_PLAY_INIT = 1
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       key1_auto_press,
    input  wire       key2_next_press,
    input  wire       key3_prev_press,
    input  wire       key4_play_press,
    input  wire [3:0] sw_select_press,
    input  wire       clip_end,
    output reg        restart_track,
    output reg  [1:0] scene_id,
    output reg        playing,
    output reg        auto_play
);

    reg clip_finished;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            scene_id  <= 2'd0;
            playing   <= 1'b1;
            auto_play <= (AUTO_PLAY_INIT != 0);
            restart_track <= 1'b0;
            clip_finished <= 1'b0;
        end else begin
            restart_track <= 1'b0;
            if (key1_auto_press)
                auto_play <= ~auto_play;
            if (sw_select_press[0]) begin
                scene_id <= 2'd0;
                playing  <= 1'b1;
                restart_track <= 1'b1;
                clip_finished <= 1'b0;
            end else if (sw_select_press[1]) begin
                scene_id <= 2'd1;
                playing  <= 1'b1;
                restart_track <= 1'b1;
                clip_finished <= 1'b0;
            end else if (sw_select_press[2]) begin
                scene_id <= 2'd2;
                playing  <= 1'b1;
                restart_track <= 1'b1;
                clip_finished <= 1'b0;
            end else if (sw_select_press[3]) begin
                scene_id <= 2'd3;
                playing  <= 1'b1;
                restart_track <= 1'b1;
                clip_finished <= 1'b0;
            end else if (key2_next_press) begin
                scene_id <= scene_id + 2'd1;
                playing  <= 1'b1;
                restart_track <= 1'b1;
                clip_finished <= 1'b0;
            end else if (key3_prev_press) begin
                scene_id <= scene_id - 2'd1;
                playing  <= 1'b1;
                restart_track <= 1'b1;
                clip_finished <= 1'b0;
            end else if (key4_play_press) begin
                if (clip_finished || clip_end) begin
                    playing <= 1'b1;
                    restart_track <= 1'b1;
                    clip_finished <= 1'b0;
                end else begin
                    playing <= ~playing;
                end
            end else if (clip_end) begin
                if (auto_play ^ key1_auto_press) begin
                    scene_id <= scene_id + 2'd1;
                    playing  <= 1'b1;
                    clip_finished <= 1'b0;
                end else begin
                    playing <= 1'b0;
                    clip_finished <= 1'b1;
                end
            end
        end
    end

endmodule
