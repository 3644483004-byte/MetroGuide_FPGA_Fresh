// Maps the active WAV sample stream onto the frame sequence for that scene.
// audio_total_samples comes from the selected WAV header, so the final frame
// timing follows the audio file's real duration rather than a rounded FPS.
module video_timeline #(
    parameter [10:0] SCENE1_FRAMES = 11'd523,
    parameter [10:0] SCENE2_FRAMES = 11'd376,
    parameter [10:0] SCENE3_FRAMES = 11'd523,
    parameter [10:0] SCENE4_FRAMES = 11'd503
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [1:0]  scene_id,
    input  wire        restart_track,
    input  wire        playing,
    input  wire        audio_sample_tick,
    input  wire        clip_end,
    input  wire [31:0] audio_total_samples,
    output reg  [10:0] frame_index,
    output reg         frame_update
);

    reg [1:0]  scene_id_q;
    reg [31:0] phase_accumulator;

    wire [10:0] scene_frame_count =
        (scene_id == 2'd0) ? SCENE1_FRAMES :
        (scene_id == 2'd1) ? SCENE2_FRAMES :
        (scene_id == 2'd2) ? SCENE3_FRAMES : SCENE4_FRAMES;

    wire [32:0] phase_sum = {1'b0, phase_accumulator} +
                             {22'd0, scene_frame_count};
    wire [32:0] total_samples_ext = {1'b0, audio_total_samples};
    wire scene_changed = (scene_id != scene_id_q);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            scene_id_q       <= 2'd0;
            phase_accumulator <= 32'd0;
            frame_index      <= 11'd0;
            frame_update     <= 1'b0;
        end else begin
            frame_update <= 1'b0;

            if (scene_changed || restart_track) begin
                scene_id_q        <= scene_id;
                phase_accumulator <= 32'd0;
                frame_index       <= 11'd0;
                frame_update      <= 1'b1;
            end else if (clip_end) begin
                phase_accumulator <= 32'd0;
                frame_index       <= scene_frame_count - 11'd1;
                frame_update      <= 1'b1;
            end else if (playing && audio_sample_tick &&
                         audio_total_samples != 32'd0) begin
                if (phase_sum >= total_samples_ext) begin
                    phase_accumulator <= phase_sum - total_samples_ext;
                    if (frame_index < scene_frame_count - 11'd1) begin
                        frame_index  <= frame_index + 11'd1;
                        frame_update <= 1'b1;
                    end
                end else begin
                    phase_accumulator <= phase_sum[31:0];
                end
            end
        end
    end

endmodule
