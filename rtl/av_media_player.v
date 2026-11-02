// Shared TF-card reader and four-scene audio/video transport. The audio sample
// count is the master timeline; video is prefetched into the inactive SDRAM bank.
module av_media_player #(
    parameter [10:0] SCENE1_FRAMES=11'd523,
    parameter [10:0] SCENE2_FRAMES=11'd376,
    parameter [10:0] SCENE3_FRAMES=11'd523,
    parameter [10:0] SCENE4_FRAMES=11'd503
) (
    input wire sys_clk, pixel_clk, rst,
    input wire [1:0] scene_id,
    input wire restart_track, playing,
    output wire audio_valid,
    output wire [23:0] audio_left, audio_right,
    output wire hdmi_audio_valid,
    output wire [23:0] hdmi_audio_left, hdmi_audio_right,
    output wire clip_end,
    output wire [31:0] audio_total_samples,
    output wire [10:0] scene_frame_index,
    output wire card_ready,
    output wire [7:0] media_error,
    output wire sd_clk,
    inout wire sd_cmd,
    input wire [3:0] sd_dat,
    output wire frame_clear,
    output wire frame_abort,
    output wire transition_request,
    output reg frame_load_start,
    input wire frame_load_ready,
    output wire frame_pixel_valid,
    output wire [15:0] frame_pixel_rgb565,
    input wire frame_load_done,
    output reg frame_swap,
    input wire frame_swap_done,
    input wire video_frame_start,
    input wire transition_busy
);
    // A scene request is a mailbox: the scene remains stable while the toggle
    // crosses clock domains. A repeated selection also restarts the same scene.
    reg [1:0] scene_q, scene_mailbox;
    reg scene_toggle;
    always @(posedge pixel_clk or posedge rst) begin
        if(rst) begin scene_q<=0; scene_mailbox<=0; scene_toggle<=0; end
        else if(scene_id!=scene_q || restart_track) begin
            scene_q<=scene_id; scene_mailbox<=scene_id; scene_toggle<=~scene_toggle;
        end
    end
    (* ASYNC_REG="TRUE" *) reg [2:0] scene_toggle_sync;
    (* ASYNC_REG="TRUE" *) reg [1:0] scene_sync1, scene_sync2;
    wire scene_event=scene_toggle_sync[2]^scene_toggle_sync[1];
    reg card_ready_q;
    wire card_lost=card_ready_q && !card_ready;
    assign frame_abort=scene_event || card_lost;
    reg [1:0] active_scene;
    reg [5:0] flush_count;
    reg start_wait, audio_start, video_start;
    wire media_reset=rst || flush_count!=0;
    always @(posedge sys_clk or posedge rst) begin
        if(rst) begin
            scene_toggle_sync<=0; scene_sync1<=0; scene_sync2<=0;
            card_ready_q<=0;
            active_scene<=0; flush_count<=63; start_wait<=1;
            audio_start<=0; video_start<=0;
        end else begin
            scene_toggle_sync<={scene_toggle_sync[1:0],scene_toggle};
            card_ready_q<=card_ready;
            scene_sync1<=scene_mailbox; scene_sync2<=scene_sync1;
            audio_start<=0; video_start<=0;
            if(scene_event || card_lost) begin
                if(scene_event) active_scene<=scene_sync2;
                flush_count<=63;
                start_wait<=1;
            end else if(flush_count!=0) flush_count<=flush_count-1'b1;
            else if(start_wait && card_ready) begin
                audio_start<=1; video_start<=1; start_wait<=0;
            end
        end
    end

    wire [87:0] wav_name=active_scene==0 ? "1       WAV" :
                              active_scene==1 ? "2       WAV" :
                              active_scene==2 ? "3       WAV" : "4       WAV";
    wire [87:0] video_name=active_scene==0 ? "V1      BIN" :
                                active_scene==1 ? "V2      BIN" :
                                active_scene==2 ? "V3      BIN" : "V4      BIN";
    wire [10:0] scene_frames=active_scene==0 ? SCENE1_FRAMES :
                              active_scene==1 ? SCENE2_FRAMES :
                              active_scene==2 ? SCENE3_FRAMES : SCENE4_FRAMES;
    wire [31:0] expected_video_bytes=active_scene==0 ? SCENE1_FRAMES*32'd614400 :
                                    active_scene==1 ? SCENE2_FRAMES*32'd614400 :
                                    active_scene==2 ? SCENE3_FRAMES*32'd614400 : SCENE4_FRAMES*32'd614400;

    wire host_start, host_busy, host_valid, host_block_end, host_done, host_error;
    wire [31:0] host_lba;
    wire [5:0] host_blocks;
    wire [7:0] host_data, sector_data;
    wire card_error;
    sd_native_reader u_sd (
        .clk(sys_clk),.rst(rst),.card_ready(card_ready),.card_error(card_error),
        .sd_clk(sd_clk),.sd_cmd(sd_cmd),.sd_dat(sd_dat),
        .read_start(host_start),.read_lba(host_lba),.read_blocks(host_blocks),
        .read_busy(host_busy),.read_data(host_data),.read_valid(host_valid),
        .block_end(host_block_end),.read_done(host_done),.read_error(host_error)
    );
    wire a_req,v_req,a_grant,v_grant;
    wire [31:0] a_lba,v_lba;
    wire [5:0] a_blocks,v_blocks;
    wire a_sec_valid,a_sec_block,a_sec_done,a_sec_error;
    wire v_sec_valid,v_sec_block,v_sec_done,v_sec_error;
    sd_read_arbiter u_card_arbiter (
        .clk(sys_clk),.rst(rst),.card_ready(card_ready),
        .a_req(a_req),.a_lba(a_lba),.a_blocks(a_blocks),.a_grant(a_grant),
        .v_req(v_req),.v_lba(v_lba),.v_blocks(v_blocks),.v_grant(v_grant),
        .host_start(host_start),.host_lba(host_lba),.host_blocks(host_blocks),
        .host_busy(host_busy),.host_data(host_data),.host_valid(host_valid),
        .host_block_end(host_block_end),.host_done(host_done),.host_error(host_error),
        .data(sector_data),.a_valid(a_sec_valid),.a_block_end(a_sec_block),
        .a_done(a_sec_done),.a_error(a_sec_error),.v_valid(v_sec_valid),
        .v_block_end(v_sec_block),.v_done(v_sec_done),.v_error(v_sec_error)
    );
    wire a_open,a_file_error,a_file_done,a_data_valid;
    wire [31:0] a_file_size,a_offset;
    wire [7:0] a_data;
    wire v_open,v_file_error,v_file_done,v_data_valid;
    wire [31:0] v_file_size,v_offset;
    wire [7:0] v_data;
    reg audio_accept,video_accept,a_wait_open,v_wait_open;
    always @(posedge sys_clk or posedge media_reset) begin
        if(media_reset) begin
            audio_accept<=0;video_accept<=0;a_wait_open<=0;v_wait_open<=0;
        end else begin
            if(audio_start) begin audio_accept<=0;a_wait_open<=1;end
            else if(a_wait_open && !a_open) begin audio_accept<=1;a_wait_open<=0;end
            if(video_start) begin video_accept<=0;v_wait_open<=1;end
            else if(v_wait_open && !v_open) begin video_accept<=1;v_wait_open<=0;end
        end
    end

    wire [11:0] audio_fifo_count;
    wire audio_fifo_full,audio_fifo_empty,audio_fifo_read,audio_sample_write;
    wire [31:0] audio_sample_data,audio_fifo_data,total_samples_sys;
    wire header_ready,wav_error;
    reg audio_file_fault,audio_overflow;
    reg audio_allow;
    always @(posedge sys_clk or posedge media_reset) begin
        if(media_reset) audio_allow<=1'b0;
        else audio_allow<=audio_accept && !audio_file_fault && !wav_error &&
                          audio_fifo_count<=12'd1536;
    end
    fat32_file_stream u_wav_file (
        .clk(sys_clk),.rst(rst),.card_ready(card_ready),.start(audio_start),
        .file_name(wav_name),.allow_read(audio_allow),.max_blocks(6'd4),
        .file_open(a_open),.file_size(a_file_size),.file_error(a_file_error),
        .file_done(a_file_done),.data(a_data),.data_valid(a_data_valid),.byte_offset(a_offset),
        .req_valid(a_req),.req_lba(a_lba),.req_blocks(a_blocks),.req_grant(a_grant),
        .sec_data(sector_data),.sec_valid(a_sec_valid),.sec_block_end(a_sec_block),
        .sec_done(a_sec_done),.sec_error(a_sec_error)
    );
    wav_pcm_ingest u_wav_unpack (
        .clk(sys_clk),.rst(media_reset),.file_size(a_file_size),.data(a_data),
        .data_valid(a_data_valid && audio_accept && !audio_start),.byte_offset(a_offset),
        .header_ready(header_ready),.total_samples(total_samples_sys),
        .format_error(wav_error),.sample_write(audio_sample_write),.sample_data(audio_sample_data)
    );
    always @(posedge sys_clk or posedge media_reset) begin
        if(media_reset) begin audio_file_fault<=0;audio_overflow<=0;end
        else begin
            if(audio_accept && a_file_error) audio_file_fault<=1;
            if(audio_sample_write && audio_fifo_full) audio_overflow<=1;
        end
    end
    av_async_fifo #(.DATA_BITS(32),.ADDR_BITS(11)) u_audio_fifo (
        .rst(media_reset),.wr_clk(sys_clk),.wr_en(audio_sample_write),
        .wr_data(audio_sample_data),.wr_full(audio_fifo_full),.wr_count(audio_fifo_count),
        .rd_clk(pixel_clk),.rd_en(audio_fifo_read),.rd_data(audio_fifo_data),.rd_empty(audio_fifo_empty)
    );

    localparam V_WAIT_FILE=3'd0,V_WAIT_READY=3'd1,V_STREAM=3'd2,
               V_WAIT_LOAD=3'd3,V_WAIT_TARGET=3'd4,V_WAIT_SWAP=3'd5,V_FINISHED=3'd6;
    reg [2:0] video_state;
    reg video_fault,video_size_fault,video_overflow,video_clear;
    reg [10:0] load_index;
    reg [19:0] frame_byte_count;
    reg [10:0] frame_sectors_left;
    reg [7:0] video_low;
    reg video_byte_lane,video_fifo_write;
    reg [15:0] video_fifo_wdata;
    wire video_fifo_full,video_fifo_empty;
    wire [12:0] video_fifo_count;
    wire [15:0] video_fifo_rdata;
    reg video_fifo_space_ok;
    // Register the occupancy threshold in the sys_clk domain. The FAT reader
    // samples allow_read only between bursts, and the threshold leaves room
    // for a maximum 8-sector burst in the 4096-word FIFO.
    always @(posedge sys_clk or posedge media_reset) begin
        if(media_reset) video_fifo_space_ok<=1'b1;
        else video_fifo_space_ok<=(video_fifo_count<=13'd2044);
    end
    wire [10:0] remaining_frame_sectors=frame_sectors_left;
    wire [5:0] video_burst_limit=remaining_frame_sectors>=8 ? 6'd8 :
                               {1'b0,remaining_frame_sectors[4:0]};
    wire video_allow=video_accept && !video_fault && video_state==V_STREAM &&
                     video_fifo_space_ok && remaining_frame_sectors!=0;
    fat32_file_stream u_video_file (
        .clk(sys_clk),.rst(rst),.card_ready(card_ready),.start(video_start),
        .file_name(video_name),.allow_read(video_allow),.max_blocks(video_burst_limit),
        .file_open(v_open),.file_size(v_file_size),.file_error(v_file_error),
        .file_done(v_file_done),.data(v_data),.data_valid(v_data_valid),.byte_offset(v_offset),
        .req_valid(v_req),.req_lba(v_lba),.req_blocks(v_blocks),.req_grant(v_grant),
        .sec_data(sector_data),.sec_valid(v_sec_valid),.sec_block_end(v_sec_block),
        .sec_done(v_sec_done),.sec_error(v_sec_error)
    );
    wire [10:0] target_frame;
    wire target_update;
    assign scene_frame_index=target_frame;
    video_timeline #(.SCENE1_FRAMES(SCENE1_FRAMES),.SCENE2_FRAMES(SCENE2_FRAMES),
                     .SCENE3_FRAMES(SCENE3_FRAMES),.SCENE4_FRAMES(SCENE4_FRAMES)) u_timeline (
        .clk(pixel_clk),.rst(media_reset),.scene_id(scene_id),.restart_track(restart_track),
        .playing(playing),.audio_sample_tick(audio_valid),.clip_end(clip_end),
        .audio_total_samples(audio_total_samples),.frame_index(target_frame),.frame_update(target_update)
    );
    reg [10:0] target_gray;
    (* ASYNC_REG="TRUE" *) reg [10:0] target_sync1,target_sync2;
    function [10:0] gray_to_frame;
        input [10:0] value;
        integer n;
        begin
            gray_to_frame[10]=value[10];
            for(n=9;n>=0;n=n-1) gray_to_frame[n]=gray_to_frame[n+1]^value[n];
        end
    endfunction
    wire [10:0] target_sys=gray_to_frame(target_sync2);
    always @(posedge pixel_clk or posedge media_reset) begin
        if(media_reset) target_gray<=0;
        else target_gray<=(target_frame>>1)^target_frame;
    end
    always @(posedge sys_clk or posedge media_reset) begin
        if(media_reset) begin target_sync1<=0;target_sync2<=0;end
        else begin target_sync1<=target_gray;target_sync2<=target_sync1;end
    end
    assign frame_clear=video_clear;
    assign transition_request=(video_state==V_WAIT_SWAP && load_index==11'd0);
    always @(posedge sys_clk or posedge media_reset) begin
        if(media_reset) begin
            video_state<=V_WAIT_FILE;frame_load_start<=0;frame_swap<=0;
            video_fault<=0;video_size_fault<=0;video_overflow<=0;
            video_clear<=0;load_index<=0;frame_byte_count<=0;frame_sectors_left<=1200;video_low<=0;
            video_byte_lane<=0;video_fifo_write<=0;video_fifo_wdata<=0;
        end else begin
            frame_load_start<=0;frame_swap<=0;video_clear<=0;video_fifo_write<=0;
            if(video_accept && v_file_error && !video_fault) begin
                video_fault<=1;video_clear<=1;video_state<=V_FINISHED;
            end else if(video_fifo_write && video_fifo_full) begin
                video_overflow<=1;video_fault<=1;video_clear<=1;video_state<=V_FINISHED;
            end else if(!video_fault) begin
                case(video_state)
                    V_WAIT_FILE: if(video_accept && v_open && !video_start) begin
                        if(v_file_size!=expected_video_bytes) begin
                            video_size_fault<=1;video_fault<=1;video_clear<=1;video_state<=V_FINISHED;
                        end else begin frame_load_start<=1;video_state<=V_WAIT_READY;end
                    end
                    V_WAIT_READY: if(frame_load_ready) video_state<=V_STREAM;
                    V_STREAM: if(v_data_valid) begin
                        if(!video_byte_lane) video_low<=v_data;
                        else begin video_fifo_wdata<={v_data,video_low};video_fifo_write<=1;end
                        video_byte_lane<=~video_byte_lane;
                        if(frame_byte_count==20'd614399) begin
                            frame_byte_count<=0;frame_sectors_left<=1200;video_state<=V_WAIT_LOAD;
                        end else begin
                            frame_byte_count<=frame_byte_count+1'b1;
                            if(frame_byte_count[8:0]==9'd511)
                                frame_sectors_left<=frame_sectors_left-1'b1;
                        end
                    end
                    V_WAIT_LOAD: if(frame_load_done) video_state<=V_WAIT_TARGET;
                    V_WAIT_TARGET: if(load_index==0 || target_sys>=load_index) begin
                        frame_swap<=1;video_state<=V_WAIT_SWAP;
                    end
                    V_WAIT_SWAP: if(frame_swap_done) begin
                        if(load_index+1'b1<scene_frames) begin
                            load_index<=load_index+1'b1;frame_load_start<=1;video_state<=V_WAIT_READY;
                        end else video_state<=V_FINISHED;
                    end
                    default: begin end
                endcase
            end
        end
    end
    // The input burst cannot be stalled. This queue reserves space for a full
    // eight-sector burst, then presents an elastic pixel to the SDRAM writer.
    reg video_read_pending,video_hold_valid;
    reg [3:0] video_write_spacing;
    reg [15:0] video_hold_data;
    wire video_loading=video_state==V_WAIT_READY || video_state==V_STREAM || video_state==V_WAIT_LOAD;
    wire video_fifo_read=video_loading && !video_read_pending && !video_hold_valid && !video_fifo_empty;
    // Keep a queued burst from monopolizing the SDRAM's write-priority port.
    // 8.33 Mpixel/s exceeds SD's 6.25 Mpixel/s payload rate while leaving
    // enough memory cycles for the currently displayed 25 MHz raster.
    assign frame_pixel_valid=video_hold_valid && video_loading && !video_fault && video_write_spacing==0;
    assign frame_pixel_rgb565=video_hold_data;
    av_async_fifo #(.DATA_BITS(16),.ADDR_BITS(12)) u_video_queue (
        .rst(media_reset),.wr_clk(sys_clk),.wr_en(video_fifo_write),.wr_data(video_fifo_wdata),
        .wr_full(video_fifo_full),.wr_count(video_fifo_count),.rd_clk(sys_clk),
        .rd_en(video_fifo_read),.rd_data(video_fifo_rdata),.rd_empty(video_fifo_empty)
    );
    always @(posedge sys_clk or posedge media_reset) begin
        if(media_reset) begin video_read_pending<=0;video_hold_valid<=0;video_hold_data<=0;video_write_spacing<=0;end
        else begin
            video_read_pending<=video_fifo_read;
            if(video_write_spacing!=0) video_write_spacing<=video_write_spacing-1'b1;
            if(frame_pixel_valid && frame_load_ready) begin
                video_hold_valid<=0;video_write_spacing<=11;
            end
            if(video_read_pending) begin video_hold_data<=video_fifo_rdata;video_hold_valid<=1;end
        end
    end
    // Wait three complete HDMI raster periods after the first visible frame
    // before releasing PCM; this compensates the display's startup latency.
    localparam [1:0] AUDIO_STARTUP_DELAY_FRAMES = 2'd3;
    reg [1:0] audio_startup_frame_count;
    reg video_started_pixel;
    always @(posedge pixel_clk or posedge media_reset) begin
        if(media_reset) begin
            video_started_pixel<=0;
            audio_startup_frame_count<=0;
        end else if(video_frame_start && !video_started_pixel && !transition_busy) begin
            if(audio_startup_frame_count == AUDIO_STARTUP_DELAY_FRAMES)
                video_started_pixel<=1;
            else
                audio_startup_frame_count<=audio_startup_frame_count+1'b1;
        end
    end

    // Some HDMI sinks need time to lock their audio clock after video appears.
    // Send 48 kHz silent samples during the first 4.5 seconds so the sink and ACR
    // path are stable before the first real PCM sample is released. This is a
    // cold-start-only sequence; scene changes keep the already-locked link.
    localparam [6:0] HDMI_PREROLL_FRAMES_PER_SECOND = 7'd60;
    reg [6:0] hdmi_preroll_frame_count;
    reg [2:0] hdmi_preroll_second_count;
    wire hdmi_preroll_second_tick =
        hdmi_preroll_frame_count==HDMI_PREROLL_FRAMES_PER_SECOND-1'b1;

    reg hdmi_preroll_started, hdmi_preroll_done, hdmi_silence_valid;
    reg [24:0] hdmi_preroll_phase;
    wire [25:0] hdmi_preroll_phase_sum =
        {1'b0,hdmi_preroll_phase} + 26'd48000;
    wire hdmi_preroll_sample_tick = hdmi_preroll_phase_sum >= 26'd25000000;
    always @(posedge pixel_clk or posedge rst) begin
        if(rst) begin
            hdmi_preroll_frame_count<=0;
            hdmi_preroll_second_count<=0;
            hdmi_preroll_started<=0;
            hdmi_preroll_done<=0;
            hdmi_silence_valid<=0;
            hdmi_preroll_phase<=0;
        end else begin
            hdmi_silence_valid<=0;
            if(!hdmi_preroll_done && video_frame_start && !transition_busy) begin
                hdmi_preroll_started<=1;
                if(hdmi_preroll_second_count==3'd4 && hdmi_preroll_frame_count==7'd29)
                    hdmi_preroll_done<=1;
                if(hdmi_preroll_second_tick) begin
                    hdmi_preroll_frame_count<=0;
                    if(hdmi_preroll_second_count<3'd4)
                        hdmi_preroll_second_count<=hdmi_preroll_second_count+1'b1;
                end else hdmi_preroll_frame_count<=hdmi_preroll_frame_count+1'b1;
            end
            if(hdmi_preroll_started && !hdmi_preroll_done) begin
                if(hdmi_preroll_sample_tick) begin
                    hdmi_preroll_phase<=hdmi_preroll_phase_sum-26'd25000000;
                    hdmi_silence_valid<=1;
                end else hdmi_preroll_phase<=hdmi_preroll_phase_sum[24:0];
            end
        end
    end
    wire audio_gate=(video_started_pixel && hdmi_preroll_done) || video_fault;
    assign hdmi_audio_valid=audio_valid || hdmi_silence_valid;
    assign hdmi_audio_left=audio_valid ? audio_left : 24'd0;
    assign hdmi_audio_right=audio_valid ? audio_right : 24'd0;
    av_pcm_consumer u_audio_output (
        .clk(pixel_clk),.rst(media_reset),.playing(playing),
        .header_ready_async(header_ready && !audio_file_fault && !audio_overflow),
        .video_ready_async(audio_gate),.total_samples_async(total_samples_sys),
        .fifo_empty(audio_fifo_empty),.fifo_data(audio_fifo_data),.fifo_read(audio_fifo_read),
        .audio_valid(audio_valid),.audio_left(audio_left),.audio_right(audio_right),
        .clip_end(clip_end),.total_samples(audio_total_samples)
    );
    assign media_error={1'b0,video_overflow,video_size_fault,video_fault,
                        audio_overflow,wav_error,audio_file_fault,card_error};
endmodule
