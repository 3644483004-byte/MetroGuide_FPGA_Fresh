module top (
    input  wire clk,          // 50 MHz board reference clock

    input  wire key1_n,       // KEY1: auto-play toggle, active low
    input  wire key2_n,       // KEY2: next scene, active low
    input  wire key3_n,       // KEY3: previous scene, active low
    input  wire key4_n,       // KEY4: play/pause, active low
    input  wire sw1,          // SW1..SW4: direct scene selection, switch up = high
    input  wire sw2,
    input  wire sw3,
    input  wire sw4,

    input  wire sd_ncs,      // Native SD DAT3 (same physical TF pin as SPI CS)
    output wire sd_dclk,
    inout  wire sd_mosi,     // Native SD CMD (same physical TF pin as SPI MOSI)
    input  wire sd_miso,
    input  wire sd_dat1,
    input  wire sd_dat2,

    output wire HDMI_CLK_P,
    output wire HDMI_D2_P,
    output wire HDMI_D1_P,
    output wire HDMI_D0_P,
    output wire HDMI_DDC_SCL,
    inout  wire HDMI_DDC_SDA
);

    wire pll_lock;
    wire pixel_clk;
    wire serial_clk;
    wire sys_clk,mem_clk,mem_clk_shift,sys_pll_lock;
    wire rst = ~pll_lock || ~sys_pll_lock;

    wire key1_press;
    wire key2_press;
    wire key3_press;
    wire key4_press;
    wire sw1_press;
    wire sw2_press;
    wire sw3_press;
    wire sw4_press;

    wire [1:0] scene_id;
    wire playing;
    wire auto_play;
    wire restart_track;
    wire clip_end_pulse;

    wire        axis_user;
    wire        axis_valid;
    wire        axis_last;
    wire [23:0] axis_data;
    wire        audio_valid;
    wire [23:0] audio_left;
    wire [23:0] audio_right;
    wire        hdmi_audio_valid;
    wire [23:0] hdmi_audio_left;
    wire [23:0] hdmi_audio_right;
    wire        acr_valid;
    wire [19:0] acr_cts;
    wire [19:0] acr_n;
    wire        sd_card_ready;
    wire [31:0] audio_total_samples;
    wire [10:0] scene_frame_index;
    wire [7:0]  media_error;
    wire frame_clear,frame_abort,frame_load_start,frame_load_ready,frame_pixel_valid;
    wire frame_transition_request;
    wire [15:0] frame_pixel_rgb565;
    wire frame_load_done,frame_swap,frame_swap_done;
    wire video_frame_start;
    wire video_transition_busy;

    wire [9:0] tmds_ch0;
    wire [9:0] tmds_ch1;
    wire [9:0] tmds_ch2;
    wire [9:0] tmds_clk;

    reg [19:0] edid_count;
    reg        edid_trig;

    video_pll u_video_pll (
        .refclk   (clk),
        .reset    (1'b0),
        .extlock  (pll_lock),
        .clk0_out (pixel_clk),
        .clk1_out (serial_clk)
    );

    sys_pll u_system_pll (
        .refclk(clk),.reset(1'b0),.extlock(sys_pll_lock),
        .clk0_out(sys_clk),.clk1_out(mem_clk),.clk2_out(mem_clk_shift)
    );

    button_debounce #(.ACTIVE_LOW(1), .DEBOUNCE_CYCLES(500_000)) u_key1_db (
        .clk(pixel_clk), .rst(rst), .pin_in(key1_n), .press_pulse(key1_press)
    );
    button_debounce #(.ACTIVE_LOW(1), .DEBOUNCE_CYCLES(500_000)) u_key2_db (
        .clk(pixel_clk), .rst(rst), .pin_in(key2_n), .press_pulse(key2_press)
    );
    button_debounce #(.ACTIVE_LOW(1), .DEBOUNCE_CYCLES(500_000)) u_key3_db (
        .clk(pixel_clk), .rst(rst), .pin_in(key3_n), .press_pulse(key3_press)
    );
    button_debounce #(.ACTIVE_LOW(1), .DEBOUNCE_CYCLES(500_000)) u_key4_db (
        .clk(pixel_clk), .rst(rst), .pin_in(key4_n), .press_pulse(key4_press)
    );
    button_debounce #(.ACTIVE_LOW(0), .DEBOUNCE_CYCLES(500_000)) u_sw1_db (
        .clk(pixel_clk), .rst(rst), .pin_in(sw1), .press_pulse(sw1_press)
    );
    button_debounce #(.ACTIVE_LOW(0), .DEBOUNCE_CYCLES(500_000)) u_sw2_db (
        .clk(pixel_clk), .rst(rst), .pin_in(sw2), .press_pulse(sw2_press)
    );
    button_debounce #(.ACTIVE_LOW(0), .DEBOUNCE_CYCLES(500_000)) u_sw3_db (
        .clk(pixel_clk), .rst(rst), .pin_in(sw3), .press_pulse(sw3_press)
    );
    button_debounce #(.ACTIVE_LOW(0), .DEBOUNCE_CYCLES(500_000)) u_sw4_db (
        .clk(pixel_clk), .rst(rst), .pin_in(sw4), .press_pulse(sw4_press)
    );

    playback_controller #(.AUTO_PLAY_INIT(1)) u_playback_controller (
        .clk              (pixel_clk),
        .rst              (rst),
        .key1_auto_press  (key1_press),
        .key2_next_press  (key2_press),
        .key3_prev_press  (key3_press),
        .key4_play_press  (key4_press),
        .sw_select_press  ({sw4_press, sw3_press, sw2_press, sw1_press}),
        .clip_end         (clip_end_pulse),
        .restart_track    (restart_track),
        .scene_id         (scene_id),
        .playing          (playing),
        .auto_play        (auto_play)
    );

    av_media_player u_media_player (
        .sys_clk(sys_clk),.pixel_clk(pixel_clk),.rst(rst),
        .scene_id(scene_id),.restart_track(restart_track),.playing(playing),
        .audio_valid(audio_valid),.audio_left(audio_left),.audio_right(audio_right),
        .hdmi_audio_valid(hdmi_audio_valid),
        .hdmi_audio_left(hdmi_audio_left),.hdmi_audio_right(hdmi_audio_right),
        .clip_end(clip_end_pulse),.audio_total_samples(audio_total_samples),
        .scene_frame_index(scene_frame_index),
        .card_ready(sd_card_ready),.media_error(media_error),
        .sd_clk(sd_dclk),.sd_cmd(sd_mosi),.sd_dat({sd_ncs,sd_dat2,sd_dat1,sd_miso}),
        .frame_clear(frame_clear),.frame_abort(frame_abort),
        .transition_request(frame_transition_request),
        .frame_load_start(frame_load_start),
        .frame_load_ready(frame_load_ready),.frame_pixel_valid(frame_pixel_valid),
        .frame_pixel_rgb565(frame_pixel_rgb565),.frame_load_done(frame_load_done),
        .frame_swap(frame_swap),.frame_swap_done(frame_swap_done),
        .video_frame_start(video_frame_start),.transition_busy(video_transition_busy)
    );

    audio_arc_calculate #(.ACR_N(6144)) u_audio_acr (
        .I_clk         (pixel_clk),
        .I_rst         (rst),
        .I_audio_valid (hdmi_audio_valid),
        .O_acr_valid   (acr_valid),
        .O_acr_cts     (acr_cts),
        .O_acr_n       (acr_n)
    );

    video_framebuffer u_video_frames (
        .sys_clk(sys_clk),.mem_clk(mem_clk),.mem_clk_shift(mem_clk_shift),
        .pixel_clk(pixel_clk),.rst(rst),.clear(frame_clear),.load_cancel(frame_abort),
        .load_start(frame_load_start),.load_ready(frame_load_ready),
        .pixel_valid(frame_pixel_valid),.pixel_rgb565(frame_pixel_rgb565),
        .load_done(frame_load_done),.swap_request(frame_swap),
        .transition_request(frame_transition_request),.swap_done(frame_swap_done),
        .visible_frame_start(video_frame_start),.transition_busy(video_transition_busy),
        .video_user(axis_user),.video_valid(axis_valid),
        .video_last(axis_last),.video_data(axis_data)
    );

    wire [23:0] osd_axis_data;
    metroguide_osd u_metroguide_osd (
        .pixel_clk(pixel_clk),.rst(rst),.scene_id(scene_id),
        .frame_index(scene_frame_index),.playing(playing),.auto_play(auto_play),
        .key4_press(key4_press),.restart_track(restart_track),
        .audio_valid(audio_valid),.audio_left(audio_left),.audio_right(audio_right),
        .s_user(axis_user),.s_valid(axis_valid),.s_last(axis_last),.s_data(axis_data),
        .m_data(osd_axis_data)
    );

    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            edid_count <= 20'd0;
            edid_trig  <= 1'b0;
        end else begin
            edid_trig <= 1'b0;
            if (edid_count < 20'd100000) begin
                edid_count <= edid_count + 1'b1;
                if (edid_count == 20'd99999)
                    edid_trig <= 1'b1;
            end
        end
    end

    hdmi_1_4b_transmitter_core_wrapper #(
        .DEVICE            ("EG"),
        .HTOTAL            (800),
        .HSA               (96),
        .HFP               (16),
        .HBP               (48),
        .HACTIVE           (640),
        .VTOTAL            (525),
        .VSA               (2),
        .VFP               (10),
        .VBP               (33),
        .VACTIVE           (480),
        .VIDEO_VIC         (1),
        .VIDEO_TPG         ("Disable"),
        .VIDEO_FORMAT      ("RGB"),
        .AUDIO_SAMPLE_RATE ("48K"),
        .IIC_SCL_DIV       (250)
    ) u_hdmi_transmitter (
        .I_pixel_clk        (pixel_clk),
        .I_rst              (rst),
        .I_edid_read_trig   (edid_trig),
        .O_edid_read_valid  (),
        .O_edid_read_data   (),
        .I_axis_s_user      (axis_user),
        .I_axis_s_valid     (axis_valid),
        .I_axis_s_last      (axis_last),
        .I_axis_s_data      (osd_axis_data),
        .O_axis_s_ready     (),
        .I_audio_valid      (hdmi_audio_valid),
        .I_audio_left_data  (hdmi_audio_left),
        .I_audio_right_data (hdmi_audio_right),
        .I_acr_valid        (acr_valid),
        .I_acr_cts          (acr_cts),
        .I_acr_n            (acr_n),
        .O_video_locked     (),
        .O_ddc_scl          (HDMI_DDC_SCL),
        .IO_ddc_sda         (HDMI_DDC_SDA),
        .O_ch0_tmds_data    (tmds_ch0),
        .O_ch1_tmds_data    (tmds_ch1),
        .O_ch2_tmds_data    (tmds_ch2),
        .O_clk_tmds_data    (tmds_clk)
    );

    hdmi_phy_wrapper #(.DEVICE("EG")) u_hdmi_phy (
        .I_pixel_clk        (pixel_clk),
        .I_serial_clk       (serial_clk),
        .I_rst              (rst),
        .I_tmds_channel_0   (tmds_ch0),
        .I_tmds_channel_1   (tmds_ch1),
        .I_tmds_channel_2   (tmds_ch2),
        .I_tmds_channel_clk (tmds_clk),
        .O_tmds_ch0_p       (HDMI_D0_P),
        .O_tmds_ch1_p       (HDMI_D1_P),
        .O_tmds_ch2_p       (HDMI_D2_P),
        .O_tmds_clk_p       (HDMI_CLK_P)
    );

endmodule

// FPGA-rendered OSD: current chapter, autoplay state, pause indicator, and
// elapsed/total time with a frame-synchronous progress bar.
module metroguide_osd #(
    // The board's 50 MHz oscillator is divided to the 25 MHz pixel clock.
    // Edit these startup values when setting the clock for a new power-up.
    parameter integer PIXEL_CLK_HZ = 25000000,
    parameter integer START_YEAR = 2026,
    parameter integer START_MONTH = 10,
    parameter integer START_DAY = 30,
    parameter integer START_HOUR = 17,
    parameter integer START_MINUTE = 38,
    parameter integer START_SECOND = 0
) (
    input  wire        pixel_clk,
    input  wire        rst,
    input  wire [1:0]  scene_id,
    input  wire [10:0] frame_index,
    input  wire        playing,
    input  wire        auto_play,
    input  wire        key4_press,
    input  wire        restart_track,
    input  wire        audio_valid,
    input  wire [23:0] audio_left,
    input  wire [23:0] audio_right,
    input  wire        s_user,
    input  wire        s_valid,
    input  wire        s_last,
    input  wire [23:0] s_data,
    output reg  [23:0] m_data
);
    reg [9:0] x;
    reg [8:0] y;
    reg [9:0] wm_x;
    reg [8:0] wm_y;
    reg [3:0] wm_frame_div;
    reg       wm_reverse;
    reg       wm_paused;
    reg [24:0] wallclock_divider;
    reg [3:0] year_thousands, year_hundreds, year_tens, year_ones;
    reg [3:0] month_tens, month_ones, day_tens, day_ones;
    reg [3:0] hour_tens, hour_ones, minute_tens, minute_ones;
    reg [3:0] second_tens, second_ones;
    wire year_is_leap = leap_year_bcd(year_thousands, year_hundreds,
                                      year_tens, year_ones);
    wire month_is_february = month_tens == 4'd0 && month_ones == 4'd2;
    wire month_has_30_days =
        (month_tens == 4'd0 &&
         (month_ones == 4'd4 || month_ones == 4'd6 || month_ones == 4'd9)) ||
        (month_tens == 4'd1 && month_ones == 4'd1);
    wire date_is_month_end =
        (month_is_february &&
         ((year_is_leap && day_tens == 4'd2 && day_ones == 4'd9) ||
          (!year_is_leap && day_tens == 4'd2 && day_ones == 4'd8))) ||
        (month_has_30_days && day_tens == 4'd3 && day_ones == 4'd0) ||
        (!month_is_february && !month_has_30_days &&
         day_tens == 4'd3 && day_ones == 4'd1);

    // BCD counters keep the display path shallow. The clock starts from the
    // editable Beijing-time values whenever the FPGA is powered/reset.
    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            wallclock_divider <= 25'd0;
            year_thousands <= (START_YEAR / 1000) % 10;
            year_hundreds <= (START_YEAR / 100) % 10;
            year_tens <= (START_YEAR / 10) % 10;
            year_ones <= START_YEAR % 10;
            month_tens <= (START_MONTH / 10) % 10;
            month_ones <= START_MONTH % 10;
            day_tens <= (START_DAY / 10) % 10;
            day_ones <= START_DAY % 10;
            hour_tens <= (START_HOUR / 10) % 10;
            hour_ones <= START_HOUR % 10;
            minute_tens <= (START_MINUTE / 10) % 10;
            minute_ones <= START_MINUTE % 10;
            second_tens <= (START_SECOND / 10) % 10;
            second_ones <= START_SECOND % 10;
        end else if (wallclock_divider == PIXEL_CLK_HZ - 1) begin
            wallclock_divider <= 25'd0;
            if (second_ones == 4'd9) begin
                second_ones <= 4'd0;
                if (second_tens == 4'd5) begin
                    second_tens <= 4'd0;
                    if (minute_ones == 4'd9) begin
                        minute_ones <= 4'd0;
                        if (minute_tens == 4'd5) begin
                            minute_tens <= 4'd0;
                            if (hour_tens == 4'd2 && hour_ones == 4'd3) begin
                                hour_tens <= 4'd0;
                                hour_ones <= 4'd0;
                                if (date_is_month_end) begin
                                    day_tens <= 4'd0;
                                    day_ones <= 4'd1;
                                    if (month_tens == 4'd1 && month_ones == 4'd2) begin
                                        month_tens <= 4'd0;
                                        month_ones <= 4'd1;
                                        if (year_ones == 4'd9) begin
                                            year_ones <= 4'd0;
                                            if (year_tens == 4'd9) begin
                                                year_tens <= 4'd0;
                                                if (year_hundreds == 4'd9) begin
                                                    year_hundreds <= 4'd0;
                                                    year_thousands <= year_thousands + 1'b1;
                                                end else begin
                                                    year_hundreds <= year_hundreds + 1'b1;
                                                end
                                            end else begin
                                                year_tens <= year_tens + 1'b1;
                                            end
                                        end else begin
                                            year_ones <= year_ones + 1'b1;
                                        end
                                    end else if (month_ones == 4'd9) begin
                                        month_ones <= 4'd0;
                                        month_tens <= month_tens + 1'b1;
                                    end else begin
                                        month_ones <= month_ones + 1'b1;
                                    end
                                end else if (day_ones == 4'd9) begin
                                    day_ones <= 4'd0;
                                    day_tens <= day_tens + 1'b1;
                                end else begin
                                    day_ones <= day_ones + 1'b1;
                                end
                            end else if (hour_ones == 4'd9) begin
                                hour_ones <= 4'd0;
                                hour_tens <= hour_tens + 1'b1;
                            end else begin
                                hour_ones <= hour_ones + 1'b1;
                            end
                        end else begin
                            minute_tens <= minute_tens + 1'b1;
                        end
                    end else begin
                        minute_ones <= minute_ones + 1'b1;
                    end
                end else begin
                    second_tens <= second_tens + 1'b1;
                end
            end else begin
                second_ones <= second_ones + 1'b1;
            end
        end else begin
            wallclock_divider <= wallclock_divider + 1'b1;
        end
    end

    // Read-only PCM amplitude monitor: max absolute stereo sample per 20 ms,
    // with a quick attack and gentle release for a legible level envelope.
    wire signed [15:0] audio_left_sample = audio_left[23:8];
    wire signed [15:0] audio_right_sample = audio_right[23:8];
    wire [15:0] audio_left_magnitude = audio_left_sample[15] ?
                                      -audio_left_sample : audio_left_sample;
    wire [15:0] audio_right_magnitude = audio_right_sample[15] ?
                                       -audio_right_sample : audio_right_sample;
    wire [15:0] audio_peak_magnitude = audio_left_magnitude > audio_right_magnitude ?
                                      audio_left_magnitude : audio_right_magnitude;
    wire [7:0] audio_sample_level = audio_peak_magnitude[15:8];
    reg [9:0] audio_window_count;
    reg [7:0] audio_peak_window;
    reg [7:0] audio_level;
    wire [7:0] audio_peak_next = (audio_sample_level > audio_peak_window) ?
                                  audio_sample_level : audio_peak_window;
    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            audio_window_count <= 10'd0;
            audio_peak_window <= 8'd0;
            audio_level <= 8'd0;
        end else if (audio_valid) begin
            if (audio_window_count == 10'd959) begin
                audio_window_count <= 10'd0;
                audio_peak_window <= 8'd0;
                if (audio_peak_next > audio_level)
                    audio_level <= audio_peak_next;
                else if (audio_level > 8'd4)
                    audio_level <= audio_level - {2'b00,audio_level[7:2]};
                else if (audio_level != 8'd0)
                    audio_level <= audio_level - 1'b1;
            end else begin
                audio_window_count <= audio_window_count + 1'b1;
                audio_peak_window <= audio_peak_next;
            end
        end
    end
    wire [10:0] audio_level_times_9 = {audio_level,3'b000} +
                                     {3'b000,audio_level};
    wire [8:0] audio_meter_height = audio_level_times_9[10:2];
    wire [8:0] audio_meter_fill_start = 9'd383 - audio_meter_height;
    wire audio_meter_panel = x >= 10'd610 && x < 10'd630 &&
                             y >= 9'd96 && y < 9'd384;
    wire audio_meter_fill = x >= 10'd614 && x < 10'd626 &&
                            y >= 9'd96 && y >= audio_meter_fill_start &&
                            y < 9'd383 &&
                            audio_meter_height != 9'd0;
    wire audio_meter_outline = audio_meter_panel &&
                               (x == 10'd610 || x == 10'd629 ||
                                y == 9'd96 || y == 9'd383);

    // Coordinates follow the registered AXI video stream. s_user marks its
    // first pixel; s_last marks the final pixel in each active line.
    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            x <= 0;
            y <= 0;
        end else if (s_valid) begin
            if (s_user) begin
                x <= 1;
                y <= 0;
            end else if (s_last) begin
                x <= 0;
                if (y == 9'd479) y <= 0;
                else y <= y + 1'b1;
            end else if (x < 10'd639) x <= x + 1'b1;
        end
    end

    // Latch the debounced KEY4 pause state independently of the video scan.
    // Restarting/selecting a clip clears the latch; stopped clips also freeze.
    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            wm_paused <= 1'b0;
        end else if (restart_track) begin
            wm_paused <= 1'b0;
        end else if (key4_press) begin
            wm_paused <= playing;
        end else if (!playing) begin
            wm_paused <= 1'b1;
        end
    end

    // Competition emblem watermark: 8 horizontal pixels per second.
    // KEY4 pause freezes its position and movement timer.
    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            wm_x <= 10'd8;
            wm_y <= 9'd297;
            wm_frame_div <= 4'd0;
            wm_reverse <= 1'b0;
        end else if (s_user) begin
            if (!playing || wm_paused || key4_press) begin
                wm_frame_div <= 4'd0;
            end else if (wm_frame_div == 4'd14) begin
                wm_frame_div <= 4'd0;
                if (!wm_reverse) begin
                    if (wm_x >= 10'd496) begin
                        wm_reverse <= 1'b1;
                    end else begin
                        wm_x <= wm_x + 10'd2;
                        wm_y <= wm_y - 1'b1;
                    end
                end else begin
                    if (wm_x <= 10'd8) begin
                        wm_reverse <= 1'b0;
                    end else begin
                        wm_x <= wm_x - 10'd2;
                        wm_y <= wm_y + 1'b1;
                    end
                end
            end else begin
                wm_frame_div <= wm_frame_div + 1'b1;
            end
        end
    end

    wire watermark_area = x >= wm_x && x < wm_x + 10'd144 &&
                          y >= wm_y && y < wm_y + 9'd50;
    wire [7:0] watermark_local_x = x - wm_x;
    wire [6:0] watermark_local_y = y - wm_y;
    wire [143:0] watermark_bits = watermark_row(watermark_local_y);
    wire watermark_pixel = watermark_area &&
                           watermark_bits[8'd143 - watermark_local_x];
    wire [9:0] watermark_red_mix = ({2'b00,s_data[23:16]} << 1) +
                                    {2'b00,s_data[23:16]} + 10'd42;
    wire [9:0] watermark_green_mix = ({2'b00,s_data[15:8]} << 1) +
                                      {2'b00,s_data[15:8]} + 10'd111;
    wire [9:0] watermark_blue_mix = ({2'b00,s_data[7:0]} << 1) +
                                     {2'b00,s_data[7:0]} + 10'd179;
    wire [23:0] watermark_blended = {watermark_red_mix[9:2],
                                      watermark_green_mix[9:2],
                                      watermark_blue_mix[9:2]};

    // Keep the title and control status in one continuous card.
    wire panel = x >= 10'd301 && x < 10'd505 &&
                 y >= 9'd14 && y < 9'd52;
    wire current_video_text = x >= 10'd312 && x < 10'd368 &&
                              y >= 9'd18 && y < 9'd32;
    wire chapter_text = x >= 10'd376 && x < 10'd432 &&
                        y >= 9'd18 && y < 9'd32;
    wire autoplay_text = x >= 10'd376 && x < 10'd432 &&
                         y >= 9'd34 && y < 9'd48;
    wire autoplay_state_text = x >= 10'd448 && x < 10'd462 &&
                               y >= 9'd34 && y < 9'd48;
    wire [5:0] label_x = current_video_text ? x - 10'd312 : x - 10'd376;
    wire [1:0] label_character = label_x < 6'd14 ? 2'd0 :
                                 label_x < 6'd28 ? 2'd1 :
                                 label_x < 6'd42 ? 2'd2 : 2'd3;
    wire [3:0] label_local_x = label_x < 6'd14 ? label_x[3:0] :
                               label_x < 6'd28 ? label_x - 6'd14 :
                               label_x < 6'd42 ? label_x - 6'd28 :
                                                label_x - 6'd42;
    wire [4:0] current_label_glyph = 5'd19 + {3'd0,label_character};
    wire [4:0] status_label_glyph = 5'd23 + {3'd0,label_character};
    wire [4:0] selected_glyph = autoplay_state_text ?
                                (auto_play ? 5'd27 : 5'd28) :
                                (current_video_text ? current_label_glyph :
                                 chapter_glyph(scene_id,label_character));
    wire [3:0] glyph_y = (current_video_text || chapter_text) ?
                         y - 9'd18 : y - 9'd34;
    wire [4:0] glyph_id = autoplay_text ? status_label_glyph : selected_glyph;
    wire [5:0] glyph_x = current_video_text || chapter_text || autoplay_text ?
                         {2'b00,label_local_x} : x - 10'd448;
    wire [3:0] glyph_source_x = shrink_glyph_index(glyph_x[3:0]);
    wire [3:0] glyph_source_y = shrink_glyph_index(glyph_y);
    wire [15:0] glyph_bits = font_line(glyph_id,glyph_source_y);
    wire glyph_area = current_video_text || chapter_text || autoplay_text ||
                      autoplay_state_text;
    wire glyph_pixel = glyph_area && glyph_bits[15-glyph_source_x];

    // Four numbered scene chips make the current selection immediately clear.
    wire [5:0] marker_x = x - 10'd312;
    wire [1:0] marker_number = marker_x < 6'd14 ? 2'd0 :
                               marker_x < 6'd28 ? 2'd1 :
                               marker_x < 6'd42 ? 2'd2 : 2'd3;
    wire [3:0] marker_local_x = marker_number == 2'd0 ? marker_x[3:0] :
                                marker_number == 2'd1 ? marker_x - 6'd14 :
                                marker_number == 2'd2 ? marker_x - 6'd28 :
                                                       marker_x - 6'd42;
    wire marker_area = x >= 10'd312 && x < 10'd368 &&
                       marker_local_x < 4'd10 &&
                       y >= 9'd34 && y < 9'd46;
    wire [3:0] marker_glyph_row = y - 9'd36;
    wire [2:0] marker_glyph_col = marker_local_x[2:0] - 3'd2;
    wire [34:0] marker_glyph_bits = ascii_5x7({1'b0,marker_number} + 4'd1);
    wire marker_digit = marker_area && marker_local_x >= 2 &&
                        marker_local_x <= 6 && y >= 9'd36 && y < 9'd43 &&
                        marker_glyph_bits[(6-marker_glyph_row)*5 +
                                          (4-marker_glyph_col)];
    wire autoplay_colon = x >= 10'd436 && x < 10'd438 &&
                          ((y >= 9'd37 && y < 9'd40) ||
                           (y >= 9'd44 && y < 9'd47));


    // Compact elapsed/total time sits below the full-width progress bar.
    wire [5:0] elapsed_seconds = frame_index / 11'd10;
    wire [3:0] elapsed_tens = elapsed_seconds / 6'd10;
    wire [3:0] elapsed_ones = elapsed_seconds % 6'd10;
    wire [3:0] total_tens = (scene_id == 2'd1) ? 4'd3 : 4'd5;
    wire [3:0] total_ones = (scene_id == 2'd1) ? 4'd7 :
                            (scene_id == 2'd3) ? 4'd0 : 4'd2;
    wire [7:0] time_offset = x - 10'd284;
    wire [3:0] time_slot = time_offset < 8'd8 ? 4'd0 :
                           time_offset < 8'd16 ? 4'd1 :
                           time_offset < 8'd24 ? 4'd2 :
                           time_offset < 8'd32 ? 4'd3 :
                           time_offset < 8'd40 ? 4'd4 :
                           time_offset < 8'd48 ? 4'd5 :
                           time_offset < 8'd56 ? 4'd6 :
                           time_offset < 8'd64 ? 4'd7 : 4'd8;
    wire [7:0] time_slot_origin = {2'b00,time_slot,3'b000};
    wire [7:0] time_local_x = time_offset - time_slot_origin;
    wire [3:0] time_glyph = time_character(time_slot,elapsed_tens,
                                           elapsed_ones,total_tens,total_ones);
    wire [34:0] time_glyph_bits = ascii_5x7(time_glyph);
    wire [2:0] time_column = time_local_x[2:0];
    wire [2:0] time_row = y - 9'd458;
    wire time_text_area = x >= 10'd284 && x < 10'd356 &&
                          y >= 9'd458 && y < 9'd465;
    wire time_text_pixel = time_text_area && time_column < 5 &&
                           time_glyph_bits[(6-time_row)*5 +
                                           (4-time_column)];

    // Date and Beijing time sit below the scene title as text-only overlays.
    // A one-pixel dark shadow keeps the digits legible over bright artwork.
    wire [5:0] date_offset = x - 10'd24;
    wire [3:0] date_slot = date_offset / 6'd6;
    wire [2:0] date_column = date_offset % 6'd6;
    wire [2:0] date_row = y - 9'd62;
    wire [3:0] date_glyph = date_character(date_slot, year_thousands,
        year_hundreds, year_tens, year_ones, month_tens, month_ones,
        day_tens, day_ones);
    wire [34:0] date_glyph_bits = ascii_5x7(date_glyph);
    wire date_area = x >= 10'd24 && x < 10'd84 &&
                     y >= 9'd62 && y < 9'd69;
    wire date_text_pixel = date_area && date_column < 5 &&
                           date_glyph_bits[(6-date_row)*5 + (4-date_column)];
    wire [5:0] date_shadow_offset = x - 10'd25;
    wire [3:0] date_shadow_slot = date_shadow_offset / 6'd6;
    wire [2:0] date_shadow_column = date_shadow_offset % 6'd6;
    wire [2:0] date_shadow_row = y - 9'd63;
    wire [34:0] date_shadow_bits = ascii_5x7(date_character(
        date_shadow_slot, year_thousands, year_hundreds, year_tens,
        year_ones, month_tens, month_ones, day_tens, day_ones));
    wire date_shadow_pixel = x >= 10'd25 && x < 10'd85 &&
                             y >= 9'd63 && y < 9'd70 &&
                             date_shadow_column < 5 &&
                             date_shadow_bits[(6-date_shadow_row)*5 +
                                              (4-date_shadow_column)];

    wire [5:0] clock_offset = x - 10'd24;
    wire [3:0] clock_slot = clock_offset / 6'd6;
    wire [2:0] clock_column = clock_offset % 6'd6;
    wire [2:0] clock_row = y - 9'd72;
    wire [3:0] clock_glyph = clock_character(clock_slot, hour_tens,
        hour_ones, minute_tens, minute_ones, second_tens, second_ones);
    wire [34:0] clock_glyph_bits = ascii_5x7(clock_glyph);
    wire clock_area = x >= 10'd24 && x < 10'd72 &&
                      y >= 9'd72 && y < 9'd79;
    wire clock_text_pixel = clock_area && clock_column < 5 &&
                            clock_glyph_bits[(6-clock_row)*5 +
                                             (4-clock_column)];
    wire [5:0] clock_shadow_offset = x - 10'd25;
    wire [3:0] clock_shadow_slot = clock_shadow_offset / 6'd6;
    wire [2:0] clock_shadow_column = clock_shadow_offset % 6'd6;
    wire [2:0] clock_shadow_row = y - 9'd73;
    wire [34:0] clock_shadow_bits = ascii_5x7(clock_character(
        clock_shadow_slot, hour_tens, hour_ones, minute_tens, minute_ones,
        second_tens, second_ones));
    wire clock_shadow_pixel = x >= 10'd25 && x < 10'd73 &&
                              y >= 9'd73 && y < 9'd80 &&
                              clock_shadow_column < 5 &&
                              clock_shadow_bits[(6-clock_shadow_row)*5 +
                                                (4-clock_shadow_column)];

    // Long, shallow progress capsule directly below the subtitle.
    wire progress_outer =
        (((y == 9'd443 || y == 9'd454) && x >= 10'd38 && x < 10'd602) ||
         ((y == 9'd444 || y == 9'd453) && x >= 10'd35 && x < 10'd605) ||
         (y >= 9'd445 && y <= 9'd452 && x >= 10'd32 && x < 10'd608));
    wire progress_region =
        (((y == 9'd445 || y == 9'd452) && x >= 10'd38 && x < 10'd602) ||
         ((y == 9'd446 || y == 9'd451) && x >= 10'd35 && x < 10'd605) ||
         (y >= 9'd447 && y <= 9'd450 && x >= 10'd32 && x < 10'd608));
    wire [9:0] progress_x = x - 10'd32;
    wire [4:0] progress_segment = progress_x / 10'd18;
    wire [10:0] progress_threshold = segment_threshold(scene_id,progress_segment);
    wire [10:0] frame_position = frame_index + 11'd1;
    wire progress_lit = frame_position >= progress_threshold;

    // KEY4 pause indicator: retain the dimmed patch and draw a custom cyan
    // right-pointing triangle with a dark outline.
    wire pause_box = !playing && x >= 10'd284 && x < 10'd356 &&
                     y >= 9'd204 && y < 9'd276;
    wire [5:0] pause_dy = y >= 9'd240 ? y - 9'd240 : 9'd240 - y;
    wire [5:0] triangle_span = (6'd23 - pause_dy) << 1;
    wire [9:0] triangle_right_x = 10'd297 + {4'd0,triangle_span};
    wire [10:0] triangle_inner_x = {1'b0,x} + 11'd5;
    wire pause_triangle_outer = pause_box && pause_dy <= 6'd23 &&
                                x >= 10'd296 && x <= triangle_right_x;
    wire pause_triangle_inner = pause_box && pause_dy <= 6'd19 &&
                                x >= 10'd302 &&
                                triangle_inner_x < {1'b0,triangle_right_x};

    always @* begin
        m_data = s_data;
        if (s_valid) begin
            if (pause_box)
                m_data = {1'b0,s_data[23:17],1'b0,s_data[15:9],1'b0,s_data[7:1]};
            if (watermark_pixel)
                m_data = watermark_blended;

            if (progress_outer)
                m_data = 24'hB7D2E2;
            if (progress_region) begin
                m_data = progress_lit ? 24'h35D6C5 : 24'h405667;
            end
            if (time_text_pixel)
                m_data = (time_slot < 4'd4) ? 24'hF3BA32 : 24'hE0E7E9;

            if (date_shadow_pixel || clock_shadow_pixel)
                m_data = 24'h10212A;
            if (date_text_pixel)
                m_data = 24'h35D6C5;
            if (clock_text_pixel)
                m_data = 24'hF4F7FA;

            if (panel) begin
                m_data = 24'h11283A;
                if (x == 10'd301 || x == 10'd504 ||
                    y == 9'd14 || y == 9'd51)
                    m_data = 24'h426278;
                if (x >= 10'd302 && x < 10'd305)
                    m_data = 24'h35D6C5;
            end
            if (marker_area)
                m_data = (marker_number == scene_id) ? 24'h278F89 : 24'h405667;
            if (marker_digit)
                m_data = 24'hF4F7FA;
            if (autoplay_colon)
                m_data = 24'hF4F7FA;
            if (glyph_pixel)
                m_data = autoplay_state_text ?
                         (auto_play ? 24'h35D6C5 : 24'hFFB45C) : 24'hF4F7FA;
            if (audio_meter_panel) begin
                m_data = 24'h102B3C;
            end
            if (audio_meter_fill)
                m_data = 24'h35D6C5;
            if (audio_meter_outline)
                m_data = 24'h426278;
            if (pause_triangle_outer)
                m_data = 24'h08253A;
            if (pause_triangle_inner)
                m_data = 24'h28D7F2;
        end
    end

    function [143:0] watermark_row;
        input [5:0] row_index;
        begin
            case (row_index)
            6'd0: watermark_row = 144'h0303FFFFF1FD800000000000000000000000;
            6'd1: watermark_row = 144'h0703FFFFF8FD800000000000000000000000;
            6'd2: watermark_row = 144'h0F03FFFFE86D800000000000000000000000;
            6'd3: watermark_row = 144'h0F07FFFFF86F800000000000000000000000;
            6'd4: watermark_row = 144'h1F07FFFEF86F800000000000000000000000;
            6'd5: watermark_row = 144'h3F07FFFDF86D800000000000000000000000;
            6'd6: watermark_row = 144'h3F07F001F86D800000000000000000000000;
            6'd7: watermark_row = 144'h3F07F001F84D800000000000000000000000;
            6'd8: watermark_row = 144'h7F07F001F800000000000000000000000000;
            6'd9: watermark_row = 144'h7F07F001F000000000000000000000000000;
            6'd10: watermark_row = 144'h7F07F001F000000000000000000000000000;
            6'd11: watermark_row = 144'h7F07F001A0001C1C1CFC001C1E1E1E000000;
            6'd12: watermark_row = 144'h7F07F00340003E3F3DFE003F3F3F3F000000;
            6'd13: watermark_row = 144'h7F07E006B00063637D8C0073737373000000;
            6'd14: watermark_row = 144'h7F07E001600063631C0C0063636361000000;
            6'd15: watermark_row = 144'h7F07E00250000363181C000363037E000000;
            6'd16: watermark_row = 144'h7F07E004B00006631818FF0763077F000000;
            6'd17: watermark_row = 144'h7F0FE00160000CE31838FE0E630E63000000;
            6'd18: watermark_row = 144'h7F0FE002D00018C31830001C631C63000000;
            6'd19: watermark_row = 144'h7F0FE005B00030C318300038633863000000;
            6'd20: watermark_row = 144'h7F0FE003700060E618600070677067000000;
            6'd21: watermark_row = 144'h7F0FE006F000FE7E1860007F7E7F7E000000;
            6'd22: watermark_row = 144'h3E07C001B000FE381840007E187F1C000000;
            6'd23: watermark_row = 144'h1C0700016000000000000000000000000000;
            6'd24: watermark_row = 144'h040200004000000000000000000000000000;
            6'd25: watermark_row = 144'h1C060000C000000000000000000000000000;
            6'd26: watermark_row = 144'h3E0D0001E000000000000000000000000000;
            6'd27: watermark_row = 144'h7E090007E000740020006188121C43018010;
            6'd28: watermark_row = 144'hFE0A4007E000FFF8F870F1BFF37E630FFBFF;
            6'd29: watermark_row = 144'hFE0C8007E000C19DDC719FBFC366630663FF;
            6'd30: watermark_row = 144'hFC09600FE000C18D8CF19FB58166231FFBFF;
            6'd31: watermark_row = 144'hFC02C00FE001818D80F1F7BFF746FFDFF9FE;
            6'd32: watermark_row = 144'hF805800FE001FD9D7CD9F7BFF77C6FCFF3FF;
            6'd33: watermark_row = 144'hFC0B600FE001FDFB7D9997BF637E631833FF;
            6'd34: watermark_row = 144'hEC06C00FE00181E30DF9B7BF636C671FF3FF;
            6'd35: watermark_row = 144'hEC0D800FE00181830DF9B73F6338660FF1B6;
            6'd36: watermark_row = 144'hBC1BC00FE00181831B19A17F6338760CC1FE;
            6'd37: watermark_row = 144'h3C07800FE0018301FB0DFF7F63EC6618F9FE;
            6'd38: watermark_row = 144'hFC0D800FE0018100F20DF71260C402107B86;
            6'd39: watermark_row = 144'hFC1F000FE000000000000000000000000000;
            6'd40: watermark_row = 144'hFC17000FE000000000000000000000000000;
            6'd41: watermark_row = 144'hFC17000FE001FE18087F9000000000000000;
            6'd42: watermark_row = 144'hFC17000FC001FEFF187FBFC0000000000000;
            6'd43: watermark_row = 144'hFC17000FC001FEFF187FB68BFCBFFDEDF7F6;
            6'd44: watermark_row = 144'hFC177FFFC00136FEFF7FBF8B5DFFAFFFA7FE;
            6'd45: watermark_row = 144'hFC17FFFFC003FEFE7F7FBF8B41522D650536;
            6'd46: watermark_row = 144'h3C15FFFFC0033EFE187FBF80000000000000;
            6'd47: watermark_row = 144'h3C15FFFFC003FEFE187FBF9FFFFEFFFBFFFE;
            6'd48: watermark_row = 144'h1C1FFFFFC003FEFC18DF841FFFFEFFFFFFBE;
            6'd49: watermark_row = 144'h180FFFFF8003FEFE18C3841FEFFEFFFBEDBE;
            default: watermark_row = 144'd0;
            endcase
        end
    endfunction

    function [3:0] shrink_glyph_index;
        input [3:0] display_index;
        begin
            case (display_index)
                0:shrink_glyph_index=0;   1:shrink_glyph_index=1;
                2:shrink_glyph_index=2;   3:shrink_glyph_index=3;
                4:shrink_glyph_index=4;   5:shrink_glyph_index=5;
                6:shrink_glyph_index=6;   7:shrink_glyph_index=8;
                8:shrink_glyph_index=9;   9:shrink_glyph_index=10;
                10:shrink_glyph_index=11; 11:shrink_glyph_index=12;
                12:shrink_glyph_index=13; default:shrink_glyph_index=15;
            endcase
        end
    endfunction

    function [4:0] chapter_glyph;
        input [1:0] scene;
        input [1:0] column;
        begin
            case (scene)
                2'd0: case (column)
                    2'd0: chapter_glyph=5'd4;
                    2'd1: chapter_glyph=5'd5;
                    2'd2: chapter_glyph=5'd6;
                    default: chapter_glyph=5'd7;
                endcase
                2'd1: case (column)
                    2'd0: chapter_glyph=5'd5;
                    2'd1: chapter_glyph=5'd8;
                    2'd2: chapter_glyph=5'd9;
                    default: chapter_glyph=5'd10;
                endcase
                2'd2: case (column)
                    2'd0: chapter_glyph=5'd11;
                    2'd1: chapter_glyph=5'd12;
                    2'd2: chapter_glyph=5'd13;
                    default: chapter_glyph=5'd14;
                endcase
                default: case (column)
                    2'd0: chapter_glyph=5'd15;
                    2'd1: chapter_glyph=5'd16;
                    2'd2: chapter_glyph=5'd17;
                    default: chapter_glyph=5'd18;
                endcase
            endcase
        end
    endfunction

    function [10:0] segment_threshold;
        input [1:0] scene;
        input [4:0] segment;
        begin
            case (scene)
                2'd1: case (segment)
                    0:segment_threshold=12;  1:segment_threshold=24;
                    2:segment_threshold=36;  3:segment_threshold=47;
                    4:segment_threshold=59;  5:segment_threshold=71;
                    6:segment_threshold=83;  7:segment_threshold=94;
                    8:segment_threshold=106; 9:segment_threshold=118;
                    10:segment_threshold=130;11:segment_threshold=141;
                    12:segment_threshold=153;13:segment_threshold=165;
                    14:segment_threshold=177;15:segment_threshold=188;
                    16:segment_threshold=200;17:segment_threshold=212;
                    18:segment_threshold=224;19:segment_threshold=235;
                    20:segment_threshold=247;21:segment_threshold=259;
                    22:segment_threshold=271;23:segment_threshold=282;
                    24:segment_threshold=294;25:segment_threshold=306;
                    26:segment_threshold=318;27:segment_threshold=329;
                    28:segment_threshold=341;29:segment_threshold=353;
                    30:segment_threshold=365;default:segment_threshold=376;
                endcase
                2'd3: case (segment)
                    0:segment_threshold=16;  1:segment_threshold=32;
                    2:segment_threshold=48;  3:segment_threshold=63;
                    4:segment_threshold=79;  5:segment_threshold=95;
                    6:segment_threshold=111; 7:segment_threshold=126;
                    8:segment_threshold=142; 9:segment_threshold=158;
                    10:segment_threshold=173;11:segment_threshold=189;
                    12:segment_threshold=205;13:segment_threshold=221;
                    14:segment_threshold=236;15:segment_threshold=252;
                    16:segment_threshold=268;17:segment_threshold=283;
                    18:segment_threshold=299;19:segment_threshold=315;
                    20:segment_threshold=331;21:segment_threshold=346;
                    22:segment_threshold=362;23:segment_threshold=378;
                    24:segment_threshold=393;25:segment_threshold=409;
                    26:segment_threshold=425;27:segment_threshold=441;
                    28:segment_threshold=456;29:segment_threshold=472;
                    30:segment_threshold=488;default:segment_threshold=503;
                endcase
                default: case (segment)
                    0:segment_threshold=17;  1:segment_threshold=33;
                    2:segment_threshold=50;  3:segment_threshold=66;
                    4:segment_threshold=82;  5:segment_threshold=99;
                    6:segment_threshold=115; 7:segment_threshold=131;
                    8:segment_threshold=148; 9:segment_threshold=164;
                    10:segment_threshold=180;11:segment_threshold=197;
                    12:segment_threshold=213;13:segment_threshold=229;
                    14:segment_threshold=246;15:segment_threshold=262;
                    16:segment_threshold=278;17:segment_threshold=295;
                    18:segment_threshold=311;19:segment_threshold=327;
                    20:segment_threshold=344;21:segment_threshold=360;
                    22:segment_threshold=376;23:segment_threshold=393;
                    24:segment_threshold=409;25:segment_threshold=425;
                    26:segment_threshold=442;27:segment_threshold=458;
                    28:segment_threshold=474;29:segment_threshold=491;
                    30:segment_threshold=507;default:segment_threshold=523;
                endcase
            endcase
        end
    endfunction

    function [3:0] date_character;
        input [3:0] slot;
        input [3:0] year_thousands_value;
        input [3:0] year_hundreds_value;
        input [3:0] year_tens_value;
        input [3:0] year_ones_value;
        input [3:0] month_tens_value;
        input [3:0] month_ones_value;
        input [3:0] day_tens_value;
        input [3:0] day_ones_value;
        begin
            case (slot)
                4'd0: date_character = year_thousands_value;
                4'd1: date_character = year_hundreds_value;
                4'd2: date_character = year_tens_value;
                4'd3: date_character = year_ones_value;
                4'd4: date_character = 4'd12;
                4'd5: date_character = month_tens_value;
                4'd6: date_character = month_ones_value;
                4'd7: date_character = 4'd12;
                4'd8: date_character = day_tens_value;
                default: date_character = day_ones_value;
            endcase
        end
    endfunction

    function [3:0] clock_character;
        input [3:0] slot;
        input [3:0] hour_tens_value;
        input [3:0] hour_ones_value;
        input [3:0] minute_tens_value;
        input [3:0] minute_ones_value;
        input [3:0] second_tens_value;
        input [3:0] second_ones_value;
        begin
            case (slot)
                4'd0: clock_character = hour_tens_value;
                4'd1: clock_character = hour_ones_value;
                4'd2: clock_character = 4'd10;
                4'd3: clock_character = minute_tens_value;
                4'd4: clock_character = minute_ones_value;
                4'd5: clock_character = 4'd10;
                4'd6: clock_character = second_tens_value;
                default: clock_character = second_ones_value;
            endcase
        end
    endfunction

    function leap_year_bcd;
        input [3:0] thousands_value;
        input [3:0] hundreds_value;
        input [3:0] tens_value;
        input [3:0] ones_value;
        reg divisible_by_four;
        reg divisible_by_four_hundreds;
        reg is_century;
        begin
            divisible_by_four = tens_value[0] ?
                (ones_value == 4'd2 || ones_value == 4'd6) :
                (ones_value == 4'd0 || ones_value == 4'd4 || ones_value == 4'd8);
            divisible_by_four_hundreds = thousands_value[0] ?
                (hundreds_value == 4'd2 || hundreds_value == 4'd6) :
                (hundreds_value == 4'd0 || hundreds_value == 4'd4 || hundreds_value == 4'd8);
            is_century = tens_value == 4'd0 && ones_value == 4'd0;
            leap_year_bcd = divisible_by_four &&
                            (!is_century || divisible_by_four_hundreds);
        end
    endfunction

    function [3:0] time_character;
        input [3:0] slot;
        input [3:0] tens;
        input [3:0] ones;
        input [3:0] total_tens_value;
        input [3:0] total_ones_value;
        begin
            case (slot)
                4'd0: time_character=4'd0;
                4'd1: time_character=4'd10;
                4'd2: time_character=tens;
                4'd3: time_character=ones;
                4'd4: time_character=4'd11;
                4'd5: time_character=4'd0;
                4'd6: time_character=4'd10;
                4'd7: time_character=total_tens_value;
                default: time_character=total_ones_value;
            endcase
        end
    endfunction


    function [34:0] ascii_5x7;
        input [3:0] character;
        begin
            case (character)
                4'd0: ascii_5x7=35'h3A33AE62E;
                4'd1: ascii_5x7=35'h11842108E;
                4'd2: ascii_5x7=35'h3A211111F;
                4'd3: ascii_5x7=35'h78217043E;
                4'd4: ascii_5x7=35'h08CA97C42;
                4'd5: ascii_5x7=35'h7E10F043E;
                4'd6: ascii_5x7=35'h3A10F462E;
                4'd7: ascii_5x7=35'h7C2222108;
                4'd8: ascii_5x7=35'h3A317462E;
                4'd9: ascii_5x7=35'h3A317842E;
                4'd10: ascii_5x7=35'h008401080;
                4'd11: ascii_5x7=35'h042222210;
                4'd12: ascii_5x7=35'h0000F8000;
                default: ascii_5x7=35'd0;
            endcase
        end
    endfunction

    function [15:0] font_line;
        input [4:0] glyph;
        input [3:0] row;
        reg [255:0] bitmap;
        begin
            case (glyph)
            5'd0: bitmap=256'h00000000000000000000FFFF0000000000000000000000000000000000000000;
            5'd1: bitmap=256'h0C0071FE11061106FDFE310630003BFE7C205420D1FE1020102013FF10000000;
            5'd2: bitmap=256'h010000807FFE400242020600FFFF0C1008301820076001C003701C1C70040000;
            5'd3: bitmap=256'h0100018000800CC00C482C0C2C046C064C024C0A4C080C080C1807F000000000;
            5'd4: bitmap=256'h2198719877FE37FE03FC7198F19C7FFF3FFF339837183318FB18FFFF4FFE0000;
            5'd5: bitmap=256'h183038301C38FF3FFF3F0630663067FE6FFE7DC67DC60DC67FFEFFFEF1FE0086;
            5'd6: bitmap=256'h03304338E73877FF3FFE0F300FFF2FFE363077FE77FF6730E738E7FF47FE0600;
            5'd7: bitmap=256'h0E001FFC3FFC7C3CEFF807F07FFFFC3E3FFC3FFC318C3FFC3FFC3FFC3FFC300C;
            5'd8: bitmap=256'h030007000E101C38381C7FFE7FFF00023FFC3FFC300C300C381C3FFC3FFC300C;
            5'd9: bitmap=256'h30FC39FC3C0C3FFF7FFF7D80FDFEFDFEFFB07FFF3FFF3C783CFC33CE33872000;
            5'd10:bitmap=256'h070006007FFE7FFE1C0019C039C03FFC3FFC01C001C0FFFFFFFF01C001C001C0;
            5'd11:bitmap=256'h30C031FC33FCFB9CFF3C33FE33FE7F66FB66F7FF3FFF30F030F877DFF78E6600;
            5'd12:bitmap=256'h7FFE7FFE0180FFFF7FFA1DBE7DBE1DBB3FFFFFFE4FF01DB8799FF18F41820000;
            5'd13:bitmap=256'h38C838DC38EEFEE6FFFF3BFE3BE43EEEFE7EFC7C38F839F73BF77B3EF81E700C;
            5'd14:bitmap=256'h00607EFE7FFE6FCE7FCE7EFC1CFC5BFF5F865DFE59FE59C6FFFEFFFEF9FE0000;
            5'd15:bitmap=256'h67FC77FE766E37FE06EE67FEF3FCF7FE37FE367E37FE37FE7E1EFFFF4FFE0000;
            5'd16:bitmap=256'h0180FFFF7FFE3FFC318C3FFC3FFC7FFC7FFEFFFFFFFF3FFE7FFE0F840F800600;
            5'd17:bitmap=256'h0198019C039CFFFFFFFF218471CE39DC1BF80FF03FB8F99E618F0F860F800E00;
            5'd18:bitmap=256'h00607E607FFE67FE67FE7E667E6666667E667E6666E666E67FCEFFFEF9BC0000;
            5'd19:bitmap=256'h0180318C399E1DBC099003C07FFE7FFE000E3FFE7FFE000E000E7FFE7FFE0006;
            5'd20:bitmap=256'h1C381C78FFFFFFFF3F067FB673B67FB67FB663B67FB67FB6738667BE6F3E6610;
            5'd21:bitmap=256'h300073FE3BFE7B06FF767B763B663B667F66FFFEF3F030F231F737BF373E3200;
            5'd22:bitmap=256'h0CFF6DFF6F386E306CFEFFFEFFD618DE7FDE7FF6DEF6CEFA3C7CF9EFE1C70000;
            5'd23:bitmap=256'h00C061C077FE3FFE03300770F7FEF7FE307037FE3FFF30707830FFFF6FFF007C;
            5'd24:bitmap=256'h30FE37FE3366FB6CFFFF37FF31FC7F6FFFFEF3FE336633FE33FEF3FEF3FE0006;
            5'd25:bitmap=256'h0C740C7E4C766C767DFE7FFF0DFE1C703C70FCF8FCD84DDC0F8E0F8F0D060000;
            5'd26:bitmap=256'h018003807FFE7FFE07E01E787FBFF9DE61820F8C7DCE6C9FEC3FEFF80FF00000;
            5'd27:bitmap=256'h7FFE7FFE0C380C380C380C38FFFFFFFF0C381C381C383C387838703820100000;
            5'd28:bitmap=256'h08301C380E707FFE7FFE03C00180FFFFFFFF03C003C00FF01E787C3F781E2000;
            default: bitmap=0;
            endcase
            font_line = bitmap >> {4'd0,(4'd15-row),4'b0000};
        end
    endfunction
endmodule
