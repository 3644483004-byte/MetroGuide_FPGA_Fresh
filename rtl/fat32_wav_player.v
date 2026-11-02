// FAT32 root-directory WAV reader for the HDMI 48 kHz stereo audio path.
// Metadata and PCM bytes are consumed as the SD controller streams each sector;
// no 512-byte register-array/multiplexer is built in the FPGA fabric.
// Card contract: FAT32, 512-byte sectors, 1.WAV..4.WAV in the root directory,
// canonical 48 kHz / stereo / signed 16-bit PCM WAVs with a 44-byte header.
module fat32_wav_player #(
    parameter integer CLK_HZ   = 25_000_000,
    parameter integer AUDIO_HZ = 48_000
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [1:0]  scene_id,
    input  wire        restart_track,
    input  wire        playing,
    output reg         audio_valid,
    output reg  [23:0] audio_left,
    output reg  [23:0] audio_right,
    output reg         clip_end,
    output reg         card_ready,
    output reg         file_error,
    output wire [31:0] audio_total_samples,
    output wire        sd_ncs,
    output wire        sd_dclk,
    output wire        sd_mosi,
    input  wire        sd_miso
);

    localparam [3:0] ST_WAIT_INIT      = 4'd0;
    localparam [3:0] ST_WAIT_SECTOR    = 4'd1;
    localparam [3:0] ST_PARSE_BPB      = 4'd2;
    localparam [3:0] ST_DIR_START      = 4'd3;
    localparam [3:0] ST_SCAN_DIR       = 4'd4;
    localparam [3:0] ST_PARSE_ROOT_FAT = 4'd5;
    localparam [3:0] ST_BEGIN_FILE     = 4'd6;
    localparam [3:0] ST_CHECK_WAV      = 4'd7;
    localparam [3:0] ST_WAIT_SPACE     = 4'd8;
    localparam [3:0] ST_PARSE_AUDIO_FAT= 4'd9;
    localparam [3:0] ST_WAIT_AUDIO_END = 4'd10;
    localparam [3:0] ST_ERROR          = 4'd11;

    localparam [2:0] RK_MBR       = 3'd0;
    localparam [2:0] RK_VBR       = 3'd1;
    localparam [2:0] RK_ROOT      = 3'd2;
    localparam [2:0] RK_ROOT_FAT  = 3'd3;
    localparam [2:0] RK_WAV       = 3'd4;
    localparam [2:0] RK_AUDIO_FAT = 3'd5;

    localparam [9:0] FIFO_CAPACITY = 10'd512;

    reg [3:0] state;
    reg [2:0] read_kind;
    reg [31:0] read_lba;
    reg        sd_sec_read;
    wire [7:0] sd_read_data;
    wire       sd_read_valid;
    wire       sd_read_end;
    wire       sd_init_done;
    reg        sd_read_busy;
    reg [8:0]  sector_byte_index;

    reg [1:0] scene_id_q;
    reg       restart_pending;
    wire      scene_changed = (scene_id != scene_id_q) || restart_track;
    reg       filesystem_ready;
    reg       wave_valid;
    reg       wav_header_pending;
    reg       wav_header_bad;
    reg       source_done;

    reg [31:0] volume_start_lba;
    reg [31:0] fat_start_lba;
    reg [31:0] first_data_lba;
    reg [31:0] root_cluster;
    reg [31:0] root_walk_cluster;
    reg [31:0] file_cluster;
    reg [31:0] file_size_bytes;
    reg [31:0] audio_bytes_remaining;
    reg [31:0] wav_data_size;
    reg [31:0] fat_word;
    reg [8:0]  fat_entry_offset;
    reg [7:0]  sectors_per_cluster;
    reg [2:0]  sectors_per_cluster_shift;
    reg [7:0]  number_of_fats;
    reg [31:0] fat_size_sectors;
    reg [7:0]  directory_sector_index;
    reg [7:0]  file_sector_index;

    // MBR and BPB fields are picked off as their byte positions pass the SD
    // reader. This avoids a large asynchronous read mux over a sector buffer.
    reg        mbr_jump_ok;
    reg        mbr_sig_55;
    reg        mbr_sig_aa;
    reg [31:0] mbr_partition_lba;
    reg [7:0]  bpb_bps_lo;
    reg [7:0]  bpb_bps_hi;
    reg [7:0]  bpb_spc;
    reg [15:0] bpb_reserved;
    reg [7:0]  bpb_num_fats;
    reg [15:0] bpb_fatsz16;
    reg [31:0] bpb_fatsz32;
    reg [31:0] bpb_root_cluster;

    // The directory parser examines 8.3 names and captures only the matching
    // entry's cluster and file size.
    reg        dir_name_match;
    reg        dir_attr_ok;
    reg        dir_terminator_seen;
    reg        dir_match_found;
    reg [15:0] dir_cluster_hi;
    reg [15:0] dir_cluster_lo;
    reg [31:0] dir_size;
    reg [31:0] dir_found_cluster;
    reg [31:0] dir_found_size;
    wire [7:0] scene_first_char = (scene_id_q == 2'd0) ? 8'h31 :
                                  (scene_id_q == 2'd1) ? 8'h32 :
                                  (scene_id_q == 2'd2) ? 8'h33 : 8'h34;
    // The accepted WAV format is 16-bit stereo: four bytes per audio sample.
    assign audio_total_samples = wav_data_size >> 2;
    wire [31:0] next_fat_cluster = fat_word & 32'h0FFFFFFF;
    wire [31:0] expected_riff_size = file_size_bytes - 32'd8;
    wire root_sector_start = sd_sec_read && read_kind == RK_ROOT;
    wire fat_sector_start = sd_sec_read &&
                            (read_kind == RK_ROOT_FAT || read_kind == RK_AUDIO_FAT);
    wire wav_header_start = sd_sec_read && read_kind == RK_WAV && wav_header_pending;

    function [31:0] cluster_to_lba;
        input [31:0] cluster;
        input [7:0]  sector_in_cluster;
        begin
            cluster_to_lba = first_data_lba +
                             ((cluster - 32'd2) << sectors_per_cluster_shift) +
                             sector_in_cluster;
        end
    endfunction

    sd_card_top #(
        .SPI_LOW_SPEED_DIV  (248),
        .SPI_HIGH_SPEED_DIV (0)
    ) u_sd_card (
        .clk                   (clk),
        .rst                   (rst),
        .SD_nCS                (sd_ncs),
        .SD_DCLK               (sd_dclk),
        .SD_MOSI               (sd_mosi),
        .SD_MISO               (sd_miso),
        .sd_init_done          (sd_init_done),
        .sd_sec_read           (sd_sec_read),
        .sd_sec_read_addr      (read_lba),
        .sd_sec_read_data      (sd_read_data),
        .sd_sec_read_data_valid(sd_read_valid),
        .sd_sec_read_end       (sd_read_end),
        .sd_sec_write          (1'b0),
        .sd_sec_write_addr     (32'd0),
        .sd_sec_write_data     (8'd0),
        .sd_sec_write_data_req (),
        .sd_sec_write_end      ()
    );

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sector_byte_index <= 9'd0;
            sd_read_busy      <= 1'b0;
        end else begin
            if (sd_sec_read)
                sector_byte_index <= 9'd0;
            else if (sd_read_valid && sector_byte_index != 9'd511)
                sector_byte_index <= sector_byte_index + 1'b1;

            if (sd_sec_read)
                sd_read_busy <= 1'b1;
            if (sd_read_end)
                sd_read_busy <= 1'b0;
        end
    end

    // Stream sector metadata directly into small registers.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mbr_jump_ok          <= 1'b0;
            mbr_sig_55           <= 1'b0;
            mbr_sig_aa           <= 1'b0;
            mbr_partition_lba    <= 32'd0;
            bpb_bps_lo           <= 8'd0;
            bpb_bps_hi           <= 8'd0;
            bpb_spc              <= 8'd0;
            bpb_reserved         <= 16'd0;
            bpb_num_fats         <= 8'd0;
            bpb_fatsz16          <= 16'd0;
            bpb_fatsz32          <= 32'd0;
            bpb_root_cluster     <= 32'd0;
            dir_name_match      <= 1'b0;
            dir_attr_ok         <= 1'b0;
            dir_terminator_seen <= 1'b0;
            dir_match_found     <= 1'b0;
            dir_cluster_hi      <= 16'd0;
            dir_cluster_lo      <= 16'd0;
            dir_size            <= 32'd0;
            dir_found_cluster   <= 32'd0;
            dir_found_size      <= 32'd0;
            fat_word            <= 32'd0;
            wav_header_bad      <= 1'b0;
            wav_data_size       <= 32'd0;
        end else if (root_sector_start) begin
            dir_name_match      <= 1'b0;
            dir_attr_ok         <= 1'b0;
            dir_terminator_seen <= 1'b0;
            dir_match_found     <= 1'b0;
        end else if (fat_sector_start) begin
            fat_word <= 32'd0;
        end else if (wav_header_start) begin
            wav_header_bad <= 1'b0;
            wav_data_size  <= 32'd0;
        end else if (sd_read_valid) begin
            if (read_kind == RK_MBR || read_kind == RK_VBR) begin
                case (sector_byte_index)
                    9'd11: bpb_bps_lo <= sd_read_data;
                    9'd12: bpb_bps_hi <= sd_read_data;
                    9'd13: bpb_spc <= sd_read_data;
                    9'd14: bpb_reserved[7:0] <= sd_read_data;
                    9'd15: bpb_reserved[15:8] <= sd_read_data;
                    9'd16: bpb_num_fats <= sd_read_data;
                    9'd22: bpb_fatsz16[7:0] <= sd_read_data;
                    9'd23: bpb_fatsz16[15:8] <= sd_read_data;
                    9'd36: bpb_fatsz32[7:0] <= sd_read_data;
                    9'd37: bpb_fatsz32[15:8] <= sd_read_data;
                    9'd38: bpb_fatsz32[23:16] <= sd_read_data;
                    9'd39: bpb_fatsz32[31:24] <= sd_read_data;
                    9'd44: bpb_root_cluster[7:0] <= sd_read_data;
                    9'd45: bpb_root_cluster[15:8] <= sd_read_data;
                    9'd46: bpb_root_cluster[23:16] <= sd_read_data;
                    9'd47: bpb_root_cluster[31:24] <= sd_read_data;
                    default: begin end
                endcase
            end

            if (read_kind == RK_MBR) begin
                case (sector_byte_index)
                    9'd0: mbr_jump_ok <= (sd_read_data == 8'hEB) || (sd_read_data == 8'hE9);
                    9'd454: mbr_partition_lba[7:0] <= sd_read_data;
                    9'd455: mbr_partition_lba[15:8] <= sd_read_data;
                    9'd456: mbr_partition_lba[23:16] <= sd_read_data;
                    9'd457: mbr_partition_lba[31:24] <= sd_read_data;
                    9'd510: mbr_sig_55 <= (sd_read_data == 8'h55);
                    9'd511: mbr_sig_aa <= (sd_read_data == 8'hAA);
                    default: begin end
                endcase
            end

            if (read_kind == RK_ROOT) begin
                case (sector_byte_index[4:0])
                    5'd0: begin
                        dir_name_match <= (sd_read_data == scene_first_char);
                        if (sd_read_data == 8'h00)
                            dir_terminator_seen <= 1'b1;
                    end
                    5'd1, 5'd2, 5'd3, 5'd4, 5'd5, 5'd6, 5'd7:
                        if (sd_read_data != 8'h20)
                            dir_name_match <= 1'b0;
                    5'd8: if (sd_read_data != 8'h57) dir_name_match <= 1'b0;
                    5'd9: if (sd_read_data != 8'h41) dir_name_match <= 1'b0;
                    5'd10: if (sd_read_data != 8'h56) dir_name_match <= 1'b0;
                    5'd11: dir_attr_ok <= (sd_read_data != 8'h0F) && !sd_read_data[4];
                    5'd20: dir_cluster_hi[7:0] <= sd_read_data;
                    5'd21: dir_cluster_hi[15:8] <= sd_read_data;
                    5'd26: dir_cluster_lo[7:0] <= sd_read_data;
                    5'd27: dir_cluster_lo[15:8] <= sd_read_data;
                    5'd28: dir_size[7:0] <= sd_read_data;
                    5'd29: dir_size[15:8] <= sd_read_data;
                    5'd30: dir_size[23:16] <= sd_read_data;
                    5'd31: begin
                        dir_size[31:24] <= sd_read_data;
                        if (dir_name_match && dir_attr_ok) begin
                            dir_match_found   <= 1'b1;
                            dir_found_cluster <= {dir_cluster_hi, dir_cluster_lo};
                            dir_found_size    <= {sd_read_data, dir_size[23:0]};
                        end
                    end
                    default: begin end
                endcase
            end

            if (read_kind == RK_ROOT_FAT || read_kind == RK_AUDIO_FAT) begin
                if (sector_byte_index == fat_entry_offset)
                    fat_word[7:0] <= sd_read_data;
                if (sector_byte_index == fat_entry_offset + 9'd1)
                    fat_word[15:8] <= sd_read_data;
                if (sector_byte_index == fat_entry_offset + 9'd2)
                    fat_word[23:16] <= sd_read_data;
                if (sector_byte_index == fat_entry_offset + 9'd3)
                    fat_word[31:24] <= sd_read_data;
            end

            if (read_kind == RK_WAV && wav_header_pending) begin
                case (sector_byte_index)
                    9'd0:  if (sd_read_data != 8'h52) wav_header_bad <= 1'b1;
                    9'd1:  if (sd_read_data != 8'h49) wav_header_bad <= 1'b1;
                    9'd2:  if (sd_read_data != 8'h46) wav_header_bad <= 1'b1;
                    9'd3:  if (sd_read_data != 8'h46) wav_header_bad <= 1'b1;
                    9'd4:  if (sd_read_data != expected_riff_size[7:0])   wav_header_bad <= 1'b1;
                    9'd5:  if (sd_read_data != expected_riff_size[15:8])  wav_header_bad <= 1'b1;
                    9'd6:  if (sd_read_data != expected_riff_size[23:16]) wav_header_bad <= 1'b1;
                    9'd7:  if (sd_read_data != expected_riff_size[31:24]) wav_header_bad <= 1'b1;
                    9'd8:  if (sd_read_data != 8'h57) wav_header_bad <= 1'b1;
                    9'd9:  if (sd_read_data != 8'h41) wav_header_bad <= 1'b1;
                    9'd10: if (sd_read_data != 8'h56) wav_header_bad <= 1'b1;
                    9'd11: if (sd_read_data != 8'h45) wav_header_bad <= 1'b1;
                    9'd12: if (sd_read_data != 8'h66) wav_header_bad <= 1'b1;
                    9'd13: if (sd_read_data != 8'h6D) wav_header_bad <= 1'b1;
                    9'd14: if (sd_read_data != 8'h74) wav_header_bad <= 1'b1;
                    9'd15: if (sd_read_data != 8'h20) wav_header_bad <= 1'b1;
                    9'd16: if (sd_read_data != 8'h10) wav_header_bad <= 1'b1;
                    9'd17: if (sd_read_data != 8'h00) wav_header_bad <= 1'b1;
                    9'd18: if (sd_read_data != 8'h00) wav_header_bad <= 1'b1;
                    9'd19: if (sd_read_data != 8'h00) wav_header_bad <= 1'b1;
                    9'd20: if (sd_read_data != 8'h01) wav_header_bad <= 1'b1;
                    9'd21: if (sd_read_data != 8'h00) wav_header_bad <= 1'b1;
                    9'd22: if (sd_read_data != 8'h02) wav_header_bad <= 1'b1;
                    9'd23: if (sd_read_data != 8'h00) wav_header_bad <= 1'b1;
                    9'd24: if (sd_read_data != 8'h80) wav_header_bad <= 1'b1;
                    9'd25: if (sd_read_data != 8'hBB) wav_header_bad <= 1'b1;
                    9'd26: if (sd_read_data != 8'h00) wav_header_bad <= 1'b1;
                    9'd27: if (sd_read_data != 8'h00) wav_header_bad <= 1'b1;
                    9'd28: if (sd_read_data != 8'h00) wav_header_bad <= 1'b1;
                    9'd29: if (sd_read_data != 8'hEE) wav_header_bad <= 1'b1;
                    9'd30: if (sd_read_data != 8'h02) wav_header_bad <= 1'b1;
                    9'd31: if (sd_read_data != 8'h00) wav_header_bad <= 1'b1;
                    9'd32: if (sd_read_data != 8'h04) wav_header_bad <= 1'b1;
                    9'd33: if (sd_read_data != 8'h00) wav_header_bad <= 1'b1;
                    9'd34: if (sd_read_data != 8'h10) wav_header_bad <= 1'b1;
                    9'd35: if (sd_read_data != 8'h00) wav_header_bad <= 1'b1;
                    9'd36: if (sd_read_data != 8'h64) wav_header_bad <= 1'b1;
                    9'd37: if (sd_read_data != 8'h61) wav_header_bad <= 1'b1;
                    9'd38: if (sd_read_data != 8'h74) wav_header_bad <= 1'b1;
                    9'd39: if (sd_read_data != 8'h61) wav_header_bad <= 1'b1;
                    9'd40: wav_data_size[7:0] <= sd_read_data;
                    9'd41: wav_data_size[15:8] <= sd_read_data;
                    9'd42: wav_data_size[23:16] <= sd_read_data;
                    9'd43: wav_data_size[31:24] <= sd_read_data;
                    default: begin end
                endcase
            end
        end
    end

    // PCM arrives little-endian as Llo,Lhi,Rlo,Rhi. The 512-frame FIFO is
    // synchronous-read so Tang Dynasty can map it into embedded RAM blocks.
    reg [31:0] sample_fifo [0:511];
    reg [8:0]  fifo_write_ptr;
    reg [8:0]  fifo_read_ptr;
    reg [9:0]  fifo_count;
    reg [31:0] frame_assembler;
    reg [1:0]  frame_lane;
    reg [31:0] fifo_read_data;
    reg        sample_pending;
    reg        last_sample_pending;
    reg        clip_end_sent;
    reg [24:0] sample_phase;
    wire [25:0] sample_phase_sum = {1'b0, sample_phase} + AUDIO_HZ;
    wire sample_tick = (sample_phase_sum >= CLK_HZ);
    wire payload_byte_valid = sd_read_valid && read_kind == RK_WAV &&
                              audio_bytes_remaining != 32'd0 &&
                              (!wav_header_pending || sector_byte_index >= 9'd44) &&
                              !scene_changed && !restart_pending;
    wire [7:0] payload_byte = sd_read_data;
    wire fifo_frame_write = payload_byte_valid && frame_lane == 2'd3;
    wire fifo_read_request = sample_tick && playing && wave_valid &&
                             (fifo_count != 10'd0) && !scene_changed;

    always @(posedge clk) begin
        if (!rst) begin
            if (fifo_frame_write)
                sample_fifo[fifo_write_ptr] <= {payload_byte, frame_assembler[23:0]};
            if (fifo_read_request)
                fifo_read_data <= sample_fifo[fifo_read_ptr];
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            fifo_write_ptr    <= 9'd0;
            fifo_read_ptr     <= 9'd0;
            fifo_count        <= 10'd0;
            frame_assembler   <= 32'd0;
            frame_lane        <= 2'd0;
            sample_pending    <= 1'b0;
            last_sample_pending <= 1'b0;
            clip_end_sent     <= 1'b0;
            sample_phase      <= 25'd0;
            audio_valid       <= 1'b0;
            audio_left        <= 24'd0;
            audio_right       <= 24'd0;
            clip_end          <= 1'b0;
        end else begin
            clip_end    <= 1'b0;
            audio_valid <= 1'b0;
            audio_left  <= 24'd0;
            audio_right <= 24'd0;

            if (scene_changed) begin
                fifo_write_ptr      <= 9'd0;
                fifo_read_ptr       <= 9'd0;
                fifo_count          <= 10'd0;
                frame_assembler     <= 32'd0;
                frame_lane          <= 2'd0;
                sample_pending      <= 1'b0;
                last_sample_pending <= 1'b0;
                clip_end_sent       <= 1'b0;
                sample_phase        <= 25'd0;
            end else begin
                if (sample_tick)
                    sample_phase <= sample_phase_sum - CLK_HZ;
                else
                    sample_phase <= sample_phase_sum[24:0];

                if (payload_byte_valid) begin
                    case (frame_lane)
                        2'd0: frame_assembler[7:0]   <= payload_byte;
                        2'd1: frame_assembler[15:8]  <= payload_byte;
                        2'd2: frame_assembler[23:16] <= payload_byte;
                        default: frame_assembler[31:24] <= payload_byte;
                    endcase
                    if (frame_lane == 2'd3)
                        frame_lane <= 2'd0;
                    else
                        frame_lane <= frame_lane + 1'b1;
                end

                if (fifo_frame_write)
                    fifo_write_ptr <= fifo_write_ptr + 1'b1;

                if (fifo_read_request)
                    fifo_read_ptr <= fifo_read_ptr + 1'b1;

                case ({fifo_frame_write, fifo_read_request})
                    2'b10: fifo_count <= fifo_count + 1'b1;
                    2'b01: fifo_count <= fifo_count - 1'b1;
                    default: fifo_count <= fifo_count;
                endcase

                sample_pending <= fifo_read_request;
                last_sample_pending <= fifo_read_request && source_done &&
                                       (fifo_count == 10'd1);
                if (source_done && fifo_count == 10'd0 && !sample_pending &&
                    !fifo_read_request && !clip_end_sent) begin
                    clip_end <= 1'b1;
                    clip_end_sent <= 1'b1;
                end
                if (sample_pending) begin
                    audio_valid <= 1'b1;
                    audio_left  <= {fifo_read_data[15:0], 8'd0};
                    audio_right <= {fifo_read_data[31:16], 8'd0};
                    if (last_sample_pending && !clip_end_sent) begin
                        clip_end <= 1'b1;
                        clip_end_sent <= 1'b1;
                    end
                end
            end
        end
    end

    // Card initialization, FAT32 directory/chain traversal, and WAV control.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state                     <= ST_WAIT_INIT;
            read_kind                 <= RK_MBR;
            read_lba                  <= 32'd0;
            sd_sec_read               <= 1'b0;
            scene_id_q                <= 2'd0;
            restart_pending           <= 1'b0;
            filesystem_ready          <= 1'b0;
            wave_valid                <= 1'b0;
            wav_header_pending        <= 1'b0;
            source_done               <= 1'b0;
            card_ready                <= 1'b0;
            file_error                <= 1'b0;
            volume_start_lba          <= 32'd0;
            fat_start_lba             <= 32'd0;
            first_data_lba            <= 32'd0;
            root_cluster              <= 32'd0;
            root_walk_cluster         <= 32'd0;
            file_cluster              <= 32'd0;
            file_size_bytes           <= 32'd0;
            audio_bytes_remaining     <= 32'd0;
            sectors_per_cluster       <= 8'd0;
            sectors_per_cluster_shift <= 3'd0;
            number_of_fats             <= 8'd0;
            fat_size_sectors          <= 32'd0;
            directory_sector_index    <= 8'd0;
            file_sector_index         <= 8'd0;
            fat_entry_offset          <= 9'd0;
        end else begin
            sd_sec_read <= 1'b0;

            if (scene_changed) begin
                scene_id_q            <= scene_id;
                file_error            <= 1'b0;
                wave_valid            <= 1'b0;
                source_done           <= 1'b0;
                audio_bytes_remaining <= 32'd0;
                wav_header_pending    <= 1'b0;
                if (sd_read_busy || sd_sec_read) begin
                    restart_pending <= 1'b1;
                    state <= ST_WAIT_SECTOR;
                end else if (filesystem_ready) begin
                    restart_pending <= 1'b0;
                    state <= ST_DIR_START;
                end else begin
                    restart_pending <= 1'b0;
                    state <= ST_WAIT_INIT;
                end
            end else if (restart_pending && !sd_read_busy && !sd_sec_read && filesystem_ready) begin
                restart_pending <= 1'b0;
                state <= ST_DIR_START;
            end else begin
                if (payload_byte_valid)
                    audio_bytes_remaining <= audio_bytes_remaining - 1'b1;
                case (state)
                    ST_WAIT_INIT: begin
                        if (sd_init_done) begin
                            read_lba          <= 32'd0;
                            read_kind         <= RK_MBR;
                            sd_sec_read       <= 1'b1;
                            state             <= ST_WAIT_SECTOR;
                        end
                    end

                    ST_WAIT_SECTOR: begin
                        if (sd_read_end) begin
                            // A scene change cannot abort an SPI sector already
                            // in flight. Once that sector finishes, discard its
                            // old-track continuation and start looking up the
                            // newly selected WAV immediately.
                            if (restart_pending) begin
                                restart_pending <= 1'b0;
                                if (filesystem_ready)
                                    state <= ST_DIR_START;
                                else
                                    state <= ST_WAIT_INIT;
                            end else begin
                            case (read_kind)
                                RK_MBR: begin
                                    if (mbr_jump_ok && bpb_bps_lo == 8'd0 &&
                                        bpb_bps_hi == 8'd2) begin
                                        volume_start_lba <= 32'd0;
                                        state <= ST_PARSE_BPB;
                                    end else if (mbr_sig_55 && mbr_sig_aa &&
                                                 mbr_partition_lba != 32'd0) begin
                                        volume_start_lba <= mbr_partition_lba;
                                        read_lba <= mbr_partition_lba;
                                        read_kind <= RK_VBR;
                                        sd_sec_read <= 1'b1;
                                    end else begin
                                        file_error <= 1'b1;
                                        state <= ST_ERROR;
                                    end
                                end

                                RK_VBR: state <= ST_PARSE_BPB;
                                RK_ROOT: state <= ST_SCAN_DIR;
                                RK_ROOT_FAT: state <= ST_PARSE_ROOT_FAT;
                                RK_AUDIO_FAT: state <= ST_PARSE_AUDIO_FAT;
                                RK_WAV: begin
                                    if (wav_header_pending) begin
                                        state <= ST_CHECK_WAV;
                                    end else if (audio_bytes_remaining == 32'd0) begin
                                        source_done <= 1'b1;
                                        state <= ST_WAIT_AUDIO_END;
                                    end else if (file_sector_index + 1'b1 < sectors_per_cluster) begin
                                        file_sector_index <= file_sector_index + 1'b1;
                                        state <= ST_WAIT_SPACE;
                                    end else begin
                                        fat_entry_offset <= {file_cluster[6:0], 2'b00};
                                        read_lba <= fat_start_lba + (file_cluster >> 7);
                                        read_kind <= RK_AUDIO_FAT;
                                        sd_sec_read <= 1'b1;
                                    end
                                end

                                default: begin
                                    file_error <= 1'b1;
                                    state <= ST_ERROR;
                                end
                            endcase
                            end
                        end
                    end

                    ST_PARSE_BPB: begin
                        if (bpb_bps_lo != 8'd0 || bpb_bps_hi != 8'd2 ||
                            bpb_reserved == 16'd0 ||
                            (bpb_num_fats != 8'd1 && bpb_num_fats != 8'd2) ||
                            bpb_fatsz16 != 16'd0 || bpb_fatsz32 == 32'd0 ||
                            (bpb_spc != 8'd1 && bpb_spc != 8'd2 &&
                             bpb_spc != 8'd4 && bpb_spc != 8'd8 &&
                             bpb_spc != 8'd16 && bpb_spc != 8'd32 &&
                             bpb_spc != 8'd64) ||
                            (bpb_root_cluster & 32'h0FFFFFFF) < 32'd2) begin
                            file_error <= 1'b1;
                            state <= ST_ERROR;
                        end else begin
                            sectors_per_cluster <= bpb_spc;
                            number_of_fats <= bpb_num_fats;
                            fat_size_sectors <= bpb_fatsz32;
                            fat_start_lba <= volume_start_lba + bpb_reserved;
                            root_cluster <= bpb_root_cluster & 32'h0FFFFFFF;
                            if (bpb_spc == 8'd1) sectors_per_cluster_shift <= 3'd0;
                            else if (bpb_spc == 8'd2) sectors_per_cluster_shift <= 3'd1;
                            else if (bpb_spc == 8'd4) sectors_per_cluster_shift <= 3'd2;
                            else if (bpb_spc == 8'd8) sectors_per_cluster_shift <= 3'd3;
                            else if (bpb_spc == 8'd16) sectors_per_cluster_shift <= 3'd4;
                            else if (bpb_spc == 8'd32) sectors_per_cluster_shift <= 3'd5;
                            else sectors_per_cluster_shift <= 3'd6;
                            if (bpb_num_fats == 8'd1)
                                first_data_lba <= volume_start_lba + bpb_reserved + bpb_fatsz32;
                            else
                                first_data_lba <= volume_start_lba + bpb_reserved +
                                                  (bpb_fatsz32 << 1);
                            filesystem_ready <= 1'b1;
                            card_ready <= 1'b1;
                            state <= ST_DIR_START;
                        end
                    end

                    ST_DIR_START: begin
                        root_walk_cluster <= root_cluster;
                        directory_sector_index <= 8'd0;
                        read_lba <= cluster_to_lba(root_cluster, 8'd0);
                        read_kind <= RK_ROOT;
                        sd_sec_read <= 1'b1;
                        state <= ST_WAIT_SECTOR;
                    end

                    ST_SCAN_DIR: begin
                        if (dir_match_found) begin
                            file_cluster <= dir_found_cluster & 32'h0FFFFFFF;
                            file_size_bytes <= dir_found_size;
                            state <= ST_BEGIN_FILE;
                        end else if (dir_terminator_seen) begin
                            file_error <= 1'b1;
                            state <= ST_ERROR;
                        end else if (directory_sector_index + 1'b1 < sectors_per_cluster) begin
                            directory_sector_index <= directory_sector_index + 1'b1;
                            read_lba <= cluster_to_lba(root_walk_cluster,
                                                       directory_sector_index + 1'b1);
                            read_kind <= RK_ROOT;
                            sd_sec_read <= 1'b1;
                            state <= ST_WAIT_SECTOR;
                        end else begin
                            fat_entry_offset <= {root_walk_cluster[6:0], 2'b00};
                            read_lba <= fat_start_lba + (root_walk_cluster >> 7);
                            read_kind <= RK_ROOT_FAT;
                            sd_sec_read <= 1'b1;
                            state <= ST_WAIT_SECTOR;
                        end
                    end

                    ST_PARSE_ROOT_FAT: begin
                        if (next_fat_cluster < 32'd2 || next_fat_cluster >= 32'h0FFFFFF0) begin
                            file_error <= 1'b1;
                            state <= ST_ERROR;
                        end else begin
                            root_walk_cluster <= next_fat_cluster;
                            directory_sector_index <= 8'd0;
                            read_lba <= cluster_to_lba(next_fat_cluster, 8'd0);
                            read_kind <= RK_ROOT;
                            sd_sec_read <= 1'b1;
                            state <= ST_WAIT_SECTOR;
                        end
                    end

                    ST_BEGIN_FILE: begin
                        if (file_cluster < 32'd2 || file_size_bytes <= 32'd44) begin
                            file_error <= 1'b1;
                            state <= ST_ERROR;
                        end else begin
                            source_done <= 1'b0;
                            wave_valid <= 1'b0;
                            wav_header_pending <= 1'b1;
                            audio_bytes_remaining <= file_size_bytes - 32'd44;
                            file_sector_index <= 8'd0;
                            read_lba <= cluster_to_lba(file_cluster, 8'd0);
                            read_kind <= RK_WAV;
                            sd_sec_read <= 1'b1;
                            state <= ST_WAIT_SECTOR;
                        end
                    end

                    ST_CHECK_WAV: begin
                        if (wav_header_bad || wav_data_size != file_size_bytes - 32'd44 ||
                            wav_data_size[1:0] != 2'b00) begin
                            file_error <= 1'b1;
                            state <= ST_ERROR;
                        end else begin
                            wav_header_pending <= 1'b0;
                            wave_valid <= 1'b1;
                            if (audio_bytes_remaining == 32'd0) begin
                                source_done <= 1'b1;
                                state <= ST_WAIT_AUDIO_END;
                            end else if (file_sector_index + 1'b1 < sectors_per_cluster) begin
                                file_sector_index <= file_sector_index + 1'b1;
                                state <= ST_WAIT_SPACE;
                            end else begin
                                fat_entry_offset <= {file_cluster[6:0], 2'b00};
                                read_lba <= fat_start_lba + (file_cluster >> 7);
                                read_kind <= RK_AUDIO_FAT;
                                sd_sec_read <= 1'b1;
                                state <= ST_WAIT_SECTOR;
                            end
                        end
                    end

                    ST_WAIT_SPACE: begin
                        if (fifo_count <= 10'd384) begin
                            read_lba <= cluster_to_lba(file_cluster, file_sector_index);
                            read_kind <= RK_WAV;
                            sd_sec_read <= 1'b1;
                            state <= ST_WAIT_SECTOR;
                        end
                    end

                    ST_PARSE_AUDIO_FAT: begin
                        if (next_fat_cluster < 32'd2 || next_fat_cluster >= 32'h0FFFFFF0) begin
                            file_error <= 1'b1;
                            state <= ST_ERROR;
                        end else begin
                            file_cluster <= next_fat_cluster;
                            file_sector_index <= 8'd0;
                            state <= ST_WAIT_SPACE;
                        end
                    end

                    ST_WAIT_AUDIO_END: begin
                        state <= ST_WAIT_AUDIO_END;
                    end

                    ST_ERROR: begin
                        file_error <= 1'b1;
                    end

                    default: state <= ST_WAIT_INIT;
                endcase
            end
        end
    end

endmodule
