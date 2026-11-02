// Read-only FAT32 root-directory file stream, with a shared sector-read backend.
// Requests stay asserted until granted.  A granted request cannot be cancelled:
// start records the new filename, discards the old bytes, and drains that request.
// Only data reads observe allow_read; mounting and FAT/directory reads continue.
// sec_error terminates a transfer.  sec_done follows the final byte (or may be
// asserted with it).  The backend supplies exactly 512 bytes per requested block.
module fat32_file_stream (
    input  wire        clk,
    input  wire        rst,
    input  wire        card_ready,
    input  wire        start,
    input  wire [87:0] file_name,
    input  wire        allow_read,
    input  wire [5:0]  max_blocks,
    output reg         file_open,
    output reg  [31:0] file_size,
    output reg         file_error,
    output reg         file_done,
    output reg  [7:0]  data,
    output reg         data_valid,
    output reg  [31:0] byte_offset,
    output reg         req_valid,
    output reg  [31:0] req_lba,
    output reg  [5:0]  req_blocks,
    input  wire        req_grant,
    input  wire [7:0]  sec_data,
    input  wire        sec_valid,
    input  wire        sec_block_end,
    input  wire        sec_done,
    input  wire        sec_error
);

    localparam [3:0] S_IDLE       = 4'd0,
                     S_SELECT     = 4'd1,
                     S_GRANT      = 4'd2,
                     S_TRANSFER   = 4'd3,
                     S_MBR_CHECK  = 4'd4,
                     S_BPB_CHECK  = 4'd5,
                     S_DIR_BEGIN  = 4'd6,
                     S_DIR_CHECK  = 4'd7,
                     S_FAT_CHECK  = 4'd8,
                     S_DATA_WAIT  = 4'd9,
                     S_DATA_NEXT  = 4'd10,
                     S_FINISHED   = 4'd11,
                     S_ERROR      = 4'd12,
                     S_META_WAIT  = 4'd13,
                     S_TRANSFER_CHECK = 4'd14;
    localparam [2:0] R_MBR        = 3'd0,
                     R_BPB        = 3'd1,
                     R_DIRECTORY  = 3'd2,
                     R_ROOT_FAT   = 3'd3,
                     R_FILE_FAT   = 3'd4,
                     R_DATA       = 3'd5;

    reg [3:0] state;
    reg [4:0] metadata_wait;
    reg [3:0] settle_destination;
    reg [2:0] read_kind;
    reg [87:0] selected_name;
    reg selected_valid;
    reg restart_pending;
    reg mounted;
    reg [8:0] sector_index;
    reg [14:0] transfer_bytes;
    reg transfer_failed;
    reg parse_valid;
    reg [7:0] parse_data;
    reg [8:0] parse_index;
    reg [2:0] parse_kind;

    reg [31:0] volume_lba, partition_sectors;
    reg [31:0] fat_lba, data_lba;
    reg [31:0] root_cluster, cluster_count, max_cluster;
    reg [7:0] cluster_sectors;
    reg [2:0] cluster_shift;
    reg [31:0] root_walk_cluster, file_walk_cluster;
    reg [7:0] root_sector, file_sector;
    reg [31:0] root_hops, file_hops;
    reg [31:0] bytes_remaining, emitted_bytes;

    // Sector zero is examined both as an MBR and as a superfloppy BPB.
    reg bpb_jump, signature_55, signature_aa;
    reg [15:0] bpb_bytes_per_sector, bpb_reserved, bpb_root_entries;
    reg [15:0] bpb_total16, bpb_fat16, bpb_flags, bpb_version;
    reg [7:0] bpb_spc, bpb_fats;
    reg [31:0] bpb_total32, bpb_fat32, bpb_root;
    reg partition_found, candidate_fat32;
    reg [31:0] candidate_lba, candidate_size;
    reg [31:0] found_partition_lba, found_partition_size;

    reg dir_match, dir_regular, dir_ended, dir_found;
    reg [15:0] dir_cluster_hi, dir_cluster_lo;
    reg [31:0] dir_size, found_cluster, found_size;
    reg [8:0] fat_entry_offset;
    reg [31:0] fat_word;

    function [2:0] spc_shift;
        input [7:0] spc;
        begin
            case (spc)
                8'd1: spc_shift = 3'd0;
                8'd2: spc_shift = 3'd1;
                8'd4: spc_shift = 3'd2;
                8'd8: spc_shift = 3'd3;
                8'd16: spc_shift = 3'd4;
                8'd32: spc_shift = 3'd5;
                8'd64: spc_shift = 3'd6;
                default: spc_shift = 3'd7; // 128 is checked separately.
            endcase
        end
    endfunction

    function [7:0] wanted_name_byte;
        input [3:0] index;
        begin
            case (index)
                4'd0: wanted_name_byte = selected_name[87:80];
                4'd1: wanted_name_byte = selected_name[79:72];
                4'd2: wanted_name_byte = selected_name[71:64];
                4'd3: wanted_name_byte = selected_name[63:56];
                4'd4: wanted_name_byte = selected_name[55:48];
                4'd5: wanted_name_byte = selected_name[47:40];
                4'd6: wanted_name_byte = selected_name[39:32];
                4'd7: wanted_name_byte = selected_name[31:24];
                4'd8: wanted_name_byte = selected_name[23:16];
                4'd9: wanted_name_byte = selected_name[15:8];
                default: wanted_name_byte = selected_name[7:0];
            endcase
        end
    endfunction

    wire valid_spc = bpb_spc == 8'd1 || bpb_spc == 8'd2 ||
                     bpb_spc == 8'd4 || bpb_spc == 8'd8 ||
                     bpb_spc == 8'd16 || bpb_spc == 8'd32 ||
                     bpb_spc == 8'd64 || bpb_spc == 8'd128;
    // Mount arithmetic is pipelined. Metadata remains stable while the state
    // machine waits twenty clocks after the final BPB byte, so all dependent
    // fields settle before validation; no long add/subtract/shift chain sits
    // between a sector parser register and a 100 MHz state register.
    reg [31:0] volume_sectors, calculated_data_sectors, calculated_clusters;
    reg [31:0] calculated_max_cluster, minimum_fat_sectors;
    reg [32:0] fats_span, overhead, volume_end;
    reg [31:0] base_fat_lba,calculated_fat_lba,calculated_data_lba;
    reg valid_bpb;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            volume_sectors<=0; fats_span<=0; overhead<=0;
            calculated_data_sectors<=0; calculated_clusters<=0;
            volume_end<=0; calculated_max_cluster<=0; minimum_fat_sectors<=0;
            valid_bpb<=0;base_fat_lba<=0;calculated_fat_lba<=0;calculated_data_lba<=0;
        end else begin
            volume_sectors<=bpb_total16!=0 ? {16'd0,bpb_total16} : bpb_total32;
            fats_span<=bpb_fats==2 ? ({1'b0,bpb_fat32}<<1) : {1'b0,bpb_fat32};
            overhead<={17'd0,bpb_reserved}+fats_span;
            calculated_data_sectors<=volume_sectors-overhead[31:0];
            calculated_clusters<=calculated_data_sectors>>spc_shift(bpb_spc);
            volume_end<={1'b0,volume_lba}+{1'b0,volume_sectors};
            calculated_max_cluster<=calculated_clusters+32'd1;
            minimum_fat_sectors<=(calculated_max_cluster>>7)+32'd1;
            base_fat_lba<=volume_lba+{16'd0,bpb_reserved};
            calculated_fat_lba<=base_fat_lba+(bpb_flags[7] && bpb_flags[3:0]==1 ? bpb_fat32 : 32'd0);
            calculated_data_lba<=volume_lba+overhead[31:0];
            valid_bpb<=bpb_jump && signature_55 && signature_aa &&
                bpb_bytes_per_sector==512 && valid_spc && bpb_reserved!=0 &&
                (bpb_fats==1 || bpb_fats==2) && bpb_root_entries==0 && bpb_fat16==0 &&
                bpb_fat32!=0 && bpb_version==0 && overhead<{1'b0,volume_sectors} &&
                !volume_end[32] && calculated_clusters>=65525 &&
                calculated_clusters<=32'h0FFFFFEE && bpb_fat32>=minimum_fat_sectors &&
                (bpb_root & 32'h0FFFFFFF)>=2 &&
                (bpb_root & 32'h0FFFFFFF)<=calculated_max_cluster &&
                (!bpb_flags[7] || {4'd0,bpb_flags[3:0]}<bpb_fats) &&
                (partition_sectors==0 || volume_sectors<=partition_sectors);
        end
    end

    wire [31:0] next_cluster = fat_word & 32'h0FFFFFFF;
    reg [31:0] root_m2,root_scaled,root_base;
    reg [31:0] walk_m2,walk_scaled,walk_base;
    reg [31:0] file_m2,file_scaled,file_base;
    reg [31:0] next_m2,next_scaled,next_base;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            root_m2<=0;root_scaled<=0;root_base<=0;
            walk_m2<=0;walk_scaled<=0;walk_base<=0;
            file_m2<=0;file_scaled<=0;file_base<=0;
            next_m2<=0;next_scaled<=0;next_base<=0;
        end else begin
            root_m2<=root_cluster-32'd2;root_scaled<=root_m2<<cluster_shift;root_base<=data_lba+root_scaled;
            walk_m2<=root_walk_cluster-32'd2;walk_scaled<=walk_m2<<cluster_shift;walk_base<=data_lba+walk_scaled;
            file_m2<=file_walk_cluster-32'd2;file_scaled<=file_m2<<cluster_shift;file_base<=data_lba+file_scaled;
            next_m2<=next_cluster-32'd2;next_scaled<=next_m2<<cluster_shift;next_base<=data_lba+next_scaled;
        end
    end
    reg valid_next_cluster,valid_found_file;
    reg [31:0] found_sectors,volume_data_sectors;
    reg found_cluster_valid;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            valid_next_cluster<=0;valid_found_file<=0;
            found_sectors<=0;volume_data_sectors<=0;found_cluster_valid<=0;
        end else begin
            valid_next_cluster<=next_cluster>=2 && next_cluster<32'h0FFFFFF0 && next_cluster<=max_cluster;
            found_sectors<=(found_size>>9)+(found_size[8:0]!=0);
            volume_data_sectors<=cluster_count<<cluster_shift;
            found_cluster_valid<=found_cluster>=2 && found_cluster<=max_cluster;
            valid_found_file<=(found_size==0 || found_cluster_valid) && found_sectors<=volume_data_sectors;
        end
    end
    wire [31:0] remaining_file_sectors = (bytes_remaining >> 9) +
                                           (bytes_remaining[8:0] != 9'd0);
    wire [7:0] remaining_cluster_sectors = cluster_sectors - file_sector;
    wire [5:0] limited_blocks = max_blocks == 6'd0 ? 6'd1 :
                               max_blocks > 6'd32 ? 6'd32 : max_blocks;
    wire [7:0] cluster_burst = remaining_cluster_sectors < {2'd0,limited_blocks} ?
                               remaining_cluster_sectors : {2'd0,limited_blocks};
    wire [5:0] data_burst = remaining_file_sectors < {24'd0,cluster_burst} ?
                            remaining_file_sectors[5:0] : cluster_burst[5:0];
    wire transfer_active = state == S_TRANSFER || (state == S_GRANT && req_grant);
    wire [15:0] expected_bytes = {1'b0,req_blocks,9'd0};
    wire [5:0] mbr_position = parse_index - 9'd446;
    wire [31:0] directory_cluster = {dir_cluster_hi,dir_cluster_lo} & 32'h0FFFFFFF;

    // Issuing a request also clears its streaming metadata parser. No sector
    // buffer or combinational 512-byte mux is needed in this module.
    task issue_read;
        input [2:0] kind;
        input [31:0] lba;
        input [5:0] blocks;
        begin
            req_valid <= 1'b1;
            req_lba <= lba;
            req_blocks <= blocks;
            read_kind <= kind;
            sector_index <= 9'd0;
            transfer_bytes <= 15'd0;
            transfer_failed <= 1'b0;
            state <= S_GRANT;
            if (kind == R_MBR || kind == R_BPB) begin
                bpb_jump <= 1'b0;
                signature_55 <= 1'b0;
                signature_aa <= 1'b0;
                bpb_bytes_per_sector <= 16'd0;
                bpb_reserved <= 16'd0;
                bpb_root_entries <= 16'd0;
                bpb_total16 <= 16'd0;
                bpb_fat16 <= 16'd0;
                bpb_flags <= 16'd0;
                bpb_version <= 16'd0;
                bpb_spc <= 8'd0;
                bpb_fats <= 8'd0;
                bpb_total32 <= 32'd0;
                bpb_fat32 <= 32'd0;
                bpb_root <= 32'd0;
                partition_found <= 1'b0;
                candidate_fat32 <= 1'b0;
                candidate_lba <= 32'd0;
                candidate_size <= 32'd0;
                found_partition_lba <= 32'd0;
                found_partition_size <= 32'd0;
            end
            if (kind == R_DIRECTORY) begin
                dir_match <= 1'b0;
                dir_regular <= 1'b0;
                dir_ended <= 1'b0;
                dir_found <= 1'b0;
                dir_cluster_hi <= 16'd0;
                dir_cluster_lo <= 16'd0;
                dir_size <= 32'd0;
                found_cluster <= 32'd0;
                found_size <= 32'd0;
            end
            if (kind == R_ROOT_FAT || kind == R_FILE_FAT)
                fat_word <= 32'd0;
        end
    endtask

    task fail_file;
        begin
            file_error <= 1'b1;
            file_open <= 1'b0;
            state <= S_ERROR;
        end
    endtask

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            parse_valid<=0;parse_data<=0;parse_index<=0;parse_kind<=0;
            metadata_wait <= 0;
            settle_destination <= S_IDLE;
            read_kind <= R_MBR;
            selected_name <= 88'd0;
            selected_valid <= 1'b0;
            restart_pending <= 1'b0;
            mounted <= 1'b0;
            file_open <= 1'b0;
            file_size <= 32'd0;
            file_error <= 1'b0;
            file_done <= 1'b0;
            data <= 8'd0;
            data_valid <= 1'b0;
            byte_offset <= 32'd0;
            req_valid <= 1'b0;
            req_lba <= 32'd0;
            req_blocks <= 6'd1;
            sector_index <= 9'd0;
            transfer_bytes <= 15'd0;
            transfer_failed <= 1'b0;
            volume_lba <= 32'd0;
            partition_sectors <= 32'd0;
            fat_lba <= 32'd0;
            data_lba <= 32'd0;
            root_cluster <= 32'd0;
            cluster_count <= 32'd0;
            max_cluster <= 32'd0;
            cluster_sectors <= 8'd0;
            cluster_shift <= 3'd0;
            root_walk_cluster <= 32'd0;
            file_walk_cluster <= 32'd0;
            root_sector <= 8'd0;
            file_sector <= 8'd0;
            root_hops <= 32'd0;
            file_hops <= 32'd0;
            bytes_remaining <= 32'd0;
            emitted_bytes <= 32'd0;
            fat_entry_offset <= 9'd0;
            fat_word <= 32'd0;
            bpb_jump <= 1'b0;
            signature_55 <= 1'b0;
            signature_aa <= 1'b0;
            bpb_bytes_per_sector <= 16'd0;
            bpb_reserved <= 16'd0;
            bpb_root_entries <= 16'd0;
            bpb_total16 <= 16'd0;
            bpb_fat16 <= 16'd0;
            bpb_flags <= 16'd0;
            bpb_version <= 16'd0;
            bpb_spc <= 8'd0;
            bpb_fats <= 8'd0;
            bpb_total32 <= 32'd0;
            bpb_fat32 <= 32'd0;
            bpb_root <= 32'd0;
            partition_found <= 1'b0;
            candidate_fat32 <= 1'b0;
            candidate_lba <= 32'd0;
            candidate_size <= 32'd0;
            found_partition_lba <= 32'd0;
            found_partition_size <= 32'd0;
            dir_match <= 1'b0;
            dir_regular <= 1'b0;
            dir_ended <= 1'b0;
            dir_found <= 1'b0;
            dir_cluster_hi <= 16'd0;
            dir_cluster_lo <= 16'd0;
            dir_size <= 32'd0;
            found_cluster <= 32'd0;
            found_size <= 32'd0;
        end else begin
            parse_valid<=0;
            data_valid <= 1'b0;
            file_done <= 1'b0;
            if (start) begin
                selected_name <= file_name;
                selected_valid <= 1'b1;
                restart_pending <= 1'b1;
                file_open <= 1'b0;
                file_size <= 32'd0;
                file_error <= 1'b0;
            end

            if (!card_ready) begin
                mounted <= 1'b0;
                req_valid <= 1'b0;
                file_open <= 1'b0;
                file_size <= 32'd0;
                file_error <= 1'b0;
                restart_pending <= 1'b0;
                state <= S_IDLE;
            end else if (start && state != S_GRANT && state != S_TRANSFER) begin
                state <= S_SELECT;
            end else begin
                case (state)
                    S_IDLE: if (selected_valid) state <= S_SELECT;
                    S_SELECT: begin
                        restart_pending <= 1'b0;
                        file_open <= 1'b0;
                        file_size <= 32'd0;
                        file_error <= 1'b0;
                        bytes_remaining <= 32'd0;
                        emitted_bytes <= 32'd0;
                        byte_offset <= 32'd0;
                        if (mounted) begin
                            metadata_wait<=4;settle_destination<=S_DIR_BEGIN;state<=S_META_WAIT;
                        end
                        else begin
                            volume_lba <= 32'd0;
                            partition_sectors <= 32'd0;
                            issue_read(R_MBR, 32'd0, 6'd1);
                        end
                    end
                    S_GRANT: begin
                        if (req_grant) begin
                            req_valid <= 1'b0;
                            if (sec_error) begin
                                if (restart_pending || start) state <= S_SELECT;
                                else fail_file;
                            end else state <= S_TRANSFER;
                        end
                    end
                    S_TRANSFER: begin
                        if (sec_error || sec_done) begin
                            transfer_failed <= sec_error;
                            state <= S_TRANSFER_CHECK;
                        end
                    end
                    // Compare on the following cycle: transfer_bytes already
                    // includes a final byte coincident with sec_done. Keeping
                    // the byte-counter adder out of this state-decode cone
                    // gives the 100 MHz sector parser a complete clock budget.
                    S_TRANSFER_CHECK: begin
                            if (restart_pending || start)
                                state <= S_SELECT;
                            else if (transfer_failed || {1'b0,transfer_bytes} != expected_bytes)
                                fail_file;
                            else begin
                                case (read_kind)
                                    R_MBR, R_BPB: begin metadata_wait<=5'd20;settle_destination<=read_kind==R_MBR ? S_MBR_CHECK : S_BPB_CHECK;state<=S_META_WAIT;end
                                    R_DIRECTORY: begin metadata_wait<=4;settle_destination<=S_DIR_CHECK;state<=S_META_WAIT;end
                                    R_ROOT_FAT, R_FILE_FAT: begin metadata_wait<=4;settle_destination<=S_FAT_CHECK;state<=S_META_WAIT;end
                                    R_DATA: begin
                                        file_sector <= file_sector + {2'd0,req_blocks};
                                        state <= S_DATA_NEXT;
                                    end
                                    default: fail_file;
                                endcase
                            end
                    end
                    S_META_WAIT: begin
                        if(metadata_wait!=0) metadata_wait<=metadata_wait-1'b1;
                        else state<=settle_destination;
                    end
                    S_MBR_CHECK: begin
                        if (valid_bpb)
                            state <= S_BPB_CHECK;
                        else if (signature_55 && signature_aa && partition_found) begin
                            volume_lba <= found_partition_lba;
                            partition_sectors <= found_partition_size;
                            issue_read(R_BPB, found_partition_lba, 6'd1);
                        end else fail_file;
                    end
                    S_BPB_CHECK: begin
                        if (!valid_bpb) fail_file;
                        else begin
                            mounted <= 1'b1;
                            cluster_sectors <= bpb_spc;
                            cluster_shift <= spc_shift(bpb_spc);
                            cluster_count <= calculated_clusters;
                            max_cluster <= calculated_max_cluster;
                            root_cluster <= bpb_root & 32'h0FFFFFFF;
                            fat_lba<=calculated_fat_lba;
                            data_lba<=calculated_data_lba;
                            metadata_wait<=4;settle_destination<=S_DIR_BEGIN;state<=S_META_WAIT;
                        end
                    end
                    S_DIR_BEGIN: begin
                        root_walk_cluster <= root_cluster;
                        root_sector <= 8'd0;
                        root_hops <= 32'd1;
                        issue_read(R_DIRECTORY, root_base,6'd1);
                    end
                    S_DIR_CHECK: begin
                        if (dir_found) begin
                            if (!valid_found_file) fail_file;
                            else begin
                                file_size <= found_size;
                                file_open <= 1'b1;
                                bytes_remaining <= found_size;
                                emitted_bytes <= 32'd0;
                                file_walk_cluster <= found_cluster;
                                file_sector <= 8'd0;
                                file_hops <= 32'd1;
                                if (found_size == 32'd0) begin
                                    file_done <= 1'b1;
                                    state <= S_FINISHED;
                                end else begin metadata_wait<=4;settle_destination<=S_DATA_WAIT;state<=S_META_WAIT;end
                            end
                        end else if (dir_ended) fail_file;
                        else if (root_sector + 8'd1 < cluster_sectors) begin
                            root_sector <= root_sector + 8'd1;
                            issue_read(R_DIRECTORY,
                                       walk_base+{24'd0,(root_sector+8'd1)},6'd1);
                        end else begin
                            fat_entry_offset <= {root_walk_cluster[6:0],2'b00};
                            issue_read(R_ROOT_FAT,fat_lba+(root_walk_cluster>>7),6'd1);
                        end
                    end
                    S_FAT_CHECK: begin
                        // EOC (0FFFFFF8..0FFFFFFF), reserved, free and bad cluster
                        // values are all invalid while more file/root data is needed.
                        if (!valid_next_cluster) fail_file;
                        else if (read_kind == R_ROOT_FAT) begin
                            if (next_cluster == root_walk_cluster || root_hops >= cluster_count)
                                fail_file;
                            else begin
                                root_walk_cluster <= next_cluster;
                                root_sector <= 8'd0;
                                root_hops <= root_hops + 32'd1;
                                issue_read(R_DIRECTORY,next_base,6'd1);
                            end
                        end else begin
                            if (next_cluster == file_walk_cluster || file_hops >= cluster_count)
                                fail_file;
                            else begin
                                file_walk_cluster <= next_cluster;
                                file_sector <= 8'd0;
                                file_hops <= file_hops + 32'd1;
                                metadata_wait<=4;settle_destination<=S_DATA_WAIT;state<=S_META_WAIT;
                            end
                        end
                    end
                    S_DATA_WAIT: begin
                        if (allow_read)
                            issue_read(R_DATA,file_base+{24'd0,file_sector},data_burst);
                    end
                    S_DATA_NEXT: begin
                        if (bytes_remaining == 32'd0) begin
                            file_done <= 1'b1;
                            state <= S_FINISHED;
                        end else if (file_sector >= cluster_sectors) begin
                            fat_entry_offset <= {file_walk_cluster[6:0],2'b00};
                            issue_read(R_FILE_FAT,fat_lba+(file_walk_cluster>>7),6'd1);
                        end else state <= S_DATA_WAIT;
                    end
                    S_FINISHED: begin end
                    S_ERROR: begin end
                    default: fail_file;
                endcase

                if (transfer_active && sec_valid && !restart_pending && !start) begin
                    sector_index <= sector_index + 9'd1;
                    transfer_bytes <= transfer_bytes + 15'd1;
                    parse_valid<=1;
                    parse_data<=sec_data;
                    parse_index<=sector_index;
                    parse_kind<=read_kind;
                end
                if (parse_valid && !restart_pending && !start) begin
                    if (parse_kind == R_MBR || parse_kind == R_BPB) begin
                        case (parse_index)
                            9'd0: bpb_jump <= parse_data == 8'hEB || parse_data == 8'hE9;
                            9'd11: bpb_bytes_per_sector[7:0] <= parse_data;
                            9'd12: bpb_bytes_per_sector[15:8] <= parse_data;
                            9'd13: bpb_spc <= parse_data;
                            9'd14: bpb_reserved[7:0] <= parse_data;
                            9'd15: bpb_reserved[15:8] <= parse_data;
                            9'd16: bpb_fats <= parse_data;
                            9'd17: bpb_root_entries[7:0] <= parse_data;
                            9'd18: bpb_root_entries[15:8] <= parse_data;
                            9'd19: bpb_total16[7:0] <= parse_data;
                            9'd20: bpb_total16[15:8] <= parse_data;
                            9'd22: bpb_fat16[7:0] <= parse_data;
                            9'd23: bpb_fat16[15:8] <= parse_data;
                            9'd32: bpb_total32[7:0] <= parse_data;
                            9'd33: bpb_total32[15:8] <= parse_data;
                            9'd34: bpb_total32[23:16] <= parse_data;
                            9'd35: bpb_total32[31:24] <= parse_data;
                            9'd36: bpb_fat32[7:0] <= parse_data;
                            9'd37: bpb_fat32[15:8] <= parse_data;
                            9'd38: bpb_fat32[23:16] <= parse_data;
                            9'd39: bpb_fat32[31:24] <= parse_data;
                            9'd40: bpb_flags[7:0] <= parse_data;
                            9'd41: bpb_flags[15:8] <= parse_data;
                            9'd42: bpb_version[7:0] <= parse_data;
                            9'd43: bpb_version[15:8] <= parse_data;
                            9'd44: bpb_root[7:0] <= parse_data;
                            9'd45: bpb_root[15:8] <= parse_data;
                            9'd46: bpb_root[23:16] <= parse_data;
                            9'd47: bpb_root[31:24] <= parse_data;
                            9'd510: signature_55 <= parse_data == 8'h55;
                            9'd511: signature_aa <= parse_data == 8'hAA;
                            default: begin end
                        endcase
                        if (parse_kind == R_MBR && parse_index >= 9'd446 && parse_index < 9'd510) begin
                            case (mbr_position[3:0])
                                4'd0: begin
                                    candidate_fat32 <= 1'b0;
                                    candidate_lba <= 32'd0;
                                    candidate_size <= 32'd0;
                                end
                                4'd4: candidate_fat32 <= parse_data == 8'h0B || parse_data == 8'h0C ||
                                                         parse_data == 8'h1B || parse_data == 8'h1C;
                                4'd8: candidate_lba[7:0] <= parse_data;
                                4'd9: candidate_lba[15:8] <= parse_data;
                                4'd10: candidate_lba[23:16] <= parse_data;
                                4'd11: candidate_lba[31:24] <= parse_data;
                                4'd12: candidate_size[7:0] <= parse_data;
                                4'd13: candidate_size[15:8] <= parse_data;
                                4'd14: candidate_size[23:16] <= parse_data;
                                4'd15: begin
                                    candidate_size[31:24] <= parse_data;
                                    if (!partition_found && candidate_fat32 && candidate_lba != 32'd0 &&
                                        {parse_data,candidate_size[23:0]} != 32'd0) begin
                                        partition_found <= 1'b1;
                                        found_partition_lba <= candidate_lba;
                                        found_partition_size <= {parse_data,candidate_size[23:0]};
                                    end
                                end
                                default: begin end
                            endcase
                        end
                    end else if (parse_kind == R_DIRECTORY) begin
                        case (parse_index[4:0])
                            5'd0: begin
                                dir_match <= !dir_ended && parse_data != 8'h00 && parse_data != 8'hE5 &&
                                             ((parse_data == wanted_name_byte(4'd0)) ||
                                              (parse_data == 8'h05 && wanted_name_byte(4'd0) == 8'hE5));
                                dir_regular <= 1'b0;
                                if (parse_data == 8'h00) dir_ended <= 1'b1;
                            end
                            5'd1,5'd2,5'd3,5'd4,5'd5,5'd6,5'd7,5'd8,5'd9,5'd10:
                                if (parse_data != wanted_name_byte(parse_index[3:0])) dir_match <= 1'b0;
                            5'd11: dir_regular <= !parse_data[3] && !parse_data[4] && !parse_data[6] && !parse_data[7];
                            5'd20: dir_cluster_hi[7:0] <= parse_data;
                            5'd21: dir_cluster_hi[15:8] <= parse_data;
                            5'd26: dir_cluster_lo[7:0] <= parse_data;
                            5'd27: dir_cluster_lo[15:8] <= parse_data;
                            5'd28: dir_size[7:0] <= parse_data;
                            5'd29: dir_size[15:8] <= parse_data;
                            5'd30: dir_size[23:16] <= parse_data;
                            5'd31: begin
                                dir_size[31:24] <= parse_data;
                                if (!dir_ended && !dir_found && dir_match && dir_regular) begin
                                    dir_found <= 1'b1;
                                    found_cluster <= directory_cluster;
                                    found_size <= {parse_data,dir_size[23:0]};
                                end
                            end
                            default: begin end
                        endcase
                    end else if (parse_kind == R_ROOT_FAT || parse_kind == R_FILE_FAT) begin
                        if (parse_index == fat_entry_offset) fat_word[7:0] <= parse_data;
                        if (parse_index == fat_entry_offset+9'd1) fat_word[15:8] <= parse_data;
                        if (parse_index == fat_entry_offset+9'd2) fat_word[23:16] <= parse_data;
                        if (parse_index == fat_entry_offset+9'd3) fat_word[31:24] <= parse_data;
                    end else if (parse_kind == R_DATA && bytes_remaining != 32'd0) begin
                        data <= parse_data;
                        data_valid <= 1'b1;
                        byte_offset <= emitted_bytes;
                        emitted_bytes <= emitted_bytes + 32'd1;
                        bytes_remaining <= bytes_remaining - 32'd1;
                    end
                end
            end
        end
    end

    // sec_block_end is intentionally not used to index bytes: validating the
    // total byte count also covers backends which pulse it beside the last byte.
endmodule


