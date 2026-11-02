`timescale 1ns/1ps

// 640x480 double buffer backed by the board's on-package 2Mx32 SDRAM.
// All control and pixel input handshakes use sys_clk. A pixel is accepted
// exactly when pixel_valid && load_ready. The producer must hold a pixel
// while load_ready is low. swap_done is returned only at vertical blank.
module video_framebuffer (
    input  wire        sys_clk,
    input  wire        mem_clk,
    input  wire        mem_clk_shift,
    input  wire        pixel_clk,
    input  wire        rst,
    input  wire        clear,
    input  wire        load_cancel,
    input  wire        load_start,
    output reg         load_ready,
    input  wire        pixel_valid,
    input  wire [15:0] pixel_rgb565,
    output reg         load_done,
    input  wire        swap_request,
    input  wire        transition_request,
    output reg         swap_done,
    output reg         visible_frame_start,
    output reg         transition_busy,
    output reg         video_user,
    output reg         video_valid,
    output reg         video_last,
    output reg  [23:0] video_data
);
    localparam [20:0] FRAME_WORDS = 21'd307200;
    localparam [3:0] ST_IDLE = 0, ST_WAIT_MEMORY = 1,
        ST_WRITE_ACK = 2, ST_WRITE_RELEASE = 3, ST_FIFO_RELEASE = 4,
        ST_STREAM = 5, ST_COMMIT = 6, ST_READY_SWAP = 7, ST_SWAP = 8,
        ST_WAIT_TRANSITION = 9;

    reg [3:0] load_state;
    reg [18:0] pixels_accepted;
    reg [3:0] fifo_release_count;
    reg [9:0] clear_guard;
    reg write_request;
    reg back_index;
    reg swap_target;
    reg swap_toggle;
    reg clear_toggle;
    reg engine_reset_toggle;
    wire write_ack;
    wire write_finish;
    wire write_full;
    wire memory_init_done;
    wire memory_refresh_valid;
    wire memory_busy;
    wire memory_engine_reset;
    wire [20:0] app_write_addr;
    wire [31:0] app_write_data;
    wire [3:0] app_write_mask;
    wire app_write_enable;
    wire [20:0] app_read_addr;
    wire app_read_enable;
    wire memory_read_valid;
    wire [31:0] memory_read_data;

    // Request data (buffer index) remains stable until the returned toggle.
    reg [1:0] write_ack_sync;
    reg [1:0] memory_ready_sync;
    reg [1:0] commit_sync;
    reg [1:0] swap_ack_sync;
    (* ASYNC_REG="TRUE" *) reg [1:0] transition_busy_sync;
    (* ASYNC_REG="TRUE" *) reg [1:0] swap_transition_sync;
    reg pixel_swap_transition;
    reg transition_seen;
    reg load_start_pending;
    reg commit_seen;
    reg swap_ack_seen;
    reg commit_toggle;
    reg pixel_swap_ack;

    always @* begin
        load_ready = (load_state == ST_STREAM) && !write_full &&
                     (clear_guard == 0) && !clear;
    end

    always @(posedge sys_clk or posedge rst) begin
        if (rst) begin
            load_state <= ST_IDLE;
            pixels_accepted <= 0;
            fifo_release_count <= 0;
            clear_guard <= 10'h3ff;
            write_request <= 0;
            back_index <= 1;
            swap_target <= 1;
            swap_toggle <= 0;
            clear_toggle <= 0;
            engine_reset_toggle <= 0;
            write_ack_sync <= 0;
            memory_ready_sync <= 0;
            commit_sync <= 0;
            swap_ack_sync <= 0;
            transition_busy_sync <= 0;
            swap_transition_sync <= 0;
            transition_seen <= 0;
            load_start_pending <= 0;
            commit_seen <= 0;
            swap_ack_seen <= 0;
            load_done <= 0;
            swap_done <= 0;
        end else begin
            write_ack_sync <= {write_ack_sync[0], write_ack};
            memory_ready_sync <= {memory_ready_sync[0],
                                 memory_init_done && !memory_engine_reset};
            commit_sync <= {commit_sync[0], commit_toggle};
            swap_ack_sync <= {swap_ack_sync[0], pixel_swap_ack};
            transition_busy_sync <= {transition_busy_sync[0], transition_busy};
            swap_transition_sync <= {swap_transition_sync[0], pixel_swap_transition};
            load_done <= 0;
            swap_done <= 0;
            if (clear_guard != 0)
                clear_guard <= clear_guard - 1'b1;

            if (clear) begin
                clear_toggle <= ~clear_toggle;
                engine_reset_toggle <= ~engine_reset_toggle;
                clear_guard <= 10'h3ff;
                load_state <= ST_IDLE;
                write_request <= 0;
                pixels_accepted <= 0;
                back_index <= 1;
                swap_target <= 1;
                commit_seen <= commit_sync[1];
                swap_ack_seen <= swap_ack_sync[1];
                transition_seen <= 1'b0;
                load_start_pending <= 1'b0;
            end else if (load_cancel && load_state >= ST_WAIT_MEMORY &&
                         load_state <= ST_READY_SWAP) begin
                // Stop an incomplete back-buffer write without blanking the
                // currently visible front buffer. The next scene can then
                // load from pixel zero and still use the push transition.
                engine_reset_toggle <= ~engine_reset_toggle;
                clear_guard <= 10'h3ff;
                load_state <= ST_IDLE;
                write_request <= 0;
                pixels_accepted <= 0;
                commit_seen <= commit_sync[1];
                swap_ack_seen <= swap_ack_sync[1];
                load_start_pending <= 1'b0;
            end else begin
                // Keep completion snapshots current even after cancellation.
                if (clear_guard != 0) begin
                    commit_seen <= commit_sync[1];
                    swap_ack_seen <= swap_ack_sync[1];
                end
                case (load_state)
                    ST_IDLE: if (load_start || load_start_pending) begin
                        load_start_pending <= 1'b0;
                        load_state <= ST_WAIT_MEMORY;
                    end
                    ST_WAIT_MEMORY: if (memory_ready_sync[1] && clear_guard == 0) begin
                        commit_seen <= commit_sync[1];
                        write_request <= 1;
                        load_state <= ST_WRITE_ACK;
                    end
                    ST_WRITE_ACK: if (write_ack_sync[1]) begin
                        write_request <= 0;
                        load_state <= ST_WRITE_RELEASE;
                    end
                    ST_WRITE_RELEASE: if (!write_ack_sync[1]) begin
                        fifo_release_count <= 4'd8;
                        load_state <= ST_FIFO_RELEASE;
                    end
                    ST_FIFO_RELEASE: if (fifo_release_count == 0) begin
                        pixels_accepted <= 0;
                        load_state <= ST_STREAM;
                    end else fifo_release_count <= fifo_release_count - 1'b1;
                    ST_STREAM: if (pixel_valid && load_ready) begin
                        if (pixels_accepted == 19'd307199)
                            load_state <= ST_COMMIT;
                        else pixels_accepted <= pixels_accepted + 1'b1;
                    end
                    ST_COMMIT: if (commit_sync[1] != commit_seen) begin
                        commit_seen <= commit_sync[1];
                        load_done <= 1;
                        load_state <= ST_READY_SWAP;
                    end
                    ST_READY_SWAP: if (swap_request) begin
                        swap_target <= back_index;
                        swap_toggle <= ~swap_toggle;
                        swap_ack_seen <= swap_ack_sync[1];
                        load_state <= ST_SWAP;
                    end
                    ST_SWAP: if (swap_ack_sync[1] != swap_ack_seen) begin
                        swap_ack_seen <= swap_ack_sync[1];
                        if (swap_transition_sync[1]) begin
                            transition_seen <= transition_busy_sync[1];
                            load_state <= ST_WAIT_TRANSITION;
                        end else begin
                            back_index <= ~back_index;
                            swap_done <= 1;
                            load_state <= ST_IDLE;
                        end
                    end
                    ST_WAIT_TRANSITION: begin
                        if (load_start) load_start_pending <= 1'b1;
                        if (transition_busy_sync[1])
                            transition_seen <= 1'b1;
                        else if (transition_seen) begin
                            back_index <= ~back_index;
                            swap_done <= 1;
                            transition_seen <= 1'b0;
                            if (load_start_pending || load_start) begin
                                load_start_pending <= 1'b0;
                                load_state <= ST_WAIT_MEMORY;
                            end else load_state <= ST_IDLE;
                        end
                    end
                    default: load_state <= ST_IDLE;
                endcase
            end
        end
    end

    // Scene changes reset only the burst/FIFO engines. Keep SDRAM refresh
    // running. Stretch reset to flush old request synchronizers and returns.
    reg [1:0] engine_reset_sync;
    reg engine_reset_seen;
    reg [6:0] memory_reset_count;
    reg [5:0] commit_drain_count;
    reg commit_pending;
    assign memory_engine_reset = (memory_reset_count != 0);

    always @(posedge mem_clk or posedge rst) begin
        if (rst) begin
            engine_reset_sync <= 0;
            engine_reset_seen <= 0;
            memory_reset_count <= 7'd127;
            commit_drain_count <= 0;
            commit_pending <= 0;
            commit_toggle <= 0;
        end else begin
            engine_reset_sync <= {engine_reset_sync[0], engine_reset_toggle};
            if (engine_reset_sync[1] != engine_reset_seen) begin
                engine_reset_seen <= engine_reset_sync[1];
                memory_reset_count <= 7'd127;
                commit_pending <= 0;
                commit_drain_count <= 0;
            end else begin
                if (memory_reset_count != 0)
                    memory_reset_count <= memory_reset_count - 1'b1;
                // The official engine signals the final application write.
                // Allow its SDRAM pipeline to drain before returning load_done.
                if (write_finish && !memory_engine_reset) begin
                    commit_pending <= 1;
                    commit_drain_count <= 6'd32;
                end else if (commit_pending) begin
                    if (commit_drain_count != 0)
                        commit_drain_count <= commit_drain_count - 1'b1;
                    else if (!memory_busy) begin
                        commit_toggle <= ~commit_toggle;
                        commit_pending <= 0;
                    end
                end
            end
        end
    end

    // VGA raster. Blank starts at line 480, leaving 45 complete lines for
    // the front-buffer selection handshake and SDRAM read FIFO prefetch.
    reg [9:0] h_count;
    reg [9:0] v_count;
    wire active = h_count < 10'd640 && v_count < 10'd480;
    wire blank_start = h_count == 0 && v_count == 10'd480;
    reg front_index;
    reg front_visible;
    reg read_request;
    reg [5:0] transition_frame_count;
    reg [9:0] transition_offset;
    reg transition_old_index, transition_new_index;
    wire read_ack;
    wire [31:0] frame_pixel;
    reg [1:0] read_ack_sync;
    reg [1:0] swap_pixel_sync;
    reg [1:0] clear_pixel_sync;
    reg swap_pixel_seen;
    reg clear_pixel_seen;
    reg pending_swap;
    reg pending_target;
    reg [4:0] pixel_clear_guard;
    reg active_d;
    reg first_d;
    reg last_d;
    reg visible_d;
    wire [15:0] rgb565 = frame_pixel[15:0];
    wire [23:0] rgb888 = {rgb565[15:11], rgb565[15:13],
                         rgb565[10:5], rgb565[10:9],
                         rgb565[4:0], rgb565[4:2]};

    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            h_count <= 0;
            v_count <= 0;
            front_index <= 0;
            front_visible <= 0;
            read_request <= 0;
            transition_busy <= 0;
            transition_frame_count <= 0;
            transition_offset <= 0;
            transition_old_index <= 0;
            transition_new_index <= 0;
            pixel_swap_transition <= 0;
            read_ack_sync <= 0;
            swap_pixel_sync <= 0;
            clear_pixel_sync <= 0;
            swap_pixel_seen <= 0;
            clear_pixel_seen <= 0;
            pixel_swap_ack <= 0;
            pending_swap <= 0;
            pending_target <= 0;
            pixel_clear_guard <= 5'd31;
            active_d <= 0;
            first_d <= 0;
            last_d <= 0;
            visible_d <= 0;
            video_valid <= 0;
            visible_frame_start <= 0;
            video_user <= 0;
            video_last <= 0;
            video_data <= 0;
        end else begin
            if (h_count == 10'd799) begin
                h_count <= 0;
                if (v_count == 10'd524) v_count <= 0;
                else v_count <= v_count + 1'b1;
            end else h_count <= h_count + 1'b1;
            read_ack_sync <= {read_ack_sync[0], read_ack};
            swap_pixel_sync <= {swap_pixel_sync[0], swap_toggle};
            clear_pixel_sync <= {clear_pixel_sync[0], clear_toggle};
            active_d <= active;
            first_d <= h_count == 0 && v_count == 0;
            last_d <= h_count == 10'd639;
            visible_d <= front_visible;
            video_valid <= active_d;
            video_user <= active_d && first_d;
            visible_frame_start <= active_d && first_d && visible_d;
            video_last <= active_d && last_d;
            video_data <= visible_d && front_visible ? rgb888 : 24'd0;

            if (read_ack_sync[1]) read_request <= 0;
            if (clear_pixel_sync[1] != clear_pixel_seen) begin
                clear_pixel_seen <= clear_pixel_sync[1];
                front_index <= 0;
                front_visible <= 0;
                read_request <= 0;
                pending_swap <= 0;
                transition_busy <= 0;
                transition_frame_count <= 0;
                transition_offset <= 0;
                transition_old_index <= 0;
                transition_new_index <= 0;
                pixel_swap_transition <= 0;
                pixel_clear_guard <= 5'd31;
                swap_pixel_seen <= swap_pixel_sync[1];
                video_data <= 0;
                visible_frame_start <= 0;
            end else if (pixel_clear_guard != 0) begin
                pixel_clear_guard <= pixel_clear_guard - 1'b1;
                swap_pixel_seen <= swap_pixel_sync[1];
                read_request <= 0;
                front_visible <= 0;
                visible_frame_start <= 0;
            end else begin
                if (swap_pixel_sync[1] != swap_pixel_seen) begin
                    swap_pixel_seen <= swap_pixel_sync[1];
                    pending_swap <= 1;
                    // Bundled index was held stable before this synchronized
                    // event and stays stable until pixel_swap_ack returns.
                    pending_target <= swap_target;
                end
                if (blank_start) begin
                    if (pending_swap) begin
                        transition_old_index <= front_index;
                        transition_new_index <= pending_target;
                        pixel_swap_transition <= front_visible && transition_request;
                        if (front_visible && transition_request) begin
                            transition_busy <= 1'b1;
                            transition_frame_count <= 6'd0;
                            transition_offset <= 10'd0;
                        end else begin
                            transition_busy <= 1'b0;
                            transition_frame_count <= 6'd0;
                            transition_offset <= 10'd0;
                        end
                        front_index <= pending_target;
                        front_visible <= 1;
                        pending_swap <= 0;
                        pixel_swap_ack <= ~pixel_swap_ack;
                        read_request <= 1;
                    end else begin
                        if (front_visible) read_request <= 1;
                        if (transition_busy) begin
                            if (transition_frame_count == 6'd29) begin
                                transition_busy <= 1'b0;
                                transition_offset <= 10'd480;
                            end else begin
                                transition_frame_count <= transition_frame_count + 1'b1;
                                transition_offset <= transition_offset + 10'd16;
                            end
                        end
                    end
                end
            end
        end
    end

    mg_frame_read_write #(
        .MEM_DATA_BITS(32), .READ_DATA_BITS(32), .WRITE_DATA_BITS(32),
        .ADDR_BITS(21), .BURST_BITS(9), .BURST_SIZE(256), .READ_BURST_SIZE(64),
        .WRITE_V_FLIP(0), .FRAME_WIDTH(640), .FRAME_HEIGHT(480)
    ) u_frame_engine (
        .rst(rst | memory_engine_reset), .mem_clk(mem_clk),
        .Sdr_init_done(memory_init_done),
        .Sdr_init_ref_vld(memory_refresh_valid), .Sdr_busy(memory_busy),
        .App_rd_en(app_read_enable), .App_rd_addr(app_read_addr),
        .Sdr_rd_en(memory_read_valid), .Sdr_rd_dout(memory_read_data),
        .read_clk(pixel_clk), .read_req(read_request),
        .read_req_ack(read_ack), .read_finish(),
        .read_addr_0(21'd0), .read_addr_1(FRAME_WORDS),
        .read_addr_2(21'd0), .read_addr_3(FRAME_WORDS),
        .read_addr_index({1'b0, front_index}), .read_len(FRAME_WORDS),
        .read_push_mode(transition_busy), .read_push_offset(transition_offset),
        .read_push_old_index({1'b0, transition_old_index}),
        .read_push_new_index({1'b0, transition_new_index}),
        .read_en(active && front_visible && pixel_clear_guard == 0),
        .read_data(frame_pixel),
        .App_wr_en(app_write_enable), .App_wr_addr(app_write_addr),
        .App_wr_din(app_write_data), .App_wr_dm(app_write_mask),
        .write_clk(sys_clk), .write_req(write_request),
        .write_req_ack(write_ack), .write_finish(write_finish),
        .write_fifo_full(write_full),
        .write_addr_0(21'd0), .write_addr_1(FRAME_WORDS),
        .write_addr_2(21'd0), .write_addr_3(FRAME_WORDS),
        .write_addr_index({1'b0, back_index}), .write_len(FRAME_WORDS),
        .write_en(pixel_valid && load_ready),
        .write_data({16'd0, pixel_rgb565})
    );

    mg_sdram u_memory (
        .Clk(mem_clk), .Clk_sft(mem_clk_shift), .Rst(rst),
        .Sdr_init_done(memory_init_done),
        .Sdr_init_ref_vld(memory_refresh_valid), .Sdr_busy(memory_busy),
        .App_wr_en(app_write_enable), .App_wr_addr(app_write_addr),
        .App_wr_dm(app_write_mask), .App_wr_din(app_write_data),
        .App_rd_en(app_read_enable), .App_rd_addr(app_read_addr),
        .Sdr_rd_en(memory_read_valid), .Sdr_rd_dout(memory_read_data)
    );
endmodule
