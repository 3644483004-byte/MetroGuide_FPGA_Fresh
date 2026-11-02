// Read-only SD memory host, native four-bit mode, 100 MHz system clock.
// Identification clock: 400 kHz. Transfer clock: 25 MHz (default speed).
// External pull-ups are required on CMD and DAT[3:0]. DAT lines are never
// driven by this read-only host. Initialization is retried after card removal/error.
module sd_native_reader (
    input wire clk, input wire rst,
    output reg card_ready, output reg card_error,
    output wire sd_clk, inout wire sd_cmd, input wire [3:0] sd_dat,
    input wire read_start, input wire [31:0] read_lba,
    input wire [5:0] read_blocks,
    output reg read_busy, output reg [7:0] read_data,
    output reg read_valid, output reg block_end,
    output reg read_done, output reg read_error
);
    // Retry initialization after a removed/unpowered card. A TF card that was
    // previously in SPI mode must be power-cycled once (remove/reinsert it).
    reg [26:0] retry_count;
    reg [5:0] retry_reset_count;
    wire link_reset=rst || retry_reset_count!=0;
    always @(posedge clk or posedge rst) begin
        if(rst) begin retry_count<=0;retry_reset_count<=0;end
        else if(retry_reset_count!=0) begin
            retry_reset_count<=retry_reset_count-1'b1;retry_count<=0;
        end else if(card_error) begin
            if(retry_count==27'd99999999) begin retry_reset_count<=32;retry_count<=0;end
            else retry_count<=retry_count+1'b1;
        end else retry_count<=0;
    end

    // The clock generator only toggles on system-clock edges. Commands are
    // changed on falling SD edges; CMD/DAT are sampled on rising SD edges.
    reg fast_clock;
    reg [7:0] clock_count;
    reg card_clock;
    wire clock_tick = (clock_count >= (fast_clock ? 8'd1 : 8'd124));
    wire sd_rise = clock_tick && !card_clock;
    wire sd_fall = clock_tick && card_clock;
    assign sd_clk = card_clock;
    always @(posedge clk or posedge link_reset) begin
        if (link_reset) begin clock_count <= 0; card_clock <= 0; end
        else if (clock_tick) begin
            clock_count <= 0;
            card_clock <= !card_clock;
        end else clock_count <= clock_count + 1'b1;
    end

    localparam RESP_NONE=2'd0, RESP_SHORT=2'd1,
               RESP_LONG=2'd2, RESP_OCR=2'd3;
    reg command_go;
    reg [5:0] command_index;
    reg [31:0] command_argument;
    reg [1:0] command_response;
    wire command_idle, command_done, command_error, command_timeout;
    wire [135:0] response;
    wire [31:0] short_payload = response[39:8];
    wire r1_error = |(short_payload & 32'hFFF98008);
    sd_native_command command_engine (
        .clk(clk), .rst(link_reset), .sd_rise(sd_rise), .sd_fall(sd_fall),
        .sd_cmd(sd_cmd), .go(command_go), .index(command_index),
        .argument(command_argument), .response_kind(command_response),
        .idle(command_idle), .done(command_done), .error(command_error),
        .timed_out(command_timeout), .response(response)
    );

    // Receive one or more native four-bit blocks. There are 1,024 payload
    // nibbles, followed by 16 independent CRC bits per lane and an end bit.
    localparam RX_IDLE=3'd0, RX_START=3'd1, RX_DATA=3'd2,
               RX_CRC=3'd3, RX_END=3'd4;
    reg [2:0] rx_state;
    reg receive_arm, receive_cancel;
    reg [5:0] blocks_latched, blocks_received;
    reg [9:0] nibble_count;
    reg [3:0] upper_nibble, crc_bit;
    reg [15:0] crc0, crc1, crc2, crc3;
    reg crc_mismatch;
    reg receive_complete, receive_fault;
    reg [22:0] data_wait;
    function [15:0] crc16_next;
        input [15:0] prior;
        input bit_in;
        begin
            crc16_next = {prior[14:0],1'b0} ^
                         ((prior[15] ^ bit_in) ? 16'h1021 : 16'h0000);
        end
    endfunction
    always @(posedge clk or posedge link_reset) begin
        if (link_reset) begin
            rx_state <= RX_IDLE; blocks_received <= 0;
            nibble_count <= 0; upper_nibble <= 0; crc_bit <= 0;
            crc0 <= 0; crc1 <= 0; crc2 <= 0; crc3 <= 0;
            crc_mismatch <= 0; receive_complete <= 0; receive_fault <= 0;
            data_wait <= 0; read_data <= 0; read_valid <= 0; block_end <= 0;
        end else begin
            read_valid <= 0;
            block_end <= 0;
            receive_complete <= 0;
            receive_fault <= 0;
            if (receive_cancel) rx_state <= RX_IDLE;
            else if (receive_arm) begin
                rx_state <= RX_START; blocks_received <= 0; data_wait <= 0;
                crc0 <= 0; crc1 <= 0; crc2 <= 0; crc3 <= 0;
                crc_mismatch <= 0; nibble_count <= 0;
            end else if (sd_rise) begin
                case (rx_state)
                    RX_IDLE: begin end
                    RX_START: begin
                        if (sd_dat == 4'b0000) begin
                            rx_state <= RX_DATA; nibble_count <= 0;
                            crc0 <= 0; crc1 <= 0; crc2 <= 0; crc3 <= 0;
                            crc_mismatch <= 0; data_wait <= 0;
                        end else if (!sd_dat[0] || data_wait == 23'd6249999) begin
                            // A malformed start bit or 250 ms data timeout.
                            receive_fault <= 1; rx_state <= RX_IDLE;
                        end else data_wait <= data_wait + 1'b1;
                    end
                    RX_DATA: begin
                        crc0 <= crc16_next(crc0, sd_dat[0]);
                        crc1 <= crc16_next(crc1, sd_dat[1]);
                        crc2 <= crc16_next(crc2, sd_dat[2]);
                        crc3 <= crc16_next(crc3, sd_dat[3]);
                        if (!nibble_count[0]) upper_nibble <= sd_dat;
                        else begin
                            read_data <= {upper_nibble,sd_dat};
                            read_valid <= 1;
                        end
                        if (nibble_count == 10'd1023) begin
                            rx_state <= RX_CRC; crc_bit <= 0;
                        end else nibble_count <= nibble_count + 1'b1;
                    end
                    RX_CRC: begin
                        if (sd_dat != {crc3[15-crc_bit],crc2[15-crc_bit],
                                       crc1[15-crc_bit],crc0[15-crc_bit]})
                            crc_mismatch <= 1;
                        if (crc_bit == 4'd15) rx_state <= RX_END;
                        else crc_bit <= crc_bit + 1'b1;
                    end
                    RX_END: begin
                        if (crc_mismatch || sd_dat != 4'b1111) begin
                            receive_fault <= 1; rx_state <= RX_IDLE;
                        end else begin
                            block_end <= 1;
                            blocks_received <= blocks_received + 1'b1;
                            if (blocks_received + 1'b1 == blocks_latched) begin
                                receive_complete <= 1; rx_state <= RX_IDLE;
                            end else begin rx_state <= RX_START; data_wait <= 0; end
                        end
                    end
                    default: rx_state <= RX_IDLE;
                endcase
            end
        end
    end

    localparam POWER=6'd0, START_CLOCKS=6'd1,
        CMD0=6'd2, WAIT0=6'd3, CMD8=6'd4, WAIT8=6'd5,
        CMD55_INIT=6'd6, WAIT55_INIT=6'd7, ACMD41=6'd8, WAIT41=6'd9,
        CMD2=6'd10, WAIT2=6'd11, CMD3=6'd12, WAIT3=6'd13,
        CMD7=6'd14, WAIT7=6'd15, BUSY7=6'd16,
        CMD16=6'd17, WAIT16=6'd18, CMD55_BUS=6'd19, WAIT55_BUS=6'd20,
        ACMD6=6'd21, WAIT6=6'd22, READY=6'd23,
        READ_COMMAND=6'd24, READ_RESPONSE=6'd25, READ_TRANSFER=6'd26,
        STOP_COMMAND=6'd27, STOP_RESPONSE=6'd28, STOP_BUSY=6'd29,
        READ_FINISH=6'd30, FATAL=6'd31;
    reg [5:0] state;
    reg [27:0] initialization_timer;
    reg [19:0] power_timer;
    reg [6:0] initial_clocks;
    reg version2, high_capacity;
    reg [15:0] relative_address;
    reg [31:0] lba_latched;
    reg transaction_failed, data_finished;
    reg [3:0] busy_guard;
    reg [23:0] busy_timer;
    // Host-wide initialization deadline: two seconds at 100 MHz.
    always @(posedge clk or posedge link_reset) begin
        if (link_reset) begin
            fast_clock <= 0; state <= POWER; initialization_timer <= 0;
            power_timer <= 0; initial_clocks <= 0;
            version2 <= 0; high_capacity <= 0; relative_address <= 0;
            card_ready <= 0; card_error <= 0; read_busy <= 0;
            read_done <= 0; read_error <= 0; command_go <= 0;
            command_index <= 0; command_argument <= 0;
            command_response <= RESP_NONE;
            receive_arm <= 0; receive_cancel <= 0;
            blocks_latched <= 0; lba_latched <= 0;
            transaction_failed <= 0; data_finished <= 0;
            busy_guard <= 0; busy_timer <= 0;
        end else begin
            command_go <= 0; receive_arm <= 0; receive_cancel <= 0;
            read_done <= 0; read_error <= 0;
            if (!card_ready && state != FATAL)
                initialization_timer <= initialization_timer + 1'b1;
            // Latch receive events even during the read-command response.
            // A fast card may start its payload before the CMD response
            // engine has completed its eight-clock command guard interval.
            if (receive_complete) data_finished <= 1;
            if (receive_fault) begin
                transaction_failed <= 1; data_finished <= 1;
            end
            case (state)
                POWER: begin
                    if (power_timer == 20'd999999) state <= START_CLOCKS;
                    else power_timer <= power_timer + 1'b1;
                end
                START_CLOCKS: if (sd_rise) begin
                    if (initial_clocks == 7'd79) state <= CMD0;
                    else initial_clocks <= initial_clocks + 1'b1;
                end
                CMD0: if (command_idle) begin
                    command_index <= 0; command_argument <= 0;
                    command_response <= RESP_NONE; command_go <= 1; state <= WAIT0;
                end
                WAIT0: if (command_done) state <= CMD8;
                CMD8: if (command_idle) begin
                    command_index <= 8; command_argument <= 32'h000001AA;
                    command_response <= RESP_SHORT; command_go <= 1; state <= WAIT8;
                end
                WAIT8: if (command_done) begin
                    if (command_timeout) begin version2 <= 0; state <= CMD55_INIT; end
                    else if (command_error) state <= FATAL;
                    else if (short_payload == 32'h000001AA) begin
                        version2 <= 1; state <= CMD55_INIT;
                    end else if (short_payload[22]) begin
                        version2 <= 0; state <= CMD55_INIT;
                    end else state <= FATAL;
                end
                CMD55_INIT: if (command_idle) begin
                    command_index <= 55; command_argument <= 0;
                    command_response <= RESP_SHORT; command_go <= 1; state <= WAIT55_INIT;
                end
                WAIT55_INIT: if (command_done) begin
                    if (command_error || r1_error || !short_payload[5]) state <= FATAL;
                    else state <= ACMD41;
                end
                ACMD41: if (command_idle) begin
                    command_index <= 41;
                    command_argument <= version2 ? 32'h40FF8000 : 32'h00FF8000;
                    command_response <= RESP_OCR; command_go <= 1; state <= WAIT41;
                end
                WAIT41: if (command_done) begin
                    if (command_error || !(|(short_payload & 32'h00FF8000))) state <= FATAL;
                    else if (short_payload[31]) begin
                        high_capacity <= version2 && short_payload[30]; state <= CMD2;
                    end else state <= CMD55_INIT;
                end
                CMD2: if (command_idle) begin
                    command_index <= 2; command_argument <= 0;
                    command_response <= RESP_LONG; command_go <= 1; state <= WAIT2;
                end
                WAIT2: if (command_done) begin
                    if (command_error) state <= FATAL; else state <= CMD3;
                end
                CMD3: if (command_idle) begin
                    command_index <= 3; command_argument <= 0;
                    command_response <= RESP_SHORT; command_go <= 1; state <= WAIT3;
                end
                WAIT3: if (command_done) begin
                    // R6 has a 16-bit status with CRC/illegal/error at 15:13.
                    if (command_error || (|short_payload[15:13]) ||
                        short_payload[31:16] == 0) state <= FATAL;
                    else begin relative_address <= short_payload[31:16]; state <= CMD7; end
                end
                CMD7: if (command_idle) begin
                    command_index <= 7; command_argument <= {relative_address,16'd0};
                    command_response <= RESP_SHORT; command_go <= 1; state <= WAIT7;
                end
                WAIT7: if (command_done) begin
                    if (command_error || r1_error) state <= FATAL;
                    else begin state <= BUSY7; busy_guard <= 0; busy_timer <= 0; end
                end
                BUSY7: if (sd_rise) begin
                    if (busy_guard != 4'd8) busy_guard <= busy_guard + 1'b1;
                    else if (sd_dat[0]) state <= high_capacity ? CMD55_BUS : CMD16;
                    if (busy_timer == 24'd12499999) state <= FATAL;
                    else busy_timer <= busy_timer + 1'b1;
                end
                CMD16: if (command_idle) begin
                    command_index <= 16; command_argument <= 512;
                    command_response <= RESP_SHORT; command_go <= 1; state <= WAIT16;
                end
                WAIT16: if (command_done) begin
                    if (command_error || r1_error) state <= FATAL; else state <= CMD55_BUS;
                end
                CMD55_BUS: if (command_idle) begin
                    command_index <= 55; command_argument <= {relative_address,16'd0};
                    command_response <= RESP_SHORT; command_go <= 1; state <= WAIT55_BUS;
                end
                WAIT55_BUS: if (command_done) begin
                    if (command_error || r1_error || !short_payload[5]) state <= FATAL;
                    else state <= ACMD6;
                end
                ACMD6: if (command_idle) begin
                    command_index <= 6; command_argument <= 2;
                    command_response <= RESP_SHORT; command_go <= 1; state <= WAIT6;
                end
                WAIT6: if (command_done) begin
                    if (command_error || r1_error) state <= FATAL;
                    else begin fast_clock <= 1; card_ready <= 1; state <= READY; end
                end
                READY: if (read_start) begin
                    if (read_blocks == 0 || read_blocks > 32 ||
                        (!high_capacity && (|read_lba[31:23]))) begin
                        read_error <= 1; read_done <= 1;
                    end else begin
                        blocks_latched <= read_blocks; lba_latched <= read_lba;
                        read_busy <= 1; transaction_failed <= 0; data_finished <= 0;
                        state <= READ_COMMAND;
                    end
                end
                READ_COMMAND: if (command_idle) begin
                    command_index <= (blocks_latched == 1) ? 17 : 18;
                    command_argument <= high_capacity ? lba_latched : (lba_latched << 9);
                    command_response <= RESP_SHORT; command_go <= 1;
                    receive_arm <= 1; state <= READ_RESPONSE;
                end
                READ_RESPONSE: if (command_done) begin
                    if (command_error) begin
                        // A missing/corrupt response leaves the card transfer
                        // state uncertain. Require reset, rather than emit a
                        // successful transaction or attempt a blind next read.
                        transaction_failed <= 1; receive_cancel <= 1;
                        state <= FATAL;
                    end else if (r1_error) begin
                        transaction_failed <= 1; receive_cancel <= 1;
                        state <= READ_FINISH;
                    end else state <= READ_TRANSFER;
                end
                READ_TRANSFER: if (data_finished || receive_complete || receive_fault) begin
                    if (blocks_latched == 1) state <= READ_FINISH;
                    else state <= STOP_COMMAND;
                end
                STOP_COMMAND: if (command_idle) begin
                    receive_cancel <= 1;
                    command_index <= 12; command_argument <= 0;
                    command_response <= RESP_SHORT; command_go <= 1; state <= STOP_RESPONSE;
                end
                STOP_RESPONSE: if (command_done) begin
                    if (command_error || r1_error) begin
                        transaction_failed <= 1; state <= FATAL;
                    end else begin state <= STOP_BUSY; busy_guard <= 0; busy_timer <= 0; end
                end
                STOP_BUSY: if (sd_rise) begin
                    if (busy_guard != 4'd8) busy_guard <= busy_guard + 1'b1;
                    else if (sd_dat[0]) state <= READ_FINISH;
                    if (busy_timer == 24'd12499999) begin
                        transaction_failed <= 1; state <= FATAL;
                    end else busy_timer <= busy_timer + 1'b1;
                end
                READ_FINISH: begin
                    read_busy <= 0; read_done <= 1;
                    read_error <= transaction_failed || receive_fault;
                    if (transaction_failed || receive_fault) begin
                        // A failed data block can leave the card/session out of
                        // sync even when CMD18/CMD12 completed. Drop card_ready
                        // so the media layer flushes both streams, then let the
                        // host retry logic reinitialize the card automatically.
                        card_ready <= 0; card_error <= 1; state <= FATAL;
                    end else state <= READY;
                end
                FATAL: begin
                    card_ready <= 0; card_error <= 1; receive_cancel <= 1;
                    if (read_busy) begin read_done <= 1; read_error <= 1; end
                    read_busy <= 0;
                end
                default: state <= FATAL;
            endcase
            if (!card_ready && state != FATAL && initialization_timer == 28'd199999999)
                state <= FATAL;
        end
    end
endmodule

// Native CMD framing/response engine. R2 CRC covers the 120 CID/CSD payload
// bits, excluding its eight leading response-header bits. R3 has no CRC.
module sd_native_command (
    input wire clk, input wire rst, input wire sd_rise, input wire sd_fall,
    inout wire sd_cmd,
    input wire go, input wire [5:0] index, input wire [31:0] argument,
    input wire [1:0] response_kind,
    output wire idle, output reg done, output reg error, output reg timed_out,
    output reg [135:0] response
);
    localparam IDLE=3'd0, SEND=3'd1, RELEASE=3'd2,
               WAIT_START=3'd3, RECEIVE=3'd4, CHECK=3'd5, GUARD=3'd6;
    reg [2:0] state;
    reg cmd_output, cmd_enable;
    reg [47:0] tx;
    reg [5:0] tx_position, expected_index;
    reg [1:0] response_type;
    reg [7:0] response_position, response_last, response_wait;
    reg [6:0] response_crc;
    reg [3:0] guard_clocks;
    assign sd_cmd = cmd_enable ? cmd_output : 1'bz;
    assign idle = (state == IDLE);
    function [6:0] crc7_next;
        input [6:0] prior;
        input bit_in;
        begin
            crc7_next = {prior[5:0],1'b0} ^
                        ((prior[6] ^ bit_in) ? 7'h09 : 7'h00);
        end
    endfunction
    function [6:0] command_crc;
        input [39:0] body;
        integer bit_number;
        reg [6:0] work_crc;
        begin
            work_crc = 0;
            for (bit_number=39; bit_number>=0; bit_number=bit_number-1)
                work_crc = crc7_next(work_crc,body[bit_number]);
            command_crc = work_crc;
        end
    endfunction
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE; cmd_output <= 1; cmd_enable <= 0;
            tx <= 0; tx_position <= 0; expected_index <= 0; response_type <= 0;
            response_position <= 0; response_last <= 0; response_wait <= 0;
            response_crc <= 0; guard_clocks <= 0;
            response <= 0; done <= 0; error <= 0; timed_out <= 0;
        end else begin
            done <= 0;
            case (state)
                IDLE: if (go) begin
                    tx <= {2'b01,index,argument,command_crc({2'b01,index,argument}),1'b1};
                    tx_position <= 47; expected_index <= index;
                    response_type <= response_kind;
                    response_last <= (response_kind == 2) ? 8'd135 : 8'd47;
                    response <= 0; response_crc <= 0; response_wait <= 0;
                    error <= 0; timed_out <= 0; guard_clocks <= 0;
                    state <= SEND;
                end
                SEND: if (sd_fall) begin
                    cmd_enable <= 1; cmd_output <= tx[tx_position];
                    if (tx_position == 0) state <= RELEASE;
                    else tx_position <= tx_position - 1'b1;
                end
                RELEASE: if (sd_fall) begin
                    cmd_enable <= 0;
                    state <= (response_type == 0) ? GUARD : WAIT_START;
                end
                WAIT_START: if (sd_rise) begin
                    if (sd_cmd == 1'b0) begin
                        response <= 0; response_position <= 1;
                        // CRC of the first response start bit (zero) is zero.
                        response_crc <= 0; state <= RECEIVE;
                    end else if (response_wait == 8'd127) begin
                        error <= 1; timed_out <= 1; state <= GUARD;
                    end else response_wait <= response_wait + 1'b1;
                end
                RECEIVE: if (sd_rise) begin
                    response <= {response[134:0],sd_cmd};
                    if ((response_type == 2 && response_position >= 8 && response_position < 128) ||
                        (response_type == 1 && response_position < 40))
                        response_crc <= crc7_next(response_crc,sd_cmd);
                    if (response_position == response_last) state <= CHECK;
                    else response_position <= response_position + 1'b1;
                end
                CHECK: begin
                    if (!response[0]) error <= 1;
                    if (response_type == 2) begin
                        if (response[135:128] != 8'h3F || response[7:1] != response_crc)
                            error <= 1;
                    end else if (response_type == 1) begin
                        if (response[47:46] != 2'b00 || response[45:40] != expected_index ||
                            response[7:1] != response_crc) error <= 1;
                    end else if (response[47:40] != 8'h3F) error <= 1;
                    state <= GUARD;
                end
                GUARD: if (sd_rise) begin
                    if (guard_clocks == 4'd7) begin done <= 1; state <= IDLE; end
                    else guard_clocks <= guard_clocks + 1'b1;
                end
                default: begin state <= IDLE; cmd_enable <= 0; error <= 1; end
            endcase
        end
    end
endmodule
