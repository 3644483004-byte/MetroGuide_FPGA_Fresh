// Validate the project's canonical 44-byte WAV header and unpack stereo PCM.
module wav_pcm_ingest (
    input wire clk, rst,
    input wire [31:0] file_size,
    input wire [7:0] data,
    input wire data_valid,
    input wire [31:0] byte_offset,
    output reg header_ready,
    output reg [31:0] total_samples,
    output reg format_error,
    output reg sample_write,
    output reg [31:0] sample_data
);
    reg header_bad;
    reg [31:0] data_size, riff_size, expected_offset;
    reg [1:0] lane;
    reg [23:0] assembler;
    wire [31:0] completed_size={data,data_size[23:0]};
    function bad_header_byte;
        input [5:0] offset;
        input [7:0] value;
        begin
            case(offset)
                0: bad_header_byte=value!="R";
                1: bad_header_byte=value!="I";
                2,3: bad_header_byte=value!="F";
                8: bad_header_byte=value!="W";
                9: bad_header_byte=value!="A";
                10: bad_header_byte=value!="V";
                11: bad_header_byte=value!="E";
                12: bad_header_byte=value!="f";
                13: bad_header_byte=value!="m";
                14: bad_header_byte=value!="t";
                15: bad_header_byte=value!=8'h20;
                16,34: bad_header_byte=value!=8'h10;
                20: bad_header_byte=value!=8'h01;
                22,30: bad_header_byte=value!=8'h02;
                24: bad_header_byte=value!=8'h80;
                25: bad_header_byte=value!=8'hBB;
                29: bad_header_byte=value!=8'hEE;
                32: bad_header_byte=value!=8'h04;
                36: bad_header_byte=value!="d";
                37,39: bad_header_byte=value!="a";
                38: bad_header_byte=value!="t";
                17,18,19,21,23,26,27,28,31,33,35:
                    bad_header_byte=value!=8'h00;
                default: bad_header_byte=1'b0;
            endcase
        end
    endfunction
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            header_ready<=0; total_samples<=0; format_error<=0;
            sample_write<=0; sample_data<=0; header_bad<=0;
            data_size<=0; riff_size<=0; expected_offset<=0; lane<=0; assembler<=0;
        end else begin
            sample_write<=0;
            if(data_valid && !format_error) begin
                expected_offset<=expected_offset+1'b1;
                if(byte_offset!=expected_offset) begin
                    format_error<=1; header_ready<=0;
                end else if(byte_offset<44) begin
                    if(bad_header_byte(byte_offset[5:0],data)) header_bad<=1;
                    case(byte_offset[5:0])
                        4: riff_size[7:0]<=data;
                        5: riff_size[15:8]<=data;
                        6: riff_size[23:16]<=data;
                        7: riff_size[31:24]<=data;
                        40: data_size[7:0]<=data;
                        41: data_size[15:8]<=data;
                        42: data_size[23:16]<=data;
                        43: begin
                            data_size[31:24]<=data;
                            if(header_bad || completed_size==0 || completed_size[1:0]!=0 ||
                               completed_size>32'hFFFFFFD3 || file_size!=completed_size+32'd44 ||
                               file_size<44 || riff_size!=file_size-32'd8) begin
                                format_error<=1; header_ready<=0;
                            end else begin
                                header_ready<=1; total_samples<=completed_size>>2;
                            end
                        end
                        default: begin end
                    endcase
                end else if(header_ready) begin
                    case(lane)
                        0: assembler[7:0]<=data;
                        1: assembler[15:8]<=data;
                        2: assembler[23:16]<=data;
                        3: begin sample_data<={data,assembler}; sample_write<=1; end
                    endcase
                    lane<=lane+1'b1;
                end
            end
        end
    end
endmodule
