// Synchronize and debounce one active-high or active-low mechanical input.
// press_pulse is asserted for one clk cycle on a debounced press only.
module button_debounce #(
    parameter integer ACTIVE_LOW       = 1,
    parameter integer DEBOUNCE_CYCLES  = 500_000
) (
    input  wire clk,
    input  wire rst,
    input  wire pin_in,
    output reg  press_pulse
);

    reg sync_ff1;
    reg sync_ff2;
    reg stable_pressed;
    reg [19:0] counter;

    wire pin_pressed = ACTIVE_LOW ? ~sync_ff2 : sync_ff2;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sync_ff1      <= ACTIVE_LOW ? 1'b1 : 1'b0;
            sync_ff2      <= ACTIVE_LOW ? 1'b1 : 1'b0;
            stable_pressed <= 1'b0;
            counter       <= 20'd0;
            press_pulse   <= 1'b0;
        end else begin
            sync_ff1    <= pin_in;
            sync_ff2    <= sync_ff1;
            press_pulse <= 1'b0;

            if (pin_pressed == stable_pressed) begin
                counter <= 20'd0;
            end else if (counter >= DEBOUNCE_CYCLES - 1) begin
                stable_pressed <= pin_pressed;
                counter <= 20'd0;
                if (pin_pressed)
                    press_pulse <= 1'b1;
            end else begin
                counter <= counter + 1'b1;
            end
        end
    end

endmodule
