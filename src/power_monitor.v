// Power monitor, estimates SoC power in microwatts.
// P = alpha * C * V^2 * f + P_leak
//   alpha = 0.2, C = 50 pF  ->  alpha*C = 10 pF
//   leakage: 10 uW (LOW), 20 uW (NORMAL), 30 uW (HIGH)
// Table values are the equation worked out for each VID/clock pair, rounded to whole uW.
// Lookup instead of multiply/divide to keep the logic small for one tile.
//
// VID:      00 = 0.6 V (LOW), 01 = 0.9 V (NORMAL), 11 = 1.1 V (HIGH)
// clk_sel:  11 = 25 MHz (base), 01 = 6.25 MHz (/4), 00 = 3.125 MHz (/8)

module power_monitor (
    input  wire [1:0] vid,      // VID actually sent to the regulator
    input  wire [1:0] clk_sel,  // clock select actually used
    input  wire       ready,    // rails up and clocks running
    output reg  [8:0] power_uw  // estimated power in uW (max 333)
);

    always @(*) begin
        if (!ready) begin
            power_uw = 9'd0;
        end else begin
            case ({vid, clk_sel})
                4'b00_00: power_uw = 9'd21;    // LOW    @ 3.125 MHz
                4'b00_01: power_uw = 9'd33;    // LOW    @ 6.25 MHz
                4'b00_11: power_uw = 9'd100;   // LOW    @ 25 MHz
                4'b01_00: power_uw = 9'd45;    // NORMAL @ 3.125 MHz
                4'b01_01: power_uw = 9'd71;    // NORMAL @ 6.25 MHz
                4'b01_11: power_uw = 9'd223;   // NORMAL @ 25 MHz
                4'b11_00: power_uw = 9'd68;    // HIGH   @ 3.125 MHz
                4'b11_01: power_uw = 9'd106;   // HIGH   @ 6.25 MHz
                4'b11_11: power_uw = 9'd333;   // HIGH   @ 25 MHz
                default:  power_uw = 9'd21;    // undriven codes, fall back to the safest point
            endcase
        end
    end

endmodule