// Energy accumulator.
// Each tick adds power_uw to a running total. If the power is in uW and the
// tick period is 1 ms, then one count = 1 nJ.
// Saturates at 2^24 - 1 instead of wrapping.

module energy_acc (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick,
    input  wire [8:0]  power_uw,
    input  wire        clr,
    output reg  [23:0] energy
);

    wire [24:0] sum = {1'b0, energy} + {16'b0, power_uw};

    always @(posedge clk) begin
        if (!rst_n || clr)
            energy <= 24'd0;
        else if (tick) begin
            if (sum[24])
                energy <= 24'hFFFFFF;
            else
                energy <= sum[23:0];
        end
    end

endmodule