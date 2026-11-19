// Fault log.
// On each fault entry (fault_pulse), records the cause and bumps a counter.
// Cause codes (from the T7 fault list):
//   0 PG timeout, 1 PG loss, 2 overcurrent, 3 critical thermal,
//   4 VREG_ACK timeout, 5 PG stuck after disable, 6 illegal state
// Count saturates at 255.

module fault_log (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       fault_pulse,
    input  wire [2:0] cause,
    output reg  [2:0] last_cause,
    output reg  [7:0] count
);

    always @(posedge clk) begin
        if (!rst_n) begin
            last_cause <= 3'd0;
            count      <= 8'd0;
        end else if (fault_pulse) begin
            last_cause <= cause;
            if (count != 8'hFF)
                count <= count + 8'd1;
        end
    end

endmodule