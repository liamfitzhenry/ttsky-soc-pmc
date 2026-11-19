// sync.v
// two flop synchronizer. every pin input goes through one.

module sync #(
    parameter RESET_VAL = 1'b0
) (
    input  wire clk,
    input  wire rst_n,
    input  wire d_async,
    output wire d_sync
);

    reg meta;
    reg stable;

    always @(posedge clk) begin
        if (!rst_n) begin
            meta   <= RESET_VAL;
            stable <= RESET_VAL;
        end else begin
            meta   <= d_async;
            stable <= meta;
        end
    end

    assign d_sync = stable;

endmodule