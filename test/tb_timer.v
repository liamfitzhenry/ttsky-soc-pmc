// timer.v
// loadable down counter. expired is high when it reaches zero and stays there.

module timer #(
    parameter BITS = 8
) (
    input  wire            clk,
    input  wire            rst_n,
    input  wire            load,
    input  wire [BITS-1:0] value,
    output wire            expired
);

    reg [BITS-1:0] cnt;

    always @(posedge clk) begin
        if (!rst_n)
            cnt <= {BITS{1'b0}};
        else if (load)
            cnt <= value;
        else if (cnt != {BITS{1'b0}})
            cnt <= cnt - 1'b1;
    end

    assign expired = (cnt == {BITS{1'b0}});

endmodule