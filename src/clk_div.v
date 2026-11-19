module clk_div (
    input  wire clk,
    input  wire rst_n,
    input  wire [1:0] div_req,
    output reg  clk_out,
    output reg  [1:0] div_cur
);

    reg [2:0] cnt;
    reg [2:0] div_max;

    always @(*) begin
        case (div_cur)
            2'b11: div_max = 3'd0;
            2'b01: div_max = 3'd1;
            2'b00: div_max = 3'd3;
            default: div_max = 3'd3;
        endcase
    end

    wire at_end = (cnt == div_max);
    wire edge_now = at_end;

    always @(posedge clk) begin
        if (!rst_n) begin
            cnt     <= 3'd0;
            clk_out <= 1'b0;
            div_cur <= 2'b00;
        end else if (edge_now) begin
            cnt     <= 3'd0;
            clk_out <= ~clk_out;
            div_cur <= div_req;
        end else begin
            cnt <= cnt + 1'b1;
        end
    end

endmodule