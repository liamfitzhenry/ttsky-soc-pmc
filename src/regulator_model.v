// fake regulator rails for the testbenches
// pg follows en after DLY cycles
// kill_x  : pg held low  (rail never comes up, or drops out)
// stuck_x : pg held high (rail wont discharge)

module regulator_model #(parameter DLY = 6) (
    input  wire clk,
    input  wire rst_n,
    input  wire en_a,
    input  wire en_b,
    input  wire en_c,
    input  wire kill_a,
    input  wire kill_b,
    input  wire kill_c,
    input  wire stuck_a,
    input  wire stuck_b,
    input  wire stuck_c,
    output reg  pg_a,
    output reg  pg_b,
    output reg  pg_c
);

    reg [7:0] cnt_a, cnt_b, cnt_c;

    always @(posedge clk) begin
        if (!rst_n || kill_a) begin
            pg_a  <= 1'b0;
            cnt_a <= 8'd0;
        end else if (stuck_a) begin
            pg_a  <= 1'b1;
            cnt_a <= 8'd0;
        end else if (en_a != pg_a) begin
            if (cnt_a >= DLY) begin
                pg_a  <= en_a;
                cnt_a <= 8'd0;
            end else
                cnt_a <= cnt_a + 8'd1;
        end else
            cnt_a <= 8'd0;
    end

    always @(posedge clk) begin
        if (!rst_n || kill_b) begin
            pg_b  <= 1'b0;
            cnt_b <= 8'd0;
        end else if (stuck_b) begin
            pg_b  <= 1'b1;
            cnt_b <= 8'd0;
        end else if (en_b != pg_b) begin
            if (cnt_b >= DLY) begin
                pg_b  <= en_b;
                cnt_b <= 8'd0;
            end else
                cnt_b <= cnt_b + 8'd1;
        end else
            cnt_b <= 8'd0;
    end

    always @(posedge clk) begin
        if (!rst_n || kill_c) begin
            pg_c  <= 1'b0;
            cnt_c <= 8'd0;
        end else if (stuck_c) begin
            pg_c  <= 1'b1;
            cnt_c <= 8'd0;
        end else if (en_c != pg_c) begin
            if (cnt_c >= DLY) begin
                pg_c  <= en_c;
                cnt_c <= 8'd0;
            end else
                cnt_c <= cnt_c + 8'd1;
        end else
            cnt_c <= 8'd0;
    end

endmodule
