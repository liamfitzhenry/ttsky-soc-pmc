`timescale 1ns/1ps

module tb_dvfs_ctrl;

    reg clk = 0;
    reg rst_n = 0;
    reg [1:0] dvfs_req = 2'b00;
    reg throttled = 0;
    reg fsm_fault = 0;
    reg vreg_ack = 0;
    reg [1:0] clk_hi_ack = 2'b00;

    wire [1:0] v_code;
    wire [1:0] clk_div_code;
    wire dvfs_busy;
    wire dvfs_fault;

    integer errors = 0;
    integer n;

    always #5 clk = ~clk;

    dvfs_ctrl dut (
        .clk(clk),
        .rst_n(rst_n),
        .dvfs_req(dvfs_req),
        .throttled(throttled),
        .fsm_fault(fsm_fault),
        .vreg_ack(vreg_ack),
        .clk_hi_ack(clk_hi_ack),
        .v_code(v_code),
        .clk_div_code(clk_div_code),
        .dvfs_busy(dvfs_busy),
        .dvfs_fault(dvfs_fault)
    );

    task step;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    task check;
        input cond;
        input [8*48-1:0] msg;
        begin
            if (!cond) begin
                errors = errors + 1;
                $display("FAIL @%0d ns: %0s", $time, msg);
            end
        end
    endtask

    task wait_idle;
        integer k;
        begin
            k = 0;
            step;
            while (dvfs_busy === 1'b1 && k < 400) begin
                step;
                k = k + 1;
            end
            check(k < 400, "timed out waiting for dvfs_busy to drop");
        end
    endtask

    task ack_vreg;
        begin
            repeat (5) step;
            vreg_ack = 1;
            step;
            vreg_ack = 0;
        end
    endtask

    task ack_clk;
        input [1:0] code;
        begin
            repeat (2) step;
            clk_hi_ack = code;
        end
    endtask

    initial begin
        $dumpfile("tb_dvfs_ctrl.vcd");
        $dumpvars(0, tb_dvfs_ctrl);

        repeat (3) step;
        rst_n = 1;
        step;
        check(v_code === 2'b00 && clk_div_code === 2'b00, "low voltage, low clock after reset");
        check(dvfs_busy === 1'b0 && dvfs_fault === 1'b0, "idle after reset");

        dvfs_req = 2'b11;
        step;
        check(dvfs_busy === 1'b1 && v_code === 2'b11, "voltage ramps first");
        check(clk_div_code === 2'b00, "clock still low during voltage ramp");
        ack_vreg;
        ack_clk(2'b11);
        wait_idle;
        check(v_code === 2'b11 && clk_div_code === 2'b11, "high perf steady state");
        check(dvfs_busy === 1'b0 && dvfs_fault === 1'b0, "not busy in high perf");

        dvfs_req = 2'b00;
        step;
        check(clk_div_code === 2'b00 && v_code === 2'b11, "clock drops first, voltage held");
        check(dvfs_busy === 1'b1, "busy while ramping down");
        ack_clk(2'b00);
        ack_vreg;
        wait_idle;
        check(v_code === 2'b00 && clk_div_code === 2'b00, "back at low perf");

        dvfs_req = 2'b11;
        ack_vreg;
        ack_clk(2'b11);
        wait_idle;
        
        throttled = 1;
        step;
        check(clk_div_code === 2'b00 && dvfs_busy === 1'b1, "throttle drops the clock first");
        ack_clk(2'b00);
        ack_vreg;
        wait_idle;
        check(v_code === 2'b00 && clk_div_code === 2'b00, "throttled ends at low perf");
        
        throttled = 0;
        ack_vreg;
        ack_clk(2'b11);
        wait_idle;
        check(v_code === 2'b11 && clk_div_code === 2'b11, "goes back up after throttle");

        if (errors == 0) $display("tb_dvfs_ctrl: PASS");
        else             $display("tb_dvfs_ctrl: FAIL (%0d errors)", errors);
        $finish;
    end

endmodule