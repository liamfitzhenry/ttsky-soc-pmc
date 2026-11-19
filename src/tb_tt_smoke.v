`timescale 1ns/1ps

module tb_tt_smoke;

    reg clk = 0;
    reg rst_n = 0;
    reg ena = 1;

    reg warn_n = 1, crit_n = 1, oc_n = 1, fault_clr = 0, vreg_ack = 0;
    reg pwr_req = 0;
    reg [1:0] perf_req = 2'b00;

    wire pg_a, pg_b, pg_c;
    wire [7:0] ui_in = {fault_clr, vreg_ack, oc_n, crit_n, warn_n, pg_c, pg_b, pg_a};
    wire [7:0] uio_in = {1'b0, perf_req[1], perf_req[0], pwr_req, 4'b0000};
    wire [7:0] uo_out, uio_out, uio_oe;

    integer errors = 0;
    integer n;

    always #5 clk = ~clk;

    regulator_model #(.DLY(8)) rails (
        .clk(clk), .rst_n(rst_n),
        .en_a(uo_out[0]), .en_b(uo_out[1]), .en_c(uo_out[2]),
        .kill_a(1'b0), .kill_b(1'b0), .kill_c(1'b0),
        .stuck_a(1'b0), .stuck_b(1'b0), .stuck_c(1'b0),
        .pg_a(pg_a), .pg_b(pg_b), .pg_c(pg_c));

    tt_um_soc_pmc dut (
        .ui_in(ui_in), .uo_out(uo_out),
        .uio_in(uio_in), .uio_out(uio_out), .uio_oe(uio_oe),
        .ena(ena), .clk(clk), .rst_n(rst_n));

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

    initial begin
        $dumpfile("tb_tt_smoke.vcd");
        $dumpvars(0, tb_tt_smoke);

        repeat (5) step;
        rst_n = 1;
        repeat (3) step;
        check(uio_oe === 8'b1000_1000, "uio_oe: only bits 7 and 3 are outputs");
        // bit 5 is the (toggling) clk_out, so mask it off before comparing
        check((uo_out & 8'b1101_1111) === 8'b0000_0000, "all outputs low after reset (ignoring clk_out)");
        check(uio_out[7] === 1'b0, "throttled low after reset");

        pwr_req = 1;
        n = 0;
        while (uo_out[7] !== 1'b1 && n < 800) begin
            step;
            n = n + 1;
        end
        check(uo_out[7] === 1'b1, "DVFS_ENABLE on uo[7] after startup");
        check(uo_out[2:0] === 3'b111, "EN_A/B/C on uo[2:0]");
        check(uo_out[6] === 1'b0, "FAULT low while running");

        warn_n = 0;
        repeat (6) step;
        check(uio_out[7] === 1'b1, "THROTTLED on uio[7]");
        warn_n = 1;
        repeat (60) step;
        check(uio_out[7] === 1'b0, "THROTTLED clears");

        oc_n = 0;
        repeat (6) step;
        check(uo_out[6] === 1'b1, "FAULT on uo[6] after overcurrent");
        check(uo_out[2:0] === 3'b000, "rails off in FAULT");
        check(uo_out[4:3] === 2'b00, "VID is 00 in FAULT");

        if (errors == 0) $display("tb_tt_smoke: PASS");
        else             $display("tb_tt_smoke: FAIL (%0d errors)", errors);
        $finish;
    end

endmodule