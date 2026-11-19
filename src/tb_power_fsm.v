`timescale 1ns/1ps

module tb_power_fsm;

    reg clk = 0;
    reg rst_n = 0;

    reg pwr_req = 0;
    reg [1:0] perf_req = 2'b00;
    reg therm_warn = 0;
    reg therm_crit = 0;
    reg oc = 0;
    reg dvfs_busy = 0;
    reg dvfs_fault = 0;
    reg fault_clr = 0;

    reg kill_a = 0, kill_b = 0, kill_c = 0;
    reg stuck_a = 0, stuck_b = 0, stuck_c = 0;

    wire pg_a, pg_b, pg_c;
    wire en_a, en_b, en_c;
    wire dvfs_enable, in_fault, in_throttled;
    wire [1:0] perf_target;

    integer errors = 0;
    integer n;
    integer bringup_faults = 0;

    always #5 clk = ~clk;

    regulator_model #(.DLY(6)) rails (
        .clk(clk), .rst_n(rst_n),
        .en_a(en_a), .en_b(en_b), .en_c(en_c),
        .kill_a(kill_a), .kill_b(kill_b), .kill_c(kill_c),
        .stuck_a(stuck_a), .stuck_b(stuck_b), .stuck_c(stuck_c),
        .pg_a(pg_a), .pg_b(pg_b), .pg_c(pg_c));

    power_fsm dut (
        .clk(clk), .rst_n(rst_n),
        .pwr_req(pwr_req), .perf_req(perf_req),
        .pg_a(pg_a), .pg_b(pg_b), .pg_c(pg_c),
        .therm_warn(therm_warn), .therm_crit(therm_crit), .oc(oc),
        .dvfs_busy(dvfs_busy), .dvfs_fault(dvfs_fault),
        .fault_clr(fault_clr),
        .en_a(en_a), .en_b(en_b), .en_c(en_c),
        .dvfs_enable(dvfs_enable), .perf_target(perf_target),
        .in_fault(in_fault), .in_throttled(in_throttled));

    // Regression guard: with no fault stimulus applied, the FSM must never
    // assert in_fault. A power-good detector that fires during normal bring-up
    // (e.g. the STARTUP_A rising-edge bug) shows up here as a non-zero count.
    wire fault_stimulus_active = oc | therm_crit | kill_a | kill_b | kill_c;

    always @(posedge clk) begin
        if (rst_n && in_fault && !fault_stimulus_active)
            bringup_faults = bringup_faults + 1;
    end

    task step;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    task fail;
        input [8*48-1:0] msg;
        begin
            errors = errors + 1;
            $display("FAIL @%0d ns: %0s", $time, msg);
        end
    endtask

    task check;
        input cond;
        input [8*48-1:0] msg;
        begin
            if (!cond) fail(msg);
        end
    endtask

    task pulse_clr;
        begin
            fault_clr = 1;
            step;
            fault_clr = 0;
            step;
        end
    endtask

    task power_up;
        integer k;
        begin
            pwr_req = 1;
            k = 0;
            while (dvfs_enable !== 1'b1 && k < 800) begin
                step;
                k = k + 1;
            end
            check(dvfs_enable === 1'b1, "power_up reaches RUN");
        end
    endtask

    task power_down;
        integer k;
        begin
            pwr_req = 0;
            k = 0;
            while ((en_a | en_b | en_c) !== 1'b0 && k < 800) begin
                step;
                k = k + 1;
            end
            check((en_a | en_b | en_c) === 1'b0, "power_down reaches all rails off");
            repeat (20) step;
        end
    endtask

    initial begin
        $dumpfile("tb_power_fsm.vcd");
        $dumpvars(0, tb_power_fsm);

        repeat (4) step;
        rst_n = 1;
        step;
        check((en_a | en_b | en_c) === 1'b0 && in_fault === 1'b0, "idle after reset");

        perf_req = 2'b11;
        power_up;
        check(bringup_faults == 0, "clean bring-up never enters FAULT");
        check(perf_target === 2'b11 && dvfs_enable === 1'b1, "RUN passes the perf request on");

        therm_warn = 1;
        step;
        check(in_throttled === 1'b1, "warning gives THROTTLED");
        check(perf_target === 2'b00, "throttle forces safe target");
        check(dvfs_enable === 1'b1, "dvfs stays on");
        
        repeat (5) step;
        therm_warn = 0;
        n = 0;
        while (in_throttled === 1'b1 && n < 100) begin
            step;
            n = n + 1;
        end
        check(n >= 32, "leaves THROTTLED after hysteresis");

        dvfs_busy = 1;
        pwr_req = 0;
        repeat (10) step;
        check(en_c === 1'b1, "shutdown waits while dvfs_busy");
        dvfs_busy = 0;
        power_down;

        power_up;
        kill_b = 1;
        n = 0;
        while (in_fault !== 1'b1 && n < 10) begin
            step;
            n = n + 1;
        end
        check(in_fault === 1'b1, "PG_B loss gives FAULT");
        check((en_a | en_b | en_c | dvfs_enable) === 1'b0, "outputs safe in FAULT");

        pwr_req = 0;
        kill_b = 0;
        repeat (30) step;
        pulse_clr;
        check(in_fault === 1'b0, "recovers to OFF");

        if (errors == 0) $display("tb_power_fsm: PASS");
        else             $display("tb_power_fsm: FAIL (%0d errors)", errors);
        $finish;
    end
endmodule