`timescale 1ns/1ps

// Full-system testbench for tt_um_soc_pmc.
// Covers bring-up, DVFS (perf requests and throttling) and the fault paths.
// Dumps every internal signal, so one VCD feeds all the Python views.

module tb_soc_full;

    reg clk = 0;
    reg rst_n = 0;
    reg ena = 1;

    reg warn_n = 1, crit_n = 1, oc_n = 1, fault_clr = 0;
    reg pwr_req = 0;
    reg [1:0] perf_req = 2'b00;
    reg vreg_ack = 0;

    // knobs for the models
    reg vreg_en = 1;                 // 0: the regulator never acks a voltage step
    reg kill_a = 0, kill_b = 0, kill_c = 0;

    wire pg_a, pg_b, pg_c;
    wire [7:0] ui_in  = {fault_clr, vreg_ack, oc_n, crit_n, warn_n, pg_c, pg_b, pg_a};
    wire [7:0] uio_in = {1'b0, perf_req[1], perf_req[0], pwr_req, 4'b0000};
    wire [7:0] uo_out, uio_out, uio_oe;

    integer errors = 0;
    integer n;
    integer cfd;     // case marker file, read by the Python views

    always #5 clk = ~clk;

    regulator_model #(.DLY(6)) rails (
        .clk(clk), .rst_n(rst_n),
        .en_a(uo_out[0]), .en_b(uo_out[1]), .en_c(uo_out[2]),
        .kill_a(kill_a), .kill_b(kill_b), .kill_c(kill_c),
        .stuck_a(1'b0), .stuck_b(1'b0), .stuck_c(1'b0),
        .pg_a(pg_a), .pg_b(pg_b), .pg_c(pg_c));

    tt_um_soc_pmc dut (
        .ui_in(ui_in), .uo_out(uo_out),
        .uio_in(uio_in), .uio_out(uio_out), .uio_oe(uio_oe),
        .ena(ena), .clk(clk), .rst_n(rst_n));

    // voltage ack model: answers each VID change (uo_out[4:3]) after 4 cycles
    reg [1:0] vid_seen = 2'b00;
    integer   vid_cnt  = 0;
    always @(posedge clk) begin
        vreg_ack <= 1'b0;
        if (!rst_n) begin
            vid_seen <= 2'b00;
            vid_cnt  <= 0;
        end else if (vid_seen != uo_out[4:3]) begin
            if (vid_cnt >= 4) begin
                vreg_ack <= vreg_en;
                vid_seen <= uo_out[4:3];
                vid_cnt  <= 0;
            end else
                vid_cnt <= vid_cnt + 1;
        end else
            vid_cnt <= 0;
    end

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

    // hold fault_clr high until the power FSM leaves FAULT (the sync stages add
    // latency). If it stays in FAULT, print what is blocking it.
    // mark the start of a test case: the Python views cut the VCD at these times
    task begin_case;
        input [8*48-1:0] name;
        begin
            $fdisplay(cfd, "%0d %0s", $time, name);
            $fflush(cfd);
            $display("case @%0d ns: %0s", $time, name);
        end
    endtask

    task clear_fault;
        input [8*48-1:0] msg;
        integer k;
        begin
            fault_clr = 1;
            k = 0;
            while (uo_out[6] !== 1'b0 && k < 60) begin
                step;
                k = k + 1;
            end
            fault_clr = 0;
            if (uo_out[6] !== 1'b0)
                $display("  diag: power_fsm=%0d dvfs=%0d dvfs_fault=%b busy=%b pg=%b%b%b pwr_req=%b kill=%b%b%b",
                         dut.u_fsm.state, dut.u_dvfs.state, dut.dvfs_fault, dut.dvfs_busy,
                         pg_a, pg_b, pg_c, pwr_req, kill_a, kill_b, kill_c);
            check(uo_out[6] === 1'b0, msg);
            step;
        end
    endtask

    task wait_vid;
        input [1:0] v;
        input integer maxc;
        input [8*48-1:0] msg;
        integer k;
        begin
            k = 0;
            while (uo_out[4:3] !== v && k < maxc) begin
                step;
                k = k + 1;
            end
            check(uo_out[4:3] === v, msg);
        end
    endtask

    task wait_div;
        input [1:0] c;
        input integer maxc;
        input [8*48-1:0] msg;
        integer k;
        begin
            k = 0;
            while (dut.u_div.div_cur !== c && k < maxc) begin
                step;
                k = k + 1;
            end
            check(dut.u_div.div_cur === c, msg);
        end
    endtask

    task wait_idle;
        input integer maxc;
        input [8*48-1:0] msg;
        integer k;
        begin
            k = 0;
            while (dut.dvfs_busy !== 1'b0 && k < maxc) begin
                step;
                k = k + 1;
            end
            check(dut.dvfs_busy === 1'b0, msg);
        end
    endtask

    task wait_on;
        input integer maxc;
        input [8*48-1:0] msg;
        integer k;
        begin
            k = 0;
            while (uo_out[7] !== 1'b1 && k < maxc) begin
                step;
                k = k + 1;
            end
            check(uo_out[7] === 1'b1, msg);
        end
    endtask

    task wait_fault;
        input integer maxc;
        input [8*48-1:0] msg;
        integer k;
        begin
            k = 0;
            while (uo_out[6] !== 1'b1 && k < maxc) begin
                step;
                k = k + 1;
            end
            check(uo_out[6] === 1'b1, msg);
        end
    endtask

    initial begin
        $dumpfile("tb_soc_full.vcd");
        $dumpvars(0, tb_soc_full);
        cfd = $fopen("tb_soc_full_cases.txt", "w");

        // ---- 1. reset ----
        begin_case("reset");
        repeat (5) step;
        rst_n = 1;
        repeat (3) step;
        check(uio_oe === 8'b1000_1000, "uio_oe: only bits 7 and 3 are outputs");
        check((uo_out & 8'b1101_1111) === 8'b0000_0000, "outputs idle after reset");

        // ---- 2. bring-up at full performance ----
        begin_case("normal startup: OFF -> RUN at perf 11");
        perf_req = 2'b11;
        pwr_req = 1;
        wait_on(800, "power up reaches RUN");
        check(uo_out[2:0] === 3'b111, "all three rails enabled");
        check(uo_out[6] === 1'b0, "no fault on bring-up");
        wait_vid(2'b11, 300, "VID 11 at full performance");
        wait_div(2'b11, 300, "clock at clk/2");
        wait_idle(300, "DVFS idle at 11");

        // ---- 3. performance steps ----
        begin_case("dvfs high (11) -> normal (01)");
        perf_req = 2'b01;
        wait_vid(2'b01, 300, "VID 01 at normal");
        wait_div(2'b01, 300, "clock at clk/4");
        wait_idle(300, "DVFS idle at 01");

        begin_case("dvfs normal (01) -> low (00)");
        perf_req = 2'b00;
        wait_vid(2'b00, 300, "VID 00 at low");
        wait_div(2'b00, 300, "clock at clk/8");
        wait_idle(300, "DVFS idle at 00");

        begin_case("dvfs low (00) -> high (11)");
        perf_req = 2'b11;
        wait_vid(2'b11, 300, "VID 11 again");
        wait_div(2'b11, 300, "clock back at clk/2");
        wait_idle(300, "DVFS idle at 11 again");

        // ---- 4. thermal warning: throttle forces the lowest level ----
        begin_case("thermal warning: high -> throttled (00) -> high");
        warn_n = 0;
        n = 0;
        while (uio_out[7] !== 1'b1 && n < 50) begin
            step;
            n = n + 1;
        end
        check(uio_out[7] === 1'b1, "THROTTLED asserted on warning");
        wait_vid(2'b00, 300, "throttle forces VID 00");
        wait_div(2'b00, 300, "throttle forces clk/8");
        wait_idle(300, "DVFS idle while throttled");

        warn_n = 1;
        n = 0;
        while (uio_out[7] !== 1'b0 && n < 200) begin
            step;
            n = n + 1;
        end
        check(uio_out[7] === 1'b0, "THROTTLED clears");
        wait_vid(2'b11, 300, "VID back to 11 after throttle");
        wait_div(2'b11, 300, "clk/2 back after throttle");
        wait_idle(300, "DVFS idle after throttle");

        // ---- 5. regulator never acks a voltage step: DVFS times out ----
        begin_case("voltage regulator never acks: fault -> recover");
        vreg_en = 0;
        perf_req = 2'b01;
        wait_fault(400, "voltage timeout gives FAULT");
        check(uo_out[2:0] === 3'b000, "rails off in FAULT");
        check(uo_out[4:3] === 2'b00, "VID 00 in FAULT");

        vreg_en = 1;
        perf_req = 2'b11;
        pwr_req = 0;
        repeat (30) step;
        clear_fault("fault clears");
        pwr_req = 1;
        wait_on(800, "power up again after voltage fault");
        wait_vid(2'b11, 300, "VID 11 after recovery");
        wait_idle(300, "DVFS idle after recovery");

        // ---- 6. rail B lost: PG_B drops, chip faults ----
        begin_case("rail B loss: fault -> recover");
        kill_b = 1;
        wait_fault(100, "PG_B loss gives FAULT");
        check(uo_out[2:0] === 3'b000, "rails off after PG_B loss");
        kill_b = 0;
        pwr_req = 0;
        repeat (30) step;
        clear_fault("fault clears after PG_B loss");
        pwr_req = 1;
        wait_on(800, "power up again after PG_B loss");
        wait_vid(2'b11, 300, "VID 11 after PG_B recovery");
        wait_idle(300, "DVFS idle after PG_B recovery");

        // ---- 7. overcurrent ----
        begin_case("overcurrent: fault -> recover");
        oc_n = 0;
        wait_fault(100, "overcurrent gives FAULT");
        oc_n = 1;
        pwr_req = 0;
        repeat (30) step;
        clear_fault("fault clears after overcurrent");

        $fclose(cfd);
        if (errors == 0) $display("tb_soc_full: PASS");
        else             $display("tb_soc_full: FAIL (%0d errors)", errors);
        $finish;
    end

    // stops the run if something hangs
    initial begin
        #5000000;
        $display("tb_soc_full: FAIL (watchdog timeout)");
        $finish;
    end

endmodule