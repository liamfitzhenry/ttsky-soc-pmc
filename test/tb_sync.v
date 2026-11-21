`timescale 1ns/1ps

// sync_tb.v
// Small testbench for the two-flop synchronizer.
//
// Clock: 10 ns period (posedge at 5, 15, 25, ... ns).
// The input is turned ON once and OFF once, both at times that do NOT line up
// with a clock edge:
//   d_async 0 -> 1 at 27.3 ns   (next posedge is 35 ns)
//   d_async 1 -> 0 at 52.7 ns   (next posedge is 55 ns)
//
// Expected: d_sync follows after exactly 2 clock edges, and does not change
// after only 1 edge (the first flop only captures into "meta").

module sync_tb;

    reg clk = 0;
    reg rst_n = 0;
    reg d_async = 0;

    wire d_sync;

    integer errors = 0;
    integer edges;

    always #5 clk = ~clk;

    sync #(.RESET_VAL(1'b0)) dut (
        .clk(clk),
        .rst_n(rst_n),
        .d_async(d_async),
        .d_sync(d_sync)
    );

    task check;
        input cond;
        input [8*56-1:0] msg;
        begin
            if (!cond) begin
                errors = errors + 1;
                $display("FAIL @%0t ns: %0s", $time, msg);
            end
        end
    endtask

    // Wait for d_sync to reach "want", counting clock edges. Expects exactly 2,
    // and checks that d_sync has not already moved after the first edge.
    task expect_after_two;
        input want;
        begin
            edges = 0;
            while (d_sync !== want && edges < 10) begin
                @(posedge clk);
                #1;
                edges = edges + 1;
                if (edges == 1)
                    check(d_sync !== want, "d_sync changed after only 1 clock edge");
            end
            check(d_sync === want, "d_sync never followed d_async");
            check(edges == 2, "d_sync did not settle after exactly 2 clock edges");
            $display("  d_sync = %b after %0d clock edges (@%0t ns)", d_sync, edges, $time);
        end
    endtask

    initial begin
        $dumpfile("sync_tb.vcd");
        $dumpvars(0, sync_tb);

        // hold reset, then release between edges
        #20;
        rst_n = 1;
        #1;
        check(d_sync === 1'b0, "d_sync not 0 after reset");

        // turn the input ON at 27.3 ns, off the clock grid
        #6.3;
        d_async = 1'b1;
        $display("d_async 0->1 at %0t ns (next posedge 35 ns)", $time);
        expect_after_two(1'b1);

        // turn the input OFF at 52.7 ns, off the clock grid
        #25.4;
        d_async = 1'b0;
        $display("d_async 1->0 at %0t ns (next posedge 55 ns)", $time);
        expect_after_two(1'b0);

        repeat (3) @(posedge clk);

        if (errors == 0) $display("sync_tb: PASS");
        else             $display("sync_tb: FAIL (%0d errors)", errors);
        $finish;
    end

    // watchdog
    initial begin
        #1000;
        $display("sync_tb: FAIL (watchdog timeout)");
        $finish;
    end

endmodule
