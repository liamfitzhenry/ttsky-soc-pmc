`timescale 1ns/1ps

module tb_clk_div;

    reg clk = 0;
    reg rst_n = 0;
    reg [1:0] div_req = 2'b00;

    wire clk_out;
    wire [1:0] div_cur;

    integer errors = 0;
    integer cyc = 0;
    integer last = -1;
    integer spacing;
    integer last_spacing = 0;
    integer n;
    reg     mon_en = 0;   // jitter monitor: 0 during a mode change, 1 in steady state

    always #5 clk = ~clk;

    clk_div dut (
        .clk(clk), .rst_n(rst_n), .div_req(div_req),
        .clk_out(clk_out), .div_cur(div_cur));

    always @(posedge clk_out) begin
        if (rst_n) begin
            if (last >= 0) begin
                spacing = cyc - last;
                last_spacing = spacing;
                // only judge spacing once a mode has settled; each mode gives 2*(div_max+1)
                if (mon_en && spacing != 2 && spacing != 4 && spacing != 8) begin
                    errors = errors + 1;
                    $display("FAIL @%0d ns: bad tick spacing %0d", $time, spacing);
                end
            end
            last = cyc;
        end
    end

    always @(posedge clk) begin
        cyc = cyc + 1;
        if (!rst_n) last = -1;
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

    // request a new divide code and wait until the output rate has settled
    // expected = rising-edge spacing in clk cycles (2*(div_max+1))
    task set_mode;
        input [1:0] code;
        input integer expected;
        input [8*48-1:0] msg;
        begin
            mon_en  = 0;                 // ignore transitional spacings while div_max changes
            div_req = code;
            n = 0;
            while (!(last_spacing == expected && div_cur === code) && n < 40) begin
                step;
                n = n + 1;
            end
            check(n < 40, "mode switch completes in time");
            check(div_cur === code, "div_cur follows the request");
            check(last_spacing == expected, msg);
            last   = -1;                 // start a fresh measurement window
            mon_en = 1;
        end
    endtask

    initial begin
        $dumpfile("tb_clk_div.vcd");
        $dumpvars(0, tb_clk_div);

        repeat (3) step;
        check(div_cur === 2'b00, "div_cur is 00 in reset (low speed)");
        check(clk_out === 1'b0, "clk_out low during reset");

        rst_n = 1;

        // --- steady divide-by-8 (code 00): rising edge every 8 clk ---
        last   = -1;
        mon_en = 1;
        n = 0;
        while (last_spacing != 8 && n < 40) begin
            step;
            n = n + 1;
        end
        check(last_spacing == 8, "one tick every 8 clocks in /8 mode");
        check(div_cur === 2'b00, "stays at /8 while div_req = 00");
        repeat (20) step;

        // --- code 11: divide-by-2 (rising edge every 2 clk) ---
        set_mode(2'b11, 2, "full rate: one tick every 2 clocks");
        repeat (20) step;

        // --- code 01: divide-by-4 (rising edge every 4 clk) ---
        set_mode(2'b01, 4, "half rate: one tick every 4 clocks");
        repeat (20) step;

        // --- back to code 00: divide-by-8 ---
        set_mode(2'b00, 8, "back to /8");
        repeat (20) step;

        // --- reset returns to low speed ---
        mon_en  = 0;
        div_req = 2'b11;
        repeat (20) step;
        rst_n = 0;
        step;
        check(div_cur === 2'b00, "reset returns to low speed");

        if (errors == 0) $display("tb_clk_div: PASS");
        else             $display("tb_clk_div: FAIL (%0d errors)", errors);
        $finish;
    end

endmodule