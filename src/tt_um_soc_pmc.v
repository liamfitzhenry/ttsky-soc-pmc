// Tiny Tapeout top level for the SoC power management controller.
// Pin map follows the project spreadsheet (uio[0..3] = SPI, uio[4..6] = requests,
// uio[7] = THROTTLED).

module tt_um_soc_pmc (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    localparam TICK_BITS = 10;   // energy tick = one pulse every 2^10 clocks

    // ---------------- input synchronisers (SPI pins are synced in spi_slave) ----------------
    wire pg_a, pg_b, pg_c, therm_warn_n, therm_crit_n, oc_n, vreg_ack, fault_clr, pwr_req;
    wire [1:0] perf_req;

    sync #(.RESET_VAL(1'b0)) u_sync_pg_a       (.clk(clk), .rst_n(rst_n), .d_async(ui_in[0]), .d_sync(pg_a));
    sync #(.RESET_VAL(1'b0)) u_sync_pg_b       (.clk(clk), .rst_n(rst_n), .d_async(ui_in[1]), .d_sync(pg_b));
    sync #(.RESET_VAL(1'b0)) u_sync_pg_c       (.clk(clk), .rst_n(rst_n), .d_async(ui_in[2]), .d_sync(pg_c));
    sync #(.RESET_VAL(1'b1)) u_sync_therm_warn (.clk(clk), .rst_n(rst_n), .d_async(ui_in[3]), .d_sync(therm_warn_n));
    sync #(.RESET_VAL(1'b1)) u_sync_therm_crit (.clk(clk), .rst_n(rst_n), .d_async(ui_in[4]), .d_sync(therm_crit_n));
    sync #(.RESET_VAL(1'b1)) u_sync_oc         (.clk(clk), .rst_n(rst_n), .d_async(ui_in[5]), .d_sync(oc_n));
    sync #(.RESET_VAL(1'b0)) u_sync_vreg_ack   (.clk(clk), .rst_n(rst_n), .d_async(ui_in[6]), .d_sync(vreg_ack));
    sync #(.RESET_VAL(1'b0)) u_sync_fault_clr  (.clk(clk), .rst_n(rst_n), .d_async(ui_in[7]), .d_sync(fault_clr));

    sync #(.RESET_VAL(1'b0)) u_sync_pwr_req    (.clk(clk), .rst_n(rst_n), .d_async(uio_in[4]), .d_sync(pwr_req));
    sync #(.RESET_VAL(1'b0)) u_sync_perf_req0  (.clk(clk), .rst_n(rst_n), .d_async(uio_in[5]), .d_sync(perf_req[0]));
    sync #(.RESET_VAL(1'b0)) u_sync_perf_req1  (.clk(clk), .rst_n(rst_n), .d_async(uio_in[6]), .d_sync(perf_req[1]));

    wire therm_warn = ~therm_warn_n;
    wire therm_crit = ~therm_crit_n;
    wire oc         = ~oc_n;

    // ---------------- SPI slave and register file ----------------
    wire       spi_miso, spi_miso_oe;
    wire [6:0] rd_addr;
    wire [7:0] rd_data;
    wire       wr_en;
    wire [6:0] wr_addr;
    wire [7:0] wr_data;
    wire       ovr_en, ovr_pwr_req;
    wire [1:0] ovr_perf_req;
    wire       spi_clr;

    spi_slave u_spi (
        .clk        (clk),
        .rst_n      (rst_n),
        .spi_cs_n   (uio_in[0]),
        .spi_sck    (uio_in[1]),
        .spi_mosi   (uio_in[2]),
        .spi_miso   (spi_miso),
        .spi_miso_oe(spi_miso_oe),
        .rd_addr    (rd_addr),
        .rd_data    (rd_data),
        .wr_en      (wr_en),
        .wr_addr    (wr_addr),
        .wr_data    (wr_data)
    );

    wire [2:0] status_state;
    wire [1:0] v_code, div_cur;
    wire       in_fault, in_throttled, dvfs_enable;
    wire [8:0] power_uw;
    wire       fault_pulse;
    wire [2:0] fault_cause;
    wire       tick;

    reg [TICK_BITS-1:0] tick_cnt;
    assign tick = &tick_cnt;
    always @(posedge clk) begin
        if (!rst_n) tick_cnt <= {TICK_BITS{1'b0}};
        else        tick_cnt <= tick_cnt + 1'b1;
    end

    pmc_regs u_regs (
        .clk        (clk),
        .rst_n      (rst_n),
        .rd_addr    (rd_addr),
        .rd_data    (rd_data),
        .wr_en      (wr_en),
        .wr_addr    (wr_addr),
        .wr_data    (wr_data),
        .state      (status_state),
        .vid        (v_code),
        .fault      (in_fault),
        .power_uw   (power_uw),
        .tick       (tick),
        .fault_pulse(fault_pulse),
        .fault_cause(fault_cause),
        .ovr_en     (ovr_en),
        .ovr_pwr_req(ovr_pwr_req),
        .ovr_perf_req(ovr_perf_req),
        .fault_clr  (spi_clr)
    );

    // SPI override: CTRL bit 0 takes the requests from the SPI register instead of the pins
    wire       pwr_req_eff  = ovr_en ? ovr_pwr_req : pwr_req;
    wire [1:0] perf_req_eff = ovr_en ? ovr_perf_req : perf_req;

    // SPI clear is held until the FSM leaves FAULT, so a pulse isn't lost while rails are still discharging
    reg  spi_clr_latch;
    wire fault_clr_eff = fault_clr | spi_clr_latch;
    always @(posedge clk) begin
        if (!rst_n)         spi_clr_latch <= 1'b0;
        else if (spi_clr)   spi_clr_latch <= 1'b1;
        else if (!in_fault) spi_clr_latch <= 1'b0;
    end

    // ---------------- control ----------------
    wire en_a, en_b, en_c;
    wire [1:0] perf_target;
    wire dvfs_busy, dvfs_fault;
    wire [1:0] clk_div_code;
    wire clk_out_wire;

    wire [1:0] dvfs_req = dvfs_enable ? perf_target : 2'b00;

    power_fsm u_fsm (
        .clk         (clk),
        .rst_n       (rst_n),
        .pwr_req     (pwr_req_eff),
        .perf_req    (perf_req_eff),
        .pg_a        (pg_a),
        .pg_b        (pg_b),
        .pg_c        (pg_c),
        .therm_warn  (therm_warn),
        .therm_crit  (therm_crit),
        .oc          (oc),
        .dvfs_busy   (dvfs_busy),
        .dvfs_fault  (dvfs_fault),
        .fault_clr   (fault_clr_eff),
        .en_a        (en_a),
        .en_b        (en_b),
        .en_c        (en_c),
        .dvfs_enable (dvfs_enable),
        .perf_target (perf_target),
        .in_fault    (in_fault),
        .in_throttled(in_throttled),
        .fault_pulse (fault_pulse),
        .fault_cause (fault_cause),
        .status_state(status_state)
    );

    dvfs_ctrl u_dvfs (
        .clk          (clk),
        .rst_n        (rst_n),
        .dvfs_req     (dvfs_req),
        .throttled    (in_throttled),
        .vreg_ack     (vreg_ack),
        .clk_hi_ack   (div_cur),
        .fsm_fault    (in_fault),
        .v_code       (v_code),
        .clk_div_code (clk_div_code),
        .dvfs_busy    (dvfs_busy),
        .dvfs_fault   (dvfs_fault)
    );

    clk_div u_div (
        .clk     (clk),
        .rst_n   (rst_n),
        .div_req (clk_div_code),
        .clk_out (clk_out_wire),
        .div_cur (div_cur)
    );

    // power estimate uses what is actually on the pins (VID and running clock divider)
    power_monitor u_pwr (
        .vid      (v_code),
        .clk_sel  (div_cur),
        .ready    (dvfs_enable),
        .power_uw (power_uw)
    );

    // ---------------- outputs ----------------
    assign uo_out[0] = en_a;
    assign uo_out[1] = en_b;
    assign uo_out[2] = en_c;
    assign uo_out[3] = v_code[0];
    assign uo_out[4] = v_code[1];
    assign uo_out[5] = clk_out_wire;
    assign uo_out[6] = in_fault;
    assign uo_out[7] = dvfs_enable;     // READY: high in RUN and THROTTLED

    assign uio_out = {in_throttled, 3'b000, spi_miso, 3'b000};
    assign uio_oe  = {1'b1, 3'b000, spi_miso_oe, 3'b000};

endmodule