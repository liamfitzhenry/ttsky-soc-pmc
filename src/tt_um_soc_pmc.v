// tt_um_soc_pmc.v
// Tiny Tapeout top. Pin map is in docs/pin_map.md.


module tt_um_soc_pmc (
    input wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input wire ena,
    input wire clk,
    input wire rst_n
);

    // asynchronous inputs need to be synchronized with the clock 
    // see sync.v 
    // see PINS in excel sheet for pin assignement mapping
    wire pg_a, pg_b, pg_c, therm_warn_n, therm_crit_n, oc_n, fault_clr, pwr_req, perf_req;

    // reset value of 0 for pgoods
    // the inputs to the fsms will be pg_a, pg_b, therm_warn_n, etc... (synced) instead of ui_in[x] (async)
    sync #(.RESET_VAL(1'b0)) u_sync_pg_a (.clk(clk), .rst_n(rst_n), .d_async(ui_in[0]), .d_sync(pg_a));
    sync #(.RESET_VAL(1'b0)) u_sync_pg_b (.clk(clk), .rst_n(rst_n), .d_async(ui_in[1]), .d_sync(pg_b));
    sync #(.RESET_VAL(1'b0)) u_sync_pg_c (.clk(clk), .rst_n(rst_n), .d_async(ui_in[2]), .d_sync(pg_c));

    // reset value of 1 for safety flags (must be confirmed clear to run)
    sync #(.RESET_VAL(1'b1)) u_sync_therm_warn (.clk(clk), .rst_n(rst_n), .d_async(ui_in[3]), .d_sync(therm_warn_n));
    sync #(.RESET_VAL(1'b1)) u_sync_therm_crit (.clk(clk), .rst_n(rst_n), .d_async(ui_in[4]), .d_sync(therm_crit_n));
    sync #(.RESET_VAL(1'b1)) u_sync_oc (.clk(clk), .rst_n(rst_n), .d_async(ui_in[5]), .d_sync(oc_n));

    // fault clearance is also active high - 0 is the safe reset value bc the fault would remain if present
    sync #(.RESET_VAL(1'b0)) u_sync_fault_clr (.clk(clk), .rst_n(rst_n), .d_async(ui_in[7]), .d_sync(fault_clr));
    // like the "on switch" for the SoC
    sync #(.RESET_VAL(1'b0)) u_sync_pwr_req (.clk(clk), .rst_n(rst_n), .d_async(uio_in[4]), .d_sync(pwr_req));
    // reads performance requests from soc
    sync #(.RESET_VAL(1'b0)) u_sync_perf_req (.clk(clk), .rst_n(rst_n), .d_async(uio_in[5]),.d_sync(perf_req));

    // convert to active-high for internal readability, industry standard is that these signals are active-low
    wire therm_warn = ~therm_warn_n;
    wire therm_crit = ~therm_crit_n;
    wire oc = ~oc_n;

    // initializing the fsms - starts with laying out all the input/output signals 

    // power_fsm outputs 
    wire en_a, en_b, en_c; // to voltage regulators
    wire dvfs_enable, perf_target, in_fault, in_throttled; // to dvfs

    // dvfs outputs
    wire dvfs_busy, dvfs_fault; // to fsm
    wire v_code, clk_div_code; // to regulators (voltgae, clock)

    // clock divider outputs
    wire div_cur; 
    wire tick; 

    wire dvfs_req = perf_target & dvfs_enable;   // power_fsm wants high perf
    wire clk_hi_ack  = ~div_cur; // divider confirmed high speed

    // fsm initizlisation from the lower modules 
    
    power_fsm u_fsm (
        .clk (clk),
        .rst_n (rst_n),
        // from SoC->power_fsm:
        .pwr_req (pwr_req),
        .perf_req (perf_req),
        // from voltage regulators->power_fsm
        .pg_a (pg_a),
        .pg_b (pg_b),
        .pg_c (pg_c),
        // from sensors->power_fsm
        .therm_warn (therm_warn),
        .therm_crit (therm_crit),
        .oc (oc),
        // from dvfs_ctrl->power_fsm
        .dvfs_busy (dvfs_busy),
        .dvfs_fault (dvfs_fault),
        .fault_clr (fault_clr),
        // from power_fsm->voltage regulators
        .en_a (en_a),
        .en_b (en_b),
        .en_c (en_c),
        // from power_fsm->dvfs_ctrl
        .dvfs_enable (dvfs_enable),
        .perf_target (perf_target),
        .in_fault (in_fault),
        .in_throttled (in_throttled)
    );

    dvfs_ctrl u_dvfs (
        .clk (clk),
        .rst_n (rst_n),
        // from power_fsm->dvfs_ctrl
        .dvfs_req (dvfs_req),       
        .throttled (in_throttled),   
        // from clk_div->dvfs_ctrl
        .clk_hi_ack (clk_hi_ack),        
        // outputs
        .fsm_fault (in_fault),       
        .v_code (v_code),
        .clk_div_code (clk_div_code),
        .dvfs_busy (dvfs_busy),
        .dvfs_fault (dvfs_fault)
    );

    // clock divider module
    clk_div u_div (
        .clk (clk),
        .rst_n (rst_n),
        .div_req (~clk_div_code),
        .tick (tick),
        .div_cur (div_cur)
    );

    // outputs to clock generator/divider
    wire clk_en = ~in_fault; // clock only while not in FAULT
    wire freq_sel = clk_hi_ack & ~in_fault;  // high speed only when confirmed
    
    
    // VID: 00 = SAFE, 01 = V_LOW, 11 = V_HIGH 
    // leaving this as 2 bits wide for now to match original plan, may change later...
    wire [1:0] vid = {v_code, 1'b1};

    wire fault_out = in_fault;
    wire throttled_out = in_throttled;
    wire ready_out = dvfs_enable;          // system up and running (RUN/THROTTLE)

    // outputs
    assign uo_out[0] = en_a;
    assign uo_out[1] = en_b;
    assign uo_out[2] = en_c;
    assign uo_out[3] = vid[0];
    assign uo_out[4] = vid[1];
    assign uo_out[5] = clk_en;
    assign uo_out[6] = tick;
    assign uo_out[7] = fault_out;

    // uio: 6 = throttled, 7 = ready are outputs, the rest are inputs
    assign uio_out = {ready_out, throttled_out, 6'b0};
    assign uio_oe  = 8'b1100_0000;


endmodule
