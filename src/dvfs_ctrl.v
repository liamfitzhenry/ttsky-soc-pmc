// Sub-FSM for the dynamic voltage & frequency scaling (DVFS) function
// FUNCTION: 
// - alter the clock frequency and supply voltage to downstream processing based on requested performance
// INPUTS:
// - clk, rst_n
// - performance request (dvfs_req), internal thermal warning & fault, clock divider handshakes
// OUTPUTS:
// - voltage request to regulator, speed request to clock producer 

module dvfs_ctrl (
    input  wire clk,
    input  wire rst_n,           

    // signals in from power_fsm
    input  wire dvfs_req,  // 1 = high performance wanted & allowed
    input  wire throttled,  // 1 = thermal warning active
    input  wire fsm_fault,   // from power_fsm, high while it is in FAULT

    // signals in from clock divider
    input  wire clk_hi_ack, // 1 = divider confirmed high speed

    // output to voltage regulator
    output reg  v_code, // 1 = V_HIGH, 0 = V_LOW

    // output to clock divider
    output reg  clk_div_code, // 1 = high speed request

    // output to power_fsm 
    output reg  dvfs_busy, // high while a step sequence is running
    output reg  dvfs_fault  // clock confirm timed out
);

    // same FSM-style encoding as in power_fsm

    localparam [2:0] NORMAL_LO_P  = 3'd0,
                     V_RAMP_UP    = 3'd1,
                     F_RAMP_UP    = 3'd2,
                     NORMAL_HI_P  = 3'd3,
                     F_RAMP_DOWN  = 3'd4,
                     V_RAMP_DOWN  = 3'd5,
                     FAULT        = 3'd6;
    
    reg [2:0] state, next_state;

    localparam TIMER_BITS = 24;
    // arbitrary parameters right now...will back this up with some relevant research 
    localparam T_V_SETTLE = 20;

    // how long the divider gets to confirm a clock change
    localparam T_CLK_SETTLE = 16;

    // timer to count settling times, etc....
    reg tmr_load;
    reg [TIMER_BITS-1:0] tmr_val;
    wire tmr_expired;

    // again, bits are a bit arbitrary right now as I haven't settled on a clock speed
    timer #(.BITS(TIMER_BITS)) u_timer (
        .clk     (clk),
        .rst_n   (rst_n),
        .load    (tmr_load),
        .value   (tmr_val),
        .expired (tmr_expired)
    );

    // create a fault flag register that tracks when a fault has occured throughout the shutdown process
    reg fault_flag;

    always @(posedge clk) begin
        if (!rst_n) begin
            state      <= NORMAL_LO_P;
            fault_flag <= 1'b0;
        end
        else begin
            state <= next_state;
            if (dvfs_fault)
                fault_flag <= 1'b1;            // remember the fault
            else if (next_state == NORMAL_LO_P)
                fault_flag <= 1'b0;            // recovered, safe again
        end
    end

    // Leave high performance if a fault is present/pending, throttled, or unwanted.
    wire shutting_down = (dvfs_fault || fault_flag || !dvfs_req || throttled);
    // FSM block, like in power_fsm
    always @(*) begin

        // assign defaults 
        next_state = state;

        case (state)

            NORMAL_LO_P: begin
                if (fsm_fault || fault_flag) next_state = FAULT;
                else if (dvfs_req) next_state = V_RAMP_UP; // if a ramp-up is requested by the power_fsm, start it
            end

            V_RAMP_UP: begin
                // wait the set settling time before moving on 
                // if throttled, just ramp the voltage back down
                if (shutting_down) next_state = V_RAMP_DOWN;
                else if (tmr_expired) next_state = F_RAMP_UP;
            end

            F_RAMP_UP: begin
                // if the "ok" is recieved from the clock divider, move on
                // if somethinf goes wrong, clock times out, flag it as a fault
                if (shutting_down) next_state = F_RAMP_DOWN;
                else if (clk_hi_ack) next_state = NORMAL_HI_P;
                // on timeout, dvfs_fault latches the fault and shutting_down
                // routes us to F_RAMP_DOWN on the next cycle
            end

            // high power mode is exited only under fault, throttled, or low-power request
            NORMAL_HI_P: begin
                if (shutting_down) next_state = F_RAMP_DOWN;
            end

            // clock comes down first, then voltage
            F_RAMP_DOWN: begin
                if (!clk_hi_ack) next_state = V_RAMP_DOWN;
                // clock never released: still bring the voltage down (dvfs_fault
                // latches the fault so V_RAMP_DOWN finishes in FAULT)
                else if (tmr_expired) next_state = V_RAMP_DOWN;
            end

            V_RAMP_DOWN: begin
                // Let the voltage settle FIRST, then choose the destination:
                // FAULT if a fault is pending, else back to low power.
                if (tmr_expired)
                    next_state = fault_flag ? FAULT : NORMAL_LO_P;
            end

            FAULT: begin
                // fault_flag is cleared in the sequential block once we
                // actually return to NORMAL_LO_P
                if (!fsm_fault) next_state = NORMAL_LO_P;
            end

            default: begin
                next_state = FAULT;
            end

        endcase
    end

    // Timer block - used for most states
    always @(*) begin

        // if entering a new state, load up the timer to track how long we're in the new state
        tmr_load = (next_state != state);

        // settling times depend on the state
        case (next_state)
            V_RAMP_UP: tmr_val = T_V_SETTLE;
            V_RAMP_DOWN: tmr_val = T_V_SETTLE;
            default: tmr_val = T_CLK_SETTLE;
        endcase

    end

    // OUTPUTS: outputs only depend on the current state, so they are logically derived
    assign v_code = (state == V_RAMP_UP) || (state == F_RAMP_UP) || (state == F_RAMP_DOWN) || (state == NORMAL_HI_P);
    assign clk_div_code = (state == F_RAMP_UP) || (state == NORMAL_HI_P);
    assign dvfs_busy = (state == V_RAMP_UP) || (state == V_RAMP_DOWN) || (state == F_RAMP_UP) || (state == F_RAMP_DOWN);

    // all fault sources: parent fsm, plus the clock timeouts on the way up and down
    wire clk_ack_timeout = (state == F_RAMP_UP)   && tmr_expired && !clk_hi_ack;
    wire clk_rel_timeout = (state == F_RAMP_DOWN) && tmr_expired &&  clk_hi_ack;
    assign dvfs_fault = fsm_fault || clk_ack_timeout || clk_rel_timeout;


endmodule
