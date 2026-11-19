module dvfs_ctrl (
    input  wire clk,
    input  wire rst_n,           

    input  wire [1:0] dvfs_req,
    input  wire throttled,
    input  wire fsm_fault,
    input  wire vreg_ack,

    input  wire [1:0] clk_hi_ack,

    output reg [1:0] v_code,
    output reg [1:0] clk_div_code,

    output wire dvfs_busy,
    output wire dvfs_fault
);

    localparam [2:0] NORMAL_LO_P  = 3'd0,
                     V_RAMP_UP    = 3'd1,
                     F_RAMP_UP    = 3'd2,
                     NORMAL_HI_P  = 3'd3,
                     F_RAMP_DOWN  = 3'd4,
                     V_RAMP_DOWN  = 3'd5,
                     FAULT        = 3'd6;
    
    reg [2:0] state, next_state;

    localparam TIMER_BITS = 6;   // max load is T_CLK_SETTLE = 16, so 6 bits (0-63) suffice
    localparam T_CLK_SETTLE = 16;

    reg tmr_load;
    reg [TIMER_BITS-1:0] tmr_val;
    wire tmr_expired;

    timer #(.BITS(TIMER_BITS)) u_timer (
        .clk     (clk),
        .rst_n   (rst_n),
        .load    (tmr_load),
        .value   (tmr_val),
        .expired (tmr_expired)
    );

    reg fault_flag;

    always @(posedge clk) begin
        if (!rst_n) begin
            state      <= NORMAL_LO_P;
            fault_flag <= 1'b0;
        end
        else begin
            state <= next_state;
            if (dvfs_fault)
                fault_flag <= 1'b1;
            else if (next_state == NORMAL_LO_P)
                fault_flag <= 1'b0;
        end
    end

    wire shutting_down = (dvfs_fault || fault_flag || (dvfs_req == 2'b00) || throttled);

    always @(*) begin
        next_state = state;

        case (state)
            NORMAL_LO_P: begin
                if (fsm_fault || fault_flag) next_state = FAULT;
                else if (dvfs_req != 2'b00) next_state = V_RAMP_UP;
            end
            V_RAMP_UP: begin
                if (shutting_down) next_state = V_RAMP_DOWN;
                else if (vreg_ack) next_state = F_RAMP_UP;
                else if (tmr_expired) next_state = FAULT;
            end
            F_RAMP_UP: begin
                if (shutting_down) next_state = F_RAMP_DOWN;
                else if (clk_hi_ack == dvfs_req) next_state = NORMAL_HI_P;
            end
            NORMAL_HI_P: begin
                if (shutting_down || dvfs_req != clk_hi_ack) next_state = F_RAMP_DOWN;
            end
            F_RAMP_DOWN: begin
                if (clk_hi_ack == 2'b00) next_state = V_RAMP_DOWN;
                else if (tmr_expired) next_state = V_RAMP_DOWN;
            end
            V_RAMP_DOWN: begin
                if (vreg_ack) next_state = fault_flag ? FAULT : NORMAL_LO_P;
                else if (tmr_expired) next_state = FAULT;
            end
            FAULT: begin
                if (!fsm_fault) next_state = NORMAL_LO_P;
            end
            default: begin
                next_state = FAULT;
            end
        endcase
    end

    always @(*) begin
        tmr_load = (next_state != state);
        tmr_val = T_CLK_SETTLE;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            v_code <= 2'b00;
            clk_div_code <= 2'b00;
        end else begin
            if (state == V_RAMP_UP || state == NORMAL_HI_P) v_code <= dvfs_req;
            else if (state == V_RAMP_DOWN || state == NORMAL_LO_P) v_code <= 2'b00;
            
            if (state == F_RAMP_UP || state == NORMAL_HI_P) clk_div_code <= dvfs_req;
            else if (state == F_RAMP_DOWN || state == NORMAL_LO_P) clk_div_code <= 2'b00;
        end
    end

    assign dvfs_busy = (state == V_RAMP_UP) || (state == V_RAMP_DOWN) || (state == F_RAMP_UP) || (state == F_RAMP_DOWN);

    wire clk_ack_timeout = (state == F_RAMP_UP)   && tmr_expired && (clk_hi_ack != dvfs_req);
    wire clk_rel_timeout = (state == F_RAMP_DOWN) && tmr_expired && (clk_hi_ack != 2'b00);
    // no VREG_ACK in time, same timer (16 clocks). Only in the voltage ramp states.
    wire vreg_timeout = ((state == V_RAMP_UP) || (state == V_RAMP_DOWN)) && tmr_expired && !vreg_ack;

    // fsm_fault is NOT included here: it is the power FSM's own fault, and feeding it back
    // would keep the power FSM in FAULT forever
    assign dvfs_fault = clk_ack_timeout || clk_rel_timeout || vreg_timeout;

endmodule