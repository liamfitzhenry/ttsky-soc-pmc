// Register file for the PMC, reached through the SPI slave.
//
// Address map:
//   0x00 CTRL       RW  [0] ovr_en [1] ovr_pwr_req [3:2] ovr_perf_req [4] fault_clr (pulse)
//   0x01 STATUS     RO  [2:0] state [4:3] vid [5] fault
//   0x02 POWER_L    RO  power_uw[7:0]
//   0x03 POWER_H    RO  power_uw[8]
//   0x04..0x06      --  reserved (state counters removed); reads return 0x00
//   0x07 FLT_CAUSE  RO  [2:0] cause of last fault
//   0x08 FLT_COUNT  RO  number of faults, saturates at 255
//   0x09 EN_SNAP    W   any write copies the energy total into the snapshot
//   0x0A EN_B0      RO  snapshot [7:0]
//   0x0B EN_B1      RO  snapshot [15:8]
//   0x0C EN_B2      RO  snapshot [23:16]
//   0x0D EN_CLR     W   any write clears the energy total
//
// State encoding: 0=OFF 1=STARTUP 2=RUN 3=THROTTLED 4=SHUTDOWN 5=FAULT

module pmc_regs (
    input  wire       clk,
    input  wire       rst_n,

    input  wire [6:0] rd_addr,
    output reg  [7:0] rd_data,
    input  wire       wr_en,
    input  wire [6:0] wr_addr,
    input  wire [7:0] wr_data,

    input  wire [2:0] state,
    input  wire [1:0] vid,
    input  wire       fault,
    input  wire [8:0] power_uw,
    input  wire       tick,           // one-clock pulse per energy step
    input  wire       fault_pulse,    // one-clock pulse on entry to FAULT
    input  wire [2:0] fault_cause,

    output wire       ovr_en,
    output wire       ovr_pwr_req,
    output wire [1:0] ovr_perf_req,
    output reg        fault_clr
);

    reg [3:0]  ctrl;
    reg [23:0] snap;

    wire [23:0] energy;
    wire [2:0]  last_cause;
    wire [7:0]  flt_count;

    wire en_clr   = wr_en && (wr_addr == 7'h0D);
    wire snap_wr  = wr_en && (wr_addr == 7'h09);

    energy_acc u_energy (
        .clk(clk), .rst_n(rst_n), .tick(tick), .power_uw(power_uw),
        .clr(en_clr), .energy(energy)
    );

    fault_log u_flt (
        .clk(clk), .rst_n(rst_n), .fault_pulse(fault_pulse), .cause(fault_cause),
        .last_cause(last_cause), .count(flt_count)
    );

    assign ovr_en       = ctrl[0];
    assign ovr_pwr_req  = ctrl[1];
    assign ovr_perf_req = ctrl[3:2];

    always @(posedge clk) begin
        if (!rst_n) begin
            ctrl      <= 4'd0;
            snap      <= 24'd0;
            fault_clr <= 1'b0;
        end else begin
            fault_clr <= 1'b0;
            if (wr_en) begin
                case (wr_addr)
                    7'h00: begin
                        ctrl      <= wr_data[3:0];
                        fault_clr <= wr_data[4];
                    end
                    7'h09: snap    <= energy;
                    default: ;
                endcase
            end
        end
    end

    always @(*) begin
        case (rd_addr)
            7'h00:   rd_data = {4'b0000, ctrl};
            7'h01:   rd_data = {2'b00, fault, vid, state};
            7'h02:   rd_data = power_uw[7:0];
            7'h03:   rd_data = {7'b0, power_uw[8]};
            7'h07:   rd_data = {5'b0, last_cause};
            7'h08:   rd_data = flt_count;
            7'h0A:   rd_data = snap[7:0];
            7'h0B:   rd_data = snap[15:8];
            7'h0C:   rd_data = snap[23:16];
            default: rd_data = 8'h00;
        endcase
    end

endmodule