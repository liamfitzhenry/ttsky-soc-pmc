// SPI slave, framing only. Register decoding lives in pmc_regs.
// Mode 0, MSB first, 16 SCK pulses per frame, CS low for the whole frame.
//   byte 0 = {rw, addr[6:0]}   rw=1 write, rw=0 read
//   byte 1 = data

module spi_slave (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       spi_cs_n,
    input  wire       spi_sck,
    input  wire       spi_mosi,
    output wire       spi_miso,
    output wire       spi_miso_oe,

    output wire [6:0] rd_addr,   // address being decoded, valid on header bit 7
    input  wire [7:0] rd_data,   // value of that register
    output reg        wr_en,     // one-clock write strobe
    output reg  [6:0] wr_addr,
    output reg  [7:0] wr_data
);

    reg [1:0] cs_s;
    reg [1:0] sck_s;
    reg [1:0] mosi_s;
    reg       sck_d;

    always @(posedge clk) begin
        if (!rst_n) begin
            cs_s   <= 2'b11;
            sck_s  <= 2'b00;
            mosi_s <= 2'b00;
            sck_d  <= 1'b0;
        end else begin
            cs_s   <= {cs_s[0], spi_cs_n};
            sck_s  <= {sck_s[0], spi_sck};
            mosi_s <= {mosi_s[0], spi_mosi};
            sck_d  <= sck_s[1];
        end
    end

    wire cs       = cs_s[1];
    wire mosi     = mosi_s[1];
    wire sck_rise = sck_s[1] & ~sck_d;
    wire sck_fall = ~sck_s[1] & sck_d;

    reg [3:0] bitcnt;
    reg [7:0] hdr;
    reg [7:0] dat;
    reg [7:0] tx;
    reg       miso_r;
    reg       rw;
    reg [6:0] addr;

    wire [6:0] addr_now = {hdr[5:0], mosi};
    wire [7:0] wr_now   = {dat[6:0], mosi};

    assign rd_addr     = addr_now;
    assign spi_miso    = miso_r;
    assign spi_miso_oe = ~cs;

    always @(posedge clk) begin
        if (!rst_n) begin
            bitcnt  <= 4'd0;
            hdr     <= 8'd0;
            dat     <= 8'd0;
            tx      <= 8'd0;
            miso_r  <= 1'b0;
            rw      <= 1'b0;
            addr    <= 7'd0;
            wr_en   <= 1'b0;
            wr_addr <= 7'd0;
            wr_data <= 8'd0;
        end else begin
            wr_en <= 1'b0;

            if (cs) begin
                bitcnt <= 4'd0;
                miso_r <= 1'b0;
            end else if (sck_rise) begin
                bitcnt <= bitcnt + 4'd1;

                if (bitcnt < 4'd8)
                    hdr <= {hdr[6:0], mosi};
                else
                    dat <= {dat[6:0], mosi};

                // last header bit: latch rw and address, load read value
                if (bitcnt == 4'd7) begin
                    rw   <= hdr[6];
                    addr <= addr_now;
                    tx   <= rd_data;
                end

                // last data bit: strobe the write
                if (bitcnt == 4'd15 && rw) begin
                    wr_en   <= 1'b1;
                    wr_addr <= addr;
                    wr_data <= wr_now;
                end

            end else if (sck_fall) begin
                if (bitcnt >= 4'd8) begin
                    miso_r <= tx[7];
                    tx     <= {tx[6:0], 1'b0};
                end
            end
        end
    end

endmodule