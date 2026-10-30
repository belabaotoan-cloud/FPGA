// ============================================================================
// Module Name: adxl345_spi_master
// Description: Hardware SPI Master with Autonomous Configuration & 
//              AUTO-RECOVERY WATCHDOG for Analog Devices ADXL345 / GY-291.
// Features:    - Automatic 10ms Power-On Reset & 5-register setup sequence
//              - Burst-read 6 bytes @ 3.2 kHz (ODR = 3200 Hz)
//              - Auto-Recovery Watchdog: Periodically monitors POWER_CTL (0x2D).
//                If sensor is unplugged or loses power and resets to Standby,
//                the FSM automatically re-wakes and re-configures the sensor
//                within milliseconds without requiring FPGA reprogramming!
// Target:      Gowin Tang Primer 20K (GW2A-LV18PG256) @ 100 MHz System Clock
// ============================================================================

`timescale 1ns / 1ps

module adxl345_spi_master #(
    parameter CLK_FREQ_HZ = 100_000_000,
    parameter SPI_SCLK_HZ = 4_166_667,    // ~4.17 MHz -> HALF=12
    parameter DATA_WIDTH  = 16
)(
    input  wire                          clk,
    input  wire                          rst_n,          // Active-Low Async Reset

    // Physical SPI Pins
    output reg                           spi_cs_n,       // Active-Low Chip Select
    output wire                          spi_sclk,       // SCK driven by spi_m3
    output wire                          spi_mosi,       // MOSI driven by spi_m3
    input  wire                          spi_miso,

    input  wire                          adxl_int1,      // DATA_READY strobe

    // Outputs to DSP pipeline
    output reg  signed [DATA_WIDTH-1:0]  accel_x,
    output reg  signed [DATA_WIDTH-1:0]  accel_y,
    output reg  signed [DATA_WIDTH-1:0]  accel_z,
    output reg                           sample_valid,   // 1-cycle strobe @ 3200 Hz
    output reg                           ready           // High after init complete
);

    // HALF = CLK / (2 * SCK) = 100_000_000 / (2 * 4_166_667) = 12
    localparam SCLK_HALF = CLK_FREQ_HZ / (2 * SPI_SCLK_HZ);

    // -------------------------------------------------------------------------
    // spi_m3 instance (SPI Mode 3, 8-bit MSB-first)
    // -------------------------------------------------------------------------
    reg        spi_start;
    reg  [7:0] spi_tx;
    wire [7:0] spi_rx;
    wire       spi_done;
    wire       spi_busy_w;

    spi_m3 #(.HALF(SCLK_HALF)) u_spi (
        .clk   (clk),
        .rst_n (rst_n),
        .start (spi_start),
        .tx    (spi_tx),
        .rx    (spi_rx),
        .done  (spi_done),
        .busy  (spi_busy_w),
        .sck   (spi_sclk),
        .mosi  (spi_mosi),
        .miso  (spi_miso)
    );

    // -------------------------------------------------------------------------
    // 3-FF synchronizer for async INT1, rising-edge detect
    // -------------------------------------------------------------------------
    reg int1_s1, int1_s2, int1_s3;
    wire int1_rising = int1_s2 && !int1_s3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            int1_s1 <= 1'b0;
            int1_s2 <= 1'b0;
            int1_s3 <= 1'b0;
        end else begin
            int1_s1 <= adxl_int1;
            int1_s2 <= int1_s1;
            int1_s3 <= int1_s2;
        end
    end

    // -------------------------------------------------------------------------
    // Configuration table: 5 x {addr, data}
    // -------------------------------------------------------------------------
    reg [7:0] cfg_addr [0:4];
    reg [7:0] cfg_data [0:4];

    initial begin
        cfg_addr[0] = 8'h2C; cfg_data[0] = 8'h0F; // BW_RATE = 3200 Hz ODR
        cfg_addr[1] = 8'h31; cfg_data[1] = 8'h0B; // DATA_FORMAT = Full-Res +/-16g
        cfg_addr[2] = 8'h2E; cfg_data[2] = 8'h80; // INT_ENABLE = DATA_READY
        cfg_addr[3] = 8'h2F; cfg_data[3] = 8'h00; // INT_MAP -> INT1
        cfg_addr[4] = 8'h2D; cfg_data[4] = 8'h08; // POWER_CTL = Measure
    end

    // -------------------------------------------------------------------------
    // FSM States
    // -------------------------------------------------------------------------
    localparam ST_IDLE          = 4'd0;
    localparam ST_POR_DELAY     = 4'd1;  // 10 ms POR wait
    localparam ST_CFG_CS_LOW    = 4'd2;  // Assert CS, 2-cycle setup
    localparam ST_CFG_SEND_ADDR = 4'd3;  // Transmit register address
    localparam ST_CFG_SEND_DATA = 4'd4;  // Transmit register value
    localparam ST_CFG_CS_HIGH   = 4'd5;  // Deassert CS, hold time
    localparam ST_WAIT_DRDY     = 4'd6;  // Wait INT1 rising edge or fallback 3.2 kHz timer
    localparam ST_BURST_CS_LOW  = 4'd7;  // Assert CS for burst read
    localparam ST_BURST_CMD     = 4'd8;  // Send 0xF2 read command
    localparam ST_BURST_RECV    = 4'd9;  // Receive 6 data bytes
    localparam ST_BURST_CS_HIGH = 4'd10; // Deassert CS
    localparam ST_PARSE         = 4'd11; // Latch X/Y/Z, pulse valid
    localparam ST_CHECK_CS_LOW  = 4'd12; // Watchdog: Assert CS to read POWER_CTL (0x2D)
    localparam ST_CHECK_CMD     = 4'd13; // Watchdog: Send 0xAD read command
    localparam ST_CHECK_RECV    = 4'd14; // Watchdog: Receive POWER_CTL byte
    localparam ST_CHECK_EVAL    = 4'd15; // Watchdog: Verify == 0x08 (Measure mode)

    reg [3:0]  state;
    reg [20:0] delay_cnt;
    reg [2:0]  cfg_step;        // 0..4: which register to configure
    reg [2:0]  byte_cnt;        // 0..5: which burst byte is being received
    reg [7:0]  rx_buf [0:5];    // DATAX0, DATAX1, DATAY0, DATAY1, DATAZ0, DATAZ1
    reg [7:0]  reg_2d_val;      // Read value of POWER_CTL register
    reg [7:0]  sample_chk_cnt;  // Counts 0..100 samples to trigger periodic health check
    reg [3:0]  err_sample_cnt;  // Counts consecutive all-0 or all-FF frames

    // Detect all-0 or all-1 (indicates sensor disconnected or power lost)
    wire frame_all_ff = (&rx_buf[0]) && (&rx_buf[1]) && (&rx_buf[2]) &&
                        (&rx_buf[3]) && (&rx_buf[4]) && (&rx_buf[5]);
    wire frame_all_00 = (~|rx_buf[0]) && (~|rx_buf[1]) && (~|rx_buf[2]) &&
                        (~|rx_buf[3]) && (~|rx_buf[4]) && (~|rx_buf[5]);

    // -------------------------------------------------------------------------
    // Main FSM
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            spi_cs_n       <= 1'b1;
            spi_start      <= 1'b0;
            spi_tx         <= 8'h00;
            sample_valid   <= 1'b0;
            ready          <= 1'b0;
            delay_cnt      <= 21'd0;
            cfg_step       <= 3'd0;
            byte_cnt       <= 3'd0;
            accel_x        <= 16'sd0;
            accel_y        <= 16'sd0;
            accel_z        <= 16'sd0;
            reg_2d_val     <= 8'h00;
            sample_chk_cnt <= 8'd0;
            err_sample_cnt <= 4'd0;
        end else begin
            // Default: de-assert 1-cycle pulses
            spi_start    <= 1'b0;
            sample_valid <= 1'b0;

            case (state)

                // --------------------------------------------------------------
                ST_IDLE: begin
                    delay_cnt      <= 21'd0;
                    cfg_step       <= 3'd0;
                    sample_chk_cnt <= 8'd0;
                    err_sample_cnt <= 4'd0;
                    ready          <= 1'b0;
                    state          <= ST_POR_DELAY;
                end

                // --------------------------------------------------------------
                // Wait 10 ms for ADXL345 internal POR
                ST_POR_DELAY: begin
                    spi_cs_n <= 1'b1;
                    if (delay_cnt < 21'd1_000_000) // 10 ms @ 100 MHz
                        delay_cnt <= delay_cnt + 1'b1;
                    else begin
                        delay_cnt <= 21'd0;
                        cfg_step  <= 3'd0;
                        state     <= ST_CFG_CS_LOW;
                    end
                end

                // --------------------------------------------------------------
                // Assert CS_N low, 2-cycle setup before first SCK
                ST_CFG_CS_LOW: begin
                    spi_cs_n  <= 1'b0;
                    delay_cnt <= delay_cnt + 1'b1;
                    if (delay_cnt >= 21'd1) begin       // 2 cycles
                        delay_cnt <= 21'd0;
                        spi_tx    <= cfg_addr[cfg_step]; // register address (write cmd)
                        spi_start <= 1'b1;
                        state     <= ST_CFG_SEND_ADDR;
                    end
                end

                // --------------------------------------------------------------
                // Wait for address byte done, then send data byte
                ST_CFG_SEND_ADDR: begin
                    if (spi_done) begin
                        spi_tx    <= cfg_data[cfg_step];
                        spi_start <= 1'b1;
                        state     <= ST_CFG_SEND_DATA;
                    end
                end

                // --------------------------------------------------------------
                // Wait for data byte done, then deassert CS
                ST_CFG_SEND_DATA: begin
                    if (spi_done) begin
                        state <= ST_CFG_CS_HIGH;
                    end
                end

                // --------------------------------------------------------------
                // Deassert CS_N, hold 2 cycles, advance to next register
                ST_CFG_CS_HIGH: begin
                    spi_cs_n  <= 1'b1;
                    delay_cnt <= delay_cnt + 1'b1;
                    if (delay_cnt >= 21'd1) begin       // 2-cycle hold
                        delay_cnt <= 21'd0;
                        if (cfg_step == 3'd4) begin
                            ready          <= 1'b1;     // All 5 registers written
                            sample_chk_cnt <= 8'd0;
                            err_sample_cnt <= 4'd0;
                            state          <= ST_WAIT_DRDY;
                        end else begin
                            cfg_step <= cfg_step + 1'b1;
                            state    <= ST_CFG_CS_LOW;
                        end
                    end
                end

                // --------------------------------------------------------------
                // Wait for INT1 DATA_READY rising edge or fallback 3.2 kHz timer (31250 cycles @ 100MHz)
                ST_WAIT_DRDY: begin
                    spi_cs_n  <= 1'b1;
                    delay_cnt <= delay_cnt + 1'b1;
                    // Trigger read on interrupt OR timer (312.5 us)
                    if (int1_rising || delay_cnt >= 21'd31_250) begin
                        delay_cnt <= 21'd0;
                        state     <= ST_BURST_CS_LOW;
                    end
                    // Watchdog: If no valid sample for 15 ms (1.5M cycles), check sensor health!
                    else if (delay_cnt >= 21'd1_500_000) begin
                        delay_cnt <= 21'd0;
                        state     <= ST_CHECK_CS_LOW;
                    end
                end

                // --------------------------------------------------------------
                // Assert CS_N for burst read
                ST_BURST_CS_LOW: begin
                    spi_cs_n  <= 1'b0;
                    delay_cnt <= delay_cnt + 1'b1;
                    if (delay_cnt >= 21'd1) begin
                        delay_cnt <= 21'd0;
                        spi_tx    <= 8'hF2; // Read=1, Multi=1, Addr=0x32 (DATAX0)
                        spi_start <= 1'b1;
                        state     <= ST_BURST_CMD;
                    end
                end

                // --------------------------------------------------------------
                // Wait for command byte done, then start receiving data
                ST_BURST_CMD: begin
                    if (spi_done) begin
                        byte_cnt  <= 3'd0;
                        spi_tx    <= 8'hFF;  // dummy TX during read
                        spi_start <= 1'b1;
                        state     <= ST_BURST_RECV;
                    end
                end

                // --------------------------------------------------------------
                // Receive 6 bytes: X0, X1, Y0, Y1, Z0, Z1
                ST_BURST_RECV: begin
                    if (spi_done) begin
                        rx_buf[byte_cnt] <= spi_rx;
                        if (byte_cnt == 3'd5) begin
                            state <= ST_BURST_CS_HIGH;  // all 6 bytes received
                        end else begin
                            byte_cnt  <= byte_cnt + 1'b1;
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;          // request next byte
                        end
                    end
                end

                // --------------------------------------------------------------
                ST_BURST_CS_HIGH: begin
                    spi_cs_n <= 1'b1;
                    state    <= ST_PARSE;
                end

                // --------------------------------------------------------------
                // Parse little-endian 16-bit values, assert sample_valid 1 cycle
                ST_PARSE: begin
                    accel_x       <= {rx_buf[1], rx_buf[0]}; // DATAX1:DATAX0
                    accel_y       <= {rx_buf[3], rx_buf[2]}; // DATAY1:DATAY0
                    accel_z       <= {rx_buf[5], rx_buf[4]}; // DATAZ1:DATAZ0
                    sample_valid  <= 1'b1;
                    delay_cnt     <= 21'd0;

                    // Monitor suspicious all-0 or all-FF frames (sensor cable disconnected)
                    if (frame_all_ff || frame_all_00) begin
                        if (err_sample_cnt < 4'd15)
                            err_sample_cnt <= err_sample_cnt + 1'b1;
                    end else begin
                        err_sample_cnt <= 4'd0;
                    end

                    // Periodic health check every 100 samples (~31ms) OR after 8 corrupted frames
                    if (err_sample_cnt >= 4'd8 || sample_chk_cnt >= 8'd100) begin
                        sample_chk_cnt <= 8'd0;
                        state          <= ST_CHECK_CS_LOW;
                    end else begin
                        sample_chk_cnt <= sample_chk_cnt + 1'b1;
                        state          <= ST_WAIT_DRDY;
                    end
                end

                // ==============================================================
                // AUTO-RECOVERY WATCHDOG: Check POWER_CTL (Register 0x2D)
                // ==============================================================
                ST_CHECK_CS_LOW: begin
                    spi_cs_n  <= 1'b0;
                    delay_cnt <= delay_cnt + 1'b1;
                    if (delay_cnt >= 21'd1) begin
                        delay_cnt <= 21'd0;
                        spi_tx    <= 8'hAD; // Read=1, Single=0, Addr=0x2D (POWER_CTL)
                        spi_start <= 1'b1;
                        state     <= ST_CHECK_CMD;
                    end
                end

                ST_CHECK_CMD: begin
                    if (spi_done) begin
                        spi_tx    <= 8'hFF; // Dummy TX to read 0x2D byte
                        spi_start <= 1'b1;
                        state     <= ST_CHECK_RECV;
                    end
                end

                ST_CHECK_RECV: begin
                    if (spi_done) begin
                        reg_2d_val <= spi_rx;
                        spi_cs_n   <= 1'b1;
                        state      <= ST_CHECK_EVAL;
                    end
                end

                ST_CHECK_EVAL: begin
                    delay_cnt <= 21'd0;
                    // If POWER_CTL == 0x08, sensor is healthy and measuring!
                    if (reg_2d_val == 8'h08) begin
                        err_sample_cnt <= 4'd0;
                        state          <= ST_WAIT_DRDY;
                    end
                    // Sensor was power-cycled into Standby (0x00) or unplugged (0xFF/other)
                    else begin
                        ready          <= 1'b0;
                        cfg_step       <= 3'd0;
                        state          <= ST_POR_DELAY; // Auto-Reconfigure to wake it up!
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
