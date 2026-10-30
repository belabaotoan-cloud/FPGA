// ============================================================================
// Module Name: gowin_top_soc
// Description: Top-Level Gowin Primer 20K Edge AI SoC Architecture
//              End-to-End Signal Chain:
//                1. ADXL345 3-Axis Accelerometer Ingest (3.2 kHz SPI Mode 3)
//                2. 32-Tap Causal Min-Phase FIR Bandpass (280 - 1400 Hz)
//                3. Causal Envelope Demodulator (Rectifier + Butterworth IIR)
//                4. 512-Point FFT Hardware Acceleration Engine (Gowin FFT IP)
//                5. Hardware Telemetry & Inference SPI DMA Master to Sonix MCU
// Target:      Gowin GW2A-LV18PG256 (20K LUTs, 48 DSP18E, 46 BSRAMs)
// ============================================================================

`timescale 1ns / 1ps

module gowin_top_soc #(
    parameter USE_ADXL345 = 1,  // 1: Live ADXL345 SPI Mode (3.2 kHz), 0: HIL / 12kHz Mode
    // FIX: String ternary is not valid Verilog-2005. Use separate parameter sets per mode.
    // With USE_ADXL345=1 (default), weight files are in roms/adxl345/
    parameter WEIGHT_FILE_BANK0 = "roms/adxl345/npu_weights_bank0.hex",
    parameter WEIGHT_FILE_BANK1 = "roms/adxl345/npu_weights_bank1.hex",
    parameter WEIGHT_FILE_BANK2 = "roms/adxl345/npu_weights_bank2.hex",
    parameter WEIGHT_FILE_BANK3 = "roms/adxl345/npu_weights_bank3.hex",
    parameter WEIGHT_FILE_BANK4 = "roms/adxl345/npu_weights_bank4.hex",
    parameter WEIGHT_FILE_BANK5 = "roms/adxl345/npu_weights_bank5.hex"
)(
    input  wire                 clk_100m,         // 100 MHz Master Clock
    input  wire                 rst_n_pin,        // Active-Low Reset Pushbutton
    
    // Physical ADXL345 / GY-291 SPI Interface (PMOD0 Header J3)
    output wire                 adxl_cs_n,        // Chip Select (Pin 1)
    output wire                 adxl_sclk,        // ~4.17 MHz SPI Clock (Pin 4)
    output wire                 adxl_mosi,        // MOSI (SDA/SDI Pin 2)
    input  wire                 adxl_miso,        // MISO (SDO Pin 3)
    input  wire                 adxl_int1,        // DATA_READY Strobe (Pin 7)
    
    // Optional Fallback / External Ingest Interface (HIL Streamer / CWRU 12kHz)
    input  wire                 sensor_sample_en, // External Sample Strobe
    input  wire signed [15:0]   sensor_data_in,   // 16-bit Raw Acceleration Sample
    
    // External Tachometer & Telemetry Inputs
    input  wire [15:0]          shaft_speed_fr,   // Rotational frequency fr (0.1 Hz)
    input  wire [15:0]          temperature_c,    // Bearing housing temperature (0.1 C)
    
    // High-Speed SPI DMA Interface to Sonix MCU (SN32F407 / SN34F788)
    output wire                 spi_sck,          // SPI Clock (12.5 MHz, Mode 0)
    output wire                 spi_cs_n,         // Active-Low SPI Chip Select
    output wire                 spi_mosi,         // Master Out Slave In
    output wire                 mcu_irq_pulse,    // Hardware IRQ Strobe to MCU GPIO
    
    // Direct UART Spectrum Streamer to PC (115200 Baud, 8N1)
    output wire                 uart_txd,
    
    // Status & Diagnostic LEDs
    output reg                  led_alive,        // Heartbeat LED
    output reg                  led_infer_busy,   // FFT / NPU processing LED
    output reg                  led_ood_alert     // Out-of-Distribution Alert LED
);

    // =========================================================================
    // 1. Clock & Reset Management
    // =========================================================================
    reg [23:0] heartbeat_cnt;
    always @(posedge clk_100m or negedge rst_n_pin) begin
        if (!rst_n_pin) begin
            heartbeat_cnt <= 24'd0;
            led_alive     <= 1'b0;
        end else begin
            heartbeat_cnt <= heartbeat_cnt + 1'b1;
            led_alive     <= heartbeat_cnt[23]; // ~6 Hz Blink
        end
    end

    // =========================================================================
    // 2. Stage 1 DSP: ADXL345 Master Ingest & 32-Tap Causal Min-Phase FIR
    // =========================================================================
    wire signed [15:0] adxl_x, adxl_y, adxl_z;
    wire               adxl_sample_valid;
    wire               adxl_ready;

    adxl345_spi_master #(
        .CLK_FREQ_HZ (100_000_000),
        .SPI_SCLK_HZ (4_166_667),
        .DATA_WIDTH  (16)
    ) u_adxl_master (
        .clk          (clk_100m),
        .rst_n        (rst_n_pin),
        .spi_cs_n     (adxl_cs_n),
        .spi_sclk     (adxl_sclk),
        .spi_mosi     (adxl_mosi),
        .spi_miso     (adxl_miso),
        .adxl_int1    (adxl_int1),
        .accel_x      (adxl_x),
        .accel_y      (adxl_y),
        .accel_z      (adxl_z),
        .sample_valid (adxl_sample_valid),
        .ready        (adxl_ready)
    );

    // Mux input source based on mode
    wire signed [15:0] sample_in     = USE_ADXL345 ? adxl_z : sensor_data_in;
    wire               sample_strobe = USE_ADXL345 ? adxl_sample_valid : sensor_sample_en;

    wire signed [15:0] fir_out;
    wire               fir_valid;

    generate
        if (USE_ADXL345) begin: gen_adxl_fir
            fir_adxl345_32 #(
                .DATA_WIDTH  (16),
                .COEFF_WIDTH (16),
                .TAPS        (32)
            ) u_fir_adxl (
                .clk        (clk_100m),
                .rst_n      (rst_n_pin),
                .sample_en  (sample_strobe),
                .x_in       (sample_in),
                .y_out      (fir_out),
                .valid_out  (fir_valid)
            );
        end else begin: gen_legacy_fir
            fir_minphase_32 #(
                .DATA_WIDTH  (16),
                .COEFF_WIDTH (16),
                .TAPS        (32)
            ) u_fir_legacy (
                .clk        (clk_100m),
                .rst_n      (rst_n_pin),
                .sample_en  (sample_strobe),
                .x_in       (sample_in),
                .y_out      (fir_out),
                .valid_out  (fir_valid)
            );
        end
    endgenerate

    // =========================================================================
    // 3. Stage 2 DSP: Causal Envelope Demodulator (Rectifier + IIR LPF)
    // =========================================================================
    wire signed [15:0] env_out;
    wire               env_valid;

    envelope_demodulator #(
        .WIDTH (16)
    ) u_envelope (
        .clk        (clk_100m),
        .rst_n      (rst_n_pin),
        .sample_en  (fir_valid),
        .y_filtered (fir_out),
        .env_out    (env_out),
        .valid_out  (env_valid)
    );

    // =========================================================================
    // 4. Stage 2.5: FFT Engine (512-Point Real Input with Handshake Controller)
    // =========================================================================
    wire                fft_start;
    wire signed [15:0]  fft_xn_re;
    wire signed [15:0]  fft_xn_im;
    wire [8:0]          fft_idx;
    wire signed [15:0]  fft_xk_re;
    wire signed [15:0]  fft_xk_im;
    wire                fft_sod;
    wire                fft_ipd;
    wire                fft_eod;
    wire                fft_busy;
    wire                fft_soud;
    wire                fft_opd;
    wire                fft_eoud;

    wire [8:0]          fft_bram_addr;
    wire [15:0]         fft_bram_data;
    wire                fft_bram_we;
    wire                fft_frame_done;
    wire                fft_ctrl_busy;

    fft_stream_controller u_fft_ctrl (
        .clk        (clk_100m),
        .rst_n      (rst_n_pin),
        .env_valid  (env_valid),
        .env_out    (env_out),
        .fft_start  (fft_start),
        .fft_xn_re  (fft_xn_re),
        .fft_xn_im  (fft_xn_im),
        .fft_idx    (fft_idx),
        .fft_xk_re  (fft_xk_re),
        .fft_xk_im  (fft_xk_im),
        .fft_sod    (fft_sod),
        .fft_ipd    (fft_ipd),
        .fft_eod    (fft_eod),
        .fft_busy   (fft_busy),
        .fft_soud   (fft_soud),
        .fft_opd    (fft_opd),
        .fft_eoud   (fft_eoud),
        .bram_addr  (fft_bram_addr),
        .bram_data  (fft_bram_data),
        .bram_we    (fft_bram_we),
        .frame_done (fft_frame_done),
        .busy       (fft_ctrl_busy)
    );

    FFT_Top u_fft (
        .clk    (clk_100m),
        .rst    (~rst_n_pin),  // FFT active-high reset
        .start  (fft_start),
        .xn_re  (fft_xn_re),
        .xn_im  (fft_xn_im),
        .idx    (fft_idx),
        .xk_re  (fft_xk_re),
        .xk_im  (fft_xk_im),
        .sod    (fft_sod),
        .ipd    (fft_ipd),
        .eod    (fft_eod),
        .busy   (fft_busy),
        .soud   (fft_soud),
        .opd    (fft_opd),
        .eoud   (fft_eoud)
    );

    // =========================================================================
    // 5. Dual-Port Synchronous Block RAM (257 bins x 16-bit)
    // =========================================================================
    reg [15:0] spectrum_bram [0:256];
    reg [15:0] spi_bram_data_q;

    // Port A: Write from FFT Stream Controller
    always @(posedge clk_100m) begin
        if (fft_bram_we && (fft_bram_addr <= 9'd256)) begin
            spectrum_bram[fft_bram_addr] <= fft_bram_data;
        end
    end

    localparam SEQ_IDLE         = 3'd0;
    localparam SEQ_FEED_SPEC    = 3'd1;
    localparam SEQ_FEED_PRIOR   = 3'd2;
    localparam SEQ_NPU_START    = 3'd3;
    localparam SEQ_NPU_WAIT     = 3'd4;

    reg [2:0]  seq_state;
    reg [8:0]  feed_idx;
    // Port B: Synchronous Read Time-Multiplexed between NPU and SPI (True Dual-Port BSRAM)
    wire [8:0]  spi_tx_bram_addr;
    wire [8:0]  bram_port_b_addr;
    wire [15:0] spi_bram_data;

    assign bram_port_b_addr = (seq_state == SEQ_FEED_SPEC) ? ((feed_idx > 9'd256) ? 9'd256 : feed_idx) : spi_tx_bram_addr;

    always @(posedge clk_100m) begin
        spi_bram_data_q <= spectrum_bram[bram_port_b_addr];
    end
    assign spi_bram_data = spi_bram_data_q;

    // =========================================================================
    // 6. Stage 3 NPU: Gowin 12-Way Systolic Acceleration Core
    // =========================================================================
    reg                 npu_start;
    wire                npu_ready;
    wire                npu_done;
    reg                 npu_in_valid;
    reg  [8:0]          npu_in_addr;
    reg  [15:0]         npu_in_data;

    wire [1:0]          npu_pred_class;
    wire signed [31:0]  npu_logits;
    wire [15:0]         npu_vacuity_q15;
    wire [15:0]         npu_rul_q15;
    wire                npu_ood_alert;


    npu_12way_core #(
        .WEIGHT_FILE_BANK0 (WEIGHT_FILE_BANK0),
        .WEIGHT_FILE_BANK1 (WEIGHT_FILE_BANK1),
        .WEIGHT_FILE_BANK2 (WEIGHT_FILE_BANK2),
        .WEIGHT_FILE_BANK3 (WEIGHT_FILE_BANK3),
        .WEIGHT_FILE_BANK4 (WEIGHT_FILE_BANK4),
        .WEIGHT_FILE_BANK5 (WEIGHT_FILE_BANK5)
    ) u_npu (
        .clk             (clk_100m),
        .rst_n           (rst_n_pin),
        .start           (npu_start),
        .ready           (npu_ready),
        .done            (npu_done),
        .in_valid        (npu_in_valid),
        .in_addr         (npu_in_addr),
        .in_data         (npu_in_data),
        .predicted_class (npu_pred_class),
        .logits_packed   (npu_logits),
        .vacuity_q15     (npu_vacuity_q15),
        .rul_q15         (npu_rul_q15),
        .ood_alert       (npu_ood_alert)
    );

    // =========================================================================
    // 7. Stage 4 SPI DMA Telemetry: Packet Streaming to Sonix MCU
    // =========================================================================
    reg         spi_start_tx;
    wire        spi_tx_busy;

    spi_master_packet #(
        .TOTAL_BYTES (524),
        .NUM_BINS    (257)
    ) u_spi_tx (
        .clk         (clk_100m),
        .rst_n       (rst_n_pin),
        .start_tx    (spi_start_tx),
        .shaft_speed (shaft_speed_fr),
        .temperature (temperature_c),
        .bram_addr   (spi_tx_bram_addr),
        .bram_data   (spi_bram_data),
        .spi_sck     (spi_sck),
        .spi_cs_n    (spi_cs_n),
        .spi_mosi    (spi_mosi),
        .irq_pulse   (mcu_irq_pulse),
        .tx_busy     (spi_tx_busy)
    );

    // =========================================================================
    // 7.1 UART Direct Spectrum Streamer (Debug Monitor to PC @ 115200 8N1)
    // =========================================================================
    spectrum_uart_streamer #(
        .CLK_HZ (100_000_000),
        .BAUD   (115200)
    ) u_uart_stream (
        .clk        (clk_100m),
        .rst_n      (rst_n_pin),
        .bram_we    (fft_bram_we),
        .bram_addr  (fft_bram_addr),
        .bram_data  (fft_bram_data),
        .frame_done (fft_frame_done),
        .uart_txd   (uart_txd)
    );

    // =========================================================================
    // 8. Master Control Sequencer & LED Indicators
    // =========================================================================
    always @(posedge clk_100m or negedge rst_n_pin) begin
        if (!rst_n_pin) begin
            seq_state      <= SEQ_IDLE;
            feed_idx       <= 9'd0;
            npu_start      <= 1'b0;
            npu_in_valid   <= 1'b0;
            npu_in_addr    <= 9'd0;
            npu_in_data    <= 16'd0;
            spi_start_tx   <= 1'b0;
            led_infer_busy <= 1'b0;
            led_ood_alert  <= 1'b0;
        end else begin
            // Visual indicators:
            led_infer_busy <= (!npu_ready) | fft_ctrl_busy;

            case (seq_state)
                SEQ_IDLE: begin
                    npu_start    <= 1'b0;
                    npu_in_valid <= 1'b0;
                    if (fft_frame_done) begin
                        if (!spi_tx_busy) spi_start_tx <= 1'b1;
                        seq_state <= SEQ_FEED_SPEC;
                        feed_idx  <= 9'd0;
                    end else begin
                        spi_start_tx <= 1'b0;
                    end
                end

                SEQ_FEED_SPEC: begin
                    spi_start_tx <= 1'b0;
                    npu_in_valid <= 1'b1;
                    if (feed_idx != 9'd0) begin
                        npu_in_valid <= 1'b1;
                        npu_in_addr  <= feed_idx - 9'd1;
                        npu_in_data  <= spi_bram_data;
                    end else begin
                        npu_in_valid <= 1'b0;
                    end

                    if (feed_idx < 9'd257) begin
                        feed_idx <= feed_idx + 9'd1;
                    end else begin
                        seq_state <= SEQ_FEED_PRIOR;
                        feed_idx  <= 9'd0;
                    end
                end

                SEQ_FEED_PRIOR: begin
                    npu_in_valid <= 1'b1;
                    npu_in_addr  <= 9'd257 + feed_idx;
                    case (feed_idx)
                        9'd0: npu_in_data <= 16'd45; // BPFO prior
                        9'd1: npu_in_data <= 16'd68; // BPFI prior
                        9'd2: npu_in_data <= 16'd29; // BSF prior
                        default: npu_in_data <= 16'd5; // FTF prior
                    endcase

                    if (feed_idx < 9'd3) begin
                        feed_idx <= feed_idx + 9'd1;
                    end else begin
                        seq_state <= SEQ_NPU_START;
                    end
                end

                SEQ_NPU_START: begin
                    npu_in_valid <= 1'b0;
                    npu_start    <= 1'b1;
                    seq_state    <= SEQ_NPU_WAIT;
                end

                SEQ_NPU_WAIT: begin
                    npu_start <= 1'b0;
                    if (npu_done) begin
                        led_ood_alert <= npu_ood_alert | (npu_pred_class != 2'b00);
                        seq_state     <= SEQ_IDLE;
                    end
                end

                default: seq_state <= SEQ_IDLE;
            endcase
        end
    end

endmodule