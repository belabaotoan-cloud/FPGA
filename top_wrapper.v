`timescale 1ns / 1ps
// =====================================================================
//  top_wrapper.v - Wrapper chua PLL 27->100 MHz + gowin_top_soc
//  Tang Primer 20K Lite (GW2A-LV18PG256C8/I7)
// =====================================================================
module top_wrapper (
    input  wire clk_27m,        // 27 MHz onboard oscillator (H11)

    // ADXL345 SPI
    output wire adxl_cs_n,
    output wire adxl_sclk,
    output wire adxl_mosi,
    input  wire adxl_miso,
    input  wire adxl_int1,

    // SPI to MCU
    output wire spi_sck,
    output wire spi_cs_n,
    output wire spi_mosi,
    output wire mcu_irq_pulse,

    // Direct UART Spectrum Streamer to PC
    output wire uart_txd,

    // LEDs
    output wire led_alive,
    output wire led_infer_busy,
    output wire led_ood_alert
);

    // ------------------------------------------------------------------
    // 1. PLL: 27 MHz -> ~100 MHz
    // ------------------------------------------------------------------
    wire clk_100m;
    wire pll_lock;

    rPLL #(
        .FCLKIN          ("27"),
        .DYN_IDIV_SEL    ("false"),
        .IDIV_SEL        (6),
        .DYN_FBDIV_SEL   ("false"),
        .FBDIV_SEL       (25),
        .DYN_ODIV_SEL    ("false"),
        .ODIV_SEL        (8),
        .PSDA_SEL        ("0000"),
        .DYN_DA_EN       ("true"),
        .DUTYDA_SEL      ("1000"),
        .CLKOUT_FT_DIR   (1'b1),
        .CLKOUTP_FT_DIR  (1'b1),
        .CLKOUT_DLY_STEP (0),
        .CLKOUTP_DLY_STEP(0),
        .CLKFB_SEL       ("internal"),
        .CLKOUT_BYPASS   ("false"),
        .CLKOUTP_BYPASS  ("false"),
        .CLKOUTD_BYPASS  ("false"),
        .DYN_SDIV_SEL    (2),
        .CLKOUTD_SRC     ("CLKOUT"),
        .CLKOUTD3_SRC    ("CLKOUT"),
        .DEVICE          ("GW2A-18C")
    ) u_pll (
        .CLKOUT  (clk_100m),
        .LOCK    (pll_lock),
        .CLKOUTP (),
        .CLKOUTD (),
        .CLKOUTD3(),
        .RESET   (1'b0),
        .RESET_P (1'b0),
        .CLKIN   (clk_27m),
        .CLKFB   (1'b0),
        .FBDSEL  (6'b000000),
        .IDSEL   (6'b000000),
        .ODSEL   (6'b000000),
        .PSDA    (4'b0000),
        .DUTYDA  (4'b0000),
        .FDLY    (4'b0000)
    );

    // ------------------------------------------------------------------
    // 2. Power-On Reset (giu reset cho den khi PLL lock)
    // ------------------------------------------------------------------
    reg [15:0] por_cnt = 16'd0;
    wire       por_done = (por_cnt == 16'hFFFF);

    always @(posedge clk_27m) begin
        if (!por_done) por_cnt <= por_cnt + 16'd1;
    end

    wire rst_req_n = por_done; // Khong phu thuoc pll_lock de tranh treo reset
    reg  [1:0] rst_sync;
    always @(posedge clk_100m or negedge rst_req_n) begin
        if (!rst_req_n) rst_sync <= 2'b00;
        else            rst_sync <= {rst_sync[0], 1'b1};
    end
    wire rst_n = rst_sync[1];

    // ------------------------------------------------------------------
    // 3. Tie-off cac input khong dung
    // ------------------------------------------------------------------
    wire [15:0] shaft_speed_fr  = 16'd0;
    wire [15:0] temperature_c   = 16'd0;
    wire        sensor_sample_en = 1'b0;
    wire signed [15:0] sensor_data_in = 16'sd0;

    // ------------------------------------------------------------------
    // 4. Instantiate gowin_top_soc
    // ------------------------------------------------------------------
    gowin_top_soc #(
        .USE_ADXL345 (1)
    ) u_soc (
        .clk_100m        (clk_100m),
        .rst_n_pin       (rst_n),

        // ADXL345
        .adxl_cs_n       (adxl_cs_n),
        .adxl_sclk       (adxl_sclk),
        .adxl_mosi       (adxl_mosi),
        .adxl_miso       (adxl_miso),
        .adxl_int1       (adxl_int1),

        // Unused fallback inputs
        .sensor_sample_en (sensor_sample_en),
        .sensor_data_in   (sensor_data_in),
        .shaft_speed_fr   (shaft_speed_fr),
        .temperature_c    (temperature_c),

        // SPI to MCU
        .spi_sck         (spi_sck),
        .spi_cs_n        (spi_cs_n),
        .spi_mosi        (spi_mosi),
        .mcu_irq_pulse   (mcu_irq_pulse),

        // Direct UART Spectrum Streamer
        .uart_txd        (uart_txd),

        // LEDs
        .led_alive       (led_alive),
        .led_infer_busy  (led_infer_busy),
        .led_ood_alert   (led_ood_alert)
    );
  
endmodule