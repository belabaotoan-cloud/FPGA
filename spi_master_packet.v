`timescale 1ns / 1ps
// ============================================================================
// Module Name: spi_master_packet
// Description: SPI Master Packetizer with Pre-Trigger DMA IRQ & CRC-16-CCITT
// Target:      Gowin GW2A-LV18PG256 (Tang Primer 20K) -> Sonix MCU SPI Slave DMA
// Clock:       100 MHz System Clock -> 12.5 MHz SPI Clock (SCK), Mode 0
// Packet:      524 Bytes total:
//              [0:1]     Sync Header: 0xAA55
//              [2:3]     Frame Counter: uint16_t
//              [4:5]     Shaft Speed fr: uint16_t (0.1 Hz units)
//              [6:7]     Temperature: int16_t (0.1 C units)
//              [8:521]   257 Bins: uint16_t * 257 (Big-Endian)
//              [522:523] CRC-16-CCITT: Polynomial 0x1021
// Timing:      Transmission of 524 Bytes @ 12.5 MHz takes exactly 335.36 us!
// Handshake:   Asserts irq_pulse 50 us BEFORE spi_cs_n pulls LOW, giving
//              Sonix MCU EXTI ISR plenty of time to arm the SPI Slave RX DMA!
// ============================================================================

module spi_master_packet #(
    parameter TOTAL_BYTES = 524,
    parameter NUM_BINS    = 257,
    parameter PRE_IRQ_CYCLES = 5000 // 5000 x 10ns = 50 us pre-trigger wait
)(
    input  wire        clk,           // 100 MHz System Clock
    input  wire        rst_n,         // Active-Low Reset
    input  wire        start_tx,      // Pulse to trigger packet transmission
    input  wire [15:0] shaft_speed,   // Shaft Speed fr from tachometer
    input  wire [15:0] temperature,   // Temperature from bearing housing
    
    // Interface to Synchronous Dual-Port Block RAM storing 257 FFT bins
    output reg  [8:0]  bram_addr,     // 0 to 256
    input  wire [15:0] bram_data,     // 16-bit Q0.15 bin magnitude
    
    // SPI Physical Interface to Sonix MCU
    output reg         spi_sck,       // SPI Clock (12.5 MHz, Mode 0)
    output reg         spi_cs_n,      // SPI Active-Low Chip Select
    output reg         spi_mosi,      // Master Out Slave In
    output reg         irq_pulse,     // Hardware interrupt pulse to MCU GPIO (P2.4)
    output reg         tx_busy        // High while transmission in progress
);

    // FSM States
    localparam STATE_IDLE      = 3'd0;
    localparam STATE_PRE_IRQ   = 3'd1;
    localparam STATE_LOAD_BYTE = 3'd2;
    localparam STATE_SHIFT_BIT = 3'd3;
    localparam STATE_DONE      = 3'd4;

    reg [2:0]  state;
    reg [2:0]  clk_div;       // 100 MHz / 8 = 12.5 MHz SCK (counts 0..7)
    reg [9:0]  byte_idx;      // 0 to 523
    reg [2:0]  bit_idx;       // 7 downto 0
    reg [7:0]  current_byte;
    reg [15:0] frame_count;
    reg [15:0] crc16_reg;
    reg [15:0] pre_cnt;

    // CRC-16-CCITT update function (Polynomial: 0x1021)
    function [15:0] update_crc(input [15:0] crc, input [7:0] data);
        integer j;
        reg [15:0] c;
        begin
            c = crc ^ (data << 8);
            for (j = 0; j < 8; j = j + 1) begin
                if (c[15])
                    c = (c << 1) ^ 16'h1021;
                else
                    c = c << 1;
            end
            update_crc = c;
        end
    endfunction

    // Combinational next-byte multiplexer
    wire [7:0] next_byte = (byte_idx == 10'd0)   ? 8'hAA :
                           (byte_idx == 10'd1)   ? 8'h55 :
                           (byte_idx == 10'd2)   ? frame_count[15:8] :
                           (byte_idx == 10'd3)   ? frame_count[7:0] :
                           (byte_idx == 10'd4)   ? shaft_speed[15:8] :
                           (byte_idx == 10'd5)   ? shaft_speed[7:0] :
                           (byte_idx == 10'd6)   ? temperature[15:8] :
                           (byte_idx == 10'd7)   ? temperature[7:0] :
                           (byte_idx <= 10'd521) ? (byte_idx[0] ? bram_data[7:0] : bram_data[15:8]) :
                           (byte_idx == 10'd522) ? crc16_reg[15:8] : crc16_reg[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= STATE_IDLE;
            clk_div      <= 3'd0;
            byte_idx     <= 10'd0;
            bit_idx      <= 3'd7;
            current_byte <= 8'd0;
            frame_count  <= 16'd0;
            crc16_reg    <= 16'hFFFF;
            spi_sck      <= 1'b0;
            spi_cs_n     <= 1'b1;
            spi_mosi     <= 1'b0;
            irq_pulse    <= 1'b0;
            pre_cnt      <= 16'd0;
            tx_busy      <= 1'b0;
            bram_addr    <= 9'd0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    spi_cs_n  <= 1'b1;
                    spi_sck   <= 1'b0;
                    spi_mosi  <= 1'b0;
                    tx_busy   <= 1'b0;
                    clk_div   <= 3'd0;
                    irq_pulse <= 1'b0;

                    if (start_tx) begin
                        // Step 1: Pre-trigger IRQ to allow MCU to arm DMA
                        state     <= STATE_PRE_IRQ;
                        irq_pulse <= 1'b1;
                        pre_cnt   <= PRE_IRQ_CYCLES[15:0];
                        tx_busy   <= 1'b1;
                    end
                end

                STATE_PRE_IRQ: begin
                    // Hold IRQ pulse high during pre-trigger window (50 us)
                    if (pre_cnt != 16'd0) begin
                        pre_cnt <= pre_cnt - 1'b1;
                    end else begin
                        irq_pulse    <= 1'b0;
                        spi_cs_n     <= 1'b0; // Assert CS# low
                        byte_idx     <= 10'd0;
                        bit_idx      <= 3'd7;
                        crc16_reg    <= 16'hFFFF;
                        bram_addr    <= 9'd0;
                        frame_count  <= frame_count + 1'b1;
                        state        <= STATE_LOAD_BYTE;
                    end
                end

                STATE_LOAD_BYTE: begin
                    current_byte <= next_byte;
                    // Compute running CRC across payload bytes 0..521
                    if (byte_idx <= 10'd521) begin
                        crc16_reg <= update_crc(crc16_reg, next_byte);
                    end

                    // Prefetch next BRAM address on even byte boundaries
                    if (byte_idx >= 10'd9 && byte_idx <= 10'd521 && byte_idx[0]) begin
                        bram_addr <= bram_addr + 1'b1;
                    end

                    bit_idx  <= 3'd7;
                    clk_div  <= 3'd0;
                    spi_sck  <= 1'b0;
                    spi_mosi <= next_byte[7]; // Setup MSB before first SCK rising edge
                    state    <= STATE_SHIFT_BIT;
                end

                STATE_SHIFT_BIT: begin
                    clk_div <= clk_div + 1'b1;

                    // 12.5 MHz SCK Timing (100 MHz / 8):
                    // clk_div == 0: SCK Low, drive MOSI
                    if (clk_div == 3'd0) begin
                        spi_sck  <= 1'b0;
                        spi_mosi <= current_byte[bit_idx];
                    end
                    // clk_div == 4: SCK High (Rising edge @ middle of bit cell for Mode 0)
                    else if (clk_div == 3'd4) begin
                        spi_sck <= 1'b1;
                    end
                    // clk_div == 7: End of current bit
                    else if (clk_div == 3'd7) begin
                        if (bit_idx == 3'd0) begin
                            if (byte_idx == TOTAL_BYTES - 1) begin
                                state <= STATE_DONE;
                            end else begin
                                byte_idx <= byte_idx + 1'b1;
                                state    <= STATE_LOAD_BYTE;
                            end
                        end else begin
                            bit_idx <= bit_idx - 1'b1;
                        end
                    end
                end

                STATE_DONE: begin
                    spi_cs_n  <= 1'b1; // Release CS# high
                    spi_sck   <= 1'b0;
                    spi_mosi  <= 1'b0;
                    tx_busy   <= 1'b0;
                    state     <= STATE_IDLE;
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

endmodule
