`timescale 1ns / 1ps
// ============================================================================
// Module Name: fft_stream_controller
// Description: Dual-Buffer Ingest & Hardware Handshake Controller for Gowin FFT IP
//              - Ingests 512 envelope samples @ 3.2 kHz (env_valid strobe)
//              - Stores samples into 512-word dual-port BRAM buffer
//              - Drives Gowin FFT_Top IP handshake (start -> sod/ipd -> eod) @ 100 MHz
//              - Captures output bins 0..256 during opd window (soud -> eoud)
//              - Calculates bin magnitude and writes into spectrum_bram
//              - Generates clean 1-cycle frame_done strobe to trigger SPI DMA packet
// Target:      Gowin Tang Primer 20K (GW2A-LV18PG256)
// ============================================================================

module fft_stream_controller (
    input  wire                clk,            // 100 MHz Core Clock
    input  wire                rst_n,          // Active-Low Reset
    
    // Input interface from envelope_demodulator
    input  wire                env_valid,      // 3.2 kHz sample strobe
    input  wire signed [15:0]  env_out,        // 16-bit demodulated envelope
    
    // Interface to Gowin FFT_Top IP Core
    output reg                 fft_start,      // Pulse 1 cycle to initiate FFT
    output reg  signed [15:0]  fft_xn_re,      // Real input sample to FFT
    output wire signed [15:0]  fft_xn_im,      // Imaginary input (0 for real signal)
    input  wire [8:0]          fft_idx,        // Output bin index (0..511)
    input  wire signed [15:0]  fft_xk_re,      // Real part of FFT bin
    input  wire signed [15:0]  fft_xk_im,      // Imaginary part of FFT bin
    input  wire                fft_sod,        // Start of Download (Input phase start)
    input  wire                fft_ipd,        // In-Process Data (Input active window)
    input  wire                fft_eod,        // End of Download (Input phase end)
    input  wire                fft_busy,       // Core busy calculating
    input  wire                fft_soud,       // Start of Unload Data (Output start)
    input  wire                fft_opd,        // Output Process Data (Output active window)
    input  wire                fft_eoud,       // End of Unload Data (Output end)
    
    // Interface to Dual-Port Spectrum BRAM
    output reg  [8:0]          bram_addr,      // Bin address (0..256)
    output reg  [15:0]         bram_data,      // 16-bit Q0.15 bin magnitude
    output reg                 bram_we,        // BRAM Write Enable
    output reg                 frame_done,     // Pulse 1 cycle when 257 bins are stored
    output reg                 busy            // Controller busy flag
);

    assign fft_xn_im = 16'sd0;

    // -------------------------------------------------------------------------
    // 1. 512-Sample Input Buffer (Inferred as Gowin BSRAM)
    // -------------------------------------------------------------------------
    reg signed [15:0] in_buf [0:511];
    reg [8:0]         sample_wr_ptr;
    reg [8:0]         sample_rd_ptr;
    reg               buf_ready;

    // Ingest process @ 3.2 kHz
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_wr_ptr <= 9'd0;
            buf_ready     <= 1'b0;
        end else begin
            buf_ready <= 1'b0; // Default pulse low
            if (env_valid) begin
                in_buf[sample_wr_ptr] <= env_out;
                if (sample_wr_ptr == 9'd511) begin
                    sample_wr_ptr <= 9'd0;
                    buf_ready     <= 1'b1; // 512 samples collected
                end else begin
                    sample_wr_ptr <= sample_wr_ptr + 1'b1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // 2. FSM States for Gowin FFT IP Handshake
    // -------------------------------------------------------------------------
    localparam FSM_IDLE       = 3'd0;
    localparam FSM_START_FFT  = 3'd1;
    localparam FSM_FEED_INPUT = 3'd2;
    localparam FSM_WAIT_CALC  = 3'd3;
    localparam FSM_STORE_BINS = 3'd4;
    localparam FSM_DONE       = 3'd5;

    reg [2:0] state;

    // Alpha max + beta min approximate magnitude: |Re| + 0.5*|Im|
    wire [15:0] abs_re = fft_xk_re[15] ? (-fft_xk_re) : fft_xk_re;
    wire [15:0] abs_im = fft_xk_im[15] ? (-fft_xk_im) : fft_xk_im;
    wire [15:0] bin_magnitude = (abs_re > abs_im) ? (abs_re + (abs_im >> 1)) : 
                                                    (abs_im + (abs_re >> 1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= FSM_IDLE;
            fft_start     <= 1'b0;
            fft_xn_re     <= 16'sd0;
            sample_rd_ptr <= 9'd0;
            bram_addr     <= 9'd0;
            bram_data     <= 16'd0;
            bram_we       <= 1'b0;
            frame_done    <= 1'b0;
            busy          <= 1'b0;
        end else begin
            fft_start  <= 1'b0;
            bram_we    <= 1'b0;
            frame_done <= 1'b0;

            case (state)
                FSM_IDLE: begin
                    busy <= 1'b0;
                    if (buf_ready) begin
                        fft_start     <= 1'b1; // Trigger FFT core
                        fft_xn_re     <= in_buf[9'd0]; // PRELOAD mau 0: phai co san trong chu ky ipd dau tien
                        sample_rd_ptr <= 9'd1;
                        busy          <= 1'b1;
                        state         <= FSM_START_FFT;
                    end
                end

                FSM_START_FFT: begin
                    // Wait for FFT core to assert sod or ipd
                    // mau 0 da nam tren xn_re tu luc start; tai canh nay IP dang lay mau 0 -> nap mau 1
                    if (fft_sod || fft_ipd) begin
                        fft_xn_re     <= in_buf[sample_rd_ptr];
                        sample_rd_ptr <= sample_rd_ptr + 1'b1;
                        state         <= FSM_FEED_INPUT;
                    end
                end

                FSM_FEED_INPUT: begin
                    // Stream samples ONLY while FFT IP asserts ipd (accepting input)
                    // FIX: was (fft_ipd || !fft_eod) which always fed data incorrectly
                    if (fft_ipd) begin
                        fft_xn_re     <= in_buf[sample_rd_ptr];
                        sample_rd_ptr <= sample_rd_ptr + 1'b1;
                    end
                    // Transition only on eod from IP, not on sample_rd_ptr overflow
                    if (fft_eod) begin
                        state <= FSM_WAIT_CALC;
                    end
                end

                FSM_WAIT_CALC: begin
                    // Wait for calculation to finish and output to start
                    if (fft_soud || fft_opd) begin
                        state <= FSM_STORE_BINS;
                        // soud va opd co the len cung chu ky (bin 0): phai chup luon o day, neu khong se mat bin 0
                        if (fft_opd && (fft_idx <= 9'd256)) begin
                            bram_addr <= fft_idx;
                            bram_data <= bin_magnitude;
                            bram_we   <= 1'b1;
                        end
                    end
                end

                FSM_STORE_BINS: begin
                    // Capture bins during opd active window
                    if (fft_opd && (fft_idx <= 9'd256)) begin
                        bram_addr <= fft_idx;
                        bram_data <= bin_magnitude;
                        bram_we   <= 1'b1;
                    end

                    // Completion detection: eoud pulse or idx == 256
                    if (fft_eoud || (fft_idx == 9'd256 && !fft_opd)) begin
                        state <= FSM_DONE;
                    end
                end

                FSM_DONE: begin
                    frame_done <= 1'b1; // Pulse frame_done to trigger SPI TX
                    busy       <= 1'b0;
                    state      <= FSM_IDLE;
                end

                default: state <= FSM_IDLE;
            endcase
        end
    end

endmodule