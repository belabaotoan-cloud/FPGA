`timescale 1ns / 1ps
// ============================================================================
// Module Name: envelope_demodulator
// Description: Causal Envelope Demodulation Engine
//              Full-Wave Absolute Value Rectifier + 2nd-Order Direct-Form I IIR LPF
// Target:      Gowin GW2A-LV18PG256 (Tang Primer 20K)
// Architecture: 3-Stage Pipelined IIR Architecture with Saturation & Q2.14 Coeffs
// Cutoff:      Fc ~ 1000 Hz @ Fs = 13.333 kHz / ~ 240 Hz @ Fs = 3.2 kHz (Butterworth 2nd-Order)
// ============================================================================

module envelope_demodulator #(
    parameter WIDTH = 16
)(
    input  wire                          clk,        // System Clock (100 MHz)
    input  wire                          rst_n,      // Active-Low Reset
    input  wire                          sample_en,  // Sample Strobe
    input  wire signed [WIDTH-1:0]       y_filtered, // Bandpass Filtered Sample
    output reg  signed [WIDTH-1:0]       env_out,    // Demodulated Baseband Envelope
    output reg                           valid_out   // Output Valid Strobe
);

    // Step 1: Full-Wave Absolute Value Rectification (with overflow protection)
    wire signed [WIDTH-1:0] v_rect = (y_filtered == -16'sd32768) ? 16'sd32767 :
                                     (y_filtered < 16'sd0) ? -y_filtered : y_filtered;

    // Fixed-Point Butterworth 2nd-Order LPF Coefficients
    // Scaled in Q2.14 Format (1 sign, 1 int, 14 fractional bits, range [-2.0, 2.0))
    localparam signed [15:0] B0 = 16'sh0451; // +0.067455 * 16384 = +1105
    localparam signed [15:0] B1 = 16'sh08A2; // +0.134911 * 16384 = +2210
    localparam signed [15:0] B2 = 16'sh0451; // +0.067455 * 16384 = +1105
    localparam signed [15:0] A1 = 16'shB6D9; // -1.142981 * 16384 = -18727
    localparam signed [15:0] A2 = 16'sh1A6B; // +0.412802 * 16384 = +6763

    reg signed [WIDTH-1:0] v_z1, v_z2;
    reg signed [WIDTH-1:0] e_z1, e_z2;

    // Pipeline Registers
    reg signed [31:0] prod_b0, prod_b1, prod_b2;
    reg signed [31:0] prod_a1, prod_a2;
    reg signed [35:0] accum_stage1;
    reg [1:0] pipe_valid;

    // Output Saturation Logic
    wire signed [35:0] shifted_accum = accum_stage1 >>> 14;
    wire signed [WIDTH-1:0] clamped_out = (shifted_accum > 36'sd32767) ? 16'sd32767 :
                                          (shifted_accum < -36'sd32768) ? -16'sd32768 :
                                          shifted_accum[15:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_z1         <= 16'sd0;
            v_z2         <= 16'sd0;
            e_z1         <= 16'sd0;
            e_z2         <= 16'sd0;
            prod_b0      <= 32'sd0;
            prod_b1      <= 32'sd0;
            prod_b2      <= 32'sd0;
            prod_a1      <= 32'sd0;
            prod_a2      <= 32'sd0;
            accum_stage1 <= 36'sd0;
            env_out      <= 16'sd0;
            valid_out    <= 1'b0;
            pipe_valid   <= 2'b00;
        end else begin
            // Stage 1: Register multiplier products on sample strobe
            if (sample_en) begin
                v_z1    <= v_rect;
                v_z2    <= v_z1;
                prod_b0 <= v_rect * B0;
                prod_b1 <= v_z1 * B1;
                prod_b2 <= v_z2 * B2;
                prod_a1 <= e_z1 * A1;
                prod_a2 <= e_z2 * A2;
                pipe_valid[0] <= 1'b1;
            end else begin
                pipe_valid[0] <= 1'b0;
            end

            // Stage 2: Parallel Accumulation: y[n] = (B0*x0 + B1*x1 + B2*x2) - (A1*y1 + A2*y2)
            if (pipe_valid[0]) begin
                accum_stage1  <= (prod_b0 + prod_b1 + prod_b2) - (prod_a1 + prod_a2);
                pipe_valid[1] <= 1'b1;
            end else begin
                pipe_valid[1] <= 1'b0;
            end

            // Stage 3: Saturation, output register and recursive state update
            if (pipe_valid[1]) begin
                env_out   <= clamped_out;
                e_z1      <= clamped_out;
                e_z2      <= e_z1;
                valid_out <= 1'b1;
            end else begin
                valid_out <= 1'b0;
            end
        end
    end

endmodule
