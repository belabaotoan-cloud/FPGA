// ============================================================================
// Module Name: fir_minphase_32
// Description: Fallback / Stub 32-Tap FIR Filter Module for Gowin Primer 20K
// ============================================================================
`timescale 1ns / 1ps

module fir_minphase_32 #(
    parameter DATA_WIDTH  = 16,
    parameter COEFF_WIDTH = 16,
    parameter TAPS        = 32
)(
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          sample_en,
    input  wire signed [DATA_WIDTH-1:0]  x_in,
    output wire signed [DATA_WIDTH-1:0]  y_out,
    output wire                          valid_out
);
    // Directly delegate to fir_adxl345_32 implementation
    fir_adxl345_32 #(
        .DATA_WIDTH  (DATA_WIDTH),
        .COEFF_WIDTH (COEFF_WIDTH),
        .TAPS        (TAPS)
    ) u_impl (
        .clk        (clk),
        .rst_n      (rst_n),
        .sample_en  (sample_en),
        .x_in       (x_in),
        .y_out      (y_out),
        .valid_out  (valid_out)
    );
endmodule
