// ============================================================================
// Module Name: npu_mac12_unit
// Description: 12-Way Parallel 16-bit x INT8 Signed MAC Array
// Target:      Gowin Primer 20K (GW2A-LV18PG256) DSP18E Multiplier Slices
// Features:    - 12 parallel 16-bit unsigned x 8-bit signed multipliers
//              - 40-bit accumulators with synchronous clear/load
//              - Direct 480-bit parallel accumulator output bus
// ============================================================================

`timescale 1ns / 1ps

module npu_mac12_unit (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 mac_en,       // MAC enable strobe
    input  wire                 acc_clear,    // Clear accumulators (start of new output point)
    input  wire [15:0]          act_in,       // 16-bit spectrum magnitude sample
    input  wire signed [95:0]   weights_in,   // 12 parallel weights (12 x 8-bit = 96-bit)
    input  wire signed [31:0]   bias_in,      // Bias for requantization channel (unused)
    input  wire [31:0]          requant_mult, // Requantization multiplier (unused)
    input  wire [5:0]           requant_shift,// Requantization shift amount (unused)
    input  wire                 relu_en,      // 1: apply ReLU (unused)
    input  wire                 requant_en,   // Requantization trigger (unused)
    input  wire [3:0]           channel_sel,  // Channel select (unused)
    output reg  signed [7:0]    act_out,      // Requantized 8-bit signed activation output
    output reg                  act_valid,    // Valid strobe for act_out
    output wire signed [479:0]  accum_all_out // Direct 12-channel accumulators output (12 x 40-bit)
);

    // 12 parallel 40-bit accumulators (prevents overflow for 257 x 16-bit x 8-bit accumulation)
    reg signed [39:0] accum [0:11];
    integer i;

    // Unpack 12 weights
    wire signed [7:0] w [0:11];
    genvar g;
    generate
        for (g = 0; g < 12; g = g + 1) begin : GEN_WEIGHTS
            assign w[g] = weights_in[(g*8) +: 8];
        end
    endgenerate

    // 12-Way 16-bit x 8-bit Multiply-Accumulate Datapath
    // act_in is unsigned magnitude [15:0], mapped to 17-bit positive signed
    wire signed [16:0] act_signed = $signed({1'b0, act_in});

    always @(posedge clk) begin
        if (!rst_n) begin
            for (i = 0; i < 12; i = i + 1) begin
                accum[i] <= 40'sd0;
            end
        end else begin
            for (i = 0; i < 12; i = i + 1) begin
                if (acc_clear) begin
                    accum[i] <= (mac_en) ? (act_signed * w[i]) : 40'sd0;
                end else if (mac_en) begin
                    accum[i] <= accum[i] + (act_signed * w[i]);
                end
            end
        end
    end

    // Direct Parallel Output of All 12 Accumulators (12 x 40-bit = 480-bit)
    assign accum_all_out = {
        accum[11], accum[10], accum[9], accum[8],
        accum[7],  accum[6],  accum[5], accum[4],
        accum[3],  accum[2],  accum[1], accum[0]
    };

    // Requantizer output ports tied off
    always @(posedge clk) begin
        if (!rst_n) begin
            act_out   <= 8'sd0;
            act_valid <= 1'b0;
        end else begin
            act_out   <= 8'sd0;
            act_valid <= 1'b0;
        end
    end

endmodule
