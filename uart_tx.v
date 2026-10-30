`timescale 1ns / 1ps
// UART TX 8N1. valid=1 (1 chu ky) khi ready=1 -> gui 1 byte.
module uart_tx #(parameter CLK_HZ = 100_000_000, parameter BAUD = 115200) (
    input  wire       clk, rst_n,
    input  wire [7:0] data,
    input  wire       valid,
    output wire       ready,
    output reg        txd
);
    localparam integer DIV = (CLK_HZ + BAUD/2) / BAUD;
    reg [15:0] cnt;
    reg [3:0]  bitn;
    reg [9:0]  sh;
    reg        busy;
    assign ready = ~busy;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin txd <= 1'b1; busy <= 1'b0; cnt <= 0; bitn <= 0; sh <= 10'h3FF; end
        else if (!busy) begin
            if (valid) begin sh <= {1'b1, data, 1'b0}; busy <= 1'b1; cnt <= 0; bitn <= 0; txd <= 1'b0; end
        end else begin
            if (cnt == DIV-1) begin
                cnt <= 0;
                if (bitn == 4'd9) begin busy <= 1'b0; txd <= 1'b1; end
                else begin bitn <= bitn + 1'b1; txd <= sh[bitn + 1'b1]; end
            end else cnt <= cnt + 1'b1;
        end
    end
endmodule
