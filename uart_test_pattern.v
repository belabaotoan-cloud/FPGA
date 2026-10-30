`timescale 1ns / 1ps
module uart_test_pattern #(parameter CLK_HZ = 100_000_000, parameter BAUD = 115200) (
    input wire clk, rst_n, output wire uart_txd);
    reg valid; wire ready;
    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u (.clk(clk), .rst_n(rst_n), .data(8'h55), .valid(valid), .ready(ready), .txd(uart_txd));
    always @(posedge clk or negedge rst_n) if (!rst_n) valid <= 1'b0; else valid <= ready & ~valid;
endmodule