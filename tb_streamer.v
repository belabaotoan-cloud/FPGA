`timescale 1ns / 1ps
module tb_streamer;
    localparam CLK_HZ = 100_000_000, BAUD = 2_000_000;   // BAUD cao cho mo phong nhanh; mach that dung 115200
    reg clk = 0, rst_n = 0; always #5 clk = ~clk;
    reg we = 0; reg [8:0] addr = 0; reg [15:0] dat = 0; reg fd = 0; wire txd;
    spectrum_uart_streamer #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) dut (.clk(clk), .rst_n(rst_n),
        .bram_we(we), .bram_addr(addr), .bram_data(dat), .frame_done(fd), .uart_txd(txd));
    // bo thu UART mo phong
    integer BIT = CLK_HZ/BAUD; reg [7:0] rx; integer n = 0; integer fh; integer i;
    reg [7:0] buf_ [0:2047];
    initial begin fh = $fopen("uart_dump.txt","w"); end
    always @(negedge txd) begin
        repeat (BIT + BIT/2) @(posedge clk);
        for (i = 0; i < 8; i = i + 1) begin rx[i] = txd; repeat (BIT) @(posedge clk); end
        buf_[n] = rx; $fdisplay(fh, "%02x", rx); n = n + 1;
    end
    integer k, frame, errs; reg [7:0] x; reg [15:0] expv;
    function [15:0] val(input integer f, input integer b); val = (b*37 + f*1000 + 5) & 16'hFFFF; endfunction
    initial begin
        #100 rst_n = 1; #100;
        for (frame = 0; frame < 2; frame = frame + 1) begin
            for (k = 0; k <= 256; k = k + 1) begin @(posedge clk); we <= 1; addr <= k; dat <= val(frame,k); end
            @(posedge clk); we <= 0; repeat (3) @(posedge clk); fd <= 1; @(posedge clk); fd <= 0;
            wait (n == 518*(frame+1)); repeat (200) @(posedge clk);
        end
        errs = 0;
        for (frame = 0; frame < 2; frame = frame + 1) begin
            if (buf_[frame*518] !== 8'hAA || buf_[frame*518+1] !== 8'h55 || buf_[frame*518+2] !== frame) errs = errs + 1;
            x = buf_[frame*518+2];
            for (k = 0; k <= 256; k = k + 1) begin
                expv = val(frame,k);
                if ({buf_[frame*518+3+2*k+1], buf_[frame*518+3+2*k]} !== expv) errs = errs + 1;
                x = x ^ buf_[frame*518+3+2*k] ^ buf_[frame*518+3+2*k+1];
            end
            if (buf_[frame*518+517] !== x) errs = errs + 1;
        end
        $fclose(fh);
        if (errs == 0 && n == 1036) $display("tb_streamer: PASS (2 khung x 518 byte, header/seq/du lieu/checksum dung)");
        else $display("tb_streamer: FAIL errs=%0d bytes=%0d", errs, n);
        $finish;
    end
    initial begin #50_000_000; $display("tb_streamer: TIMEOUT bytes=%0d", n); $finish; end
endmodule
