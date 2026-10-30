`timescale 1ns / 1ps
// =====================================================================
//  spi_m3.v  -  SPI master 8 bit, MODE 3 (CPOL=1, CPHA=1), MSB truoc.
//  - SCK nghi o muc CAO. MOSI doi o suon XUONG, MISO duoc lay mau o suon LEN.
//  - Tan so SCK = f_clk / (2 * HALF). Voi clk ~100 MHz, HALF=12 -> ~4.17 MHz.
// =====================================================================
module spi_m3 #(
    parameter integer HALF = 12
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,   // xung 1 chu ky; chi nhan khi busy = 0
    input  wire [7:0] tx,      // byte can gui
    output reg  [7:0] rx,      // byte nhan duoc (hop le khi done = 1)
    output reg        done,    // xung 1 chu ky khi xong 8 bit
    output reg        busy,
    output reg        sck,
    output reg        mosi,
    input  wire       miso
);
    reg [7:0] sh;              // thanh ghi dich gui
    reg [2:0] bitn;            // dem bit 0..7
    reg [7:0] cnt;             // dem nua chu ky SCK

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sh     <= 8'h00;
            bitn   <= 3'd0;
            cnt    <= 8'd0;
            rx     <= 8'h00;
            done   <= 1'b0;
            busy   <= 1'b0;
            sck    <= 1'b1;    // Mode 3: CPOL = 1
            mosi   <= 1'b0;
        end else begin
            done   <= 1'b0;

            if (!busy) begin
                sck <= 1'b1;
                if (start) begin
                    busy <= 1'b1;
                    sh   <= tx;
                    bitn <= 3'd0;
                    cnt  <= 8'd0;
                end
            end else begin
                if (cnt == HALF - 1) begin
                    cnt <= 8'd0;
                    if (sck) begin
                        // Suon XUONG (Leading Edge): dua bit MSB ra MOSI
                        sck  <= 1'b0;
                        mosi <= sh[7];
                    end else begin
                        // Suon LEN (Trailing Edge): lay mau truc tiep MISO
                        sck  <= 1'b1;
                        rx   <= {rx[6:0], miso};
                        sh   <= {sh[6:0], 1'b0};
                        bitn <= bitn + 3'd1;
                        if (bitn == 3'd7) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                        end
                    end
                end else begin
                    cnt <= cnt + 8'd1;
                end
            end
        end
    end
endmodule
