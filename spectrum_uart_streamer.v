`timescale 1ns / 1ps
// Chup 257 bin tu fft_stream_controller (bram_we/addr/data) va gui len PC qua UART khi frame_done.
// Khung gui (518 byte):
//   AA 55 | SEQ | 257 x (lo, hi) = 514 byte | CKSUM
//   CKSUM = XOR cua SEQ va 514 byte du lieu.  Bin k ung voi tan so k * 6.25 Hz (fs=3200, N=512).
module spectrum_uart_streamer #(
    parameter CLK_HZ = 100_000_000,
    parameter BAUD   = 115200
) (
    input  wire        clk, rst_n,
    input  wire        bram_we,
    input  wire [8:0]  bram_addr,
    input  wire [15:0] bram_data,
    input  wire        frame_done,
    output wire        uart_txd
);
    reg [15:0] mem [0:256];
    always @(posedge clk) if (bram_we && bram_addr <= 9'd256) mem[bram_addr] <= bram_data;

    reg  [7:0] tx_data; reg tx_valid; wire tx_ready;
    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst_n(rst_n), .data(tx_data), .valid(tx_valid), .ready(tx_ready), .txd(uart_txd));

    localparam S_IDLE=0, S_FETCH=1, S_LOAD=2, S_SEND=3, S_W1=4, S_W2=5;
    reg [2:0]  st;
    reg [9:0]  bi;            // chi so byte 0..517
    reg [7:0]  seq, ck;
    reg [8:0]  rd_addr;
    reg [15:0] mem_q;
    wire [9:0] p = bi - 10'd3; // chi so byte trong payload

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; bi <= 0; seq <= 0; ck <= 0; tx_valid <= 0; tx_data <= 0; rd_addr <= 0; mem_q <= 0;
        end else begin
            tx_valid <= 1'b0;
            case (st)
                S_IDLE:  if (frame_done) begin bi <= 0; ck <= 0; st <= S_FETCH; end
                S_FETCH: begin rd_addr <= (bi >= 10'd3 && bi <= 10'd516) ? p[9:1] : 9'd0; st <= S_LOAD; end
                S_LOAD:  begin mem_q <= mem[rd_addr]; st <= S_SEND; end
                S_SEND:  if (tx_ready) begin
                    tx_valid <= 1'b1;
                    if      (bi == 10'd0)   tx_data <= 8'hAA;
                    else if (bi == 10'd1)   tx_data <= 8'h55;
                    else if (bi == 10'd2)   begin tx_data <= seq; ck <= ck ^ seq; end
                    else if (bi == 10'd517) tx_data <= ck;
                    else begin
                        tx_data <= p[0] ? mem_q[15:8] : mem_q[7:0];
                        ck      <= ck ^ (p[0] ? mem_q[15:8] : mem_q[7:0]);
                    end
                    st <= S_W1;
                end
                S_W1: st <= S_W2;
                S_W2: if (tx_ready) begin
                    if (bi == 10'd517) begin seq <= seq + 1'b1; st <= S_IDLE; end
                    else begin bi <= bi + 1'b1; st <= S_FETCH; end
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
