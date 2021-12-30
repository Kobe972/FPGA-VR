`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/12/15 11:19:55
// Design Name: 
// Module Name: top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module top(
input clk,rst_n,
inout [1:0]sda,
output [1:0]scl,
input               rx,
output              tx,
output [5:0]led
    );
    
wire                tx_ready;
wire        [7:0]   tx_data;
wire        [7:0]   rx_data;
wire [15:0]ac_x;
wire [15:0]ac_y;
wire [15:0]ac_z;

wire [15:0]gc_x;
wire [15:0]gc_y;
wire [15:0]gc_z;
mpu6050 t1(
.clk(clk),
.rst_n(rst_n),
.scl(scl[0]),
.sda(sda[0]),
.ACC_X(ac_x),
.ACC_Y(ac_y),
.ACC_Z(ac_z),
.GYRO_X(gc_x),
.GYRO_Y(gc_y),
.GYRO_Z(gc_z)
);
wire [15:0]ac_x2;
wire [15:0]ac_y2;
wire [15:0]ac_z2;

wire [15:0]gc_x2;
wire [15:0]gc_y2;
wire [15:0]gc_z2;

mpu6050 t2(
.clk(clk),
.rst_n(rst_n),
.scl(scl[1]),
.sda(sda[1]),
.ACC_X(ac_x2),
.ACC_Y(ac_y2),
.ACC_Z(ac_z2),
.GYRO_X(gc_x2),
.GYRO_Y(gc_y2),
.GYRO_Z(gc_z2)
);
reg [35:0]cnt;
reg [4:0]out_period;
rx                  rx_inst(
.clk                (clk),
.rst_n                (rst_n),
.rx                 (rx),
.rx_vld             (rx_vld),
.rx_data            (rx_data)
);                     
tx                  tx_inst(
.clk                (clk),
.rst_n                (rst_n),
.tx                 (tx ),
.tx_ready           (tx_ready),
.tx_rd              (tx_rd),
.tx_data            (tx_data)
);
always@(posedge clk)
begin
if(cnt <= 100000)
    cnt <= cnt + 1;
else
    begin
        cnt <= 0;
        out_period <= out_period + 1;
        if(out_period >= 28)
            out_period <= 0;
    end
end
assign tim = (cnt == 0);
reg tx_r;
reg [7:0]tx_d;
always@(posedge clk)
begin
    if(tim) 
    begin
        tx_r <= 1;
        case (out_period)
        5'b00000: tx_d <= 8'h0a;
        5'b00001: tx_d <= ac_x[15:8];
        5'b00010: tx_d <= ac_x[7:0];
        5'b00011: tx_d <= ac_y[15:8];
        5'b00100: tx_d <= ac_y[7:0];
        5'b00101: tx_d <= ac_z[15:8];
        5'b00110: tx_d <= ac_z[7:0];
        5'b00111: tx_d <= gc_x[15:8];
        5'b01000: tx_d <= gc_x[7:0];
        5'b01001: tx_d <= gc_y[15:8];
        5'b01010: tx_d <= gc_y[7:0];
        5'b01011: tx_d <= gc_z[15:8];
        5'b01100: tx_d <= gc_z[7:0];
        5'b01101: tx_d <= ac_x2[15:8];
        5'b01110: tx_d <= ac_x2[7:0];
        5'b01111: tx_d <= ac_y2[15:8];
        5'b10000: tx_d <= ac_y2[7:0];
        5'b10001: tx_d <= ac_z2[15:8];
        5'b10010: tx_d <= ac_z2[7:0];
        5'b10011: tx_d <= gc_x2[15:8];
        5'b10100: tx_d <= gc_x2[7:0];
        5'b10101: tx_d <= gc_y2[15:8];
        5'b10110: tx_d <= gc_y2[7:0];
        5'b10111: tx_d <= gc_z2[15:8];
        5'b11000: tx_d <= gc_z2[7:0];
        5'b11001: tx_d <= 8'h0a;
        5'b11010: tx_d <= 8'h0a;
        5'b11011: tx_d <= 8'h0a;
        default tx_d <= 8'h0a;
        endcase
    end
    else 
    begin 
    tx_r <= tx_rd;
    end
end
assign tx_ready = tx_r;
assign tx_data = tx_d;
assign led = {ac_x[7], out_period};
endmodule
