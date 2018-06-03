`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    09:49:24 06/02/2018 
// Design Name: 
// Module Name:    uart_tb 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module uart_tb();

	reg clk;
    reg rst;
    reg [7:0] tx_byte;
    reg tx_en;
    wire tx_ready;
    wire tx_pin;
	
	uart UART0 (
		.clk(clk),
		.rst(rst),
		.tx_byte(tx_byte),
		.tx_en(tx_en),
		.tx_ready(tx_ready),
		.tx_pin(tx_pin)
	);
	
	initial
	begin
		clk = 0;
		rst = 0;	
		tx_byte = 0;
		tx_en = 0;
		#40 rst = 1;
		#20 rst = 0;

	end

	always 
		#10 clk = !clk;
		
	always begin
	
		#100 rst = 0;
		#10 tx_byte = tx_byte + 8'h01; tx_en = 1; 
		#20 tx_en = 0;
		
	end
		


endmodule
