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
module mcp3201_tb();

    reg clk;
    reg rst;
    reg start;
    wire [11:0] data_out;
    wire busy;
    wire new_data;
	
	reg  data_in_pin;
	wire clk_pin;
	wire cs_pin_n;
	
	mcp3201_spi #(.CLK_DIV(3)) SPI0 (
		.clk(clk),
		.rst(rst),
		.start(start),
		.data_out(data_out),
		.busy(busy),
		.new_data(new_data),
		.data_in_pin(data_in_pin),
		.clk_pin(clk_pin),
		.cs_pin_n(cs_pin_n)
	);
	
	initial
	begin
		clk = 0;
		rst = 0;	
		start = 0;
		data_in_pin = 1;
		#25 rst = 1;
	end

	always 
		#1 clk = !clk;
		
	always begin

		#250 rst = 0;
	
		#20 start = 1; 
		#20 start = 0; 
		
		
	end
		


endmodule
