`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    19:44:29 06/27/2018 
// Design Name: 
// Module Name:    sampler 
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
	
	
module sampler #(parameter CLK_DIV = 1)(
	input clk,
	input rst,
	input start,
	output busy,
	output new_data,
	output data_out[11:0],
	
	input  voltage0_miso_pin, 
	output voltage0_clkout_pin,
	output voltage0_cs_pin,
	
	input current0_miso_pin, 
	output current0_clkout_pin,
	output current0_cs_pin

    );


	mcp3201_spi #(.CLK_DIV(60)) SPI0 ( //7 = ~781kHz
		.clk(clk),
		.rst(rsp),
		.data_in_pin(voltage0_miso_pin),
		.clk_pin(voltage0_clkout_pin),
		.start(start),
		.data_out(),
		.busy(busy),
		.new_data(spi0_new_data),
		.cs_pin_n(voltage0_cs_pin)
	);




assign 





endmodule
