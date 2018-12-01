`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   23:27:42 11/30/2018
// Design Name:   meter
// Module Name:   /home/robertoalcantara/Documents/Xilinx/P1/meter_tb.v
// Project Name:  P1
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: meter
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module meter_tb;

	// Inputs
	reg rst;
	reg clk;
	reg start;
	reg voltage0_miso_pin;
	reg current0_miso_pin;

	// Outputs
	wire busy;
	wire [21:0] data_v;
	wire [21:0] data_i;
	wire [35:0] data_p;
	wire voltage0_clkout_pin;
	wire voltage0_cs_pin;
	wire current0_clkout_pin;
	wire current0_cs_pin;

	// Instantiate the Unit Under Test (UUT)
	meter uut (
		.rst(rst), 
		.clk(clk), 
		.start(start), 
		.busy(busy), 
		.data_v(data_v), 
		.data_i(data_i), 
		.data_p(data_p), 
		.voltage0_miso_pin(voltage0_miso_pin), 
		.voltage0_clkout_pin(voltage0_clkout_pin), 
		.voltage0_cs_pin(voltage0_cs_pin), 
		.current0_miso_pin(current0_miso_pin), 
		.current0_clkout_pin(current0_clkout_pin), 
		.current0_cs_pin(current0_cs_pin)
	);
		
				
	initial begin
		// Initialize Inputs
		rst = 1;
		clk = 0;
		start = 0;
		voltage0_miso_pin = 0;
		current0_miso_pin = 0;
		#5 rst = 0;
		#10 start = 1;
	end


	always 
		#1 clk = !clk;

      
endmodule

