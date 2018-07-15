`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   10:54:07 07/15/2018
// Design Name:   clk_divider
// Module Name:   /home/robertoalcantara/Documents/Xilinx/P1/src/clk/clk_divider_tb.v
// Project Name:  P1
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: clk_divider
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module clk_divider_tb;

	// Inputs
	reg clk;
	reg rst;

	// Outputs
	wire clk_out;

	// Instantiate the Unit Under Test (UUT)
	clk_divider #(.CLK_DIV(1000)) uut (
		.clk(clk), 
		.clk_out(clk_out), 
		.rst(rst)
	);

	initial begin
		// Initialize Inputs
		clk = 0;
		rst = 1;

		// Wait 100 ns for global reset to finish
		#100;
		rst = 0;
	        
		// Add stimulus here

	end
	
	
	always 
		#5 clk = !clk;
      
endmodule

