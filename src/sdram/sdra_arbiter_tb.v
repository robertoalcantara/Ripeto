`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   18:27:45 03/23/2019
// Design Name:   sdram_arbiter
// Module Name:   /home/robertoalcantara/Documents/Xilinx/P1/src/sdram/sdra_arbiter_tb.v
// Project Name:  P1
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: sdram_arbiter
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module sdra_arbiter_tb;

	// Inputs
	reg clk;
	reg rst;
	reg req1;
	reg req2;

	// Outputs
	wire ack1;
	wire ack2;

	// Instantiate the Unit Under Test (UUT)
	sdram_arbiter uut (
		.clk(clk), 
		.rst(rst), 
		.req1(req1), 
		.ack1(ack1), 
		.req2(req2), 
		.ack2(ack2)
	);

	initial begin
		// Initialize Inputs
		clk = 0;
		rst = 1;
		req1 = 0;
		req2 = 0;

		// Wait 100 ns for global reset to finish
		#1;
		rst=0;
		#4;
	end
	
	always 
		#1 clk = !clk;
		
	always begin
		#10 req1=1; req2=0;		
		#25 req1 = 0;
		#30 req2 = 1;
		#35 req2 = 0;
		#40 req1 = 1; req2=1;
		#45 req1 = 0; req2=1;
		#50 req2=0;
	end
	
	
	
      
endmodule

