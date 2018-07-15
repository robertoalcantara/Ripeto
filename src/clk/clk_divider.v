`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    10:44:03 07/15/2018 
// Design Name: 
// Module Name:    clk_divider 
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

module clk_divider  #(parameter CLK_DIV = 2)(
    input wire clk,
    output reg clk_out,
    input wire rst
    );

reg [16:0] cnt_clk;

always @(posedge clk) begin
	if (rst) begin
		cnt_clk <= 0;
		clk_out <= 0;
	end
	else begin
		if (cnt_clk == (CLK_DIV/2)-1) begin
			cnt_clk <= 0;
			clk_out <= ~clk_out;
		end 
		else begin
			cnt_clk <= cnt_clk+1;
		end
	end

end


endmodule
