`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    19:04:14 06/17/2018 
// Design Name: 
// Module Name:    mcp3201_spi 
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
module mcp3201_spi #(parameter CLK_DIV = 2)(
    input clk,
    input rst,
    input start,
    output [11:0] data_out,
    output busy,
    output new_data,
	
	input  data_in_pin,
	output clk_pin,
	output cs_pin_n
    );
	
reg clk_slow;
reg [7:0] cnt_clk;

(* IOB = "TRUE" *)
reg cs_r;

reg new_data_r;
reg [11:0] data_out_r;

assign cs_pin_n = cs_r;
assign busy = (main_state != 0);
(* IOB = "TRUE" *)
reg clk_enable;

assign clk_pin = clk_slow & clk_enable;

assign new_data = new_data_r;
assign data_out = data_out_r;

reg[5:0] main_state;

reg [4:0] cnt;


always @(posedge clk) begin
	if (rst) begin
		cnt_clk <= 0;
		clk_slow <= 0;
	end
	else begin
		if (cnt_clk == CLK_DIV) begin
			cnt_clk <= 0;
			clk_slow <= ~clk_slow;
		end 
		else begin
			cnt_clk <= cnt_clk+1;
		end
	end

end

always @(posedge clk_slow or posedge rst) begin
	if (rst) begin
		data_out_r <= 12'd0;
		new_data_r <= 0;
		cs_r <= 1; //enable low
		main_state <= 0;
		cnt <= 0;
		clk_enable <= 0;
	end
	else begin
		case (main_state)
			0: begin//idle
				if (start) begin
					main_state <= 1;
					cs_r <= 0; //enable low. 
					cnt <= 0;
					data_out_r <= 12'd0;
					new_data_r <= 0;
				end 
			end
			1: begin
				clk_enable <= 1;  //enable clk to device
				main_state <= main_state+1; //dummy byte 1
			end
			2: begin
				main_state <= main_state+1; //dummy byte 2
			end
			3: begin
				main_state <= main_state+1; //null byte 
			end
			4: begin
				if (cnt == 12) begin
					main_state <= 5;
					new_data_r <= 1; //data is available
				end
				else begin
					cnt <= cnt + 1;
					data_out_r <= (data_out_r << 1) | data_in_pin;
				end
			end
			5: begin
				cs_r <= 1; //disable cs
				clk_enable <= 0; //disabe clock
				
				main_state <= main_state+1; //wait
			end
			6: begin
				main_state <= 0; //wait
			end
			
			default: begin
			end
		endcase

	end

end





endmodule
