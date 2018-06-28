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
	output [21:0] data_out,
	
	input  voltage0_miso_pin, 
	output voltage0_clkout_pin,
	output voltage0_cs_pin,
	
	input current0_miso_pin, 
	output current0_clkout_pin,
	output current0_cs_pin

    );
	
	reg [21:0] data_out_f;



	reg start_msp;
	wire busy_msp;
	wire [11:0] v0_data_out;
	wire [11:0] i0_data_out;
	wire v0_new_data;
	

	mcp3201_spi #(.CLK_DIV(CLK_DIV)) SPI0_V ( 
		.clk(clk),
		.rst(rst),
		.start(start_msp),
		.data_out(v0_data_out),
		.busy(busy_msp),
		.new_data(v0_new_data),
		
		.cs_pin_n(voltage0_cs_pin),
		.data_in_pin(voltage0_miso_pin),
		.clk_pin(voltage0_clkout_pin)
	);

	mcp3201_spi #(.CLK_DIV(CLK_DIV)) SPI1_I ( 
		.clk(clk),
		.rst(rst),
		.start(start_msp),
		.data_out(i0_data_out),
		.busy(),
		.new_data(),
		
		.cs_pin_n(current0_cs_pin),
		.data_in_pin(current0_miso_pin),
		.clk_pin(current0_clkout_pin)
	);


parameter SPI_IDLE 		= 0;
parameter SPI_RUNNING	= 1;

reg [7:0] state_ctl;

assign busy = (state_ctl != 0);

always @(posedge clk or negedge rst) begin

	if ( !rst ) begin
		state_ctl <= 8'd0;
		start_msp <= 0;
	end 
	else begin

		case (state_ctl)
			SPI_IDLE: begin
				if (start && !busy_msp) begin  
					start_msp <= 1;
					state_ctl <= SPI_RUNNING;
				end	

			end
			
			SPI_RUNNING: begin
				if (v0_new_data == 1)  begin //como sao sincronos, verificando apenas o v0
					data_out_f <= ( (v0_data_out<<12) | i0_data_out );
					start_msp <= 0;
					state_ctl <= SPI_IDLE;
				end
			end	
		endcase
	end
end



endmodule
