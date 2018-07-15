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
	input wire clk,
	input wire rst,
	input wire start,
	output wire busy,
	output reg new_data,
	output reg [21:0] data_out,
	
	input  wire voltage0_miso_pin, 
	output wire voltage0_clkout_pin,
	output wire voltage0_cs_pin,
	
	input wire current0_miso_pin, 
	output wire current0_clkout_pin,
	output wire current0_cs_pin

    );


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
parameter SPI_CALC		= 2;

reg [7:0] state_ctl;
assign busy = (state_ctl != 0);


parameter NUM_SAMPLES = 32; //power of 2
reg [7:0] samples_cnt;
reg [31:0] v0_data_sum;
reg [31:0] i0_data_sum;

always @(posedge clk or posedge rst) begin

	if ( rst ) begin
		state_ctl <= 8'd0;
		start_msp <= 0;
		data_out <= 0;
	end 
	else begin

		case (state_ctl)
			SPI_IDLE: begin
				if (start && !busy_msp) begin  
					start_msp <= 1;
					state_ctl <= SPI_RUNNING;
					new_data <= 0;
					v0_data_sum <= 12'h0;
					i0_data_sum <= 12'h0;
					samples_cnt <= 8'h0;
				end	

			end
			
			SPI_RUNNING: begin
				if (v0_new_data == 1)  begin //como sao sincronos, verificando apenas o v0					
					if (samples_cnt == NUM_SAMPLES ) begin
						v0_data_sum <= v0_data_sum >> $clog2(NUM_SAMPLES);
						i0_data_sum <= i0_data_sum >> $clog2(NUM_SAMPLES);
						start_msp <= 0;
						state_ctl <= SPI_CALC;
					end 
					else begin
						v0_data_sum <= v0_data_sum + v0_data_out;
						i0_data_sum <= i0_data_sum + i0_data_out;
						samples_cnt <= samples_cnt + 1;
					end
				end
			end
			
			SPI_CALC: begin
				data_out <= { v0_data_sum[11:0], i0_data_sum[11:0] };
				new_data <= 1;
				state_ctl <= SPI_IDLE;
			end
		endcase
	end
end



endmodule
