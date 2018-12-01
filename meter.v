`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    22:14:22 11/30/2018 
// Design Name: 
// Module Name:    meter 
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
module meter(
    input wire rst,
    input wire clk,
    input wire start,
    output wire busy,
    output wire [21:0] data_v,
    output wire [21:0] data_i,
    output wire [35:0] data_p,
		
	input  wire voltage0_miso_pin, 
	output wire voltage0_clkout_pin,
	output wire voltage0_cs_pin,
	
	input wire current0_miso_pin, 
	output wire current0_clkout_pin,
	output wire current0_cs_pin
	
    );


	mult_12_constV MULT_CONST_V ( //const 805
		.a(data_out_v), //12bit
		.p(data_v) //22bit   
	);
		
	reg start_mcp, start_mcp_next;
	wire busy_mcp;
	wire [11:0] v0_data_out;
	wire [11:0] i0_data_out;
	wire v0_new_data;
	
	parameter CLK_DIV = 0;
	parameter NUM_SAMPLES_AVERAGE = 3; //2^n 
	
		
	mcp3201_spi #(.CLK_DIV(CLK_DIV)) SPI0_V ( 
		.clk(clk),
		.rst(rst),
		.start(start_mcp),
		.data_out(v0_data_out),
		.busy(busy_mcp),
		.new_data(v0_new_data),
		
		.cs_pin_n(voltage0_cs_pin),
		.data_in_pin(voltage0_miso_pin),
		.clk_pin(voltage0_clkout_pin)
	);

	mcp3201_spi #(.CLK_DIV(CLK_DIV)) SPI1_I ( 
		.clk(clk),
		.rst(rst),
		.start(start_mcp),
		.data_out(i0_data_out),
		.busy(),
		.new_data(),
		
		.cs_pin_n(current0_cs_pin),
		.data_in_pin(current0_miso_pin),
		.clk_pin(current0_clkout_pin)
	);



assign busy = (state_ctl != 0);
assign data_i = data_out_i;
assign data_p = data_out_p;

parameter SPI_IDLE 		= 0;
parameter SPI_RUNNING	= 1;
parameter SPI_STORE		= 2;
parameter SAMPLES_AVERAGE = 3;

reg [7:0] state_ctl, state_ctl_next;

reg [11:0] data_out_v, data_out_v_next;
reg [11:0] data_out_i, data_out_i_next;
reg [22:0] data_out_p, data_out_p_next;

reg [11:0] tmp_vout, tmp_vout_next;
reg [11:0] tmp_iout, tmp_iout_next;

reg [23:0] acum_vout, acum_vout_next;
reg [23:0] acum_iout, acum_iout_next;



reg [9:0] samples_counter, samples_counter_next;

always @(posedge clk or rst) begin
	if (rst) begin
		state_ctl <= 8'd0;
		state_ctl_next <= 8'd0;
		start_mcp <= 0;
		start_mcp_next <= 0;
		data_out_v <= 0;
		data_out_v_next <= 0;
		data_out_i <= 0;
		data_out_i_next <= 0;
		data_out_p <= 0;
		data_out_p_next <= 0;
		samples_counter <= 0;
		samples_counter_next <= 0;		
		tmp_vout <= 0;
		tmp_vout_next <= 0;
		tmp_iout <= 0;
		tmp_iout_next <= 0;
		acum_vout <= 0;
		acum_vout_next <= 0;
		acum_iout <= 0;
		acum_iout_next <= 0;
		
	end
	else begin
		state_ctl <= state_ctl_next;
		start_mcp <= start_mcp_next;
		data_out_v <= data_out_v_next;
		data_out_i <= data_out_i_next;
		data_out_p <= data_out_p_next;
		samples_counter <= samples_counter_next;
		tmp_vout <= tmp_vout_next;
		tmp_iout_next <= tmp_iout_next;
		acum_vout <= acum_vout_next;
		acum_iout <= acum_iout_next;	
		
	end
end	

always @(*) begin
	state_ctl_next = state_ctl;
	start_mcp_next = start_mcp;
	data_out_v_next = data_out_v;
	data_out_i_next = data_out_i;
	data_out_p_next = data_out_p;
	samples_counter_next = samples_counter;
	tmp_vout_next = tmp_vout;
	tmp_iout_next = tmp_iout;
	acum_vout_next = acum_vout;
	acum_iout_next = acum_iout;
	
	case (state_ctl)
			SPI_IDLE: begin
				start_mcp_next = 0;
				if (start==1) begin  
					start_mcp_next = 1;
					state_ctl_next = SPI_RUNNING;
				end	
			end
	
			SPI_RUNNING: begin
				if (v0_new_data==1)  begin //como os ad's sincronos, verificando apenas o v0					
						start_mcp_next = 0;
						state_ctl_next = SPI_STORE;
				end 
			end
			
			SPI_STORE: begin
				tmp_vout_next = 12'd500;//v0_data_out[11:0];
				tmp_iout_next = i0_data_out[11:0];
				if (busy_mcp==0) begin
					state_ctl_next = SAMPLES_AVERAGE;
				end
	
			end
			
			SAMPLES_AVERAGE: begin
				if ( samples_counter == (2**NUM_SAMPLES_AVERAGE) ) begin
					//finalizou
					state_ctl_next = SPI_IDLE;
					samples_counter_next = 0;
					data_out_v_next = acum_vout >> SAMPLES_AVERAGE;
					data_out_i_next = acum_iout >> SAMPLES_AVERAGE;
					//data_out_p_next = acum_out_v_next * data_out_i_next;
					samples_counter_next = 0;
					acum_vout_next = 0;
					acum_iout_next = 0;
				end else begin 
					//acumulando
					acum_vout_next = tmp_vout + acum_vout;
					acum_iout_next = tmp_iout + acum_iout;
					samples_counter_next = samples_counter + 1;
					start_mcp_next = 1;
					state_ctl_next = SPI_RUNNING;
				end
			
			end
			
	endcase;
			
				
end
	
	
	
	
endmodule
