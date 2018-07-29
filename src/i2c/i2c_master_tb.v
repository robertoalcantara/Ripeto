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
module i2c_master_tb();

    reg clk;
    reg rst;

// Instantiate the module
	reg [6:0] dac_cmd_address;
	reg dac_cmd_start;
	reg dac_cmd_read;
	reg dac_cmd_write;
	reg dac_cmd_write_multiple;
	reg dac_cmd_stop;
	reg dac_cmd_valid;
	wire dac_cmd_ready;
	
	reg [7:0] dac_data_in;
	reg dac_data_in_valid;
	wire dac_data_in_ready;
	reg dac_data_in_last;
	
	wire [7:0] dac_data_out;
	wire dac_data_out_valid;
	reg dac_data_out_ready;
	wire dac_data_out_last;
	
	wire scl_i;
	wire scl_o;
	wire scl_t;
	wire sda_i;
	reg sda_o;
	wire sda_t;
	
	wire dac_busy;
	wire dac_bus_control;
	wire dac_missed_ack;

	i2c_master dac (
		.clk(clk), 
		.rst(rst), 
		.cmd_address(dac_cmd_address), 
		.cmd_start(dac_cmd_start), 
		.cmd_read(dac_cmd_read), 
		.cmd_write(dac_cmd_write), 
		.cmd_write_multiple(dac_cmd_write_multiple), 
		.cmd_stop(dac_cmd_stop), 
		.cmd_valid(dac_cmd_valid), 
		.cmd_ready(dac_cmd_ready), 
		.data_in(dac_data_in), 
		.data_in_valid(dac_data_in_valid), 
		.data_in_ready(dac_data_in_ready), 
		.data_in_last(dac_data_in_last), 
		.data_out(dac_data_out), 
		.data_out_valid(dac_data_out_valid), 
		.data_out_ready(dac_data_out_ready), 
		.data_out_last(dac_data_out_last), 
		.scl_i(scl_i), 
		.scl_o(scl_o), 
		.scl_t(scl_t), 
		.sda_i(sda_i), 
		.sda_o(sda_o), 
		.sda_t(sda_t), 
		.busy(dac_busy), 
		.bus_control(dac_bus_control), 
		.bus_active(dac_bus_active), 
		.missed_ack(dac_missed_ack), 
		.prescale(15'd500), 
		.stop_on_idle(1'b1)
		);

	
	assign scl_i = dac_scl;
	assign dac_scl = scl_o ? 1'bz : 1'b0;
	assign sda_i = dac_sda;
	assign dac_sda = sda_o ? 1'bz : 1'b0;

	reg [7:0] dac_test_ctl;

	initial
	begin
		clk = 0;
		rst = 1;	
			
		 dac_cmd_address <= 0;
		 dac_cmd_start <= 0;
		 dac_cmd_read <= 0;
		 dac_cmd_write<= 0;
		 dac_cmd_write_multiple <= 0;
		 dac_cmd_stop <= 0;
		 dac_cmd_valid <= 0;
		 dac_data_in <= 0 ;
		 dac_data_in_valid <= 0;
		 dac_data_in_last <= 0;
		 dac_data_out_ready <= 0;		
		 dac_test_ctl <= 0;

		#5 rst = 0;
		
		#10 dac_test_ctl <= 1;
		
		#250 sda_o <= 0;
		#260 sda_o <= 1;
		
		#30000 clk = 0;
	end

	always 
		#10 clk = !clk;
	

	parameter MCP47FEB_ID			= 7'b110_0000; 
	parameter DAC0_REG				= 5'b0;
	parameter DAC1_REG				= 5'b1;
	parameter CMD_WRITE				= 2'b0;
	parameter CMD_READ				= 2'b11;
	
	always begin
	#20	case (dac_test_ctl)
			0: begin
				dac_test_ctl <= 1;
			end
			
			1: begin
				if ( dac_cmd_ready ) begin
					dac_cmd_write_multiple <= 1;
					dac_cmd_start <= 1;
					dac_cmd_stop <= 1;
					dac_cmd_valid <= 1;
					dac_data_in_last <= 0;
					dac_cmd_address <= 8'h01;//MCP47FEB_ID;
					dac_test_ctl <= 2;
				end
			end
			2: begin
					dac_cmd_valid <= 0;
					dac_data_in_last <= 0;
					dac_data_in_valid <= 1;
					dac_data_in <= 8'h03;//{DAC0_REG, CMD_WRITE, 1'b0};
					if (dac_data_in_ready) dac_test_ctl <= 3;
			end
			
			3: begin
					dac_data_in_last <= 0;
					dac_data_in_valid <= 1;
					dac_data_in <= 8'h05;//{DAC0_REG, CMD_WRITE, 1'b0};
					if (dac_data_in_ready) dac_test_ctl <= 4;
			end
			
		
			4: begin
					dac_data_in_last <= 1;
					dac_data_in_valid <= 1;
					dac_data_in <= 8'h09;//{DAC0_REG, CMD_WRITE, 1'b0};
					if (!dac_busy) dac_test_ctl <= 5;
			end
						
			5: begin
				dac_data_in_last <= 0;
				dac_cmd_valid <= 0;			
				dac_cmd_write_multiple <= 0;
			end
			
			
			default: begin
			
			end
			
		
		endcase
		
	end
		


endmodule
