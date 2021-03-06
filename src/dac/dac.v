`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    18:01:12 07/29/2018 
// Design Name: 
// Module Name:    dac 
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
module dac(
    input wire clk,
    input wire rst,
	 input wire enable,
    input wire [11:0] ch_value,
	output wire busy,
    inout wire i2c_scl_pin,
    inout wire i2c_sda_pin	
    );
	
		
	// Instantiate the module
	reg [6:0] dac_cmd_address, dac_cmd_address_next;
	reg dac_cmd_start, dac_cmd_start_next;
	reg dac_cmd_read, dac_cmd_read_next;
	reg dac_cmd_write, dac_cmd_write_next;
	reg dac_cmd_write_multiple, dac_cmd_write_multiple_next;
	reg dac_cmd_stop, dac_cmd_stop_next;
	reg dac_cmd_valid, dac_cmd_valid_next;
	reg [7:0] dac_data_in, dac_data_in_next;
	reg dac_data_in_valid, dac_data_in_valid_next;
	reg dac_data_in_last, dac_data_in_last_next;

	wire dac_cmd_ready;
	wire dac_data_in_ready;
	
	wire [7:0] dac_data_out;
	wire dac_data_out_valid;
	wire dac_data_out_last;
	
	wire scl_i;
	wire scl_o;
	wire scl_t;
	wire sda_i;
	wire sda_o;
	wire sda_t;
	
	wire dac_busy;
	wire dac_bus_control;
	wire dac_missed_ack;
	
	reg [11:0] dac_value, dac_value_next;
	
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
		.data_out_ready(0), 
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
		.prescale(15'd20),  //200 funciona  //62?   12 funciona as vezes?
		.stop_on_idle(1'b0)  //1
		);

	assign scl_i = i2c_scl_pin;
	assign i2c_scl_pin = scl_o ? 1'bz : 1'b0;
	assign sda_i = i2c_sda_pin;
	assign i2c_sda_pin = sda_o ? 1'bz : 1'b0;
	
	assign busy = !(dac_ctl == DAC_START) || dac_busy;
////////////////////////
	
	reg [7:0] dac_ctl, dac_ctl_next;

	parameter MCP47FEB_ID			= 7'b110_0000; 
	parameter DAC0_REG				= 5'b0;
	parameter DAC1_REG				= 5'b1;
	parameter CMD_WRITE				= 2'b0;
	parameter CMD_READ				= 2'b11;


	parameter DAC_IDLE		= 0;
	parameter DAC_START		= 1;
	parameter DAC_CMD		= 2;
	parameter DAC_VL1		= 3;
	parameter DAC_VL2		= 4;
	parameter DAC_STOP		= 5;
	parameter DAC_CMD_CONTINUOUS		= 6;

	always @(posedge clk or posedge rst) begin
			
		if (rst) begin
			dac_ctl <= DAC_START;
			
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
			dac_value <= 0;
			
		end
		else begin
			dac_ctl <= dac_ctl_next;
			dac_cmd_address <= dac_cmd_address_next;
			dac_cmd_start <= dac_cmd_start_next;
			dac_cmd_read <= dac_cmd_read_next;
			dac_cmd_write<= dac_cmd_write_next;
			dac_cmd_write_multiple <= dac_cmd_write_multiple_next;
			dac_cmd_stop <= dac_cmd_stop_next;
			dac_cmd_valid <= dac_cmd_valid_next;
			dac_data_in <= dac_data_in_next;
			dac_data_in_valid <= dac_data_in_valid_next;
			dac_data_in_last <= dac_data_in_last_next;	
			dac_value <= dac_value_next;
		
		end
				
	end
	
	
	always @(*) begin
		dac_ctl_next = dac_ctl;
		dac_cmd_address_next = dac_cmd_address;
		dac_cmd_start_next = dac_cmd_start;
		dac_cmd_read_next = dac_cmd_read;
		dac_cmd_write_next = dac_cmd_write;
		dac_cmd_write_multiple_next = dac_cmd_write_multiple;
		dac_cmd_stop_next = dac_cmd_stop;
		dac_cmd_valid_next = dac_cmd_valid;
		dac_data_in_next = dac_data_in;
		dac_data_in_valid_next = dac_data_in_valid;
		dac_data_in_last_next = dac_data_in_last;
		dac_value_next = dac_value;


		case (dac_ctl)
			DAC_IDLE: begin

			end
			
			DAC_START: begin
				if ( dac_cmd_ready && enable) begin
					dac_cmd_write_multiple_next = 1;
					dac_cmd_valid_next = 1;
					dac_cmd_stop_next = 0;
					dac_cmd_start_next = 1;
					dac_data_in_last_next = 0;
					dac_cmd_address_next = MCP47FEB_ID;
					dac_ctl_next = DAC_CMD;
					dac_value_next = ch_value;
				end
			end
			
			DAC_CMD: begin
					dac_cmd_valid_next = 0;
					dac_data_in_last_next = 0;
					dac_data_in_valid_next = 1;
					dac_data_in_next = {DAC0_REG, CMD_WRITE, 1'b0};
					
					if (dac_missed_ack) begin
						dac_ctl_next = DAC_STOP;
						dac_cmd_valid_next = 0;			
					end else begin
						if (dac_data_in_ready) 
							dac_ctl_next = DAC_VL1;
					end
			end
			
			DAC_VL1: begin
					dac_data_in_last_next = 0;
					dac_data_in_valid_next = 1;
					dac_data_in_next = {4'b0000, dac_value[11:8]};
					if (dac_data_in_ready) 
						dac_ctl_next= DAC_VL2;
			end
	
			DAC_VL2: begin
					dac_data_in_last_next = 1;//1
					dac_data_in_valid_next = 1;
					dac_data_in_next = dac_value[7:0];
					
					if (dac_data_in_ready)
						dac_ctl_next = DAC_STOP;
						
					/*if (dac_data_in_ready)   //!dac_busy
						dac_ctl_next = DAC_CMD_CONTINUOUS; //DAC_STOP*/
			end
			
			/*DAC_CMD_CONTINUOUS: begin
					dac_data_in_last_next = 0;
					dac_data_in_valid_next = 1;
					dac_data_in_next = {DAC1_REG, CMD_WRITE, 1'b0};
					if (dac_data_in_ready) 
						dac_ctl_next= DAC_VL1;		
			end*/
						
			DAC_STOP: begin
				dac_data_in_last_next = 0;
				dac_cmd_valid_next = 0;			
				dac_cmd_write_multiple_next = 0;
				dac_ctl_next = DAC_START;
			end
			
			default: begin
			
			end
			
		
		endcase
	end	


endmodule
