`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    18:23:55 12/30/2018 
// Design Name: 
// Module Name:    top_meter 
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
module top_meter(
	input clk, 
	input rst,
  
	output led1, 
	output led2,
	output debug7,
	output debug11,
	
	output uart_tx_pin,
	
	input spi0_miso, //U3 current
	output spi0_clkout,
	output spi0_cs,

	input spi1_miso,  //U1 voltage
	output spi1_clkout,
	output spi1_cs	
);


	assign rst_p = ~rst;

	reg led1_debug, led2_debug, debug7q, debug11q;

	wire clk100;		
	clk_wiz_v3_6 clkPLL (
	  .CLK_IN1(clk), // IN
	  .CLK_OUT1(clk100) // OUT
	);

	reg [7:0] tx_byte;  
	reg tx_en, tx_en_next;
	wire tx_ready;
	
	uart UART0(
		.clk(clk100),
		.rst(rst_p),
		.tx_byte(tx_byte),
		.tx_en(tx_en),
		.tx_ready(tx_ready),
		.tx_pin(uart_tx_pin)
    );
	
  reg meter_start;
  reg meter_start_next;	
  wire meter_busy;
  wire [11:0] meter_data_v;
  wire [11:0] meter_data_i;  

  meter METER0(
	 .rst(rst_p),
    .clk(clk100),
	 .start(meter_start),
    .busy(meter_busy),
    .data_v(meter_data_v),
    .data_i(meter_data_i),
		
	.voltage0_miso_pin(spi1_miso), 
	.voltage0_clkout_pin(spi1_clkout),
	.voltage0_cs_pin(spi1_cs),
	
	.current0_miso_pin(spi0_miso), 
	.current0_clkout_pin(spi0_clkout),
	.current0_cs_pin(spi0_cs)
  ); 



parameter IDLE			= 3'd0; 
parameter RUNNING    = 3'd1;
parameter DONE			= 3'd2; 


reg [3:0] state_ctl, state_ctl_next;

reg [21:0] meter0_v, meter0_v_next;
reg [21:0] meter0_i, meter0_i_next;

reg [31:0] led1_counter, led1_counter_next;

always @(posedge clk100 or posedge rst_p) begin
	if (rst_p) begin
		state_ctl <= IDLE;
		meter_start <= 0;
		led1_debug <= 1;
		led2_debug <= 0;
		meter0_v <= 0;
		meter0_i <= 0;
		led1_counter <= 0;
		tx_en <= 0;
	end
	else begin
		state_ctl <= state_ctl_next;
		meter_start <= meter_start_next;
		meter0_v <= meter0_v_next;
		meter0_i <= meter0_i_next;
		
		tx_en <= tx_en_next;
		
		led1_counter <= led1_counter_next;
		if (led1_counter_next == 0)
			led1_debug <= ~ led1_debug;
			
	end

end


always @(*) begin
	state_ctl_next = state_ctl;
	meter_start_next = meter_start;
	meter0_v_next = meter0_v;
	meter0_i_next = meter0_i;
	led1_counter_next = led1_counter;
	tx_en_next = tx_en;
	
	if (led1_counter == 32'd10000000)
		led1_counter_next = 0;
	else
		led1_counter_next = led1_counter + 1;
	
	
	
	
	case (state_ctl) 
		IDLE: begin
			state_ctl_next = RUNNING;
			meter_start_next = 1;
		end
		
		RUNNING: begin

			meter_start_next = 0;
			if (meter_busy==0) begin
				state_ctl_next = DONE;
				meter0_v_next = meter_data_v;
				meter0_i_next = meter_data_i;
				
				if ( tx_ready ) begin//debug
				  tx_en_next = 1;
				  tx_byte = meter_data_i[7:0];
				end

			end
		end
		
		DONE: begin
			tx_en_next = 0;
			state_ctl_next = IDLE;
		end
	endcase

end




	
	assign led1 = led1_debug;
	assign led2 =  led2_debug;	
	assign debug7 = meter_busy;
	assign debug11 = debug11q;
	


endmodule
