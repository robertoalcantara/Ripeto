`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:27:50 05/21/2018 
// Design Name: 
// Module Name:    controller_test 
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

module controller_test(
   
	input clk, 
	input rst,
	input sw2,
    
	output sdram_clk,
    output sdram_cle,
    output sdram_cs,
    output sdram_cas,
    output sdram_ras,
    output sdram_we,
    output [1:0]sdram_dqm,
    output [1:0] sdram_ba,
    output  [12:0] sdram_a,
    inout [15:0] sdram_dq,
	
	output led1, 
	output led2,
	output debug7,
	output debug11,
	
	output uart_tx_pin	
);

    wire clk100;		
	clk_wiz_v3_6 clkPLL (
        .CLK_IN1(clk), // IN
        .CLK_OUT1(clk100) // OUT
    );
	

	assign rst_p = ~rst;
	
	//memory user interface
	reg [22:0] addr, addr_next;      // address to read/write
	reg rw, rw_next;               // 1 = write, 0 = read
	reg [31:0] data_in, data_in_next;   // data from a read
	wire [31:0] data_out; // data for a write
	wire ready;
	wire out_valid;        // pulses high when data from read is valid
	reg enable, enable_next;
	reg [31:0] data_read, data_read_next;

	SDRAM_Controller_v SDRAM (
	   .clk(clk100),   .reset(rst_p),
	   // command and write port
	   .cmd_ready(ready), .cmd_enable(enable), .cmd_wr(rw), .cmd_byte_enable(4'b1111), .cmd_address(addr), .cmd_data_in(data_in),
	   // Read data port
	   .data_out(data_out), .data_out_ready(out_valid),
	   // SDRAM signals
	   .SDRAM_CLK(sdram_clk),  .SDRAM_CKE(sdram_cle),  .SDRAM_CS(sdram_cs),  .SDRAM_RAS(sdram_ras),  .SDRAM_CAS(sdram_cas),
	   .SDRAM_WE(sdram_we), .SDRAM_DQM(sdram_dqm), .SDRAM_ADDR(sdram_a), .SDRAM_BA(sdram_ba), .SDRAM_DATA(sdram_dq)
	);

	reg [7:0] tx_byte, tx_byte_next;  
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
	
  	
	reg[3:0] led1_mode, led1_mode_next;
	reg led1_fast, led1_fast_next;
	wire led1_busy;
	led_flash LED1 (
    .clk(clk100),
    .rst(rst_p),
    .mode(led1_mode),
	 .mode_fast(led1_fast),
	 .busy(led1_busy),
    .led_pin(led1)
    );	
	
	reg[3:0] led2_mode, led2_mode_next;
	reg led2_fast, led2_fast_next;
	wire led2_busy;
	led_flash LED2 (
    .clk(clk100),
    .rst(rst_p),
    .mode(led2_mode),
	 .mode_fast(led2_fast),
	 .busy(led2_busy),
    .led_pin(led2)
    );		
	 
	 
    wire sw2_state;
	 sw_debouncer SW2(
    .clk(clk100),
    .PB(sw2),  // "PB" is the glitchy, asynchronous to clk, active low push-button signal
    // from which we make three outputs, all synchronous to the clock
    .PB_state(),  // 1 as long as the push-button is active (down)
    .PB_down(),  // 1 for one clock cycle when the push-button goes down (i.e. just pushed)
    .PB_up(sw2_state)   // 1 for one clock cycle when the push-button goes up (i.e. just released)
);	

(* IOB = "TRUE" *)
reg debug7q, debug11q;
reg debug7q_next, debug11q_next;

reg [7:0] state_ctl;
	
	
reg[31:0] cnt_seg; 
parameter CTL_START = 0;
parameter WAITING_MEMORY_TEST = 1;
parameter WAITING_SPI_TEST = 2;

reg [7:0] memory_test_ctl, memory_test_ctl_next;
parameter MEMORY_TEST_IDLE 		= 0;
parameter MEMORY_TEST_START	    = 1;
parameter MEMORY_TEST_WAIT 		= 2;
parameter MEMORY_TEST_LOOP 		= 3;
parameter MEMORY_TEST_CHECK		= 4;
parameter MEMORY_TEST_FINISHED  = 5;
parameter MEMORY_TEST_FAULT     = 6;

reg[31:0] data_tmp, data_tmp_next;

always @(posedge clk100 or posedge rst_p) begin

	if ( rst_p ) begin
		cnt_seg <= 0;

		memory_test_ctl <= MEMORY_TEST_IDLE;
		addr <= 0;
		rw <= 0;
		data_in <= 0;
		enable <= 0;
		data_tmp <= 0;
		
		tx_byte <= 0;
		tx_en <= 0;
		
		debug11q <= 0;
		debug7q <= 0;

		led1_mode <= 1;
		led1_fast <= 0;
		led2_mode <= 0;
		led2_fast <= 0;		
	end 
	else begin
	
		tx_byte <= tx_byte_next;
		tx_en <= tx_en_next;
	
		debug11q <= debug11q_next;
		debug7q <= debug7q_next;
	
		addr <= addr_next;
		rw <= rw_next;
		data_in <= data_in_next;
		enable <= enable_next;
		data_tmp <= data_tmp_next;
		memory_test_ctl <= memory_test_ctl_next;
		
		led1_mode <= led1_mode_next;
		led1_fast <= led1_fast_next;
		led2_mode <= led2_mode_next;
		led2_fast <= led2_fast_next;
		
		if (sw2_state) memory_test_ctl <= MEMORY_TEST_START;

	end
end




always @(*) begin

		addr_next = addr;
		rw_next = rw;
		data_in_next = data_in;
		enable_next = enable;
		data_tmp_next = data_tmp;
		memory_test_ctl_next = memory_test_ctl;
		led1_mode_next = led1_mode;
		led1_fast_next = led1_fast;
		led2_mode_next = led2_mode;
		led2_fast_next = led2_fast;
		
		tx_byte_next = tx_byte;
		tx_en_next = tx_en;
		debug11q_next = debug11q;
		debug7q_next = debug7q;
		
				
		case (memory_test_ctl) 
			MEMORY_TEST_IDLE: begin
				tx_en_next = 0; //debug
				led2_mode_next = 1;	led2_fast_next = 1;
			end
			
			MEMORY_TEST_START: begin
				if ( ready ) begin
					data_in_next = data_tmp;
					rw_next = 1;
					enable_next = 1;
					memory_test_ctl_next = MEMORY_TEST_WAIT;
				end
			end
			
			MEMORY_TEST_WAIT: begin
				if ( ready ) begin
					rw_next = 0;
					enable_next = 0;
					if ( ready ) begin
						memory_test_ctl_next = MEMORY_TEST_LOOP;
					end
				end
			end
			
			MEMORY_TEST_LOOP: begin
				if ( ready ) begin
					enable_next = 1;
					memory_test_ctl_next = MEMORY_TEST_CHECK;
				end
			end
				
			MEMORY_TEST_CHECK: begin
				if (out_valid ) begin				
				
					if ( tx_ready ) begin //debug
						tx_byte_next = data_out[7:0];//debug
						tx_en_next = 1;//debug
					end//debug
				
					if ( data_out != data_tmp ) begin
							memory_test_ctl_next = MEMORY_TEST_FAULT;
					end 
					else begin 
					
						enable_next = 0;
						data_tmp_next = data_tmp + 7'd1;

						if (addr ==  8388608-1) begin
							memory_test_ctl_next = MEMORY_TEST_FINISHED;
							addr_next = 0;
						end
						else begin
							addr_next = addr + 23'd1;
							memory_test_ctl_next = MEMORY_TEST_START;
						end
					end
				end
			end
			MEMORY_TEST_FINISHED: begin
				led2_mode_next = 3; led2_fast_next = 1;
				debug11q_next = 1;

			end

			MEMORY_TEST_FAULT: begin
				led2_mode_next = 15; led2_fast_next = 1;
			end
					
				
			default: begin			
			end
			
		endcase		
	
end

	
	assign debug7 = debug7q;
	assign debug11 = debug11q;
	

	
endmodule
