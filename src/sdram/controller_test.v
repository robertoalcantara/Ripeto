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
	reg [20:0] addr, addr_next;      // address to read/write
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
	
(* IOB = "TRUE" *)
reg led1_debug, led2_debug;
reg led1_debug_next, led2_debug_next;

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

		memory_test_ctl <= MEMORY_TEST_START;
		addr <= 20'd0;
		rw <= 0;
		data_in <= 0;
		enable <= 0;
		data_tmp <= 0;
		
		tx_byte <= 0;
		tx_en <= 0;
		
		debug11q <= 0;
		debug7q <= 0;
		led1_debug <= 1; //apaga
		led2_debug <= 1; //apaga
		
	end 
	else begin
	
		tx_byte <= tx_byte_next;
		tx_en <= tx_en_next;
	
		debug11q <= debug11q_next;
		debug7q <= debug7q_next;
		//led1_debug <= led1_debug_next;
		led2_debug <= led2_debug_next;
	
		addr <= addr_next;
		rw <= rw_next;
		data_in <= data_in_next;
		enable <= enable_next;
		data_tmp <= data_tmp_next;
		memory_test_ctl <= memory_test_ctl_next;
		

	   cnt_seg <= cnt_seg + 32'd1;
		if (cnt_seg == (32'd25000000)/2) begin 
			cnt_seg <= 0;
			led1_debug <= ~led1_debug; //led pulse
		end 		
		
	end
end




always @(*) begin

		addr_next = addr;
		rw_next = rw;
		data_in_next = data_in;
		enable_next = enable;
		data_tmp_next = data_tmp;
		memory_test_ctl_next = memory_test_ctl;
		
		tx_byte_next = tx_byte;
		tx_en_next = tx_en;
		debug11q_next = debug11q;
		debug7q_next = debug7q;
		led1_debug_next = led1_debug;
		led2_debug_next = led2_debug;
		
				
		case (memory_test_ctl) 
			
			MEMORY_TEST_START: begin
				tx_en_next = 0; //debug

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

						if (addr == 88606) begin
							memory_test_ctl_next = MEMORY_TEST_FINISHED;
						end
						else begin
							addr_next = addr + 1;
							memory_test_ctl_next = MEMORY_TEST_START;
						end
					end
				end
			end
			MEMORY_TEST_FINISHED: begin
				debug11q_next = 1;
			end

			MEMORY_TEST_FAULT: begin
				led2_debug_next = 0; //acende
			end
					
				
			default: begin			
			end
			
		endcase		
	
end

	
	assign led1 = led1_debug;
	assign led2 =  led2_debug;	
	assign debug7 = debug7q;
	assign debug11 = debug11q;
	

	
endmodule
