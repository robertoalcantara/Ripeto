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
	//parameter ONE_SEC_DLY = 64'd50000000;
	//parameter HALF_SEC_DLY = 64'd12000000; //25
	
	//clock aux
	reg [7:0] clk_1M_cnt;
	reg clk_1M;
	reg [7:0] clk_500k_cnt;
	reg clk_500k;
	
    wire clk100;		
	clk_wiz_v3_6 clkPLL (
        .CLK_IN1(clk), // IN
        .CLK_OUT1(clk100) // OUT
    );

	assign rst_p = ~rst;
	//memory user interface
	reg [22:0] addr;      // address to read/write
	reg rw;               // 1 = write, 0 = read
	reg [31:0] data_in;   // data from a read
	wire [31:0] data_out; // data for a write
	wire ready;
	wire out_valid;        // pulses high when data from read is valid
	reg enable;
	reg [31:0] data_read;

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

	reg [7:0] tx_byte;  
	reg tx_en;
	wire tx_ready;
	
	uart UART0(
    .clk(clk100),
    .rst(rst_p),
    .tx_byte(tx_byte),
    .tx_en(tx_en),
    .tx_ready(tx_ready),
    .tx_pin(uart_tx_pin)
    );
	
	reg led1_debug, led2_debug;

	(* IOB = "TRUE" *)
	reg debug7q;
	(* IOB = "TRUE" *)
	reg debug11q;

	reg [1:0]tx_state;	
	reg tx_go;
	
	reg [7:0] state_ctl;
	

	always @(posedge clk100) begin

	if ( !rst ) begin
		clk_1M_cnt <= 8'd0;
		clk_1M <= 0;
		clk_500k_cnt <= 8'd0;
		clk_500k <= 0;
	end
	else begin
		if  (clk_1M_cnt == 8'd50)  begin 
			clk_1M <= ~clk_1M;
			clk_1M_cnt <= 8'd0;
		end 
		else begin
			clk_1M_cnt <= clk_1M_cnt + 8'd1;
		end
		
		if (clk_500k_cnt == 8'd100) begin
			clk_500k <= ~clk_500k;
			clk_500k_cnt <= 8'd0;
		end
		else begin
			clk_500k_cnt <= clk_500k_cnt + 8'd1;
		end
		
	end
end
	
	
reg[31:0] cnt_seg; 
parameter CTL_START = 0;
parameter WAITING_MEMORY_TEST = 1;


reg [7:0] memory_test_ctl;
parameter MEMORY_TEST_IDLE 		= 0;
parameter MEMORY_TEST_START	    = 1;
parameter MEMORY_TEST_WAIT 		= 2;
parameter MEMORY_TEST_LOOP 		= 3;
parameter MEMORY_TEST_CHECK		= 4;
parameter MEMORY_TEST_FINISHED  = 5;
parameter MEMORY_TEST_FAULT     = 6;

reg[31:0] data_tmp;
reg[7:0] cnt_tmp;
	
always @(posedge clk_1M or negedge rst) begin

	if ( !rst ) begin
		state_ctl <= 8'd0;
		debug7q <= 0;
		cnt_seg <= 0;
		led1_debug <= 1;
		
		memory_test_ctl <= MEMORY_TEST_IDLE;
		addr <= 23'd0;
		data_in <= 0;
		rw <= 0;
		led2_debug <= 1; //apaga
		debug11q <= 0;
		data_tmp <= 0;
		
		cnt_tmp <= 0;
	end 
	else begin
	
		case (state_ctl) 
			CTL_START: begin
				state_ctl <= WAITING_MEMORY_TEST;
				memory_test_ctl <= MEMORY_TEST_START;
			end
			WAITING_MEMORY_TEST: begin
				//if (memory_test_ctl == MEMORY_TEST_FINISHED) state_ctl <= 3;
			end
			
			default: begin
			end
		
		endcase
		
		
		case (memory_test_ctl) 
			MEMORY_TEST_IDLE: begin
			end
			
			MEMORY_TEST_START: begin
				if ( ready ) begin
					data_in <= 32'hAAAA;
					rw <= 1;
					enable <= 1;
					addr <= addr+23'h1;
					memory_test_ctl <= MEMORY_TEST_WAIT;
				end
			end
			
			MEMORY_TEST_WAIT: begin
				rw <= 0;
				enable <= 0;
				if ( ready ) begin
					memory_test_ctl <= MEMORY_TEST_LOOP;
				end
			end
			
			MEMORY_TEST_LOOP: begin
				rw <= 0;
				enable <= 1;
				memory_test_ctl <= MEMORY_TEST_CHECK;
				if (tx_ready) begin
					tx_byte <= addr[15:8];
					tx_en <= 1;
				end
			end
				
			MEMORY_TEST_CHECK: begin
				tx_en <= 0;

				if (out_valid) begin
				
					if ( data_out != 32'hAAAA ) begin
							led2_debug <= 0; //acende
							//fica parado, erro
							memory_test_ctl <= MEMORY_TEST_FAULT;
					end 
					else begin 
					
						enable <= 0;
						if (addr == 23'd8388606-1) begin
						//if (addr == 23'd10) begin
							memory_test_ctl <= MEMORY_TEST_FINISHED;
						end
						else begin
							memory_test_ctl <= MEMORY_TEST_START;
						end
					end
				end
			end
			MEMORY_TEST_FINISHED: begin
				debug11q <= 1;
			end

			MEMORY_TEST_FAULT: begin
			end
					
				
			default: begin			
			end
			
		endcase		
	
	
	    cnt_seg <= cnt_seg + 32'd1;
		if (cnt_seg == 32'd500000) begin
			cnt_seg <= 0;
			led1_debug <= ~led1_debug; //led pulse
		
		end
	
	end
end

	/*
always @(posedge out_valid or negedge rst) begin
	if (!rst) begin
		data_readed <= 0;
	
	end
	else begin
		data_readed <= data_out;
	end
		
end*/
	
	assign led1 = led1_debug;
	assign led2 = led2_debug;	
	assign debug7 = out_valid;//debug7q;
	assign debug11 = debug11q;
	

	
endmodule
