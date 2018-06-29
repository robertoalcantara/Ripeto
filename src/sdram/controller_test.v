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
	output uart_tx_pin,
	
	input spi0_miso, //U3 current
	output spi0_clkout,
	output spi0_cs,

	input spi1_miso,  //U1 voltage
	output spi1_clkout,
	output spi1_cs	
	
);
	//parameter ONE_SEC_DLY = 64'd50000000;
	//parameter HALF_SEC_DLY = 64'd12000000; //25
	
	//clock aux
	reg [7:0] clk_25M_cnt;
	reg clk_25M;

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
	
	wire [21:0] sampler_data_out;
	reg sampler_start;
	wire sampler_busy;
	wire sampler_new_data;
	
	sampler #(.CLK_DIV(60)) SAMPLER0 ( 
		.clk(clk100),
		.rst(rst_p),
		.start(sampler_start),
		.busy(sampler_busy),
		.new_data(sampler_new_data),
		.data_out(sampler_data_out),
		.voltage0_miso_pin(spi1_miso),
		.voltage0_clkout_pin(spi1_clkout),
		.voltage0_cs_pin(spi1_cs),
		.current0_miso_pin(spi0_miso),
		.current0_clkout_pin(spi0_clkout),
		.current0_cs_pin(spi0_cs)		
	);
	
	
	/*
	wire [11:0] spi0_data_out;
	reg spi0_start;
	wire spi0_busy;
	wire spi0_new_data;
	
	mcp3201_spi #(.CLK_DIV(60)) SPI0 ( //7 = ~781kHz
		.clk(clk100),
		.rst(rsp_p),
		.data_in_pin(spi0_miso),
		.clk_pin(spi0_clkout),
		.start(spi0_start),
		.data_out(spi0_data_out),
		.busy(spi0_busy),
		.new_data(spi0_new_data),
		.cs_pin_n(spi0_cs)
	);
	

	wire [11:0] spi1_data_out;
	reg spi1_start;
	wire spi1_busy;
	wire spi1_new_data;
	
	mcp3201_spi #(.CLK_DIV(60)) SPI1 ( //7 = ~781kHz
		.clk(clk100),
		.rst(rsp_p),
		.data_in_pin(spi1_miso),
		.clk_pin(spi1_clkout),
		.start(spi1_start),
		.data_out(spi1_data_out),
		.busy(spi1_busy),
		.new_data(spi1_new_data),
		.cs_pin_n(spi1_cs)
	);*/


//debug
(* IOB = "TRUE" *)
reg led1_debug, led2_debug;
reg debug7q;
reg debug11q;

reg [7:0] state_ctl;
	
always @(posedge clk100) begin

	if ( !rst ) begin
		clk_25M_cnt <= 8'd0;
		clk_25M <= 0;

	end
	else begin
		if  (clk_25M_cnt == 8'd2)  begin 
			clk_25M <= ~clk_25M;
			clk_25M_cnt <= 8'd0;
		end 
		else begin
			clk_25M_cnt <= clk_25M_cnt + 8'd1;
		end
	
	end
end
	
	
reg[31:0] cnt_seg; 
parameter CTL_START = 0;
parameter WAITING_MEMORY_TEST = 1;
parameter WAITING_SPI_TEST = 2;


reg [7:0] memory_test_ctl;
parameter MEMORY_TEST_IDLE 		= 0;
parameter MEMORY_TEST_START	    = 1;
parameter MEMORY_TEST_WAIT 		= 2;
parameter MEMORY_TEST_LOOP 		= 3;
parameter MEMORY_TEST_CHECK		= 4;
parameter MEMORY_TEST_FINISHED  = 5;
parameter MEMORY_TEST_FAULT     = 6;

reg [7:0] spi_test_ctl;
parameter SPI_TEST_IDLE 		= 0;
parameter SPI_TEST_START		= 1;
parameter SPI_TEST_RUNNING		= 2;
parameter SPI_TEST_RUNNING2		= 3;
parameter SPI_TEST_PRINT		= 4;
parameter SPI_TEST_PRINT2		= 5;
parameter SPI_TEST_PRINT3		= 6;
parameter SPI_TEST_PRINT4		= 7;
parameter SPI_TEST_PRINT5		= 8;
parameter SPI_TEST_PRINT6		= 9;
parameter SPI_TEST_FAULT 		= 19;
parameter SPI_TEST_FINISHED		= 20;


reg[31:0] data_tmp;
reg[20:0] cnt_tmp;

reg [11:0] amost_output_r;
//wire [11:0] amost_output;
//assign amost_output[15:0] = { (amost_output1&8'b00011111), amost_output2[7:1] };   

always @(posedge clk_25M or negedge rst) begin

	if ( !rst ) begin
		state_ctl <= 8'd0;
		debug7q <= 0;
		cnt_seg <= 0;
		led1_debug <= 1;
		
		memory_test_ctl <= MEMORY_TEST_IDLE;
		spi_test_ctl <= SPI_TEST_IDLE;
		state_ctl <= CTL_START;
		
		addr <= 23'd0;
		data_in <= 0;
		rw <= 0;
		//led2_debug <= 1; //apaga
		debug11q <= 0;
		data_tmp <= 0;
		amost_output_r <= 0;
		
		cnt_tmp <= 0;
	end 
	else begin
		case (state_ctl) 
			CTL_START: begin
				state_ctl <= WAITING_SPI_TEST;
				//state_ctl <= WAITING_MEMORY_TEST;
				//memory_test_ctl <= MEMORY_TEST_START;
				spi_test_ctl <= SPI_TEST_IDLE;

			end
			WAITING_MEMORY_TEST: begin
				if (memory_test_ctl == MEMORY_TEST_FINISHED) state_ctl <= WAITING_SPI_TEST;
			end
			WAITING_SPI_TEST: begin
				if (spi_test_ctl == SPI_TEST_FINISHED) state_ctl <= state_ctl+1;
			end
			
			default: begin
			end
		
		endcase


		case (spi_test_ctl)
			SPI_TEST_IDLE: begin
			
				cnt_tmp <= cnt_tmp +1;
				if (cnt_tmp == 65000) begin
					sampler_start <= 1;
					spi_test_ctl <= SPI_TEST_START;
					cnt_tmp <= 0;
				end
			
				
			end
			SPI_TEST_START: begin
				tx_en <= 0;
				if (! sampler_busy) begin
					spi_test_ctl <= SPI_TEST_RUNNING;
				end
			end
			SPI_TEST_RUNNING: begin
				if (sampler_new_data == 1)  begin
					amost_output_r <= sampler_data_out[11:0];
					spi_test_ctl <= SPI_TEST_PRINT;
					sampler_start <= 0;
				end
			end
			SPI_TEST_PRINT: begin
				if (amost_output_r[9] == 1 )
					led2_debug <= 0; //acende
				else
					led2_debug <= 1; //apaga
					
				if (tx_ready) begin
					tx_byte <= 8'hAA; //TEST flag
					tx_en <= 1;
					spi_test_ctl <= SPI_TEST_PRINT2;
				end
			end
			SPI_TEST_PRINT2: begin
				tx_en <= 0;
				spi_test_ctl <= SPI_TEST_PRINT3;
			end
			SPI_TEST_PRINT3: begin
				if (tx_ready) begin
					tx_byte <= { 4'b0000, amost_output_r[11:8] };
					tx_en <= 1;
					spi_test_ctl <= SPI_TEST_PRINT4;
				end
			end
			SPI_TEST_PRINT4: begin
				tx_en <= 0;
				spi_test_ctl <= SPI_TEST_PRINT5;
			end
			
			SPI_TEST_PRINT5: begin
				if (tx_ready) begin
					tx_byte <= amost_output_r[7:0];
					tx_en <= 1;
					spi_test_ctl <= SPI_TEST_PRINT6;
				end
			end
			SPI_TEST_PRINT6: begin
				tx_en <= 0;
				spi_test_ctl <= SPI_TEST_IDLE;
			end			
			
			SPI_TEST_FAULT:begin
				//do nothing
			end
			
			SPI_TEST_FINISHED: begin
			end
			
			default: begin
			end
		endcase //spi_test


		case (memory_test_ctl) 
			MEMORY_TEST_IDLE: begin
			end
			
			MEMORY_TEST_START: begin
				if ( ready ) begin
					data_in <= data_tmp;
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
				
					if ( data_out != data_tmp ) begin
							led2_debug <= 0; //acende
							//fica parado, erro
							memory_test_ctl <= MEMORY_TEST_FAULT;
					end 
					else begin 
					
						enable <= 0;
						data_tmp <= data_tmp + 7'd1;

						if (addr == 23'd8388606-1) begin
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
