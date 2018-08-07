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
	
	inout dac_sda,
	inout dac_scl,
	
	input spi0_miso, //U3 current
	output spi0_clkout,
	output spi0_cs,

	input spi1_miso,  //U1 voltage
	output spi1_clkout,
	output spi1_cs	
	
);

    wire clk100;		
	clk_wiz_v3_6 clkPLL (
        .CLK_IN1(clk), // IN
        .CLK_OUT1(clk100) // OUT
    );

	/*wire clk_dac;
	clk_divider #(.CLK_DIV(1000)) clk_div_dac (
		.clk(clk100), 
		.clk_out(clk_dac), 
		.rst(rst_p)
    );*/
	
	wire clk_25M;
	clk_divider #(.CLK_DIV(4)) clk_div_25M (
		.clk(clk100), 
		.clk_out(clk_25M), 
		.rst(rst_p)
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
	
	/*reg dac_enable;
	reg [6:0] dac_addr;
	reg dac_rw;
	reg [7:0] dac_data_wr;
	wire dac_busy;
	wire [7:0] dac_data_rd;
	wire dac_ack_error;*/
	
	/*i2c_master dac (
		.clk(clk100), 
		.reset_n(rst), 
		.ena(dac_enable), 
		.addr(dac_addr), 
		.rw(dac_rw), 
		.data_wr(dac_data_wr), 
		.busy(dac_busy), 
		.data_rd(dac_data_rd), 
		.ack_error(dac_ack_error), 
		.sda(dac_sda), 
		.scl(dac_scl)
	);*/
	
		
	// Instantiate the module
	/*reg [6:0] dac_cmd_address;
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
	wire sda_o;
	wire sda_t;
	
	wire dac_busy;
	wire dac_bus_control;
	wire dac_missed_ack;
	
	i2c_master dac (
		.clk(clk100), 
		.rst(rst_p), 
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
		.prescale(15'd250), 
		.stop_on_idle(1'b1)
		);

	assign scl_i = dac_scl;
	assign dac_scl = scl_o ? 1'bz : 1'b0;
	assign sda_i = dac_sda;
	assign dac_sda = sda_o ? 1'bz : 1'b0;*/




	reg [15:0] dac_value;
	// Instantiate the module
	dac dac1 (
		.clk(clk100), 
		.rst(rst_p), 
		.ch_value(dac_value), 
		.i2c_scl_pin(dac_scl), 
		.i2c_sda_pin(dac_sda)
		);


//debug
(* IOB = "TRUE" *)
reg led1_debug, led2_debug;
reg debug7q;
reg debug11q;

reg [7:0] state_ctl;
	
	
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

/*
reg [7:0] dac_test_ctl;
parameter MCP47FEB_ID			= 7'b110_0000; 
parameter DAC0_REG				= 5'b0;
parameter DAC1_REG				= 5'b1;
parameter CMD_WRITE				= 2'b0;
parameter CMD_READ				= 2'b11;


parameter DAC_TEST_IDLE			= 0;
parameter DAC_TEST_START		= 1;


reg[31:0] data_tmp;
reg[20:0] cnt_tmp;
reg [15:0] dac_value;

reg [11:0] amost_output_r;

reg [20:0] startup_delay;
*/

reg [15:0] dac_cnt;

always @(posedge clk100 or posedge rst_p) begin

	if ( rst_p ) begin
		state_ctl <= 8'd0;
		debug7q <= 0;
		cnt_seg <= 0;
		led1_debug <= 1;
		
		memory_test_ctl <= MEMORY_TEST_IDLE;
		spi_test_ctl <= SPI_TEST_IDLE;
		state_ctl <= CTL_START;
//		dac_test_ctl <= DAC_TEST_IDLE;
		
		addr <= 23'd0;
		data_in <= 0;
		rw <= 0;
//		amost_output_r <= 0;

		
		led2_debug <= 1; //apaga
		debug11q <= 0;
//		data_tmp <= 0;
		/*
		cnt_tmp <= 0;
		
		dac_value <=0;		

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
		startup_delay <= 0;*/
		
		dac_value <= 0;
		dac_cnt <= 0;
	end 
	else begin
			
		/*case (dac_test_ctl)
			DAC_TEST_IDLE: begin
			led2_debug <= 1; //apaga
				startup_delay <= startup_delay + 1;
				if (startup_delay > 100_000) dac_test_ctl <= DAC_TEST_START;
			end
			
			1: begin
				if ( dac_cmd_ready ) begin
					dac_cmd_write_multiple <= 1;
					dac_cmd_valid <= 1;
					dac_cmd_stop <= 1;
					dac_cmd_start <= 1;
					dac_data_in_last <= 0;
					dac_cmd_address <= MCP47FEB_ID;
					dac_test_ctl <= 2;
				end
			end
			
			2: begin
					dac_cmd_valid <= 0;
					dac_data_in_last <= 0;
					dac_data_in_valid <= 1;
					dac_data_in <= {DAC1_REG, CMD_WRITE, 1'b0};
					if (dac_data_in_ready) dac_test_ctl <= 3;
			end
			
			3: begin
					dac_data_in_last <= 0;
					dac_data_in_valid <= 1;
					dac_data_in <= dac_value[15:8];
					if (dac_data_in_ready) dac_test_ctl <= 4;
			end
	
			4: begin
					dac_data_in_last <= 1;
					dac_data_in_valid <= 1;
					dac_data_in <= dac_value[7:0];
					if (!dac_busy) dac_test_ctl <= 5;
			end
						
			5: begin
				dac_data_in_last <= 0;
				dac_cmd_valid <= 0;			
				dac_cmd_write_multiple <= 0;
				led2_debug <= 0; //acende
				dac_value <= dac_value +1;
				dac_test_ctl <= 1;

			end
			
			default: begin
			
			end
			
		
		endcase*/
	
	
	
/*		case (state_ctl) 
			CTL_START: begin
				//state_ctl <= WAITING_SPI_TEST;
				//state_ctl <= WAITING_MEMORY_TEST;
				//memory_test_ctl <= MEMORY_TEST_START;
				//spi_test_ctl <= 20;// SPI_TEST_IDLE;

			end
			WAITING_MEMORY_TEST: begin
				//if (memory_test_ctl == MEMORY_TEST_FINISHED) state_ctl <= WAITING_SPI_TEST;
			end
			WAITING_SPI_TEST: begin
				//if (spi_test_ctl == SPI_TEST_FINISHED) state_ctl <= state_ctl+1;
			end
			
			default: begin
			end
		
		endcase
*/
/*
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
*/

/*		case (memory_test_ctl) 
			MEMORY_TEST_IDLE: begin
				led2_debug <= 1; //apaga
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
			
		endcase*/		
	
		dac_cnt <= dac_cnt + 15'd1;
		if (dac_cnt == (32'd25000)/2) begin
			dac_cnt <= 0;
			dac_value <= 512;//dac_value + 50;
		end
		
		
	    cnt_seg <= cnt_seg + 32'd1;
		if (cnt_seg == (32'd25000000)/2) begin 
			cnt_seg <= 0;
			led1_debug <= ~led1_debug; //led pulse
		end
	
	end
end

	
	assign led1 = led1_debug;
	assign led2 =  led2_debug;	
	assign debug7 = debug7q;
	assign debug11 = debug11q;
	

	
endmodule
