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
	
	output uart_tx_pin,
	input uart_rx_pin,

	input spi0_miso, //AMOST2 U3 current
	output spi0_clkout,
	output spi0_cs,

	input spi1_miso,  //AMOST2 U1 voltage
	output spi1_clkout,
	output spi1_cs,
	
	output dac_lat_pin, //DAC
	inout dac_scl_pin, 
	inout dac_sda_pin, 
	output dac_up1_pin,
	output dac_up2_pin,	
	
	input [15:0] fpin, //logic analizer
	
	input replay_on_pin,
	output replay_up_pin
	
);

   wire clk100;		
	clk_wiz_v3_6 clkPLL (
        .CLK_IN1(clk), // IN
        .CLK_OUT1(clk100) // OUT
    );
	
	assign rst_p = ~rst;

	assign replay_up_pin = 1;

   //DRAM memory organization
	parameter ADDR_LOW_LOG = 0;
	parameter ADDR_HIGH_LOG = 4194303;
	parameter ADDR_LOW_CURVE = ADDR_HIGH_LOG+1;
	parameter ADDR_HIGH_CURVE = 8388607;

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
	

	reg amost2_start, amost2_start_next;	
	wire amost2_busy;
	wire [11:0] amost2_data_v;
	wire [11:0] amost2_data_i;  
	wire [5:0] amost2_checksum;
	assign amost2_checksum = amost2_data_v[11:4] + {amost2_data_v[3:0],amost2_data_i[11:8]} + amost2_data_i[7:0]; 

	meter AMOST2(
	 .rst(rst_p),
	 .clk(clk100),
	 .start(amost2_start),
	 .busy(amost2_busy),
	 .data_v(amost2_data_v),
	 .data_i(amost2_data_i),
		
	.voltage0_miso_pin(spi1_miso), 
	.voltage0_clkout_pin(spi1_clkout),
	.voltage0_cs_pin(spi1_cs),

	.current0_miso_pin(spi0_miso), 
	.current0_clkout_pin(spi0_clkout),
	.current0_cs_pin(spi0_cs)
	); 
	
	wire [15:0] logic_data;
	wire logic_event;
	wire [15:0] f_pin; // data for a write
	reg logic_event_saved, logic_event_saved_next;
	reg [31:0] logic_pack, logic_pack_next;
	reg logic_event_ack, logic_event_ack_next;
	wire[5:0] logic_checksum;
	assign logic_checksum = f_pin[15:8] + f_pin[7:0]; 

	logic_capture LOGIG_CAP(
		.rst(rst_p),
		.clk(clk100),
		.data(logic_data),
		.event_detected(logic_event),
		.f_pin(fpin)
	);
	
	
	reg[2:0] dump_type, dump_type_next;
	reg [7:0] tx_byte, tx_byte_next;  
	reg tx_en, tx_en_next;
	wire tx_ready;
	wire tx_active;
	assign tx_ready = !tx_active;
	
	uart_tx UARTTX0(
		.i_Clock(clk100),
		.i_Tx_Byte(tx_byte),
		.i_Tx_DV(tx_en),
		.o_Tx_Active(tx_active),
		.o_Tx_Serial(uart_tx_pin)
    );	
	 
	wire rx_dv;
	wire [7:0] rx_byte;
	uart_rx UARTRX0(
		.i_Clock(clk100),
		.o_Rx_DV(rx_dv),
		.o_Rx_Byte(rx_byte),
		.i_Rx_Serial(uart_rx_pin)
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

	// Instantiate the module
	reg [11:0] dac_value, dac_value_next;
	wire dac_busy;
	wire clk_dac;
	reg dac_enable, dac_enable_next;
	dac DAC (
		.clk(clk100), 
		.rst(rst_p), 
		.enable(dac_enable),
		.ch_value(dac_value), 
		.busy(dac_busy), 
		.i2c_scl_pin(dac_scl_pin), 
		.i2c_sda_pin(dac_sda_pin)
		);
		
	assign dac_lat_pin = 0;
	assign dac_up1_pin = 1;
	assign dac_up2_pin = 1;	
	


(* IOB = "TRUE" *)
reg debug7q, debug11q;
reg debug7q_next, debug11q_next;

reg [7:0] state_main, state_main_next;
parameter MAIN_IDLE = 0;  parameter MAIN_MEMORY_CLEANUP = 1;
parameter MAIN_SAMPLING = 2; parameter MAIN_DUMPING = 3; parameter MAIN_LOAD_CURVE = 5;
parameter MAIN_REPLAY = 6;

reg [7:0] sampling_ctl, sampling_ctl_next;
parameter SAMPLER_IDLE = 0;
parameter SAMPLER_WAITING_START = 2; parameter SAMPLER_SAMPLING = 3; parameter SAMPLER_SAMPLING_SYNC=4;
parameter SAMPLER_SAMPLING_SAVE= 5 ; parameter SAMPLER_SAMPLING_DONE = 6; 

reg [7:0] sampling_logic_ctl, sampling_logic_ctl_next;
parameter SAMPLER_LOGIC_IDLE=0; parameter SAMPLER_LOGIC_RUNNING=1; 
parameter SAMPLER_LOGIC_SAVE=2; parameter SAMPLER_LOGIC_DONE=3;

reg [7:0] memory_test_ctl, memory_test_ctl_next;
parameter MEMORY_CLEANUP_IDLE = 0;  parameter MEMORY_CLEANUP_START = 1;
parameter MEMORY_CLEANUP_WAIT = 2;  parameter MEMORY_CLEANUP_LOOP  = 3;
parameter MEMORY_CLEANUP_CHECK = 4;  parameter MEMORY_CLEANUP_FINISHED = 5;
parameter MEMORY_CLEANUP_FAULT = 6;

reg [7:0] serial_dump_ctl, serial_dump_ctl_next;
parameter SERIAL_DUMP_IDLE = 0; parameter SERIAL_DUMP_SETUP=1;parameter SERIAL_DUMP_RUNNING =2; parameter SERIAL_DUMP_RUNNING1 =3;  
parameter SERIAL_DUMP_RUNNING2 =4; parameter SERIAL_DUMP_RUNNING3 =5;  parameter SERIAL_DUMP_RUNNING4=6; parameter SERIAL_DUMP_RUNNING5=7; 
parameter SERIAL_DUMP_TX=8; parameter SERIAL_DUMP_DONE=9;

reg [7:0] load_curve_ctl, load_curve_ctl_next;
parameter LOAD_CURVE_IDLE = 0; parameter LOAD_CURVE_WAIT_SOH = 1;  parameter LOAD_CURVE_WAIT_RX=2; parameter LOAD_CURVE_RECORD=3;
parameter LOAD_CURVE_CHECK=4; parameter LOAD_CURVE_RECORD2 =5; parameter LOAD_CURVE_FINISHED = 6; parameter LOAD_CURVE_ERROR = 7; 

reg [7:0] replay_curve_ctl, replay_curve_ctl_next;
parameter REPLAY_CURVE_IDLE=0; parameter REPLAY_CURVE_RUN=1;  parameter REPLAY_LOOKUP=3; parameter REPLAY_UPDATE=4;

reg[7:0] load_curve_byte_count, load_curve_byte_count_next;
reg[13:0] load_curve_reg_count, load_curve_reg_count_next;
reg [7:0] load_byte [4:0];
wire[7:0] load_checksum = load_byte[0] + load_byte[1] + load_byte[2] + load_byte[3]; 

always @(posedge clk100 or posedge rst_p) begin

	if ( rst_p ) begin
	
		state_main <= 0;
		memory_test_ctl <= MEMORY_CLEANUP_IDLE;
		sampling_ctl <= 0;
		serial_dump_ctl <= 0;
		sampling_logic_ctl <= 0;
		load_curve_ctl <= 0;
		replay_curve_ctl <= 0;
		
		addr <= 0;
		rw <= 0;
		data_in <= 0;
		enable <= 0;
		
		amost2_start <= 0;
		
		logic_event_saved <= 0;
		logic_pack <= 0;	
		logic_event_saved <= 0;
		logic_event_ack <= 0;
		
		tx_byte <= 0;
		tx_en <= 0;
		dump_type <= 0;
				
		load_curve_byte_count <= 0;
		load_curve_reg_count <= 0;
				
		debug11q <= 0;
		debug7q <= 0;

		led1_mode <= 0;
		led1_fast <= 0;
		led2_mode <= 0;
		led2_fast <= 0;

		dac_enable <= 0;	
		dac_value <= 0;		
	end 
	else begin
	
		tx_byte <= tx_byte_next;
		tx_en <= tx_en_next;
		dump_type <= dump_type_next;
	
		debug11q <= debug11q_next;
		debug7q <= debug7q_next;
	
		addr <= addr_next;
		rw <= rw_next;
		data_in <= data_in_next;
		enable <= enable_next;
		
		amost2_start <= amost2_start_next;
		
		state_main <= state_main_next;
		memory_test_ctl <= memory_test_ctl_next;
		sampling_ctl <= sampling_ctl_next;
		sampling_logic_ctl <= sampling_logic_ctl_next;
		serial_dump_ctl <= serial_dump_ctl_next;
		load_curve_ctl <= load_curve_ctl_next;
		replay_curve_ctl <= replay_curve_ctl_next;
		
	
		load_curve_byte_count <= load_curve_byte_count_next;
		load_curve_reg_count <= load_curve_reg_count_next;

		dac_enable <= dac_enable_next;
		dac_value <= dac_value_next;
	
		logic_event_saved <= logic_event_saved_next;
		logic_pack <= logic_pack_next;	
		logic_event_ack <= logic_event_ack_next;

		led1_mode <= led1_mode_next;
		led1_fast <= led1_fast_next;
		led2_mode <= led2_mode_next;
		led2_fast <= led2_fast_next;
		
	end
end




always @(*) begin

	addr_next = addr;
	rw_next = rw;
	data_in_next = data_in;
	enable_next = enable;
	amost2_start_next = amost2_start;
	
	logic_event_ack_next = logic_event_ack;
	
	state_main_next = state_main;
	memory_test_ctl_next = memory_test_ctl;
	sampling_ctl_next = sampling_ctl;
	sampling_logic_ctl_next = sampling_logic_ctl;
	serial_dump_ctl_next = serial_dump_ctl;
	load_curve_ctl_next = load_curve_ctl;
	replay_curve_ctl_next = replay_curve_ctl;


	load_curve_byte_count_next = load_curve_byte_count;
	load_curve_reg_count_next = load_curve_reg_count;

	led1_mode_next = led1_mode;
	led1_fast_next = led1_fast;
	led2_mode_next = led2_mode;
	led2_fast_next = led2_fast;

	tx_byte_next = tx_byte;
	tx_en_next = tx_en;
	dump_type_next = dump_type;

	dac_enable_next = dac_enable;
	dac_value_next = dac_value;
	
	debug11q_next = debug11q;
	debug7q_next = debug7q;

	/* //loop back serial test 
	if (rx_dv) begin
		tx_byte_next = rx_byte;
		tx_en_next = 1;
	end 
	else tx_en_next = 0;*/

	case (state_main)

		MAIN_IDLE: begin
			led1_mode_next = 1; led1_fast_next = 0;
			led2_mode_next = 1; led2_fast_next = 0;
			
			/**** debug adc
			dac_enable_next = 1;
			if (dac_value == 0 ) begin
				dac_value_next = 2048;
				debug7q_next = 1;
			end
			else begin
				dac_value_next = 0;	
				debug7q_next = 0;
			end
			debug7q_next = !debug7q;
			dac_value_next = dac_value + 5;
			state_main_next = MAIN_MEMORY_CLEANUP;
          *** end debug adc */
			
			if (sw2_state)	begin
				state_main_next = MAIN_MEMORY_CLEANUP;
				memory_test_ctl_next = MEMORY_CLEANUP_START;
			end
		end //MAIN_IDLE
			
		MAIN_MEMORY_CLEANUP: begin /***** M E M O R Y  C L E A N  UP ****/
			
			/*debug adc if (!dac_busy) begin
				state_main_next = MAIN_IDLE;//debug adc
				//dac_enable_next = 0; //debug adc
			end*/
			
			case (memory_test_ctl) 
				MEMORY_CLEANUP_IDLE: begin
				end
				MEMORY_CLEANUP_START: begin
					led2_mode_next = 1;	led2_fast_next = 0;

					if ( ready ) begin
						data_in_next = 32'd0; //zero all memory
						rw_next = 1;
						enable_next = 1;
						memory_test_ctl_next = MEMORY_CLEANUP_WAIT;
					end
				end
				MEMORY_CLEANUP_WAIT: begin
					if ( ready ) begin
						rw_next = 0;
						enable_next = 0;
						memory_test_ctl_next = MEMORY_CLEANUP_LOOP;
					end
				end
				MEMORY_CLEANUP_LOOP: begin
					if ( ready ) begin
						enable_next = 1;
						memory_test_ctl_next = MEMORY_CLEANUP_CHECK;
					end
				end
				MEMORY_CLEANUP_CHECK: begin
					if (out_valid ) begin				
						if ( data_out != 0 ) begin
								memory_test_ctl_next = MEMORY_CLEANUP_FAULT;
						end 
						else begin 
							enable_next = 0;
							if (addr ==  8388608-1) begin
								memory_test_ctl_next = MEMORY_CLEANUP_FINISHED;
								addr_next = 0;
							end
							else begin
								addr_next = addr + 23'd1;
								memory_test_ctl_next = MEMORY_CLEANUP_START;
							end
						end
					end
				end
				MEMORY_CLEANUP_FINISHED: begin
					led2_mode_next = 3; led2_fast_next = 1;
					debug11q_next = 1;
					addr_next = 0;
					rw_next = 0;
					data_in_next = 0;
					enable_next = 0;	
					
					if ( ! replay_on_pin ) begin   //SAMPLING OR REPLAY
						state_main_next = MAIN_SAMPLING;
						sampling_ctl_next = SAMPLER_WAITING_START;
					end
					else begin
						state_main_next = MAIN_LOAD_CURVE;
						load_curve_ctl_next = LOAD_CURVE_IDLE;
					end
					
				end
				MEMORY_CLEANUP_FAULT: begin
					led2_mode_next = 15; led2_fast_next = 1;
					//just stop
				end		
			endcase //case (memory_test_ctl)
		end//MAIN_MEMORY_CLEANUP
		
		                               /****** S A M P L I N G ******/
		
		MAIN_SAMPLING:	begin    
			case (sampling_ctl) 
				SAMPLER_IDLE: begin
				end 
				SAMPLER_WAITING_START: begin	
					if (sw2_state) sampling_ctl_next = SAMPLER_SAMPLING_SYNC;
				end 
				SAMPLER_SAMPLING_SYNC: begin
						amost2_start_next = 1;
						sampling_ctl_next = SAMPLER_SAMPLING;
						sampling_logic_ctl_next = SAMPLER_LOGIC_RUNNING;
				end
				SAMPLER_SAMPLING: begin
					led2_mode_next = 2; led2_fast_next = 1;
					amost2_start_next = 0;
					if (sw2_state) sampling_ctl_next = SAMPLER_SAMPLING_DONE;

					if (amost2_busy==0) begin
						//formatar pacote
						data_in_next = {amost2_data_v, amost2_data_i,amost2_checksum,1'b0,1'b1}; //DATA!
						sampling_ctl_next = SAMPLER_SAMPLING_SAVE;
						rw_next = 1;
						enable_next = 1;
					end
				end
				SAMPLER_SAMPLING_SAVE: begin
					if ( ready ) begin //DRAM ready
						rw_next = 0;
						enable_next = 0;
						if (addr ==  ADDR_HIGH_LOG) begin
							//memory full. stop
							sampling_ctl_next = SAMPLER_SAMPLING_DONE;
						end 
						else begin
							addr_next = addr + 23'd1;
							sampling_ctl_next = SAMPLER_SAMPLING_SYNC;
							amost2_start_next = 1;
						end
					end
				end
			
				SAMPLER_SAMPLING_DONE: begin
					led2_mode_next = 2; led2_fast_next = 0;
					sampling_logic_ctl_next = SAMPLER_LOGIC_DONE; //finaliza tambem o analizador logico
					state_main_next = MAIN_DUMPING;
					serial_dump_ctl_next = SERIAL_DUMP_IDLE;

				end
			endcase //case (sampling_ctl)
			
			case (sampling_logic_ctl)   // logic analizer
				SAMPLER_LOGIC_IDLE: begin
				end
				SAMPLER_LOGIC_RUNNING: begin
					logic_event_ack_next = 0;
					if (logic_event_saved) begin
						if ( ready ) begin //DRAM ready
							rw_next = 1;
							enable_next = 1;
							data_in_next = logic_pack;
							sampling_logic_ctl_next = SAMPLER_LOGIC_SAVE;
						end
					end
				end
				
				SAMPLER_LOGIC_SAVE: begin
					if ( ready ) begin //DRAM ready
						rw_next = 0;
						enable_next = 0;
						logic_event_ack_next = 1;

						if (addr ==  ADDR_HIGH_LOG) begin
							//memory full. stop
							sampling_ctl_next = SAMPLER_SAMPLING_DONE; //finaliza tambem o sampler meter
							sampling_logic_ctl_next = SAMPLER_LOGIC_DONE;
						end 
						else begin
							addr_next = addr + 23'd1;
							sampling_logic_ctl_next = SAMPLER_LOGIC_RUNNING;
						end
					end
				end
					
				SAMPLER_LOGIC_DONE: begin
				end
						
			endcase //case sampling_logic_ctl
			
		end // MAIN_SAMPLING
	

		MAIN_DUMPING: begin            /****  D U M P   S E R I A L   *****/
			case (serial_dump_ctl)
				SERIAL_DUMP_IDLE: begin
					led2_mode_next = 1; led2_fast_next = 1;
					if (sw2_state)  serial_dump_ctl_next = SERIAL_DUMP_SETUP;
				end
				SERIAL_DUMP_SETUP: begin
					led2_mode_next = 3; led2_fast_next = 0;

					addr_next = ADDR_LOW_LOG;
					rw_next = 0;
					enable_next = 1;
					serial_dump_ctl_next = SERIAL_DUMP_RUNNING;
				end
				
				SERIAL_DUMP_RUNNING: begin
					if ( out_valid ) begin	
						if (data_out[0]!=1) begin
								serial_dump_ctl_next = SERIAL_DUMP_DONE; //valid register
								tx_en_next = 0;						
						end
						else begin
							if (tx_ready) begin
								if (data_out[1]==0) begin
									tx_byte_next = 8'h0A;
									dump_type_next = 0; //data sample pack
								end
								else begin
									tx_byte_next = 8'h0C;
									dump_type_next = 2; //data logic pack
								end
									
								tx_en_next = 1;
								serial_dump_ctl_next = SERIAL_DUMP_RUNNING1;
							end
						end
					end
				end				
	
				SERIAL_DUMP_RUNNING1: begin
					tx_en_next = 0;
					if (tx_ready && out_valid) begin
						case (dump_type)
							0: tx_byte_next = {4'b0000, data_out[31:28]};
							2: tx_byte_next = data_out[31:24];
						endcase
						tx_en_next = 1;
						serial_dump_ctl_next = SERIAL_DUMP_RUNNING2;
					end
				end
				SERIAL_DUMP_RUNNING2: begin
					tx_en_next = 0;
					if (tx_ready && out_valid) begin
						case (dump_type)
							0: tx_byte_next = data_out[27:20];
							2: tx_byte_next = data_out[23:16];
						endcase						
						tx_en_next = 1;
						serial_dump_ctl_next = SERIAL_DUMP_RUNNING3;
					end
				end					
				SERIAL_DUMP_RUNNING3: begin
					tx_en_next = 0;
					if (tx_ready && out_valid) begin
						case (dump_type)
							0:	tx_byte_next = {4'b0000, data_out[19:16]};
							2: tx_byte_next = data_out[15:8];
						endcase
						tx_en_next = 1;
						serial_dump_ctl_next = SERIAL_DUMP_RUNNING4;
					end
				end	
				SERIAL_DUMP_RUNNING4: begin
					tx_en_next = 0;
					if (tx_ready && out_valid) begin
						case (dump_type)
							0:	tx_byte_next = data_out[15:8];
							2: tx_byte_next = 8'd0;
						endcase					
						tx_en_next = 1;
						serial_dump_ctl_next = SERIAL_DUMP_RUNNING5;
					end
				end
				SERIAL_DUMP_RUNNING5: begin
					tx_en_next = 0;
					if (tx_ready && out_valid) begin
						case (dump_type)
							0:	tx_byte_next = {2'b0, data_out[7:2]};
							2: tx_byte_next = {2'b0, data_out[7:2]};
						endcase
						tx_en_next = 1;
						serial_dump_ctl_next = SERIAL_DUMP_TX;
					end
				end

				
				SERIAL_DUMP_TX: begin
					tx_en_next = 0;
					if (addr == ADDR_HIGH_LOG) begin
						serial_dump_ctl_next = SERIAL_DUMP_DONE;
						addr_next = 0;
					end
					else begin
						addr_next = addr + 23'd1;
						serial_dump_ctl_next = SERIAL_DUMP_RUNNING;

					end

				end
				SERIAL_DUMP_DONE: begin
						led2_mode_next = 5; led2_fast_next = 0;
				end
			endcase //case (serial_dump_ctl)
		end//MAIN_DUMPING
		
		
										
		MAIN_LOAD_CURVE: begin      /*  RECEIVE IV CURVE ON MEMORY */
			case (load_curve_ctl)
				LOAD_CURVE_IDLE: begin
					load_curve_reg_count_next = 0;
					addr_next = ADDR_LOW_CURVE; //dram start addr
					load_curve_ctl_next = LOAD_CURVE_WAIT_SOH;
				end
				
				LOAD_CURVE_WAIT_SOH: begin
					led2_mode_next = 1; led2_fast_next = 1;
					if (rx_dv && rx_byte==8'h0f) begin
						load_curve_ctl_next = LOAD_CURVE_WAIT_RX;
						load_curve_byte_count_next = 0;
					end
				end
				LOAD_CURVE_WAIT_RX: begin
					if (rx_dv) begin
						load_byte[ load_curve_byte_count ] = rx_byte;
						load_curve_ctl_next = LOAD_CURVE_CHECK;
					end
				end
				LOAD_CURVE_CHECK: begin
					if (load_curve_byte_count == 4) begin
						//received 5 bytes
						if ( load_byte[4] == load_checksum ) begin
							load_curve_ctl_next = LOAD_CURVE_RECORD;
						end
						else begin
							load_curve_ctl_next = LOAD_CURVE_ERROR;
						end
					end
					else begin
						load_curve_byte_count_next = load_curve_byte_count+1;
						load_curve_ctl_next = LOAD_CURVE_WAIT_RX;
					end
				end
				LOAD_CURVE_RECORD: begin
					if ( ready ) begin //DRAM ready
						rw_next = 1;
						enable_next = 1;
						data_in_next = { load_byte[4], load_byte[3], load_byte[2], load_byte[1]}; //few bits lost here (24 bit real data)
						load_curve_reg_count_next = load_curve_reg_count + 1;
						load_curve_ctl_next = LOAD_CURVE_RECORD2;
					end
				end
				LOAD_CURVE_RECORD2: begin
					if ( ready ) begin //DRAM ready
						enable_next = 0;
						rw_next = 0;

						if ( load_curve_reg_count==4094 || addr==ADDR_HIGH_CURVE ) begin
							load_curve_ctl_next = LOAD_CURVE_FINISHED;
						end
						else begin
							load_curve_ctl_next = LOAD_CURVE_WAIT_SOH;
							addr_next = addr + 1;
						end
					end
				end
				
				LOAD_CURVE_FINISHED: begin
					led2_mode_next = 3; led2_fast_next = 1;
					replay_curve_ctl_next = REPLAY_CURVE_IDLE;
					state_main_next = MAIN_REPLAY;
					

				end
				LOAD_CURVE_ERROR: begin
					led2_mode_next = 15; led2_fast_next = 1;
				end
				
			endcase		
		end //MAIN_LOAD_CURVE
		
		MAIN_REPLAY: begin
			case (replay_curve_ctl)
				REPLAY_CURVE_IDLE: begin
						dac_value_next = 0;
						dac_enable_next = 1;
							
					if (sw2_state) begin
						replay_curve_ctl_next = REPLAY_CURVE_RUN;
						led2_mode_next = 2; led2_fast_next = 1;
						enable_next = 1;
						rw_next = 0;
						addr_next = ADDR_LOW_CURVE;
						amost2_start_next = 1;
					end
				end
				
				REPLAY_CURVE_RUN: begin
				//	if (amost2_busy==0) begin
						if ( ready ) begin //DRAM ready
							addr_next = ADDR_LOW_CURVE + amost2_data_i;
							rw_next = 0;
							enable_next = 1;
							replay_curve_ctl_next = REPLAY_LOOKUP;
						end
					end
				//end
				
				REPLAY_LOOKUP: begin
					//amost2_start_next = 0;
					if ( out_valid ) begin //DRAM ready
							dac_value_next = data_out[11:0];
							replay_curve_ctl_next = REPLAY_UPDATE;
					end
				end

				REPLAY_UPDATE: begin
					//dac_enable_next = 0;
					enable_next = 0;
					replay_curve_ctl_next = REPLAY_CURVE_RUN;
				end				
				
				
			endcase
		
		end //MAIN_REPLAY
		
		
		
		
		
	endcase //case (state_main)
	
end



always @(*) begin	
	logic_event_saved_next = logic_event_saved;
	logic_pack_next = logic_pack;
	
	if (logic_event && !logic_event_ack && !logic_event_saved ) begin
			//novo evento ocorreu e nao existe evento pendente
			logic_event_saved_next  = 1;
			logic_pack_next = {logic_data, 8'b0, logic_checksum, 2'b11};
	end
	if (logic_event_ack) logic_event_saved_next = 0;
	
end


	assign debug7 = debug7q;
	assign debug11 = debug11q;
	
endmodule
