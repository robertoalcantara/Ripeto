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

    //user interface
        reg [22:0] addr;      // address to read/write
        reg rw;               // 1 = write, 0 = read
        reg [31:0] data_in;   // data from a read
        wire [31:0] data_out; // data for a write
        //wire busy;            // controller is busy when high
		wire ready;
        reg in_valid;         // pulse high to initiate a read/write
        wire out_valid;        // pulses high when data from read is valid
	
	
	reg [31:0] data_read;
	
	//aux
	reg [63:0] clk_slow_cnt;
	reg clk_slow;
	reg enable;
	
	parameter ONE_SEC_DLY = 64'd50000000;
	parameter HALF_SEC_DLY = 64'd12000000; //25
	assign rst_p = ~rst;

/////////////////////
    wire clk100;	
	
	clk_wiz_v3_6 clkPLL (
        .CLK_IN1(clk), // IN
        .CLK_OUT1(clk100) // OUT
    );    
    // INST_TAG_END ------ End INSTANTIATION Template ---------

///////////////////////	
/*   sdram DUT (
        .clk(clk100),
        .rst(rst_p),
        .sdram_clk(sdram_clk),
        .sdram_cle(sdram_cle),
        .sdram_cs(sdram_cs),
        .sdram_cas(sdram_cas),
        .sdram_ras(sdram_ras),
        .sdram_we(sdram_we),
        .sdram_dqm(sdram_dqm),
        .sdram_ba(sdram_ba),
        .sdram_a(sdram_a),
        .sdram_dq(sdram_dq),
        .addr(addr),
        .rw(rw),
        .data_in(data_in),
        .data_out(data_out),
        .busy(busy),
        .in_valid(in_valid),
        .out_valid(out_valid)
    );*/
////////////////////

SDRAM_Controller_v DUT (
   .clk(clk100),   .reset(rst_p),
   // command and write port
   .cmd_ready(ready), .cmd_enable(enable), .cmd_wr(rw), .cmd_byte_enable(4'b1111), .cmd_address(addr), .cmd_data_in(data_in),
   // Read data port
   .data_out(data_out), .data_out_ready(out_valid),
   // SDRAM signals
   .SDRAM_CLK(sdram_clk),  .SDRAM_CKE(sdram_cle),  .SDRAM_CS(sdram_cs),  .SDRAM_RAS(sdram_ras),  .SDRAM_CAS(sdram_cas),
   .SDRAM_WE(sdram_we), .SDRAM_DQM(sdram_dqm), .SDRAM_ADDR(sdram_a), .SDRAM_BA(sdram_ba), .SDRAM_DATA(sdram_dq)
);



///////////////
   reg i_Tx_DV;
   reg [7:0] i_Tx_Byte;
   wire      o_Tx_Active;
   wire      o_Tx_Done;
   
   
uart_tx tx_serial (
   .i_Clock (clk100),
   .i_Tx_DV (i_Tx_DV),
   .i_Tx_Byte(i_Tx_Byte),
   .o_Tx_Active(o_TX_Active),
   .o_Tx_Serial(uart_tx_pin),
   .o_Tx_Done(o_Tx_Done)
  );


//////////////
	reg led1_debug, led2_debug;
	reg debug7q, debug11q;
	reg [7:0] state_cnt;;
	
reg [1:0]tx_state;	
reg tx_go;
	
always @(posedge clk100) 
	begin
	
		if ( !rst ) begin
			led1_debug <= 1;
			clk_slow_cnt <= 64'd0;
			clk_slow <= 0;
			tx_state <= 2'd0;
		end
		else begin
			clk_slow_cnt <= clk_slow_cnt + 64'd1;
			if  (clk_slow_cnt == HALF_SEC_DLY)  begin 
				led1_debug <= ~led1_debug;
				clk_slow <= ~clk_slow;
				clk_slow_cnt <= 64'd0;
			end 
			
			case (tx_state)
			0:  begin
				if (tx_go ==1)
					tx_state <= 2'd1;
			end
			1: begin
				  i_Tx_DV <= 1;
  				tx_state <= 2'd2;
			end
			
			2: begin
				i_Tx_DV <= 0;
	
				if ( tx_go == 0)
					tx_state <= 2'd0;
			end
			
			3: begin
			if (tx_go==0)
					tx_state <= 2'd0;
			end
		endcase
			
		end
end
	
	
 always @(posedge clk_slow or negedge rst) begin

	 if ( !rst ) begin
		state_cnt <= 8'd0;
		debug7q <= 1;
		debug11q <= 1;	
		addr <= 23'd0;
		data_in <= 32'd0;
		in_valid <= 0;
		tx_go <= 0;
		enable <= 0;
     end 
	 else begin
		case (state_cnt)
			0: begin		
				tx_go <= 0;
				addr <= 23'd0;
				rw <= 0;
				data_in <= 32'd0;
				if ( ready ) begin  
					state_cnt <= 8'd1;
				end
  
			end
			
			1: begin
				addr <= 23'd100;
				data_in <= 32'd666;
				rw <= 1;
				enable <= 1;
				state_cnt <= 8'd2;
			end
			
			2: begin
				  if ( ready ) begin
					state_cnt <= 8'd3;
					in_valid <= 0;
					rw <= 0;
					enable <= 0;
				  end
				end
				
			3: begin
					addr <= 23'd100;
					rw <= 0;
					enable <= 1;
					if ( ready ) begin
						state_cnt <= 8'd4;
					end
			   end
		   
		   4: begin
				enable <= 0;
			    i_Tx_Byte <= 8'd1;
			    tx_go <= 1;
			    state_cnt <= state_cnt + 1;
			end
			
		   5: begin
			    tx_go <= 0;
			    state_cnt <= state_cnt + 1;
			end
			
			6: begin			
				i_Tx_Byte <= data_out[7:0];
				tx_go <= 1;
				state_cnt <= 0;
			end
			
		  default: state_cnt <= 0;
	  endcase
	 end
    end
	
	
	always @(posedge out_valid or negedge rst) begin
		if (!rst) begin
			led2_debug <= 1; //desliga
			data_read <= 0;
		
		end
		else begin
			data_read <= data_out;
			if (data_read == 32'd666) begin
			end
			led2_debug <= 0; //liga

		end
		
	end
	
	assign led1 = led1_debug;
	assign led2 = led2_debug;	
	//assign debug7 = debug7q;
	//assign debug11 = debug11q;
	assign debug7 = state_cnt[0];
	assign debug11 = state_cnt[1];
	

	
endmodule
