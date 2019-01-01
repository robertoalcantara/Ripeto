`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    04:39:32 01/01/2019 
// Design Name: 
// Module Name:    led_flash 

// ** led is on on low level
//////////////////////////////////////////////////////////////////////////////////
module led_flash(
    input clk,
    input rst,
    input [3:0] mode,
	 input mode_fast,
	 output busy,
    output led_pin
    );
	 
   parameter CLK = 100000000;
	parameter LED_TICK = CLK>>2 ;

	reg[3:0] mode_saved, mode_next;
	reg led, led_next;
	reg fast, fast_next;
	
	reg[40:0] cnt, cnt_next;


	reg[3:0] state, state_next;
	parameter OFF=0, SETUP=1, WAIT=2, WAIT2=6, ACT=3, DONE=4, IDLE=5;

always @(posedge clk or posedge rst) begin

	if (rst) begin
		mode_saved <= 0;
		led <= 1; //apagado
		cnt <= 0;
		state <= 0;
		fast <= 0;
	end
	else begin
		led <= led_next;
		mode_saved <= mode_next;
		cnt <= cnt_next;
		state <= state_next;
		fast <= fast_next;
	end
end


always @(*) begin

	led_next = led;
	mode_next = mode;
	state_next = state;
	cnt_next = cnt;
	mode_next = mode_saved;
	fast_next = fast;
 
	case (state) 
			IDLE: begin
				led_next = 1; //apagado
				if (mode != 0) begin
					 mode_next = mode;
					 fast_next = mode_fast;
					 state_next = SETUP;	

				end
			end
			
			SETUP: begin
				led_next = 0; //acende
				cnt_next = LED_TICK >> fast;
				state_next = WAIT;
			end
			
			WAIT: begin
				if (cnt > 0)
				  cnt_next = cnt - 40'd1;
				else begin
				 led_next = 1; //apaga
			     state_next = ACT;
				  cnt_next = LED_TICK >> fast;
				  state_next = WAIT2;					
				end
			end
			
			WAIT2: begin
				if (cnt > 0)
					cnt_next = cnt - 40'd1;
				else
					state_next = ACT;
			end			
			
			ACT: begin
	
				if (mode_saved > 1) begin
					mode_next = mode_saved - 4'd1;
					state_next = SETUP;
				end 
				else begin 
				    if (fast)
						cnt_next = LED_TICK<<1;
					 else
					   cnt_next = LED_TICK<<2;
					state_next = OFF;
				end
			end
			
			OFF: begin
				led_next = 1; //apaga

				if (cnt > 0)
					cnt_next = cnt - 32'd1;
				else
					state_next = IDLE;				
			end
		
	endcase


end


assign led_pin = led;
assign busy = (state!=IDLE);

endmodule
