`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    17:28:52 03/23/2019 
// Design Name: 
// Module Name:    sdram_arbiter 
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
module sdram_arbiter(
    input clk,
    input rst,
    input req1,
    output ack1,
    input req2,
    output ack2
    );

	 
	 
	parameter IDLE = 0;
	parameter S1 = 1;	
	parameter S2 = 2;
	
	reg[7:0] state, state_next; 
	 
	assign ack1 = (state==S1);
	assign ack2 = (state==S2);

always @(posedge clk or posedge rst) begin

	if ( rst ) begin	
		state <= IDLE;

	end 
	else begin
		state <= state_next;
	end
end


always @(*) begin
	state_next = state;
	
	case (state)
		IDLE: begin
			if (req1) state_next = S1;
			else 
				if (req2) state_next = S2;
				else	state_next = IDLE;
		end
		
		S1: begin
			if (req1) state_next = S1;
			else state_next = IDLE;				
		end
		
		S2: begin
			if (req2) state_next = S2;
			else state_next = IDLE;
		end
		
	endcase
	
	

end



endmodule





