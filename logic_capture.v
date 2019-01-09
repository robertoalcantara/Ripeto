`timescale 1ns / 1ps

module logic_capture(
    input clk,
	 input rst,
	 output reg [15:0] data,
	 output reg event_detected,	 
	 input[15:0] f_pin
);

reg [15:0] data_next;
reg event_detected_next;


always @(posedge clk or negedge clk or posedge rst) begin

	if ( rst ) begin
		data <= 0;
		event_detected <= 0;
	end
	else begin
		data <= data_next;
		event_detected <= event_detected_next;
	end
end



always @(*) begin
	
	event_detected_next = event_detected;
	data_next = data;
	
	if ( data != f_pin ) begin
			data_next = f_pin;
			event_detected_next = 1;
	end
	
	if (event_detected == 1) event_detected_next = 0;
	

end

	
endmodule
