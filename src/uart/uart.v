`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////: 
// from nandland.com
//////////////////////////////////////////////////////////////////////////////////
module uart(
    input clk,
    input rst,
    input [7:0] tx_byte,
    input tx_en,
    output tx_ready,
    output tx_pin
    );
	reg tx_ready_r;
	assign tx_ready = tx_ready_r;
	reg i_Tx_DV;
	reg [7:0] i_Tx_Byte;
	wire o_Tx_Active;
	wire o_Tx_Done;

	uart_tx UART_TX (
		.i_Clock (clk),
		.i_Tx_DV (i_Tx_DV),
		.i_Tx_Byte(i_Tx_Byte),
		.o_Tx_Active(o_Tx_Active),
		.o_Tx_Serial(tx_pin),
		.o_Tx_Done(o_Tx_Done)
	);  
	
	reg f_we;
	reg f_re;
	reg [7:0] f_data_in;
	wire [7:0] f_data_out;
	wire f_full;
	wire f_empty;
	
	fifo FIFO (
		.dout(f_data_out),
		.full(f_full),
		.empty(f_empty),
		.clock( clk ),
		.reset( rst ),
		.wr( f_we ),
		.rd( f_re ),
		.din( f_data_in )
	);  
	  
	  
	  
reg[7:0] main_tx_state;
reg[7:0] saved_byte;
always @(posedge clk or posedge rst) begin
	if ( rst ) begin	
		tx_ready_r <= 1;
		main_tx_state <= 0;
		saved_byte <= 0;
	end
	else begin
		case (main_tx_state) 
			0: begin
				if ( tx_en ) begin
					main_tx_state <= 1;
					saved_byte <= tx_byte;
				end
			end
			1: begin
				tx_ready_r <= 0;
				if ( f_full == 0 ) begin 
					f_data_in <= saved_byte;
					f_we <= 1; //salvar na fifo
					main_tx_state <= 2;
				end else begin
					//wait
				end
			end
			2: begin
				//wait to deassign tx_en
				f_we <= 0;
				if (tx_en == 0) begin
					main_tx_state <= 3; 
				end
			end
			3: begin
				tx_ready_r <= 1;
				main_tx_state <= 0;
			end
			
			default: begin
			end
		endcase
	end
end
	
		
reg[7:0] serial_tx_state;
always @(posedge clk or posedge rst) begin

	if ( rst ) begin
		f_re <= 0;
		serial_tx_state <= 0;
		i_Tx_DV <= 0;
		i_Tx_Byte <= 0;	
	end
	else begin
		case (serial_tx_state)
			0: begin
				if ( !f_empty )	begin
					serial_tx_state <= 1; //have data to send
					f_re <= 1;
				end
			end
			1: begin
				i_Tx_Byte <= f_data_out;
				f_re <= 0;
				i_Tx_DV <= 1;
				serial_tx_state <= 2;
			end
			2: begin
				i_Tx_DV <= 0;
				if (o_Tx_Done)
					serial_tx_state <= 0; //wait tx done
			end
			
			default: begin
			
			end
			
			
		endcase
	end

end

endmodule
