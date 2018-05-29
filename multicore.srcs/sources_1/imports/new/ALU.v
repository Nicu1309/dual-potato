`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: UPC 
// Engineer: David Nicuesa Aranda
// Engineer: Ana Lasheras Mas
// 
// Create Date:    17:53:13 10/22/2017 
// Design Name: 
// Module Name:    ALU 
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
module ALU(
    enable,
	din0,
	din1,
	opcode,
	dout,
	zero,
	uflow,
	oflow,
	greater_than,
	less_than,
	equals
);
	
	parameter BITS_DATA = 32;
	parameter BITS_OPCODE = 5;
	
	localparam OPCODE_ADD = 0;
	localparam OPCODE_SUB = 5'b01000;
	localparam  OPCODE_OR = 5'b00110;
	localparam OPCODE_AND = 5'b00111;
	localparam OPCODE_LSL = 5'b00001;
	localparam OPCODE_LSR = 5'b00101;
	localparam OPCODE_MUL = 5'b11111;
	
	input enable;
	input signed [BITS_DATA-1:0] din0, din1;
	input signed [BITS_OPCODE-1:0] opcode;
	output reg signed [BITS_DATA-1:0] dout;
	output zero, uflow, oflow, greater_than, less_than, equals; //wire
	
	wire oflow_add, oflow_sub, uflow_add, uflow_sub;
	
	always@(din0 or din1 or opcode or enable)
	begin
	   if(enable)
		case(opcode)
			OPCODE_ADD:
				dout = din0 + din1; //ADD = 0x0
			OPCODE_SUB:
				dout = din1 - din0; //SUB = 0x1
			OPCODE_OR:
				dout = din1 | din0; //OR = 0x3
			OPCODE_AND:
				dout = din1 & din0; //AND = 0x4
			OPCODE_LSL:
				dout = din0 << din1 ; //LSL = 0x5
			OPCODE_LSR:
                dout = din0 >> din1 ; //LSR = 0x6
		    OPCODE_MUL:
                dout = din0 * din1 ; // MUL
		endcase
	end
	
	// pos + pos = neg
	assign oflow_add = (opcode == 0) & ~din0[BITS_DATA-1] & ~din1[BITS_DATA-1] & dout[BITS_DATA-1];
	// pos - neg = neg
	assign oflow_sub = (opcode == 1) & ~din0[BITS_DATA-1] & din1[BITS_DATA-1] & dout[BITS_DATA-1];
	// neg + neg = pos
	assign uflow_add = (opcode == 0) & din0[BITS_DATA-1] & din1[BITS_DATA-1] & ~dout[BITS_DATA-1];
	// neg - pos = pos
	assign uflow_sub = (opcode == 1) & din0[BITS_DATA-1] & ~din1[BITS_DATA-1] & ~dout[BITS_DATA-1];
	
	assign zero = enable?(dout == 0):1'b0; 
	assign oflow = oflow_add | oflow_sub;
	assign uflow = uflow_add | uflow_sub;
	assign greater_than = din0 > din1;
	assign less_than = din0 < din1;
	assign equals = (din0 == din1);
	
endmodule
