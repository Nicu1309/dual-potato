`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: UPC
// Engineer: David Nicuesa Aranda
// Engineer: Ana Lasheras Mas
// 
// Create Date: 27.09.2017 15:49:58
// Design Name: 
// Module Name: regBank
// Project Name: PariscV
// Target Devices: 
// Tool Versions: 
// Description: 
//      Configurable by instantiation registers bank
//
// Parameters:
//      BITS_REG_SELECT  Number of bits for selecting register
//      BITS_REG_DATA    Registers size in bits
//
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Revision 1.0 - Integer regbank tested
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module Register_File(
    clk,
    reset,
    rs,
    rt,
    dout0,
    dout1,
    din,
    reg_in,
    we
    );
    
    parameter BITS_REG_SELECT = 2; // Bits for number of register
    parameter BITS_REG_DATA = 32; // Size of registers
    
    localparam FILE_SIZE = 2 ** BITS_REG_SELECT;
    
    input clk, we, reset;
    input [BITS_REG_SELECT-1:0] rs, rt, reg_in;
    input [BITS_REG_DATA-1:0] din;
    output [BITS_REG_DATA-1:0] dout0, dout1;

    wire [BITS_REG_DATA-1:0] outs [FILE_SIZE-1:0]; 
    wire [FILE_SIZE-1:0] enables; 
    
    /* Select register to write in */
    Decoder #(.BITS_IN(BITS_REG_SELECT),.BITS_OUT(FILE_SIZE)) reg_write_select(
            .decoder_in(reg_in),
            .decoder_out(enables),
            .enable(1'b1),
            .reset(reset));
            

    // Write behavior on clock's rising edge
    // as RiscV ISA, reg0 is not writtable 
    generate
        genvar r;
        
        // Generate all registers except for the register zero
        for(r=1;r<FILE_SIZE; r=r+1)begin: registers_no_zero
            Register #(.BITS_SIZE(BITS_REG_DATA)) register_n(
                .clk(clk),
                .enable(enables[r] & we),
                .reset(reset),
                .din(din),
                .dout(outs[r])
            );
        end
    endgenerate
    
    assign outs[0] = 0;
    
    // Data out
    assign dout0 = outs[rs];
    assign dout1 = outs[rt];
    
endmodule
