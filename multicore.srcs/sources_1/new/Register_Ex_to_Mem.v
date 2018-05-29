`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 21.05.2018 18:23:50
// Design Name: 
// Module Name: Register_Ex_to_Mem
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module Register_Ex_to_Mem(
        clk,
        enable,
        reset,
        effective_address_in,
        is_byte_in,
        memread_en_in,
        memwrite_en_in,
        data_in,
        rd_in,
        regwrite_en_in,
        effective_address_out,
        is_byte_out,
        memread_en_out,
        memwrite_en_out,
        data_out,
        rd_out,
        regwrite_en_out
    );
    
    parameter BITS_ADDRESS = 32;
    parameter BITS_REG_SELECT = 5;
    
    input clk, enable, reset, is_byte_in, memread_en_in, memwrite_en_in, regwrite_en_in;
    input[BITS_ADDRESS-1:0] effective_address_in, data_in;
    input[BITS_REG_SELECT-1:0] rd_in;
    
    output is_byte_out, memread_en_out, memwrite_en_out, regwrite_en_out;
    output[BITS_ADDRESS-1:0] effective_address_out, data_out;
    input[BITS_REG_SELECT-1:0] rd_out;
    
    wire[3:0] bit_signals;
    
    Register #(.BITS_SIZE(BITS_ADDRESS)) effective_address(
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(effective_address_in),
        .dout(effective_address_out)
    );

     Register #(.BITS_SIZE(4)) one_bit_signals(
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din({regwrite_en_in,memwrite_en_in,memread_en_in,is_byte_in}),
        .dout(bit_signals)
    );   
    assign is_byte_out = bit_signals[0];
    assign memread_en_out = bit_signals[1];
    assign memwrite_en_out = bit_signals[2];
    assign regwrite_en_out = bit_signals[3];
    
    Register #(.BITS_SIZE(BITS_ADDRESS)) data_store(
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(data_in),
        .dout(data_out)
    );
    
    Register #(.BITS_SIZE(BITS_REG_SELECT)) reg_rd (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(rd_in),
        .dout(rd_out)
    );
    
endmodule
