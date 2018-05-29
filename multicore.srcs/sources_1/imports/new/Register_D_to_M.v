`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.11.2017 22:32:08
// Design Name: 
// Module Name: Register_D_to_M
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


module Register_D_to_M(
        clk,
        enable,
        reset,
        effective_address_in,
        is_byte_in,
        memread_en_in,
        memwrite_en_in,
        data_store_in,
        is_exception_in,
        exception_type_in,
        rob_entry_in,
        reg_rw_in,
        effective_address_out,
        is_byte_out,
        memread_en_out,
        memwrite_en_out,
        data_store_out,
        is_exception_out,
        exception_type_out,
        rob_entry_out,
        reg_rw_out,
        invalidate_in,
        invalidate_out
    );
    
    parameter BITS_ADDRESS = 32;
    parameter BITS_EXCEPTION = 5;
    parameter BITS_ROB_ENTRY_SELECT = 4;
    parameter BITS_REG_SELECT = 5;
    
    input clk, enable, reset, is_byte_in, memread_en_in, memwrite_en_in, is_exception_in, invalidate_in;
    input[BITS_ADDRESS-1:0] effective_address_in, data_store_in;
    input[BITS_EXCEPTION-1:0] exception_type_in;
    input[BITS_ROB_ENTRY_SELECT-1:0] rob_entry_in;
    input[BITS_REG_SELECT-1:0] reg_rw_in;
    
    output is_byte_out, memread_en_out, memwrite_en_out, is_exception_out, invalidate_out;
    output[BITS_ADDRESS-1:0] effective_address_out, data_store_out;
    output[BITS_EXCEPTION-1:0] exception_type_out;
    output[BITS_ROB_ENTRY_SELECT-1:0] rob_entry_out;
    input[BITS_REG_SELECT-1:0] reg_rw_out;
    
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
        .din({is_exception_in,memwrite_en_in,memread_en_in,is_byte_in}),
        .dout(bit_signals)
    );   
    assign is_byte_out = bit_signals[0];
    assign memread_en_out = bit_signals[1];
    assign memwrite_en_out = bit_signals[2];
    assign is_exception_out = bit_signals[3];
    
    Register #(.BITS_SIZE(BITS_ADDRESS)) data_store(
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(data_store_in),
        .dout(data_store_out)
    );
    
    Register #(.BITS_SIZE(BITS_EXCEPTION)) exception_type(
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(exception_type_in),
        .dout(exception_type_out)
    );

    Register #(.BITS_SIZE(BITS_ROB_ENTRY_SELECT)) rob_entry(
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(rob_entry_in),
        .dout(rob_entry_out)
    );    
    
    Register #(.BITS_SIZE(BITS_REG_SELECT)) reg_rw (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(reg_rw_in),
        .dout(reg_rw_out)
    );
    
    Register #(.BITS_SIZE(1)) invalidate_bit(
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(invalidate_in),
        .dout(invalidate_out)
    );
    
endmodule
