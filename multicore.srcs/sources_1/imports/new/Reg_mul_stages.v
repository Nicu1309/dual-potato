`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02.12.2017 18:40:18
// Design Name: 
// Module Name: Reg_mul_stages
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

module Reg_mul_stages(
    enable,
    reset,
    clk,
    rd_in,
    rd_out,
    din,
    dout,
    zero_in,
    zero_out,
    finished_in,
    finished_out,
    rob_entry_in,
    rob_entry_out,
    invalidate_in,
    invalidate_out
);

    parameter BITS_DATA = 32;
    parameter BITS_ROB_ENTRY_SELECT = 4;
    parameter BITS_REG_SELECT = 5;
    
    input enable, reset, clk, invalidate_in;
    input signed [BITS_DATA-1:0] din;
    input zero_in, finished_in;
    input[BITS_ROB_ENTRY_SELECT-1:0] rob_entry_in;
    input [BITS_REG_SELECT-1:0] rd_in;
    
    output signed [BITS_DATA-1:0] dout;
    output zero_out, finished_out, invalidate_out;
    output[BITS_ROB_ENTRY_SELECT-1:0]rob_entry_out;
    output [BITS_REG_SELECT-1:0] rd_out;
    
    Register #(.BITS_SIZE(BITS_DATA)) data(
        .enable(enable),
        .reset(reset),
        .clk(clk),
        .din(din),
        .dout(dout)
    );
    
    Register #(.BITS_SIZE(2)) bit_signals(
        .enable(enable),
        .reset(reset),
        .clk(clk),
        .din({zero_in,finished_in}),
        .dout({zero_out,finished_out})
    ); 
    
    Register #(.BITS_SIZE(1)) invalidate_bit(
        .enable(enable),
        .reset(reset),
        .clk(clk),
        .din(invalidate_in),
        .dout(invalidate_out)
    ); 

    Register #(.BITS_SIZE(BITS_ROB_ENTRY_SELECT)) rob_entry(
        .enable(enable),
        .reset(reset),
        .clk(clk),
        .din(rob_entry_in),
        .dout(rob_entry_out)
    ); 
    
    Register #(.BITS_SIZE(BITS_REG_SELECT)) rd(
        .enable(enable),
        .reset(reset),
        .clk(clk),
        .din(rd_in),
        .dout(rd_out)
    ); 
    
endmodule
