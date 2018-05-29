`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 29.05.2018 17:04:34
// Design Name: 
// Module Name: Register_Mem_to_Writeback
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


module Register_Mem_to_Writeback(
        clk,
        enable,
        reset,
        rd_in,
        data_in,
        en_wb_in,
        rd_out,
        data_out,
        en_wb_out
    );
    
    parameter BITS_REG_SELECT = 5;
    parameter BITS_DATA = 32;
    
    input en_wb_in, clk, enable, reset;
    input [BITS_REG_SELECT-1:0] rd_in;
    input [BITS_DATA-1:0] data_in;
    output [BITS_DATA-1:0] data_out;
    output [BITS_REG_SELECT-1:0] rd_out;
    output en_wb_out;
    
     Register #(.BITS_SIZE(BITS_DATA)) data (
           .clk(clk),
           .reset(reset),
           .enable(enable),
           .din(data_in),
           .dout(data_out)
       );

     Register #(.BITS_SIZE(BITS_REG_SELECT)) rd (
           .clk(clk),
           .reset(reset),
           .enable(enable),
           .din(rd_in),
           .dout(rd_out)
       );

     Register #(.BITS_SIZE(1)) reg_enable (
           .clk(clk),
           .reset(reset),
           .enable(enable),
           .din(en_wb_in),
           .dout(en_wb_out)
       );
    
endmodule
