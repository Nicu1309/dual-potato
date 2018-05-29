`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 15.10.2017 17:43:07
// Design Name: 
// Module Name: Register_Pipe_Mem
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


module Register_Res_Mem_Data(
        clk,
        rst,
        is_data_in,
        ready_mem_in,
        addr_mem_in,
        block_in,
        ready_mem_out,
        addr_mem_out,
        block_out,
        is_data_out
    );
    
    parameter BITS_ADDR = 32;
    parameter BITS_DATA = 32;
    parameter BITS_ADDRESS_MEM = 27;
    parameter BITS_BLOCK = 128;
    
    input [BITS_ADDRESS_MEM-1:0] addr_mem_in;
    input [BITS_BLOCK-1:0] block_in;
    input clk, rst, ready_mem_in, is_data_in;
    
    output reg [BITS_ADDRESS_MEM-1:0] addr_mem_out;
    output reg [BITS_BLOCK-1:0] block_out;
    output reg ready_mem_out, is_data_out;    
    
    
    always @(posedge clk) begin
        if (~rst) begin
            addr_mem_out <= addr_mem_in;
            block_out <= block_in;
            ready_mem_out <= ready_mem_in;
            is_data_out <= is_data_in;
        end else begin
            addr_mem_out <= 0;
            block_out <= 0;
            ready_mem_out <= 0;
            is_data_out <= 0;
        end
    end

endmodule

