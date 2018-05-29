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


module Register_Req_Mem_Data(
        clk,
        rst,
        is_data_in,
        read_mem_in,
        write_mem_in,
        addr_req_mem_in,
        addr_write_mem_in,
        block_in,
        read_mem_out,
        write_mem_out,
        addr_req_mem_out,
        addr_write_mem_out,
        block_out,
        is_data_out
    );
    
    parameter BITS_ADDR = 32;
    parameter BITS_DATA = 32;
    parameter BITS_ADDRESS_MEM = 27;
    parameter BITS_BLOCK = 128;
    
    input [BITS_ADDRESS_MEM-1:0] addr_req_mem_in, addr_write_mem_in;
    input [BITS_BLOCK-1:0] block_in;
    input clk, rst, read_mem_in, write_mem_in, is_data_in;
    
    output reg [BITS_ADDRESS_MEM-1:0] addr_req_mem_out, addr_write_mem_out;
    output reg [BITS_BLOCK-1:0] block_out;
    output reg read_mem_out, write_mem_out, is_data_out;    
    
    
    always @(posedge clk) begin
        if (~rst) begin
            addr_req_mem_out <= addr_req_mem_in;
            addr_write_mem_out <= addr_write_mem_in;
            block_out <= block_in;
            read_mem_out <= read_mem_in;
            write_mem_out <= write_mem_in;
            is_data_out <= is_data_in;
        end else begin
            addr_req_mem_out <= 0;
            addr_write_mem_out <= 0;
            block_out <= 0;
            read_mem_out <= 0;
            write_mem_out <= 0; 
            is_data_out <= 0;      
        end
    end

endmodule

