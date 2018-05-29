`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.09.2017 22:03:33
// Design Name: 
// Module Name: Main_Memory
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

`define BYTE 8

module Main_Memory(
        clk,
        rst,
        addr_read,
        addr_write,
        we,
        re,
        valid,
        din,
        dout
    );
    
    parameter BITS_BYTE_IN_BLOCK = 4;
    parameter BITS_ADDRESS_BYTE = 32;
    localparam BITS_ADDRESS = BITS_ADDRESS_BYTE - BITS_BYTE_IN_BLOCK;
    localparam BLOCK_SIZE_BITS = `BYTE * (2**BITS_BYTE_IN_BLOCK);
    localparam MEM_ENTRIES = 2**BITS_ADDRESS;
    
    input we, re, clk, rst;
    output valid;
    input [BLOCK_SIZE_BITS-1:0] din;
    input [BITS_ADDRESS-1:0] addr_read, addr_write;
    output [BLOCK_SIZE_BITS-1:0] dout;
    
    reg [BLOCK_SIZE_BITS-1:0] memory [0:MEM_ENTRIES-1];
    
    initial begin
      $readmemh("RAM_buff_sum.mem", memory);
    end
    
    always @(posedge clk) begin
        if (we && ~rst)
            memory[addr_write] <= din[BLOCK_SIZE_BITS-1:0];
    end
    
    assign dout = re?memory[addr_read]:0;
    assign valid = re;
    
endmodule
