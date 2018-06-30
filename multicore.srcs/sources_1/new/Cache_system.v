`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.06.2018 14:08:20
// Design Name: 
// Module Name: Cache_system
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


`include "constants.vh"

module Cache_system(
        clk,
        reset,
        addr_data,
        addr_inst,
        hit_data,
        hit_inst,
        is_byte,
        we,
        re_data,
        re_inst,
        dout_data,
        dout_inst,
        din,
        addr_req_mem_inst,
        block_mem_in_inst,
        ready_mem_inst,
        read_mem_inst,
        invalidate_req_inst,
        coherence_bus_request,
        coherence_bus_granted,
        coherence_bus_addr,
        coherence_bus_data,
        coherence_bus_msg
    );
    
    parameter BITS_ADDR = 32;
    parameter BITS_DATA = 32;
    parameter BITS_ADDRESS_MEM = 28;
    parameter BITS_BLOCK = 128;
    parameter STAGES = 2;
    parameter BITS_WAY = 2; // 4 ways
    parameter BITS_SET = 2; // 4 sets
    localparam NUM_WAYS = 2**BITS_WAY;
    localparam NUM_SETS = 2**BITS_SET;
  
    input is_byte, we, re_data, re_inst, clk, reset, ready_mem_inst, invalidate_req_inst;
    input [BITS_ADDR-1:0] addr_data, addr_inst;
    input [BITS_DATA-1:0] din;
    input [BITS_BLOCK-1:0] block_mem_in_inst;
    input [NUM_SETS*NUM_WAYS-1:0]  coherence_bus_granted;
    output hit_data, hit_inst, read_mem_inst;
    output [BITS_DATA-1:0] dout_data, dout_inst;
    output [BITS_ADDRESS_MEM-1:0] addr_req_mem_inst;
    output [NUM_SETS*NUM_WAYS-1:0] coherence_bus_request;
   
    inout [BITS_BLOCK-1:0] coherence_bus_data;
    inout [BITS_ADDRESS_MEM-1:0] coherence_bus_addr;
    inout [`BITS_COHERENCE_MSG_TYPES-1:0] coherence_bus_msg;
    
    Cache_Data_RR cache_data(
        .clk(clk),
        .reset(reset),
        .flush(1'b0),
        .we(we),
        .re(re_data),
        .is_byte(is_byte),
        .bus_request(coherence_bus_request),
        .bus_granted(coherence_bus_granted),
        .bus_addr(coherence_bus_addr),
        .bus_data(coherence_bus_data),
        .bus_msg(coherence_bus_msg),
        .addrin(addr_data),
        .din(din),
        .dout(dout_data),
        .hit(hit_data)
        
        );
    
    //Set parameters for the inst cache
    Cache_Inst_RR cache_Inst(.clk(clk),
        .reset(reset),
        .re(re_inst),
        .addrin(addr_inst),
        .addr_req_mem(addr_req_mem_inst),
        .block_mem_in(block_mem_in_inst),
        .ready_mem(ready_mem_inst),
        .read_mem(read_mem_inst),
        .dout(dout_inst),
        .hit(hit_inst),
        .bus_request(),
        .bus_granted(1'b1),
        .invalidate_request(invalidate_req_inst));
    
    
endmodule
