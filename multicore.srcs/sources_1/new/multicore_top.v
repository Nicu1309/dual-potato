`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.06.2018 12:32:47
// Design Name: 
// Module Name: multicore_top
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

module multicore_top(
    input clk,
    input reset
    );

    parameter BITS_ADDR = 32;
    parameter BITS_DATA = 32;
    parameter BITS_ADDRESS_MEM = 28;
    parameter BITS_BLOCK = 128;
    parameter BITS_WAY = 2; // 4 ways
    parameter BITS_SET = 2; // 4 sets
    localparam BITS_REQUESTS = 1 + BITS_WAY + BITS_SET + 1; // memory + WAYS*SET + log(num_proc)
    localparam NUM_WAYS = 2**BITS_WAY;
    localparam NUM_SETS = 2**BITS_SET;
    localparam REQUEST_PER_CORE = NUM_WAYS*NUM_SETS;
    localparam TOTAL_REQUESTS = (2 * REQUEST_PER_CORE) + 1; // CORES + MEMORY
    
    wire [BITS_BLOCK-1:0] inst_data_pc0, inst_data_pc1;
    wire [BITS_ADDRESS_MEM-1:0] inst_addr_pc0, inst_addr_pc1;
    wire [1:0] read_mem_inst, ready_mem_inst;
    
    wire [TOTAL_REQUESTS-1:0] coherence_bus_granted;
    wire [TOTAL_REQUESTS-1:0] coherence_bus_request;
    wire [BITS_BLOCK-1:0] coherence_bus_data;
    wire [BITS_ADDRESS_MEM-1:0] coherence_bus_addr;
    wire [`BITS_COHERENCE_MSG_TYPES-1:0] coherence_bus_msg;
    
    
top_core pc0 (    
    .clk(clk),
    .reset(reset),
    .addr_req_mem_inst(inst_addr_pc0),
    .block_mem_in_inst(inst_data_pc0),
    .ready_mem_inst(read_mem_inst[0]),
    .read_mem_inst(ready_mem_inst[0]),
    .coherence_bus_request(coherence_bus_request[REQUEST_PER_CORE:1]),
    .coherence_bus_granted(coherence_bus_granted[REQUEST_PER_CORE:1]),
    .coherence_bus_addr(coherence_bus_addr),
    .coherence_bus_data(coherence_bus_data),
    .coherence_bus_msg(coherence_bus_msg)
  );
  
top_core pc1 (    
      .clk(clk),
      .reset(reset),
      .addr_req_mem_inst(inst_addr_pc1),
      .block_mem_in_inst(inst_data_pc1),
      .ready_mem_inst(read_mem_inst[1]),
      .read_mem_inst(ready_mem_inst[1]),
      .coherence_bus_request(coherence_bus_request[TOTAL_REQUESTS-1:REQUEST_PER_CORE+1]),
      .coherence_bus_granted(coherence_bus_granted[TOTAL_REQUESTS-1:REQUEST_PER_CORE+1]),
      .coherence_bus_addr(coherence_bus_addr),
      .coherence_bus_data(coherence_bus_data),
      .coherence_bus_msg(coherence_bus_msg)
);

Coherence_Bus_Arbiter #(.BITS_REQUEST(6)) arbiter(
    .clk(clk),    
    .request_signals(coherence_bus_request),
    .grant_signals(coherence_bus_granted)
);

Main_Memory memory(       
        .clk(clk),
        .rst(reset),
        .re_pc0(read_mem_inst[0]),
        .re_pc1(read_mem_inst[1]),
        .addr_read_pc0(inst_addr_pc0),
        .ready_pc0(ready_mem_inst[0]),
        .addr_read_pc1(inst_addr_pc1),
        .ready_pc1(ready_mem_inst[1]),
        .coherence_bus_msg_type(coherence_bus_msg),
        .coherence_bus_data(coherence_bus_data),
        .coherence_bus_addr(coherence_bus_addr),
        .coherence_bus_granted(coherence_bus_request[0]),
        .coherence_bus_request(coherence_bus_request[1]),
        .dout_pc0(inst_data_pc0),
        .dout_pc1(inst_data_pc1));


endmodule
