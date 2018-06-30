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

`include "constants.vh"

module Main_Memory(
        clk,
        rst,
        re_pc0,
        re_pc1,
        addr_read_pc0,
        ready_pc0,
        addr_read_pc1,
        ready_pc1,
        coherence_bus_msg_type,
        coherence_bus_data,
        coherence_bus_addr,
        coherence_bus_granted,
        coherence_bus_request,
        dout_pc0,
        dout_pc1
    );
    
    parameter BITS_BYTE_IN_BLOCK = 4;
    parameter BITS_ADDRESS_BYTE = 32;
    localparam BITS_ADDRESS = BITS_ADDRESS_BYTE - BITS_BYTE_IN_BLOCK;
    localparam BITS_BLOCK = `BYTE * (2**BITS_BYTE_IN_BLOCK);
    localparam MEM_ENTRIES = 2**BITS_ADDRESS;
    localparam READY = 1;
    localparam WAIT_BUS = 0;
    
    input clk, rst, re_pc0, re_pc1, coherence_bus_granted;
    output reg coherence_bus_request;
    input [BITS_ADDRESS-1:0] addr_read_pc0, addr_read_pc1;
    output [BITS_BLOCK-1:0] dout_pc0, dout_pc1;
    output ready_pc0, ready_pc1;
    
    inout [BITS_BLOCK-1:0] coherence_bus_data;
    inout [BITS_ADDRESS-1:0] coherence_bus_addr;
    inout [`BITS_COHERENCE_MSG_TYPES-1:0] coherence_bus_msg_type;
    
    reg [BITS_BLOCK-1:0] memory [0:MEM_ENTRIES-1];
    reg control [0:MEM_ENTRIES-1];
    reg control_nxt, update_control, must_send_data, mem_state;
    
    reg [BITS_ADDRESS-1:0] write_bus_addr;
    reg [BITS_BLOCK-1:0] write_bus_data;
    reg [`BITS_COHERENCE_MSG_TYPES-1:0] write_bus_msg_type;
    
    integer i;
    
    initial begin
      mem_state = 1'b1;
      $readmemh("RAM_buff_sum.mem", memory);
      for(i=0; i<MEM_ENTRIES; i=i+1)
          control[i] = 1'b1;
    end
    

    assign coherence_bus_addr = (coherence_bus_granted) ? write_bus_addr : {BITS_ADDRESS{1'bz}};
    assign coherence_bus_msg_type = (coherence_bus_granted) ? write_bus_msg_type : {`BITS_COHERENCE_MSG_TYPES{1'bz}};
    assign coherence_bus_data = (must_send_data && coherence_bus_granted) ? write_bus_data : {BITS_BLOCK{1'bz}};
    
    
    always @(posedge clk) begin
        if(coherence_bus_msg_type == `EVICT_DATA) begin
            memory[coherence_bus_addr] <= coherence_bus_data;
        end
        if(update_control)
            control[coherence_bus_addr] <= control_nxt;
    end
    
    always @(*) begin
        case(mem_state)
            READY:begin
                update_control = 1'b0;
                casez(coherence_bus_msg_type) 
                    `EVICT_DATA:begin
                        control_nxt = 1'b1;
                        update_control = 1'b1;
                    end
                    `READ_DATA:begin
                        if(control[coherence_bus_addr])begin //We have data
                            write_bus_addr = coherence_bus_addr;
                            coherence_bus_request = 1'b1;
                            mem_state = WAIT_BUS;
                        end
                    end
                endcase
            end
            WAIT_BUS:begin
                if(coherence_bus_granted) begin
                    update_control = 1'b1;
                    control_nxt = 1'b0; //We dont have data anymore
                    write_bus_data = memory[write_bus_addr];
                    write_bus_msg_type = `WRITE_DATA;
                    coherence_bus_request = 1'b0;
                end
            end
        endcase
        
    end
    
    assign dout_pc0 = re_pc0?memory[addr_read_pc0]:0;
    assign dout_pc1 = re_pc1?memory[addr_read_pc1]:0;
    assign ready_pc0 = re_pc0;
    assign ready_pc1 = re_pc1;
    
endmodule
