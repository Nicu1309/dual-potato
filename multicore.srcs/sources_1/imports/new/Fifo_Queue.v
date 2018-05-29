`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07.11.2017 20:06:35
// Design Name: 
// Module Name: Fifo_Queue
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


module Fifo_Queue(
    data_in,
    init_fill,
    clk,
    push,
    pop,
    reset,
    data_out,
    empty,
    full
    );

    parameter BITS_DATA = 4;
    parameter BITS_ENTRY_SELECT = 2;
    
    localparam NUM_ENTRIES = 2**BITS_ENTRY_SELECT;    
    
    input push, pop, reset, clk, init_fill;
    input [BITS_DATA-1:0] data_in;
    output [BITS_DATA-1:0] data_out;
    output full, empty;
    
    reg [BITS_DATA-1:0] queue [NUM_ENTRIES-1:0];
    reg [BITS_ENTRY_SELECT-1:0] pt_head, pt_tail;
    reg [BITS_ENTRY_SELECT:0] count; // Count is full means = 4, so needs additional bit
    
    integer e;
    
    always @(posedge clk)begin
        if(reset)begin
            for(e=0;e<NUM_ENTRIES;e=e+1)
                if(init_fill) begin
                    queue[e] <= e;
                    count <= NUM_ENTRIES;
                end
                else begin
                    queue[e] <= 0;
                    count <= 0;
                end
            pt_head <= 0;
            pt_tail <= 0;
        end
        else if(push && pop) begin
            queue[pt_tail] <= data_in;
            pt_tail <= pt_tail + 1;
            pt_head <= pt_head + 1;
        end 
        else if(push && !full)begin
            count <= count + 1;
            queue[pt_tail] <= data_in;
            pt_tail <= pt_tail + 1;
        end
        else if(pop && !empty)begin
            count <= count - 1;
            pt_head <= pt_head + 1;
        end
    end
    
    assign empty = (pt_tail == pt_head) & !full;
    assign full = (count == NUM_ENTRIES);
    assign data_out = queue[pt_head];
    
endmodule
