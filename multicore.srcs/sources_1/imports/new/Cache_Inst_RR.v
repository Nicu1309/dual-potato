`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 22.11.2017 16:13:03
// Design Name: 
// Module Name: Cache_Inst_RR
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
module Cache_Inst_RR(
        clk,
        reset,
        re,
        addrin,
        bus_granted,
        bus_request,
        addr_req_mem,
        block_mem_in,
        ready_mem,
        read_mem,
        dout,
        hit,
        invalidate_request
    );
    
    parameter BITS_ADDRESS_IN = 32;
    parameter BITS_BYTES_IN_WORD = 2; // bits for select byte in word, words of 32 bits
    parameter BITS_BYTES_IN_BLOCK = 4; // 16 bytes per block, 4 bits select byte
    parameter BITS_WAY = 1; // 2 ways
    parameter BITS_SET = 2; // 4 sets
         
    localparam BITS_ADDRESS_MEM = BITS_ADDRESS_IN - BITS_BYTES_IN_BLOCK; // bits to address blocks in memory (8 bytes per block)
    localparam BITS_TAG = BITS_ADDRESS_IN - BITS_BYTES_IN_BLOCK - BITS_SET;
    localparam BLOCK_SIZE_BYTES = 2**BITS_BYTES_IN_BLOCK; // bytes in block, default 8 bytes
    localparam CACHE_SIZE_BYTES = 2**(BITS_BYTES_IN_BLOCK + BITS_WAY + BITS_SET); // cache size default 64 bytes
    localparam NUM_WAYS = 2**BITS_WAY;
    localparam NUM_SETS = 2**BITS_SET;
    localparam MEM_ENTRIES = 2**BITS_ADDRESS_IN;
    localparam WORD_SIZE_BYTES = 2**BITS_BYTES_IN_WORD;
    localparam NUM_WORDS_IN_BLOCK = 2**(BITS_BYTES_IN_BLOCK-BITS_BYTES_IN_WORD);
    localparam BITS_WORD = `BYTE*WORD_SIZE_BYTES;
    localparam VALID_BIT = BITS_TAG; //index for valid bit
    
    // States for the control unit
    localparam BITS_STATES = 3;
    localparam RESET = 0;
    localparam READY = 1;
    localparam WAIT_BUS = 2;
    localparam MISS = 3;
               
    input re, clk, reset, ready_mem, bus_granted, invalidate_request;
    output hit;
    output reg bus_request, read_mem;
    input [BLOCK_SIZE_BYTES*`BYTE-1:0] block_mem_in;
    input [BITS_ADDRESS_IN-1:0] addrin;
    output reg [BITS_ADDRESS_MEM-1:0] addr_req_mem;
    output [BITS_WORD-1:0] dout;
    
    
    integer i,k;
    wire wh;
    wire [BITS_WAY-1:0] victim_way[NUM_SETS-1:0];
    wire [NUM_SETS-1:0] evict;
    wire [NUM_WAYS-1:0] hits; // 1-bit wires for hit on each way
    wire [BITS_TAG-1:0] addr_tag = addrin[BITS_ADDRESS_IN-1:BITS_BYTES_IN_BLOCK+BITS_SET];
    wire [BITS_SET-1:0] addr_set = addrin[BITS_SET+BITS_BYTES_IN_BLOCK-1:BITS_BYTES_IN_BLOCK];
    wire [BITS_BYTES_IN_BLOCK-BITS_BYTES_IN_WORD-1:0] num_word_in_block = addrin[BITS_BYTES_IN_BLOCK-1:BITS_BYTES_IN_WORD];
    wire [BITS_BYTES_IN_WORD-1:0] addr_byte_in_word = addrin[BITS_BYTES_IN_WORD-1:0];
    
    // selected block 
    wire [`BYTE-1:0] word_out [WORD_SIZE_BYTES-1:0];
      
    // data structure 
    reg [`BYTE-1:0] data [0:NUM_SETS-1][0:NUM_WAYS-1][NUM_WORDS_IN_BLOCK-1:0][WORD_SIZE_BYTES-1:0];
    
    // control structure with (valid + tag) No dirty bit here, it's Read-Only
    reg [BITS_TAG:0] control [0:NUM_SETS-1][0:NUM_WAYS-1];
    
    // state machine
    reg [BITS_STATES-1:0] state = RESET;
   
    /*
    * Control Unit FSM
    */
    always @(posedge clk)
    begin    
     case(state)
         RESET:begin 
             bus_request <= 1'b0;
             addr_req_mem <= 1'b0;
             read_mem <= 1'b0;
             for(i=0;i<NUM_SETS;i=i+1)
                 for(k=0;k<NUM_WAYS;k=k+1)begin
                     control[i][k] <= {BITS_TAG+1{1'b0}};
                 end    
             if (~reset)
                 state <= READY;
         end
         READY:begin
             if (reset) //reset
                 state <= RESET;
             else if (~hit && ~invalidate_request)begin  //miss 
                 state <= WAIT_BUS;
                 bus_request <= 1'b1;
                 addr_req_mem <= {addr_tag,addr_set}; // Block addr to fill cache
                 read_mem <= 1'b1;; //Read signal to mem 
             end
         end
         WAIT_BUS:begin
             if (invalidate_request)begin
                 bus_request <= 1'b0;
                 addr_req_mem <= 0;
                 read_mem <= 1'b0;
                 state <= READY;
             end
             else if(bus_granted)begin
                 state <= MISS;
                 bus_request <= 1'b0;
                 addr_req_mem <= 0;
                 read_mem <= 1'b0;
             end
         end
         MISS:begin
             if (invalidate_request) begin
                state <= READY;
             end
             else if(ready_mem)begin
                 // Write received blocked from memory
                 for(i=0;i<NUM_WORDS_IN_BLOCK;i=i+1)begin
                     for(k=1;k<=WORD_SIZE_BYTES;k=k+1)begin
                        data[addr_set][victim_way[addr_set]][i][k-1] <= block_mem_in[WORD_SIZE_BYTES*`BYTE*i + `BYTE*k-1 -:`BYTE];
                     end
                 end
                 control[addr_set][victim_way[addr_set]] <= addr_tag;
                 control[addr_set][victim_way[addr_set]][VALID_BIT] <= 1'b1;
                 state <= READY;
             end
         end
     endcase
    end
    
    /*
    * Generate connections for way and data selection 
    */
    generate
     genvar w,b;
     
     // Each way hit      
     for(w=0;w<NUM_WAYS;w=w+1)begin:hit_out
         assign hits[w] = (control[addr_set][w][BITS_TAG-1:0] == addr_tag);
     end
     
     // Encoder for way select
     Encoder #(NUM_WAYS,BITS_WAY) way_selector(
             .encoder_in(hits),
             .encoder_out(wh),
             .enable(1),
             .reset(reset)
         );
             
     // Decoder for check what set is active by addr on miss
     Decoder #(BITS_SET,NUM_SETS) set_selector(
              .decoder_in(addr_set),
              .decoder_out(evict),
              .enable(~hit),
              .reset(reset)
         );
         
     // Block out from hit way
     for(b=0;b<WORD_SIZE_BYTES;b=b+1)begin: byte_out
         assign word_out[b] = data[addr_set][wh][num_word_in_block][b];
     end
     
     // Pack the word out 
     for(b=1;b<=WORD_SIZE_BYTES;b=b+1)begin: unpacked_to_word
         assign dout[`BYTE*b-1 -:`BYTE] = word_out[b-1];
     end
     
     //One round robin queue per set to select the victim way
     for(b=0;b<NUM_SETS;b=b+1)begin:round_robin_queue_per_module
         Fifo_Queue #(BITS_WAY,BITS_WAY) victim_way_queue (
             .clk(clk),
             .init_fill(1),
             .reset(reset),
             .pop(evict[b] && ready_mem),
             .push(evict[b] && ready_mem),
             .data_in(victim_way[b]),
             .data_out(victim_way[b])
          );
     end 
     
    endgenerate
    
    assign hit = ~reset?( (|hits && control[addr_set][wh][VALID_BIT]) | ~re | invalidate_request):1'b0; //hit = any hit AND block is valid
    
endmodule