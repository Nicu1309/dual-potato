`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.09.2017 23:10:55
// Design Name: 
// Module Name: Cache_Data_RR
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

module Cache_Data_RR(
        clk,
        reset,
        flush,
        we,
        re,
        is_byte,
        bus_request,
        bus_granted,
        bus_addr,
        bus_data,
        bus_msg,
        addrin,
        ready_mem,
        read_mem,
        write_mem,
        din,
        dout,
        hit
    );
    
     parameter BITS_ADDRESS_IN = 32;
     parameter BITS_BYTES_IN_WORD = 2; // bits for select byte in word, words of 32 bits
     parameter BITS_BYTES_IN_BLOCK = 4; // 16 bytes per block, 4 bits select byte
     parameter BITS_WAY = 2; // 4 ways
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
     localparam VALID_BIT = BITS_TAG+1; //index for valid bit
     localparam DIRTY_BIT = BITS_TAG; //index for dirty bit
     localparam BITS_BLOCK = NUM_WORDS_IN_BLOCK * 32;
     
     // States for the control unit
     localparam BITS_STATES = 3;
     localparam RESET = 0;
     localparam READY = 1;
     localparam WAIT_BUS = 2;
     localparam MISS = 3;
                   
     input we, re, clk, flush, reset, is_byte, ready_mem, bus_granted;
     output hit;
     output bus_request;
     input [BITS_WORD-1:0] din;
     input [BITS_ADDRESS_IN-1:0] addrin;
     output reg read_mem, write_mem;
     output [BITS_WORD-1:0] dout;
     inout [BITS_BLOCK-1:0] bus_data;
     inout [BITS_ADDRESS_MEM-1:0] bus_addr;
     inout [`BITS_COHERENCE_MSG_TYPES-1:0] bus_msg;
     
     
     integer i,k;
     wire [BITS_WAY-1:0] wh;
     wire [BITS_WAY-1:0] victim_way[NUM_SETS-1:0];
     //wire [BITS_WAY-1:0] DEBUG_WAY_EVICT;
     wire [NUM_SETS-1:0] evict;
     wire [BITS_WORD-1:0] whole_word_out, block_word_out;
     wire [BITS_WORD-1:0] unpacked_block_out [NUM_WORDS_IN_BLOCK-1:0];
     wire [`BYTE-1:0] unpacked_word_out [WORD_SIZE_BYTES-1:0];
     wire [NUM_WAYS-1:0] hits; // 1-bit wires for hit on each way
     wire [BITS_TAG-1:0] addr_tag = addrin[BITS_ADDRESS_IN-1:BITS_BYTES_IN_BLOCK+BITS_SET];
     wire [BITS_SET-1:0] addr_set = addrin[BITS_SET+BITS_BYTES_IN_BLOCK-1:BITS_BYTES_IN_BLOCK];
     wire [BITS_BYTES_IN_BLOCK-BITS_BYTES_IN_WORD-1:0] num_word_in_block = addrin[BITS_BYTES_IN_BLOCK-1:BITS_BYTES_IN_WORD];
     wire [BITS_BYTES_IN_WORD-1:0] addr_byte_in_word = addrin[BITS_BYTES_IN_WORD-1:0];
     
     // Write from coherence unit received blck
     wire write_data [NUM_SETS-1:0][NUM_WAYS-1:0];
     wire [BITS_BLOCK-1:0] data_to_cache [NUM_SETS-1:0][NUM_WAYS-1:0];
      
     // selected block 
     wire [BITS_BLOCK-1:0] block_out;
     
     // Coherence states wires from units
     wire [`BITS_COHERENCE_STATES-1:0] coherence_states_out [0:NUM_SETS-1][0:NUM_WAYS-1], coherence_states_in [0:NUM_SETS-1][0:NUM_WAYS-1];          
          
     // data structure 
     reg [BITS_BLOCK-1:0] data [0:NUM_SETS-1][0:NUM_WAYS-1];
    
     // control structure with (dirty bit + tag)
     reg [BITS_ADDRESS_MEM:0] control [0:NUM_SETS-1][0:NUM_WAYS-1];
      
     
     // state machine
     reg [BITS_STATES-1:0] state = READY;
     
     /*
       * Control Unit FSM
     */
     /*always @(posedge clk)
     begin    
        case(state)
            RESET:begin 
                bus_request <= 1'b0;
                read_mem <= 1'b0;
                write_mem <= 1'b0;
                addr_req_mem <= 1'b0;
                addr_write_mem <= 1'b0;
                for(i=0;i<NUM_SETS;i=i+1)
                    for(k=0;k<NUM_WAYS;k=k+1)begin
                        control[i][k] <= {BITS_TAG+2{1'b0}};
                    end    
                if (~reset)
                    state <= READY;
            end
            READY:begin
                if (reset) //reset
                    state <= RESET;
                else if (~hit)begin  //miss 
                    state <= WAIT_BUS;
                    bus_request <= 1'b1;
                    addr_req_mem <= {addr_tag,addr_set}; // Block addr to fill cache
                    read_mem <= 1'b1;; //Read signal to mem 
                    write_mem <= (control[addr_set][victim_way[addr_set]][DIRTY_BIT]);// write signal to mem
                    addr_write_mem <= control[addr_set][victim_way[addr_set]][BITS_TAG-1:0]; //Addr to mem which write evicted block into
                end
            end
            WAIT_BUS:begin
                if(bus_granted)begin
                    state <= MISS;
                    bus_request <= 1'b0;
                    addr_req_mem <= 0;
                    read_mem <= 1'b0;
                    write_mem <= 1'b0;
                end
            end
            MISS:begin
                if(ready_mem)begin
                    // Write received blocked from memory
                    data[addr_set][victim_way[addr_set]] <= block_mem_in;
                    control[addr_set][victim_way[addr_set]] <= addr_tag;
                    control[addr_set][victim_way[addr_set]][VALID_BIT] <= 1'b1;
                    state <= READY;
                end
            end
        endcase
     end*/
     
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
        assign block_out = data[addr_set][wh];
              
        // Unpack block, array to bitstream
        for(w=0; w<NUM_WORDS_IN_BLOCK; w=w+1) begin: unpack_block
            assign unpacked_block_out[w] = block_out[((w+1)*BITS_WORD)-1 : -BITS_WORD]; 
        end
        
        assign whole_word_out = unpacked_block_out[num_word_in_block];
        
         // Unpack word, array to bitstream
        for(b=0; b<WORD_SIZE_BYTES; b=b+1) begin: unpack_word
           assign unpacked_word_out[b] = block_out[((b+1)*`BYTE)-1 : -`BYTE]; 
        end
        
        //One round robin queue per set to select the victim way
        for(b=0;b<NUM_SETS;b=b+1)begin:round_robin_queue_per_set
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
        
        //Coherence units
        for(b=0; b<NUM_SETS; b=b+1) begin: coherence_by_set
            for(w=0; w<NUM_WAYS; w=w+1) begin: coherence_by_way
                Register #(`BITS_COHERENCE_STATES) register_coherence_state(
                    .clk(clk),
                    .enable(1'b1),
                    .reset(reset),
                    .din(coherence_states_in[b][w]),
                    .dout(coherence_states_out[b][w])
                );
                
                Coherence_unit #(.BITS_ADDR_BUS(BITS_ADDRESS_MEM)) block_coherence(
                    .clk(clk),
                    .reset(reset),
                    .state_in(coherence_states_out[b][w]), //coherence_state[addr_set][victim_way[addr_set]]
                    .state_out(coherence_states_in[b][w]),
                    .coherence_bus_granted(bus_granted),
                    .coherence_bus_req(bus_request),
                    .coherence_bus_addr(bus_addr),
                    .coherence_bus_data(bus_data),
                    .coherence_bus_msg_type(bus_msg),
                    .block_id_in(control[b][w][BITS_ADDRESS_MEM-1:0]),
                    .block_id_req({addr_tag,addr_set}),
                    .data_in(data[b][w]),
                    .data_out(data_to_cache[b][w]),
                    .evict_block(evict[b] & (victim_way[b] == w)),
                    .write_data_cache(write_data[b][w])
                );   
            end
        end   
      endgenerate
      
      /* FSM data */
      always @(posedge clk) begin
        for(i=0; i<NUM_SETS; i=i+1)
            for(k=0; k<NUM_WAYS; k=k+1)
                if(reset)
                    control[i][k] <= {BITS_ADDRESS_MEM+1{1'b0}};
                else if(write_data[i][k])
                    data[i][k] <= data_to_cache[i][k];
      end
      
      /* Signals output from cache */
      assign dout = is_byte?unpacked_word_out[addr_byte_in_word]:whole_word_out; //word out = full word or byte in word
      assign hit = ~reset?((|hits && coherence_states_out[addr_set][wh] == `VALID) || (~re && ~we)):1'b0; //hit = any hit AND block is valid
      
endmodule