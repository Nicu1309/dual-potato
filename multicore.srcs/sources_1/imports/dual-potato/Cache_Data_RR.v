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
                   
     input we, re, clk, flush, reset, is_byte;
     input [NUM_SETS*NUM_WAYS-1:0] bus_granted;
     output hit;
     output [NUM_SETS*NUM_WAYS-1:0] bus_request;
     input [BITS_WORD-1:0] din;
     input [BITS_ADDRESS_IN-1:0] addrin;
     output [BITS_WORD-1:0] dout;
     inout [BITS_BLOCK-1:0] bus_data;
     inout [BITS_ADDRESS_MEM-1:0] bus_addr;
     inout [`BITS_COHERENCE_MSG_TYPES-1:0] bus_msg;
     
     
     integer i,k;
     wire [BITS_WAY-1:0] wh;
     wire [BITS_WAY-1:0] victim_way[NUM_SETS-1:0];
     wire [NUM_SETS*NUM_WAYS-1:0] active_unit;
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
     
     wire [NUM_SETS-1:0] data_ready;
     
     // Write from coherence unit received blck
     wire [NUM_WAYS-1:0] write_data [NUM_SETS-1:0];
     wire [BITS_BLOCK-1:0] data_to_cache [NUM_SETS-1:0][NUM_WAYS-1:0];
      
     // selected block 
     wire [BITS_BLOCK-1:0] block_out;
     
     // Coherence states wires from units
     wire [`BITS_COHERENCE_STATES-1:0] coherence_states_out [0:NUM_SETS-1][0:NUM_WAYS-1], coherence_states_in [0:NUM_SETS-1][0:NUM_WAYS-1];          
          
     // data structure 
     reg [BITS_BLOCK-1:0] data [0:NUM_SETS-1][0:NUM_WAYS-1];
    
     // control structure with (dirty bit + tag)
     reg [BITS_ADDRESS_MEM-1:0] control [0:NUM_SETS-1][0:NUM_WAYS-1];
      
     
     // state machine
     reg [BITS_STATES-1:0] state = READY;
     
     /*
       * Control Unit FSM
     */
     always @(posedge clk)
     begin    
        case(state)
            RESET:begin 
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
                else if (hit) // write on hit
                    for(i=0; i<BITS_BLOCK; i=i+1)begin
                        if((i >= num_word_in_block*BITS_WORD) && (i < (num_word_in_block+1)*BITS_WORD))
                            data[addr_set][wh][i] <= din[i];
                    end
                else 
                    state <= MISS;
            end
            MISS:begin
                if (reset)
                    state <= RESET;
                    if(|data_ready)begin
                        data[addr_set][victim_way[addr_set]] <= data_to_cache[addr_set][victim_way[addr_set]];
                        control[addr_set][victim_way[addr_set]] <= {addr_tag,addr_set};
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
            assign hits[w] = (control[addr_set][w] == {addr_tag,addr_set});
        end
        
        // Signals to write received block from coherence unit
        for(b=0; b<NUM_SETS; b=b+1)begin:ready_data_set
            assign data_ready[b] = |(write_data[b]);
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
        assign unpacked_block_out[0] = block_out[31:0];
        assign unpacked_block_out[1] = block_out[63:32];
        assign unpacked_block_out[2] = block_out[95:64];
        assign unpacked_block_out[3] = block_out[127:96];        
        
        assign whole_word_out = unpacked_block_out[num_word_in_block];
        
         // Unpack word, array to bitstream
        for(b=1; b<=WORD_SIZE_BYTES; b=b+1) begin: unpack_word
           assign unpacked_word_out[b] = whole_word_out[(b*`BYTE)-1 : -`BYTE]; 
        end
        
        //One round robin queue per set to select the victim way
        for(b=0;b<NUM_SETS;b=b+1)begin:round_robin_queue_per_set
            Fifo_Queue #(BITS_WAY,BITS_WAY) victim_way_queue (
                .clk(clk),
                .init_fill(1),
                .reset(reset),
                .pop(evict[b] && data_ready[b]),
                .push(evict[b] && data_ready[b]),
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
                assign active_unit[b*NUM_WAYS+w] = ~hit & (victim_way[b] == w) & (addr_set == b) & ~(|(data_ready));
                Coherence_unit #(.BITS_ADDR_BUS(BITS_ADDRESS_MEM)) block_coherence(
                    .clk(clk),
                    .reset(reset),
                    .active_request(active_unit[b*NUM_WAYS+w]),
                    .state_in(coherence_states_out[b][w]), //coherence_state[addr_set][victim_way[addr_set]]
                    .state_out(coherence_states_in[b][w]),
                    .coherence_bus_granted(bus_granted[b*NUM_WAYS+w]),
                    .coherence_bus_req(bus_request[b*NUM_WAYS+w]),
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
      
      /* Signals output from cache */
      assign dout = is_byte?unpacked_word_out[addr_byte_in_word]:whole_word_out; //word out = full word or byte in word
      assign hit = ~reset?((|hits && coherence_states_out[addr_set][wh] == `VALID) || (~re && ~we)):1'b1; //hit = any hit AND block is valid
      
endmodule