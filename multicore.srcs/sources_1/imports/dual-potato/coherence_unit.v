`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/21/2018 07:55:42 PM
// Design Name: 
// Module Name: coherence_unit
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

module coherence_unit(
        clk,
        state_in, //coherence_state[addr_set][victim_way[addr_set]]
        state_out,
        coherence_bus_granted,
        coherence_bus_req,
        coherence_bus_addr,
        coherence_bus_data,
        coherence_bus_msg_type,
        block_id_in,
        data_in,
        data_out,
        evict_block,
        write_data_cache
    );
    
    parameter BITS_ADDR_BUS = 30;
    parameter BITS_DATA_BUS = 128;
    
    input [`BITS_COHERENCE_STATES-1:0] state_in;
    input coherence_bus_granted, evict_block, clk;
    input [BITS_ADDR_BUS-1:0] block_id_in;
    input [BITS_DATA_BUS-1:0] data_in;
    output reg [BITS_DATA_BUS-1:0] data_out;
    output reg coherence_bus_req, write_data_cache;
    output reg [`BITS_COHERENCE_STATES-1:0] state_out;
    inout [BITS_ADDR_BUS-1:0] coherence_bus_addr;
    inout [BITS_DATA_BUS-1:0] coherence_bus_data;
    inout [`BITS_COHERENCE_MSG_TYPES-1:0] coherence_bus_msg_type;
    
    wire must_send_data;
    reg [`BITS_COHERENCE_STATES-1:0] actual_state;
    reg [BITS_ADDR_BUS-1:0] write_bus_addr;
    reg [BITS_DATA_BUS-1:0] write_bus_data;
    reg [`BITS_COHERENCE_MSG_TYPES-1:0] write_bus_msg_type;
    
    assign must_send_data = ((coherence_bus_addr == block_id_in) && (coherence_bus_msg_type == `READ_DATA)) | evict_block;
    assign coherence_bus_addr = (coherence_bus_granted) ? write_bus_addr : {BITS_ADDR_BUS{1'bz}};
    assign coherence_bus_msg_type = (coherence_bus_granted) ? write_bus_msg_type : {`BITS_COHERENCE_MSG_TYPES{1'bz}};
    assign coherence_bus_data = (must_send_data && coherence_bus_granted) ? write_bus_data : {BITS_DATA_BUS{1'bz}};
    
    initial begin
        actual_state = `INVALID;
    end
    
    /*
    * Control Unit Coherence protocol (Valid/Invalid)
    */
    always @(*) begin    
     case(actual_state)
         `INVALID:begin 
            // Move to valid state if that line is invalid (MISS)
            coherence_bus_req = 1'b1; // Request access to coherence bus
            state_out = `WAIT_COHERENCE_BUS_INVALID;
         end
         `WAIT_COHERENCE_BUS_INVALID: begin
            if(coherence_bus_granted) begin
                coherence_bus_req = 1'b0;
                write_bus_addr = block_id_in;
                write_bus_msg_type = `READ_DATA; // Ask for the block of data
                state_out = `IV_DATA;
            end
         end
         `VALID:begin
            write_data_cache = 1'b0;
            // Move to invalid state if evict data
            // Move to invalid state if there is a "READ_DATA" of the block not mine
            if (must_send_data) begin
                coherence_bus_req = 1'b1; // Request access to coherence bus
                state_out = `WAIT_COHERENCE_BUS_VALID;
            end
         end
         `WAIT_COHERENCE_BUS_VALID: begin
            if(coherence_bus_granted) begin
                coherence_bus_req = 1'b0;
                write_bus_data = data_in; 
                state_out = `INVALID;
                write_bus_addr = block_id_in;
                if (evict_block) begin
                    write_bus_msg_type = `EVICT_DATA;
                end else begin
                    write_bus_msg_type = `WRITE_DATA;
                end
            end   
         end
         `IV_DATA:begin
            if((coherence_bus_addr == block_id_in) && (coherence_bus_msg_type == `WRITE_DATA)) begin
                // Write received blocked from the bus
                data_out = coherence_bus_data;
                write_data_cache = 1'b1;   
                state_out = `VALID;                
            end   
         end
     endcase
    end
    
    always @(posedge clk) begin
        actual_state <= state_out;
    end
    
endmodule
