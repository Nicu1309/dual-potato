`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 15.10.2017 17:25:52
// Design Name: 
// Module Name: Memory_System
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


module Memory_System(
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
        invalidate_req_inst
    );
    parameter BITS_ADDR = 32;
    parameter BITS_DATA = 32;
    parameter BITS_ADDRESS_MEM = 27;
    parameter BITS_BLOCK = 128;
    parameter STAGES = 2;
    
    input is_byte, we, re_data, re_inst, clk, reset, invalidate_req_inst;
    input [BITS_ADDR-1:0] addr_data, addr_inst;
    input [BITS_DATA-1:0] din;
    output hit_data, hit_inst;
    output [BITS_DATA-1:0] dout_data, dout_inst;
    
    wire [STAGES-1:0] ready_mem_main, read_mem_main, aux_ready_mem_main, aux_read_mem_main, write_mem_main, req_is_data, res_is_data;    
    wire [BITS_ADDRESS_MEM-1:0] addr_req_mem_main [STAGES-1:0];
    wire [BITS_ADDRESS_MEM-1:0] addr_write_mem_main [STAGES-1:0];
    wire [BITS_ADDRESS_MEM-1:0] addr_req_mem_inst, addr_req_mem_data, addr_write_mem_data;
    wire [BITS_BLOCK-1:0] block_mem_in_main [STAGES-1:0];
    wire [BITS_BLOCK-1:0] block_mem_out_main [STAGES-1:0];
    
    wire ready_mem_data, ready_mem_inst, read_mem_data, read_mem_inst, write_mem_data;
    wire [BITS_BLOCK-1:0] block_mem_in_data, block_mem_out_data, block_mem_in_inst;
    
    wire bus_request_data, bus_request_inst, bus_granted_data, bus_granted_inst;
    
    //Set parameters for the data cache
    Cache_Data_RR cache_data(.clk(clk),
            .reset(reset),
            .we(we),
            .re(re_data),
            .flush(1'b0),
            .is_byte(is_byte),
            .addrin(addr_data),
            .addr_req_mem(addr_req_mem_data),
            .addr_write_mem(addr_write_mem_data),
            .block_mem_in(block_mem_in_data),
            .block_mem_out(block_mem_out_data),
            .ready_mem(ready_mem_data),
            .read_mem(read_mem_data),
            .write_mem(write_mem_data),
            .din(din),
            .dout(dout_data),
            .hit(hit_data),
            .bus_request(bus_request_data),
            .bus_granted(bus_granted_data));

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
            .bus_request(bus_request_inst),
            .bus_granted(bus_granted_inst),
            .invalidate_request(invalidate_req_inst));

    Memory_Arbiter arbiter(
        .req_data(bus_request_data),
        .req_inst(bus_request_inst),
        .grant_data(bus_granted_data),
        .grant_inst(bus_granted_inst)
    );

    //Main Memory
    Main_Memory main_memory(
            .clk(clk),
            .rst(reset),
            .addr_read(addr_req_mem_main[STAGES-1]),
            .addr_write(addr_write_mem_main[STAGES-1]),
            .we(write_mem_main[STAGES-1]),
            .re(read_mem_main[STAGES-1]),
            .valid(aux_ready_mem_main[0]),
            .din(block_mem_in_main[STAGES-1]),
            .dout(block_mem_out_main[0])
    );
    // Bypass whether is data request or not
    assign res_is_data[0] = req_is_data[STAGES-1];
    
    // Assign request wires from chaches
    assign req_is_data[0] = bus_granted_data;
    assign read_mem_main[0] = bus_granted_data?read_mem_data:bus_granted_inst?(read_mem_inst & ~invalidate_req_inst):1'b0;
    assign ready_mem_main[0] = res_is_data[0] | (~res_is_data[0] & aux_ready_mem_main[0]);
    assign aux_read_mem_main[0] = bus_granted_data?read_mem_data:bus_granted_inst?(read_mem_inst & ~invalidate_req_inst):1'b0;
    assign write_mem_main[0] = bus_granted_data?write_mem_data:1'b0;
    assign block_mem_in_main[0] = bus_granted_data?block_mem_out_data:0;
    assign addr_req_mem_main[0] = bus_granted_data?addr_req_mem_data:addr_req_mem_inst;
    assign addr_write_mem_main[0] = bus_granted_data?addr_write_mem_data:0;
    
    // Assign  response wires from memory to caches
    assign block_mem_in_data = res_is_data[STAGES-1]?block_mem_out_main[STAGES-1]:0;
    assign block_mem_in_inst = ~res_is_data[STAGES-1]?block_mem_out_main[STAGES-1]:0;
    assign ready_mem_data = res_is_data[STAGES-1] & ready_mem_main[STAGES-1];
    assign ready_mem_inst = ~res_is_data[STAGES-1] & ready_mem_main[STAGES-1];
    

    generate
        for(genvar r=1; r<STAGES; r=r+1)begin: pipe_req_mem_data
        
            assign read_mem_main[r] = (req_is_data[r] | (~req_is_data[r] & ~invalidate_req_inst)) & aux_read_mem_main[r];
            assign ready_mem_main[r] = (res_is_data[r] | (~res_is_data[r] & ~invalidate_req_inst)) & aux_ready_mem_main[r];
            
            Register_Req_Mem_Data pipe_req_data(
                    .clk(clk), 
                    .rst(reset),
                    .is_data_in(req_is_data[r-1]),
                    .read_mem_in(read_mem_main[r-1]),
                    .write_mem_in(write_mem_main[r-1]),
                    .block_in(block_mem_in_main[r-1]),
                    .addr_req_mem_in(addr_req_mem_main[r-1]),
                    .addr_write_mem_in(addr_write_mem_main[r-1]),
                    .read_mem_out(aux_read_mem_main[r]),
                    .write_mem_out(write_mem_main[r]),
                    .block_out(block_mem_in_main[r]),
                    .addr_req_mem_out(addr_req_mem_main[r]),
                    .addr_write_mem_out(addr_write_mem_main[r]),
                    .is_data_out(req_is_data[r])
            );
        end
        
        for(genvar r=1; r<STAGES; r=r+1)begin: pipe_res_mem_data
            Register_Res_Mem_Data pipe_res_data(
                    .clk(clk), 
                    .rst(reset),
                    .ready_mem_in(ready_mem_main[r-1]),
                    .block_in(block_mem_out_main[r-1]),
                    .is_data_in(res_is_data[r-1]),
                    .ready_mem_out(aux_ready_mem_main[r]),
                    .block_out(block_mem_out_main[r]),
                    .is_data_out(res_is_data[r])
             );
        end
        
    endgenerate
    
endmodule
