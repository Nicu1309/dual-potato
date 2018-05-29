`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02.12.2017 17:51:54
// Design Name: 
// Module Name: Multiplier
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

module Multiplier(
    enable,
    reset,
    clk,
    din0,
    din1,
    rd_in,
    check_rs0,
    check_rs1,
    dout,
    zero,
    finished,
    rob_entry_in,
    rob_entry_out,
    rd_out,
    hazard_register,
    invalidate_in,
    invalidate_out
);

    parameter BITS_DATA = 32;
    parameter NUM_STAGES = 6; 
    parameter BITS_ROB_ENTRY_SELECT = 4;
    parameter BITS_REG_SELECT = 5;
    
    input enable, reset, clk, invalidate_in;
    input signed [BITS_DATA-1:0] din0, din1;
    input [BITS_ROB_ENTRY_SELECT-1:0] rob_entry_in;
    input [BITS_REG_SELECT-1:0] rd_in, check_rs0, check_rs1;
    
    output signed [BITS_DATA-1:0] dout;
    output zero, finished, hazard_register, invalidate_out; //wire
    output[BITS_ROB_ENTRY_SELECT-1:0] rob_entry_out;
    output [BITS_REG_SELECT-1:0] rd_out;
    
    wire[BITS_DATA-1:0] data[NUM_STAGES-1:0];
    wire[NUM_STAGES-1:0] aux_zero, aux_finished;
    wire[BITS_ROB_ENTRY_SELECT-1:0] aux_rob_entry[NUM_STAGES-1:0];
    wire[BITS_REG_SELECT-1:0] aux_rd [NUM_STAGES-1:0];
    wire[NUM_STAGES-2:0] hits_hazard;
    wire[NUM_STAGES-1:0] aux_invalidate;

    assign data[0] = (enable && ~reset) ? (din0*din1) : {BITS_DATA{1'b0}};
    assign aux_zero[0] = (enable && ~reset && (data[0] == 0));
    assign aux_finished[0] = (enable && ~reset);
    assign aux_rob_entry[0] = rob_entry_in;
    assign aux_rd[0] = rd_in;
    assign aux_invalidate[0] = invalidate_in;
            
    generate 
        for (genvar r=0; r<NUM_STAGES-1; r=r+1) begin:check_for_hazards
           assign hits_hazard[r] = (aux_rd[r] == check_rs0) || (aux_rd[r] == check_rs1);
        end
        for (genvar r=1; r<NUM_STAGES; r=r+1) 
            begin: aux_reg_mul
                Reg_mul_stages #(
                            .BITS_DATA(BITS_DATA),
                            .BITS_ROB_ENTRY_SELECT(BITS_ROB_ENTRY_SELECT)) mutliplier_stage(
                    .enable(1'b1),
                    .reset(reset),
                    .clk(clk),
                    .din(data[r-1]),
                    .dout(data[r]),
                    .rd_in(aux_rd[r-1]),
                    .rd_out(aux_rd[r]),
                    .zero_in(aux_zero[r-1]),
                    .zero_out(aux_zero[r]),
                    .finished_in(aux_finished[r-1]),
                    .finished_out(aux_finished[r]),
                    .rob_entry_in(aux_rob_entry[r-1]),
                    .rob_entry_out(aux_rob_entry[r]),
                    .invalidate_in(aux_invalidate[r-1]),
                    .invalidate_out(aux_invalidate[r])
                );
            end
    endgenerate
    
    assign dout = data[NUM_STAGES -1];
    assign zero = aux_zero[NUM_STAGES -1]; 
    assign finished = aux_finished[NUM_STAGES -1]; 
    assign rob_entry_out = aux_rob_entry[NUM_STAGES-1];
    assign rd_out = aux_rd[NUM_STAGES-1];
    assign hazard_register = |hits_hazard;
    assign invalidate_out = aux_invalidate[NUM_STAGES-1];
    
endmodule
