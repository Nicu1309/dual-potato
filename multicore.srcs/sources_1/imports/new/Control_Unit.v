`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 23.11.2017 19:31:48
// Design Name: 
// Module Name: Control_Unit
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
/*
    ******************* Instruction types encoding *******************
    
    * R-type
    
      31        25 24     20 19     15 14  12 11      7 6           0
     +------------+---------+---------+------+---------+-------------+
     | funct7     | rs1     | rs0     |funct3| rd      | opcode      |
     +------------+---------+---------+------+---------+-------------+
    
    * I-type
    
      31                  20 19     15 14  12 11      7 6           0
     +----------------------+---------+------+---------+-------------+
     | imm                  | rs0     |funct3| rd      | opcode      |
     +----------------------+---------+------+---------+-------------+
    
    * S-type
    
      31        25 24     20 19     15 14  12 11      7 6           0
     +------------+---------+---------+------+---------+-------------+
     | imm        | rs1     | rs0     |funct3| imm     | opcode      |
     +------------+---------+---------+------+---------+-------------+
    
    * U-type
    
      31                                      11      7 6           0
     +---------------------------------------+---------+-------------+
     | imm                                   | rd      | opcode      |
     +---------------------------------------+---------+-------------+
    
 from: 
 https://web.csl.cornell.edu/courses/ece5745/handouts/ece5745-tinyrv-isa.txt
Types of immediates: I, S, B, U, J. Here U is not used, not LUI support
*/

//////////////////////////////////////////////////////////////////////////////////


`include "constants.vh"

module Control_Unit(
        reset,
        instruction,
        alu_en,
        alu_opcode,
        use_rs1,
        rs0,
        rs1,
        rd,
        regwrite_en,
        immediate,
        memread_en,
        memwrite_en,
        is_byte,
        is_branch,
        branch_mask,
        is_jump
    );
    
    parameter BITS_INSTRUCTION = 32;
    parameter BITS_DATA = 32;
    parameter BITS_REG_SELECT = 5;
    parameter MSB_INST_CODE = 6;
    parameter LSB_INST_CODE = 0;
    parameter BITS_ALU_OPCODE = 5;
    
    localparam FN3_SIZE = 3;
    localparam FN7_SIZE = 7;
    localparam FN12_SIZE = 12;
    
    
    localparam RD_START = MSB_INST_CODE+1;
    localparam RD_END = RD_START+BITS_REG_SELECT-1;
    localparam FN3_START = RD_END+1;
    localparam FN3_END = FN3_START+FN3_SIZE-1;
    localparam RS0_START = FN3_END+1;
    localparam RS0_END = RS0_START+BITS_REG_SELECT-1;
    localparam RS1_START = RS0_END+1;
    localparam RS1_END = RS1_START+BITS_REG_SELECT-1;
    localparam FN7_START = RS1_END+1;
    localparam FN7_END = FN7_START+FN7_SIZE-1;
    
    localparam I_IMM = 0; 
    localparam S_IMM = 1;
    localparam B_IMM = 2;
    localparam U_IMM = 3;
    localparam J_IMM = 4;
        
    input reset;
    input[BITS_INSTRUCTION-1:0] instruction;
    output[BITS_DATA-1:0] immediate;
    output reg memread_en, memwrite_en, alu_en, regwrite_en, is_branch, is_jump;
    output is_byte, use_rs1;
    output[BITS_REG_SELECT-1:0] rs0, rs1, rd;
    output reg [BITS_ALU_OPCODE-1:0] alu_opcode;
    output reg[2:0] branch_mask; //GT,LT,EQ
    
    
    /* Number of different types for immediates */
    localparam NUM_IMMEDIATES = 5;
    localparam BITS_SELECT_IMMEDIATES = 3;
    // To select which type of immediate we are using 
    wire[BITS_DATA-1:0] immediate_types [NUM_IMMEDIATES-1:0];
    reg[BITS_SELECT_IMMEDIATES-1:0] immediate_select;
    
    wire[MSB_INST_CODE:LSB_INST_CODE] inst_opcode;
    wire[FN3_SIZE-1:0] inst_fn3;
    wire[FN7_SIZE-1:0] inst_fn7;
    wire[FN12_SIZE-1:0] inst_fn12;
    
    
    assign inst_opcode = instruction[MSB_INST_CODE:LSB_INST_CODE];
    assign rd = instruction[RD_END:RD_START];
    assign inst_fn3 = instruction[FN3_END:FN3_START];
    assign rs0 = instruction[RS0_END:RS0_START];
    assign rs1 = instruction[RS1_END:RS1_START];
    assign inst_fn7 = instruction[FN7_END:FN7_START];
    assign inst_fn12 = instruction[BITS_DATA-1 -: FN12_SIZE];
    
    assign immediate_types[I_IMM] = {{21{instruction[31]}},instruction[30:20]};
    assign immediate_types[S_IMM] = {{21{instruction[31]}},instruction[30:25],instruction[11:7]};
    assign immediate_types[B_IMM] = {{20{instruction[31]}},instruction[7],instruction[30:25],instruction[11:8],1'b0};
    assign immediate_types[U_IMM] = {instruction[31:12],12'b0};
    assign immediate_types[J_IMM] = {{12{instruction[31]}},instruction[19:12],instruction[20],instruction[30:21],1'b0};
    
    always @(instruction,reset)begin
        if (reset) begin
            memread_en = 1'b0; 
            memwrite_en = 1'b0;
            alu_en = 1'b0;
            regwrite_en = 1'b0;
            immediate_select = I_IMM;
            alu_opcode = 1'b0;
            is_branch = 1'b0;
            is_jump = 1'b0;
            branch_mask = 3'b000;
        end else
            casez({inst_fn7[0],inst_opcode})   
                `LOAD:begin
                    memread_en = 1'b1;
                    memwrite_en = 0;
                    alu_en = 0;
                    regwrite_en = 1'b1;
                    immediate_select = I_IMM;
                    alu_opcode = 0;
                    is_branch = 0;
                    is_jump = 0;
                    branch_mask = 3'b000;
                end
                `STORE:begin
                    memread_en = 0;
                    memwrite_en = 1'b1;
                    alu_en = 0;
                    regwrite_en = 0;
                    immediate_select = S_IMM;
                    alu_opcode = 0;
                    is_branch = 0;
                    is_jump = 0;
                    branch_mask = 3'b000;
                end
                `MUL:begin
                    memread_en = 0;
                    memwrite_en = 0;
                    alu_en = 1'b1;
                    regwrite_en = 1'b1;
                    // MUL -> alu_opcode = 1xxxx
                    alu_opcode = {BITS_ALU_OPCODE{1'b1}};
                    is_branch = 0;
                    is_jump = 0;
                    branch_mask = 3'b000; 
                end
                `ARIT_REG:begin
                    memread_en = 0;
                    memwrite_en = 0;
                    alu_en = 1'b1;
                    regwrite_en = 1'b1;
                    // if SUB -> opcode=01000, else 00xxx
                    alu_opcode = {1'b0,inst_fn7[5],inst_fn3};
                    is_branch = 0;
                    is_jump = 0;
                    branch_mask = 3'b000; 
                 end
                `ARIT_IMM:begin
                    memread_en = 0;
                    memwrite_en = 0;
                    alu_en = 1'b1;
                    regwrite_en = 1'b1;
                    immediate_select = I_IMM;
                    alu_opcode = {2'b00,inst_fn3};
                    is_branch = 0;
                    is_jump = 0;
                    branch_mask = 3'b000; 
                end
                `BRANCH:begin
                    memread_en = 0;
                    memwrite_en = 0;
                    alu_en = 1'b1;
                    regwrite_en = 0;
                    immediate_select = B_IMM;
                    alu_opcode = 0;
                    is_branch = 1'b1;
                    is_jump = 0;
                    case(inst_fn3)
                        3'b000:begin
                            branch_mask = 3'b001; //BEQ
                        end
                        3'b001:begin 
                            branch_mask = 3'b110; //BNE
                        end
                        3'b100:begin
                            branch_mask = 3'b010; //BLT
                        end
                        3'b101:begin
                            branch_mask = 3'b101; //BGE
                        end
                        default:begin
                            branch_mask = 3'b000; //mark exception illegal instruction
                        end
                    endcase
                end
                `JAL:begin
                    memread_en = 0;
                    memwrite_en = 0;
                    alu_en = 1'b1;
                    regwrite_en = 0;
                    immediate_select = J_IMM;
                    is_branch = 0;
                    is_jump = 1'b1;
                    branch_mask = 3'b000;    
                end 
                default begin
                    memread_en = 0;
                    memwrite_en = 0;
                    alu_opcode = 0;
                    alu_en = 0;
                    regwrite_en = 0;
                    is_branch = 0;
                    is_jump = 0;
                    immediate_select = 0;
                    branch_mask = 3'b000;
                end
            endcase
    end
    
    assign is_signed = inst_fn3[2]; //Used ? I dont know if need to impl signed byte loads
    assign is_byte = ~(inst_fn3[1] | inst_fn3[0]); //if LB -> fn3 = x00
    assign use_rs1 = inst_opcode[5]; //Diff between ADDI and ADD is (0010011 and 0110011)
    assign immediate = immediate_types[immediate_select];
    
endmodule