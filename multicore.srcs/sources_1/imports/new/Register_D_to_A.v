`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.11.2017 17:25:59
// Design Name: 
// Module Name: Register_D_to_A
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


module Register_D_to_A(
        clk,
        enable,
        reset,
        alu_en_in,
        is_branch_in,
        branch_mask_in,
        is_jump_in,
        alu_opcode_in,
        rs0_val_in,
        rs1_val_in,
        immediate_in,
        use_rs1_in,
        regwrite_en_in,
        rd_in,
        save_pc_in,
        effective_address_in,
        memread_en_in,
        memwrite_en_in,
        alu_en_out,
        is_branch_out,
        branch_mask_out,
        is_jump_out,
        alu_opcode_out,
        rs0_val_out,
        rs1_val_out,
        immediate_out,
        use_rs1_out,
        regwrite_en_out,
        rd_out,
        save_pc_out,
        effective_address_out,
        memread_en_out,
        memwrite_en_out
    );
    
    parameter BITS_ALU_OPCODE = 4;
    parameter BITS_DATA = 32;
    parameter BITS_REG_SELECT = 5;
    parameter BITS_PC = 32;
    parameter BITS_ROB_ENTRY_SELECT = 4;
    
    input clk, enable, reset, is_branch_in, is_jump_in, regwrite_en_in, use_rs1_in, alu_en_in, memread_en_in, memwrite_en_in;
    input[2:0] branch_mask_in;
    input[BITS_REG_SELECT-1:0] rd_in;
    input[BITS_ALU_OPCODE-1:0] alu_opcode_in;
    input[BITS_DATA-1:0] rs0_val_in, rs1_val_in, immediate_in, effective_address_in;
    input[BITS_PC-1:0] save_pc_in;
    
    output is_branch_out, is_jump_out, regwrite_en_out, use_rs1_out, alu_en_out, memread_en_out, memwrite_en_out;
    output[BITS_ALU_OPCODE-1:0] alu_opcode_out;
    output[BITS_REG_SELECT-1:0] rd_out;
    output[BITS_DATA-1:0] rs0_val_out, rs1_val_out, immediate_out, effective_address_out;
    output[2:0] branch_mask_out;
    output[BITS_PC-1:0] save_pc_out;
    
    wire[BITS_DATA*2-1:0] source_regs;
    wire[6:0] bit_signals;
    
    Register #(.BITS_SIZE(BITS_DATA)) immediate (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(immediate_in),
        .dout(immediate_out)
    );

    Register #(.BITS_SIZE(BITS_DATA*2)) register_values (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din({rs0_val_in,rs1_val_in}),
        .dout(source_regs)
    );
    assign rs0_val_out = source_regs[63:32];
    assign rs1_val_out = source_regs[31:0];
    
    Register #(.BITS_SIZE(BITS_ALU_OPCODE)) alu_opcode(
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(alu_opcode_in),
        .dout(alu_opcode_out)    
    );
    
    Register #(.BITS_SIZE(BITS_REG_SELECT)) destination_reg (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(rd_in),
        .dout(rd_out)
    );

    Register #(.BITS_SIZE(3)) branch_mask (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(branch_mask_in),
        .dout(branch_mask_out)
    );

    Register #(.BITS_SIZE(7)) one_bit_signals (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din({memread_en_in,memwrite_en_in,alu_en_in,use_rs1_in,is_jump_in,is_branch_in,regwrite_en_in}),
        .dout(bit_signals)
    );   
        
    assign regwrite_en_out = bit_signals[0];
    assign is_branch_out = bit_signals[1];
    assign is_jump_out = bit_signals[2];
    assign use_rs1_out = bit_signals[3];
    assign alu_en_out = bit_signals[4];
    assign memread_en_out = bit_signals[5];
    assign memwrite_en_out = bit_signals[6];
    
    Register #(.BITS_SIZE(BITS_PC)) save_pc (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(save_pc_in),
        .dout(save_pc_out)
    );    

    
endmodule