`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02.12.2017 19:45:52
// Design Name: 
// Module Name: Register_D_to_MUL
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

module Register_D_to_MUL(
    clk,
    reset,
    enable,
    mul_en_in,
    rs0_val_in,
    rs1_val_in,
    regwrite_en_in,
    rd_in,
    save_pc_in,
    rob_entry_in,
    mul_en_out,
    rs0_val_out,
    rs1_val_out,
    regwrite_en_out,
    rd_out,
    save_pc_out,
    rob_entry_out,
    invalidate_in,
    invalidate_out
);

    parameter BITS_DATA = 32;
    parameter BITS_REG_SELECT = 5;
    parameter BITS_PC = 32; 
    parameter BITS_ROB_ENTRY_SELECT = 4;
    
    input  clk, enable, reset, mul_en_in, regwrite_en_in, invalidate_in;
    //Register number to write the result of the multiplication
    input[BITS_REG_SELECT-1:0] rd_in;
    input[BITS_DATA-1:0] rs0_val_in, rs1_val_in;
    input[BITS_PC-1:0] save_pc_in;
    input[BITS_ROB_ENTRY_SELECT-1:0] rob_entry_in;
    
    output mul_en_out, regwrite_en_out, invalidate_out;
    output[BITS_DATA-1:0] rs0_val_out, rs1_val_out;
    output[BITS_REG_SELECT-1:0] rd_out;
    output[BITS_PC-1:0] save_pc_out;
    output[BITS_ROB_ENTRY_SELECT-1:0] rob_entry_out;
    
    wire[BITS_DATA*2-1:0] source_regs;
    wire[1:0] bit_signals;
    
    Register #(.BITS_SIZE(BITS_DATA*2)) register_values (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din({rs0_val_in,rs1_val_in}),
        .dout(source_regs)
    );
    
    assign rs0_val_out = source_regs[63:32];
    assign rs1_val_out = source_regs[31:0];
    
    Register #(.BITS_SIZE(BITS_REG_SELECT)) destiny_reg (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(rd_in),
        .dout(rd_out)
    );
    
    Register #(.BITS_SIZE(2)) one_bit_signals (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din({mul_en_in,regwrite_en_in}),
        .dout(bit_signals)
    );  
    
    assign mul_en_out = bit_signals[0];
    assign regwrite_en_out = bit_signals[1];
    
    Register #(.BITS_SIZE(BITS_PC)) save_pc (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(save_pc_in),
        .dout(save_pc_out)
    ); 

    Register #(.BITS_SIZE(BITS_ROB_ENTRY_SELECT)) rob_entry (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(rob_entry_in),
        .dout(rob_entry_out)
    ); 
    
    Register #(.BITS_SIZE(1)) invalidate_bit (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .din(invalidate_in),
        .dout(invalidate_out)
    );  
       
endmodule
