`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05.12.2017 20:19:41
// Design Name: 
// Module Name: Register_F_to_D
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


module Register_F_to_D(
        clk,
        enable,
        reset,
        fetched_instruction_in,
        fetched_instruction_out,
        save_pc_in,
        save_pc_out
    );
    
    parameter BITS_ADDRESS = 32;
    
    input clk, enable, reset;
    input[BITS_ADDRESS-1:0] fetched_instruction_in, save_pc_in;
    
    output[BITS_ADDRESS-1:0] fetched_instruction_out, save_pc_out;
    
    /* Instruction */
    Register #(.BITS_SIZE(BITS_ADDRESS)) instruction(
        .clk(clk),
        .enable(enable),
        .reset(reset),
        .din(fetched_instruction_in),
        .dout(fetched_instruction_out)
    );
    
    /* Saved PC */
    Register #(.BITS_SIZE(BITS_ADDRESS)) pc(
        .clk(clk),
        .enable(enable),
        .reset(reset),
        .din(save_pc_in),
        .dout(save_pc_out)
    );

endmodule
