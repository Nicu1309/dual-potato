`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 23.11.2017 17:30:09
// Design Name: 
// Module Name: Register
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


module Register(
        clk,
        enable,
        reset,
        din,
        dout
    );
    
    parameter BITS_SIZE = 32;
    
    input clk, enable, reset;
    input[BITS_SIZE-1:0] din;
    output[BITS_SIZE-1:0] dout;
    
    generate
        genvar r;
        
        for(r=0; r<BITS_SIZE; r=r+1)begin: instantiate_ff
            FF_D flip_flop(
                .clk(clk),
                .reset(reset),
                .enable(enable),
                .din(din[r]),
                .dout(dout[r])
            );
        end
    endgenerate
endmodule
