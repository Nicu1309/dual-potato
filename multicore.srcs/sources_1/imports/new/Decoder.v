`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12.11.2017 18:00:59
// Design Name: 
// Module Name: Decoder
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


module Decoder(
        decoder_in,
        decoder_out,
        enable,
        reset
    );
    
    parameter BITS_IN = 1; //Number of input bits
    parameter BITS_OUT = 2; //Number of output bits
    
    integer i;
    
    input[BITS_IN-1:0] decoder_in;
    input enable, reset;
    output reg [BITS_OUT-1:0] decoder_out;
    
    always @(decoder_in or enable or reset) begin
        if(reset)
            decoder_out = 0;
        else
            for(i=0;i<BITS_OUT;i=i+1)
                decoder_out[i] = (i==decoder_in) && enable ;
    end
endmodule
