`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: UPC
// Engineer: David Nicuesa Aranda
// Engineer: Ana Lasheras Mas
// 
// Create Date: 05.10.2017 21:30:48
// Design Name: Paraterized Encoder
// Module Name: Encoder
// Project Name: PariscV
// Target Devices: 
// Tool Versions: 
// Description: 
//      Parameterized encoder
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module Encoder(
        encoder_in,
        encoder_out,
        enable,
        reset
    );
    
    parameter BITS_IN = 2; //Number of input bits
    parameter BITS_OUT = 1; //Number of output bits
    
    integer i;
    
    input[BITS_IN-1:0] encoder_in;
    input enable, reset;
    output reg [BITS_OUT-1:0] encoder_out;
    
    // ON change of enable or input, encode
    always@(encoder_in)begin
        if(reset)
            encoder_out = 0;
        else if(enable)
            for(i=0;i<BITS_IN;i=i+1)begin
                if(encoder_in[i])
                    encoder_out = i;
            end
    end
endmodule
