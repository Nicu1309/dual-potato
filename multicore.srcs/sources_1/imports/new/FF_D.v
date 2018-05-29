`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 23.11.2017 17:27:16
// Design Name: 
// Module Name: FF_D
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


module FF_D(
        clk,
        reset,
        din,
        dout,
        enable
    );
    
    input clk, reset, din, enable;
    output reg dout;
    
    always @(posedge clk) begin
        if(reset)
            dout <= 1'b0;
        else if(enable)
            dout <= din;
    end
    
endmodule
