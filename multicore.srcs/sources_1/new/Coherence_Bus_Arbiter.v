`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 22.05.2018 11:54:25
// Design Name: 
// Module Name: Coherence_Bus_Arbiter
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


module Coherence_Bus_Arbiter(
        clk,
        request_signals,
        grant_signals
    );
    
    parameter BITS_REQUEST = 2;
    localparam NUM_REQUEST = 2**BITS_REQUEST;
    integer i;
    
    input clk;
    input [NUM_REQUEST-1:0] request_signals;
    output reg [NUM_REQUEST-1:0] grant_signals;
    
    always @(*) begin: grant_block
        if(request_signals[0]) begin
            grant_signals = {{NUM_REQUEST-1{1'b0}},1'b1};
        end
        else begin
            // Update grant_signals
            for(i=0;i<NUM_REQUEST;i=i+1) begin
                grant_signals[i] = (request_signals[i] == 0) ? 0 : grant_signals[i];
            end
            
            //Check the prioritazed line
           /* if(request_signals[turn_state]) begin
                update_turn = 1'b1;
                gran
                
            end*/
            // Give new grant if possible
            if(grant_signals == {NUM_REQUEST{1'b0}}) begin
                for(i=0;i<NUM_REQUEST;i=i+1) begin
                    if (request_signals[i]) begin
                        grant_signals[i] = request_signals[i];
                        disable grant_block;
                    end
                end
            end
        end
    end
    
    /*always @(posedge clk) begin
        if (update_turn)
            turn_state <= turn_state + 1;
    end*/
    
endmodule
