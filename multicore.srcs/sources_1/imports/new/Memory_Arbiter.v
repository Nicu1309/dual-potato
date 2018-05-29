`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 22.11.2017 16:54:41
// Design Name: 
// Module Name: Memory_Arbiter
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


module Memory_Arbiter(
        req_data,
        req_inst,
        grant_data,
        grant_inst
    );
    
    input req_data, req_inst;
    output grant_data, grant_inst;
    
    assign grant_data = req_data;
    assign grant_inst = ~req_data && req_inst;
         
endmodule
