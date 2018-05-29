`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 21.05.2018 17:15:52
// Design Name: 
// Module Name: top_core
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

`include "constants.vh"

module top_core(
    clk,
    reset
    );
    
        parameter BITS_PC = 32;
        parameter BITS_DATA = 32;
        parameter BITS_REG_SELECT = 5;
        parameter MSB_INST_CODE = 6;
        parameter LSB_INST_CODE = 0;
        
        localparam BITS_ALU_OPCODE = 5;
        
        input clk;
        input reset;
        
        /******** Fetch  *******/
        wire z, inst_hit, hazard_fetch;
        wire [BITS_PC-1 : 0] pc_out, pc_in, pc_p4, next_pc, branch_pc;
        wire [BITS_DATA-1 : 0] fetched_inst;
        
        /******** Decode  *******/
        wire hazard_decode;
        wire [BITS_PC-1 : 0] save_pc_decode;
        wire [BITS_DATA-1 : 0] encoded_inst;
        wire no_bypass, bypass_decode_alu_r0, bypass_decode_alu_r1, bypass_decode_mem_r0, 
             bypass_decode_mem_r1, bypass_decode_wb_r0, bypass_decode_wb_r1;
        wire[BITS_DATA-1:0] rs0_val_decode_final, rs1_val_decode_final;
        wire use_rs1_decode, alu_en_decode, memread_en_decode, memwrite_en_decode, is_byte_decode;
        wire regwrite_en_decode, is_branch_decode, is_jump_decode;
        wire[BITS_DATA-1:0] data_writeback, rs0_val_decode, rs1_val_decode, immediate_extended_decode, effective_address_decode, data_store_decode;
        wire[BITS_REG_SELECT-1:0] rs0_decode, rs1_decode, rd_decode;
        wire[BITS_ALU_OPCODE-1:0] alu_opcode_decode;
        wire[2:0] branch_mask_decode;
        wire [BITS_REG_SELECT-1:0] instruction_rs0_decode, instruction_rs1_decode;
        
        /******** Execution *****/
        wire alu_en_ex, regwrite_alu_en_ex, z_flag, uflow_flag, oflow_flag, is_branch_ex, is_jump_ex, gt_ex, lt_ex, eq_ex, is_byte_ex;
        wire[BITS_DATA-1:0] op0_ex, op1_ex, alu_result_ex, rs1_val_ex, immediate_extended_ex, effective_address_ex, dout_ex;
        wire[BITS_REG_SELECT-1:0] rd_ex;
        wire[BITS_ALU_OPCODE-1:0] alu_opcode_ex;
        wire[BITS_PC-1:0] save_pc_ex; //This has pc
        wire[2:0] branch_mask_ex;
        wire use_rs1_ex, memwrite_en_ex, memread_en_ex, regwrite_en_ex;
        
        /******** Memory  *******/
        wire data_hit;
        wire[BITS_DATA-1:0] effective_address_mem, memout_mem, data_mem, mem_address_in, dout_mem;
        wire[BITS_REG_SELECT-1:0] rd_mem;
        wire is_byte_mem, memread_en_mem, memwrite_en_mem, regwrite_en_mem;
        wire valid, mem_is_byte_instruction;
        wire [BITS_DATA-1:0] address_mem_in;
        
        /******* Writeback ******/
        wire regwrite_en_wb;
        wire [BITS_DATA-1:0] data_wb;
        wire [BITS_REG_SELECT-1:0] rd_wb;
        
        
        /*******  Write-back ****/
            
        /*********************************************
        ************** Fetch Stage *******************
        ***********************************************/
            
        // Program Counter 
        Register #(.BITS_SIZE(BITS_PC)) pc (
            .clk(clk),
            .enable( (~hazard_fetch) | reset ),
            .reset(1'b0),
            .din(pc_in),
            .dout(pc_out)
        );
        
        assign pc_p4 = pc_out+4; 
        assign next_pc = z?branch_pc:pc_p4;
        assign pc_in = reset?`RESET_PC:next_pc;
        assign hazard_fetch = ~inst_hit | hazard_decode | z;
        
        
        // FETCH to DECODE
        Register_F_to_D #(.BITS_ADDRESS(BITS_PC)) Stage_F_D(
            .clk(clk),
            .enable(~hazard_decode),
            .reset(reset),
            .fetched_instruction_in(~hazard_fetch?fetched_inst:`NOP),
            .fetched_instruction_out(encoded_inst),
            .save_pc_in(pc_out),
            .save_pc_out(save_pc_decode)
        );
        
        
        
        /*********************************************
        ************** Decode Stage *******************
        ***********************************************/
        assign bypass_decode_alu_r0 = ((is_branch_decode || alu_en_decode || memwrite_en_decode || memread_en_decode) &&
                                        alu_en_ex && rd_ex == instruction_rs0_decode);
        assign bypass_decode_alu_r1 =((is_branch_decode || alu_en_decode || memwrite_en_decode) && 
                                        alu_en_ex && rd_ex == instruction_rs1_decode);
        assign bypass_decode_mem_r0 = ((is_branch_decode || alu_en_decode || memwrite_en_decode || memread_en_decode) && 
                                        memread_en_mem && rd_mem == instruction_rs0_decode);
        assign bypass_decode_mem_r1 =((is_branch_decode || alu_en_decode || memwrite_en_decode) && 
                                        memread_en_mem && rd_mem == instruction_rs1_decode);
        assign bypass_decode_wb_r0 = ((is_branch_decode || alu_en_decode || memwrite_en_decode || memread_en_decode) && 
                                        regwrite_en_wb && rd_wb == instruction_rs0_decode);
        assign bypass_decode_wb_r1 =((is_branch_decode || alu_en_decode || memwrite_en_decode) && 
                                        regwrite_en_wb && rd_wb == instruction_rs1_decode);
        /* Mem producer followed by consumer at distance 1 */
        assign no_bypass = ((is_branch_decode || alu_en_decode || memwrite_en_decode || memread_en_decode) &&
                            memread_en_ex && (rd_ex == instruction_rs0_decode || rd_ex == instruction_rs1_decode));
                            
        assign hazard_decode = ~reset && (no_bypass || (~data_hit)); 
        
        
        Control_Unit #(.BITS_INSTRUCTION(BITS_DATA),
                .BITS_DATA(BITS_DATA),
                .BITS_REG_SELECT(BITS_REG_SELECT),
                .MSB_INST_CODE(MSB_INST_CODE),
                .LSB_INST_CODE(LSB_INST_CODE),
                .BITS_ALU_OPCODE(BITS_ALU_OPCODE)) cu(.reset(reset),
            .instruction(encoded_inst),
            .alu_en(alu_en_decode),
            .alu_opcode(alu_opcode_decode),
            .use_rs1(use_rs1_decode),
            .rs0(instruction_rs0_decode),
            .rs1(instruction_rs1_decode),
            .rd(rd_decode),
            // Add NOP if dependency not satisfied with bypass
            .regwrite_en(regwrite_en_decode),
            .immediate(immediate_extended_decode),
            .memread_en(memread_en_decode),
            .memwrite_en(memwrite_en_decode),
            .is_byte(is_byte_decode),
            .is_branch(is_branch_decode),
            .branch_mask(branch_mask_decode),
            .is_jump(is_jump_decode)
        );
            
        assign rs0_val_decode_final = bypass_decode_alu_r0?alu_result_ex:
                              bypass_decode_mem_r0?memout_mem:
                              bypass_decode_wb_r0?data_wb:rs0_val_decode;
        
        assign rs1_val_decode_final = bypass_decode_alu_r1?alu_result_ex:
                              bypass_decode_mem_r1?memout_mem:
                              bypass_decode_wb_r1?data_wb:rs1_val_decode;

        // Registers File with 2 true read ports, 1 write port
        Register_File #(.BITS_REG_SELECT(BITS_REG_SELECT),.BITS_REG_DATA(BITS_DATA)) register_file (
            .clk(clk),
            .reset(reset),
            .rs(instruction_rs0_decode),
            .rt(instruction_rs1_decode),
            .dout0(rs0_val_decode),
            .dout1(rs1_val_decode),
            .din(data_wb),
            .reg_in(rd_wb),
            .we(regwrite_en_wb)
        );
        
        assign effective_address_decode = (memread_en_decode | memwrite_en_decode)?(rs0_val_decode_final+immediate_extended_decode):32'b0;
         //If we write in memory, we are store so take reg value
        assign data_store_decode = memwrite_en_decode?rs1_val_decode_final:{BITS_DATA{1'b0}};
          
        // DECODE to EX
        Register_D_to_A #(.BITS_DATA(BITS_DATA),
           .BITS_ALU_OPCODE(BITS_ALU_OPCODE),
           .BITS_REG_SELECT(BITS_REG_SELECT),
           .BITS_PC(BITS_PC)) stage_D_A(
           .clk(clk),
           .reset(reset),
           .enable(1'b1), 
           // Decode -> In
           .alu_en_in(~z & alu_en_decode & ~hazard_decode),
           .is_branch_in(~z & is_branch_decode & ~hazard_decode),
           .branch_mask_in(z?{3{1'b0}}:branch_mask_decode),
           .is_jump_in(~z & is_jump_decode & ~hazard_decode),
           .alu_opcode_in(alu_opcode_decode),
           .rs0_val_in(z?{BITS_DATA-1{1'b0}}:rs0_val_decode_final), 
           .rs1_val_in(z?{BITS_DATA-1{1'b0}}:rs1_val_decode_final), 
           .immediate_in(z?{BITS_DATA-1{1'b0}}:immediate_extended_decode),
           .use_rs1_in(~z & use_rs1_decode),
           .regwrite_en_in(~z & regwrite_en_decode & ~hazard_decode),
           .rd_in(z?{BITS_REG_SELECT-1{1'b0}}:rd_decode),
           .save_pc_in(save_pc_decode),
           .memread_en_in(memread_en_decode & ~hazard_decode),
           .memwrite_en_in(memwrite_en_decode & ~hazard_decode),
           .effective_address_in(effective_address_decode),
           // Out -> ALU
           .alu_en_out(alu_en_ex),
           .is_branch_out(is_branch_ex),
           .branch_mask_out(branch_mask_ex),
           .is_jump_out(is_jump_ex),
           .alu_opcode_out(alu_opcode_ex),
           .rs0_val_out(op0_ex),
           .rs1_val_out(rs1_val_ex),
           .immediate_out(immediate_extended_ex),
           .use_rs1_out(use_rs1_ex),
           .regwrite_en_out(regwrite_en_ex),
           .rd_out(rd_ex),    
           .save_pc_out(save_pc_ex),
           .effective_address_out(effective_address_ex),
           .memread_en_out(memread_en_ex),
           .memwrite_en_out(memwrite_en_ex)
        );
        
        /*********************************************
        ************** Execution Stage ***************
        ***********************************************/
        assign op1_ex = use_rs1_ex?rs1_val_ex:immediate_extended_ex;
        
        ALU #(.BITS_DATA(BITS_DATA),.BITS_OPCODE(BITS_ALU_OPCODE)) alu(
           .enable(alu_en_ex),
           .din0(op0_ex),
           .din1(op1_ex),
           .opcode(alu_opcode_ex),
           .dout(alu_result_ex),
           .zero(z_flag),
           .uflow(uflow_flag),
           .oflow(oflow_flag),
           .greater_than(gt_ex),
           .less_than(lt_ex),
           .equals(eq_ex)
        );

        assign z = is_branch_ex ? (|({gt_ex,lt_ex,eq_ex} & branch_mask_ex)) : 0;   
        assign branch_pc = save_pc_ex + immediate_extended_ex;  
        assign dout_ex = memwrite_en_ex?rs1_val_ex:alu_result_ex;
        // EX to MEM  
        Register_Ex_to_Mem #(.BITS_ADDRESS(BITS_DATA)) stage_A_M(
            .clk(clk),
            .reset(reset),
            // Stop if we cannot access memory
            .enable(data_hit),     
            // Alu-> In
            .effective_address_in(effective_address_ex),
            .is_byte_in(is_byte_ex),
            .memread_en_in(memread_en_ex),
            .memwrite_en_in(memwrite_en_ex),
            .data_in(dout_ex),
            .rd_in(rd_ex),
            .regwrite_en_in(regwrite_en_ex),
            // Out -> Mem
            .effective_address_out(effective_address_mem),
            .is_byte_out(is_byte_mem),
            .memread_en_out(memread_en_mem),
            .memwrite_en_out(memwrite_en_mem),
            .data_out(data_mem),
            .rd_out(rd_mem),
            .regwrite_en_out(regwrite_en_mem)
        );

        /**** EL BIT DE INVALIDATE SOLO DEBE INVALIDAR LOS MISSES DE INSTRUCCIONES *****/
        Memory_System memory_hierarchy(
            .clk(clk),
            .reset(reset),
            .addr_data(effective_address_mem),
            .addr_inst(pc_out),
            .hit_data(data_hit),
            .hit_inst(inst_hit),
            .is_byte(is_byte_mem),
            .we(memwrite_en_mem),
            .re_data(memread_en_mem),
            .re_inst(~reset), //Always fetching instruction
            .dout_data(memout_mem),
            .dout_inst(fetched_inst),
            .din(data_mem),
            .invalidate_req_inst(z)
        );  
        assign dout_mem = memread_en_mem?memout_mem:data_mem;
        Register_Mem_to_Writeback #(
                .BITS_REG_SELECT(BITS_REG_SELECT),
                .BITS_DATA(BITS_DATA)) 
            stage_M_WB(
                .clk(clk),
                .reset(reset),
                .enable(1),
                .rd_in(rd_mem),
                .rd_out(rd_wb),
                .data_in(dout_mem),
                .data_out(data_wb),
                .en_wb_in(regwrite_en_mem),
                .en_wb_out(regwrite_en_wb)
           );
      
        
endmodule
