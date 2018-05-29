`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 22.11.2017 12:37:43
// Design Name: 
// Module Name: Pipeline
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

module Pipeline(
    clk,
    reset,
    interrupt_enable,
    interrupt_type
    );
    
    parameter BITS_PC = 32;
    parameter BITS_DATA = 32;
    parameter BITS_REG_SELECT = 5;
    parameter MSB_INST_CODE = 6;
    parameter LSB_INST_CODE = 0;
    parameter BITS_EXCEPTION = 5;
    parameter BITS_INTERRUPT = 4; //To can encode 9 different interrupts
    
    localparam BITS_PWS = 7;
    localparam BIT_MODE = BITS_PWS;
    localparam BITS_ALU_OPCODE = 4;
    localparam BITS_ROB_ENTRY_SELECT = 5;
    localparam EXEC_MODE_BIT = 0;
    localparam IE_BIT = 1;
    
    input clk;
    input reset;
    // Extern input indicating if interrupts are enable
    input interrupt_enable;
    // Type of the interrupt defined as an input
    input [BITS_INTERRUPT-1:0] interrupt_type;
     
    wire new_exec_mode;
    wire pending_interrupt, available_interrupt, take_exception;
    wire [BITS_INTERRUPT-1:0] interrupt_type_out;
    wire[BITS_PC-1:0] pc_in, next_pc, pc_out;
    wire[BITS_DATA-1:0] fetched_inst, encoded_inst;
    wire[BITS_PWS-1:0] status_in, status_out, prev_status_in, prev_status_out;
    wire[BITS_PC-1:0] branch_pc, pc_p4, save_pc_decode, epc_out;
    wire is_exception_fetch, is_exception_fetch_decode;
    wire[BITS_EXCEPTION-1:0] exception_type_fetch, exception_type_fetch_decode, ecause_out;
    wire z, inst_hit, data_hit, exec_mode, hazard_fetch, hazard_decode, prev_reset, restore_context;
    wire invalidate_pipeline;
    wire invalid_decode, invalid_mem, invalid_alu, invalid_initial_mul, invalid_final_mul, stop_alu;
    wire on_fly_misses = ~inst_hit | ~data_hit;
    
    /************ CSR CONTROL ********/
    wire csr_en_read_decode, stall_for_int, jump_to_int;
    wire [1:0] csr_en_write_decode; // 2 bits for no write, write, clear, set
    wire [7:0] csr_reg_num_decode; 
    wire [BITS_DATA-1:0] csr_outs [6:0]; //We only have 7 csr registers
    wire [BITS_DATA-1:0] csr_selected;
    wire [6:0] csr_we_from_inst; //We only have 7 csr registers
    wire [BITS_DATA-1:0] csr_data_in; 
    wire [BITS_DATA-1:0] csr_possible_data_in [3:0];
    wire [BITS_DATA-1:0] page_table_pointer_in, page_table_pointer_out;
    
    /************** ROB    ********/
    wire rob_is_finished, rob_is_exception, rob_is_byte, rob_writeReg_en, rob_writeMem_en;
    wire[BITS_REG_SELECT-1:0] rob_rd;
    wire[BITS_PC-1:0] rob_pc, rob_address_mem, bad_virtual_addr_out;
    wire[BITS_DATA-1:0] rob_data;
    wire[BITS_EXCEPTION-1:0] rob_exception_type;
    wire rob_full, rob_empty, push_rob, pop_rob;
    wire rob_update_reg, rob_store_coming;
    
    /******** BYPASSES wires *********/
    wire no_bypass, bypass_decode_rob_r0, bypass_decode_rob_r1, bypass_decode_mem_r0, 
         bypass_decode_mem_r1, bypass_decode_mul_r0, bypass_decode_mul_r1,
         bypass_decode_alu_r0, bypass_decode_alu_r1;
    wire[BITS_DATA-1:0] data_bypass_rob_r0, data_bypass_rob_r1;
    wire[BITS_DATA-1:0] rs0_val_decode_final, rs1_val_decode_final;
    
    /********* DECODE wires **********/
    wire use_rs1_decode, alu_en_decode, mul_en_decode, memread_en_decode, memwrite_en_decode, is_byte_decode;
    wire regwrite_mem_en_decode, regwrite_alu_en_decode, regwrite_mul_en_decode, is_branch_decode, is_jump_decode, is_exception_inst_decode, is_exception_decode;
    wire regwrite_alu_en_writeback, regwrite_mem_en_writeback, regwrite_mul_en_writeback;
    wire inst_finished_decode, stop_mem;
    wire[BITS_DATA-1:0] data_writeback, rs0_val_decode, rs1_val_decode, immediate_extended_decode, effective_address_decode, data_store_decode;
    wire[BITS_REG_SELECT-1:0] rs0_decode, rs1_decode, rd_decode, rd_writeback;
    wire[BITS_ALU_OPCODE-1:0] alu_opcode_decode;
    wire[2:0] branch_mask_decode;
    wire[BITS_ROB_ENTRY_SELECT-1:0] rob_entry_decode;
    wire[BITS_EXCEPTION-1:0] exception_type_inst_decode, exception_type_decode;
    wire tlb_we_decode;
    wire [BITS_REG_SELECT-1:0] instruction_rs0_decode, instruction_rs1_decode;
    
    /******** ALU wires *********/
    wire alu_en_ex, regwrite_alu_en_ex, z_flag, uflow_flag, oflow_flag, is_branch_ex, is_jump_ex, gt_ex, lt_ex, eq_ex;
    wire[BITS_DATA-1:0] op0_ex, op1_ex, alu_result_ex, rs1_val_ex, immediate_extended_ex;
    wire[BITS_REG_SELECT-1:0] rd_ex;
    wire[BITS_ALU_OPCODE-1:0] alu_opcode_ex;
    wire[BITS_PC-1:0] save_pc_ex; //This has pc
    wire[2:0] branch_mask_ex;
    wire use_rs1_ex;
    wire[BITS_ROB_ENTRY_SELECT-1:0] rob_entry_alu; 
    
    /******** MEM wires *********/
    wire[BITS_DATA-1:0] effective_address_mem, data_store_mem, memout_mem, data_mem, mem_address_in;
    wire[BITS_EXCEPTION-1:0] exception_type_decode_mem, exception_type_inst_mem, exception_type_mem;
    wire[BITS_REG_SELECT-1:0] rd_mem;
    wire is_byte_mem, memread_en_mem, memwrite_en_mem, is_exception_decode_mem, is_exception_inst_mem, is_exception_mem;
    wire[BITS_ROB_ENTRY_SELECT-1:0] rob_entry_mem;
    wire valid, mem_write_from_rob, mem_is_byte_instruction, mem_finished;
    wire [BITS_DATA-1:0] address_mem_in;
    wire [BITS_DATA-1:0] mem_data_write_rob;
    
    
    /******** MUL wires ********/
    wire mul_en_ex, regwrite_mul_en_ex, mul_z_flag_ex, mul_finished;
    wire[BITS_DATA-1:0] mul_op0_ex, mul_op1_ex, mul_result_ex;
    wire[BITS_REG_SELECT-1:0] rd_ex_m;
    wire[BITS_PC-1:0] save_pc_ex_m;
    wire[BITS_ROB_ENTRY_SELECT-1:0] rob_entry_inst_mul, rob_entry_mul;
        
    // Program Counter 
    Register #(.BITS_SIZE(BITS_PC)) pc (
        .clk(clk),
        .enable(invalidate_pipeline | (~hazard_fetch & ~stall_for_int) | reset),
        .reset(1'b0),
        .din(pc_in),
        .dout(pc_out)
    );
    
    // Status register
    Register #(.BITS_SIZE(BITS_PWS)) pws (
        .clk(clk),
        .enable(reset | (invalidate_pipeline | (jump_to_int && ~hazard_fetch) | restore_context) | csr_we_from_inst[`CSR_PWS]),
        .reset(1'b0),
        .din(status_in),
        .dout(status_out)
    );
    // Previous status register for restoring context
    Register #(.BITS_SIZE(BITS_PWS)) prev_pws (
        .clk(clk),
        .enable(invalidate_pipeline | jump_to_int),
        .reset(reset),
        .din(status_out),
        .dout(prev_status_out)
    );
    
    // Exception program Counter 
    Register #(.BITS_SIZE(BITS_PC)) epc (
        .clk(clk),
        .enable(invalidate_pipeline | (jump_to_int && ~hazard_fetch)),
        .reset(reset),
        .din(invalidate_pipeline?rob_pc:pc_out),
        .dout(epc_out)
    );

    // Exception cause 
    Register #(.BITS_SIZE(BITS_EXCEPTION)) ecause (
        .clk(clk),
        .enable(invalidate_pipeline),
        .reset(reset),
        .din(rob_exception_type),
        .dout(ecause_out)
    );
    
    // Bad virtual address cause of exception
    Register #(.BITS_SIZE(BITS_DATA)) badvaddr (
        .clk(clk),
        .enable(invalidate_pipeline),
        .reset(reset),
        .din(rob_address_mem),
        .dout(bad_virtual_addr_out)
    );
    
    // Load interrupt registers if interrupts are enabled
    //Interrupt register
    Register #(.BITS_SIZE(BITS_INTERRUPT)) itype_reg ( 
            .clk(clk),
            .enable((interrupt_enable & status_out[IE_BIT]) | csr_we_from_inst[`CSR_ITYPE]),
            .reset(reset),
            .din(interrupt_type),
            .dout(interrupt_type_out)
    );

    // Pending interrupt register
    Register #(.BITS_SIZE(1)) pir ( 
            .clk(clk),
            .enable((~pending_interrupt & status_out[IE_BIT]) | csr_we_from_inst[`CSR_PEND_INT]),
            .reset(reset | (jump_to_int && ~hazard_fetch)),
            .din(interrupt_enable),
            .dout(pending_interrupt)
    );
    
    // Pointer to the page table
    Register #(.BITS_SIZE(BITS_DATA)) ept ( 
            .clk(clk),
            .enable(csr_we_from_inst[`CSR_EPT]),
            .reset(reset),
            .din(page_table_pointer_in),
            .dout(page_table_pointer_out)
    );
    
    
    // Data out from csr registers zero-extended
    assign csr_outs[`CSR_PC] = pc_out;
    assign csr_outs[`CSR_PWS] = {{BITS_DATA-BITS_PWS{1'b0}},status_out};
    assign csr_outs[`CSR_EPC] = epc_out;
    assign csr_outs[`CSR_ECAUSE] = {{BITS_DATA-BITS_EXCEPTION{1'b0}},ecause_out};
    assign csr_outs[`CSR_BADVADDR] = bad_virtual_addr_out;
    assign csr_outs[`CSR_ITYPE] = {{BITS_DATA-BITS_INTERRUPT{1'b0}},interrupt_type_out};
    assign csr_outs[`CSR_PEND_INT] = {{30{1'b0}},pending_interrupt};
    assign csr_outs[`CSR_EPT] = {page_table_pointer_out};
    
    // Dout from csr register
    assign csr_selected = csr_outs[csr_reg_num_decode];
    //Enables for writing into csr register
    generate
        genvar s;
        for (s=0; s<8; s=s+1)begin:enable_for_csr_write
            assign csr_we_from_inst[s] = |csr_en_write_decode && (s == csr_reg_num_decode) && ~invalid_decode && ~is_exception_decode;
        end
    endgenerate
    // Select correct operation for data in
    assign csr_possible_data_in[0] = csr_selected; //No write
    assign csr_possible_data_in[1] = rs0_val_decode_final; //Write whole register
    assign csr_possible_data_in[2] = rs0_val_decode_final | csr_selected; //Set bits
    assign csr_possible_data_in[3] = ~rs0_val_decode_final & csr_selected; //Clear bits. If 1 in rs0, bit must be 0 in csr
    assign csr_data_in = csr_possible_data_in[csr_en_write_decode];
    
     /***********************************************************************************
     ************************************** Fetch ***************************************
     ************************************************************************************/

    // Pipeline full if write_enable, read_enable mul_enable or reg_dst != r0 
    
    // CHECK pc_out when no interrupt
    
    // Wire that indicates the apperance of an interrupt (interrupt_active = 1)
    assign available_interrupt = pending_interrupt & status_out[IE_BIT]; //If interrupts are enabled by processor
    assign stall_for_int = available_interrupt & (~rob_empty | (is_branch_decode | is_jump_decode | is_branch_ex | is_jump_ex));
    assign jump_to_int = available_interrupt & rob_empty;
    assign pc_p4 = pc_out+4; 
    // TODO: Check priority list to adapt the assignment of next_pc (cond_1?if:(cond_2?elseif:else))
    assign next_pc = z?branch_pc:(available_interrupt)?`INTERRUPT_PC:restore_context?epc_out:csr_we_from_inst[`CSR_PC]?csr_data_in:pc_p4;
    assign pc_in = reset?`RESET_PC:invalidate_pipeline?`EXCEPTION_PC:next_pc;
    assign hazard_fetch = ~inst_hit | hazard_decode;
    
    // Status is previous status if restoring context, or higher privilege mode, or the status word if no changes
    // At reset, mode = 0, and interrupts enabled          
    assign status_in = reset?{1'b0,1'b1,1'b0}:
                        restore_context?prev_status_out:
                       (invalidate_pipeline | jump_to_int)?{1'b0,1'b0,status_out[2]}:
                        csr_we_from_inst[`CSR_PWS]?csr_data_in:{exec_mode,2'b10}; //Status word 
    assign page_table_pointer_in = csr_we_from_inst[`CSR_EPT]?csr_data_in:`EPT;
    assign exec_mode = status_out[EXEC_MODE_BIT];//Execution mode
    assign is_exception_fetch = 1'b0;
    assign exception_type_fetch = 0;
    assign take_exception = (rob_is_exception & rob_is_finished);
    assign invalidate_pipeline =  take_exception;
    

    /***********************************************************************************
    ************************************** Decode **************************************
    ************************************************************************************/
    // FETCH to DECODE
    Register_F_to_D #(.BITS_ADDRESS(BITS_PC),.BITS_EXCEPTION(BITS_EXCEPTION)) Stage_F_D(
        .clk(clk),
        .enable(~hazard_decode),
        .reset(reset),
        .fetched_instruction_in(~hazard_fetch?fetched_inst:`NOP),
        .fetched_instruction_out(encoded_inst),
        .save_pc_in(pc_out),
        .save_pc_out(save_pc_decode),
        .is_exception_in(is_exception_fetch),
        .is_exception_out(is_exception_fetch_decode),
        .exception_type_in(exception_type_fetch),
        .exception_type_out(exception_type_fetch_decode),
        .invalidate_in(z | hazard_fetch | invalidate_pipeline | restore_context  | ((jump_to_int || stall_for_int) && ~take_exception)),
        .invalidate_out(invalid_decode)
    );
   
    Register #(.BITS_SIZE(1)) previous_reset(
         .clk(clk),
         .reset(1'b0),
         .enable(1'b1),
         .din(reset),
         .dout(prev_reset)
    );
    
    assign hazard_decode = no_bypass // Cannot take data from bypass 
            || rob_full
            || ((memread_en_decode | memwrite_en_decode) && stop_mem);
    assign stop_mem = rob_full //Cannot push instruction
                        ||((memread_en_decode | memwrite_en_decode) && rob_store_coming) // Store comming and we are mem instruction
                        || (~data_hit && (memread_en_mem || memwrite_en_mem)); // We are memory instruction and previous mem instruction missed
    assign push_rob = ~reset && ~prev_reset && ~hazard_decode && ~invalid_decode && ~z;
    // Pop rob if instruction has finished, there are instructions and it is not committing store, or 
    // committing store is hit in cache
    assign pop_rob = rob_is_finished && ~rob_empty && (~rob_writeMem_en || (rob_writeMem_en & data_hit));
    
    /***** Bypasses for: PRODUCT - CONSUMER dependencies *****/
    assign bypass_decode_mem_r0 = ((is_branch_decode || alu_en_decode || mul_en_decode || memwrite_en_decode || memread_en_decode) && 
                                    memread_en_mem && rd_mem == rs0_decode)?1'b1:1'b0;
    assign bypass_decode_mem_r1 =((is_branch_decode || alu_en_decode || mul_en_decode || memwrite_en_decode) && 
                                    memread_en_mem && rd_mem == rs1_decode)?1'b1:1'b0;
    assign bypass_decode_mul_r0 = ((is_branch_decode ||alu_en_decode || mul_en_decode || memwrite_en_decode || memread_en_decode) 
                                   && mul_finished && mul_en_ex & rd_ex_m == rs0_decode)?1'b1:1'b0;
    assign bypass_decode_mul_r1 =((is_branch_decode || alu_en_decode || mul_en_decode || memwrite_en_decode) && 
                                    mul_finished && mul_en_ex && rd_ex_m == rs1_decode)?1'b1:1'b0;        
    assign bypass_decode_alu_r0 = ((is_branch_decode || alu_en_decode || mul_en_decode || memwrite_en_decode || memread_en_decode) &&
                                    alu_en_ex && rd_ex == rs0_decode)?1'b1:1'b0;
    assign bypass_decode_alu_r1 =((is_branch_decode || alu_en_decode || mul_en_decode || memwrite_en_decode) && 
                                    alu_en_ex && rd_ex == rs1_decode)?1'b1:1'b0;
    /******* Check if bypass is possible *********/
    assign no_bypass = ~(bypass_decode_mem_r0 || bypass_decode_mem_r1 || bypass_decode_mul_r0 
    || bypass_decode_mul_r1 || bypass_decode_mul_r1 || bypass_decode_alu_r0 || bypass_decode_alu_r1) &&
    ((mul_en_ex && ~mul_finished && (rd_ex_m==rs0_decode || rd_ex_m==rs1_decode)) 
    || ((memread_en_mem || memwrite_en_mem) && ~data_hit && (rd_mem==rs0_decode || rd_mem==rs1_decode)));
    /****** Value to use in ALU - MUL - MEM (bypass or register file value ******/
    assign rs0_val_decode_final = bypass_decode_mul_r0?mul_result_ex:
                                  bypass_decode_alu_r0?alu_result_ex:
                                  bypass_decode_mem_r0?memout_mem:
                                  bypass_decode_rob_r0?data_bypass_rob_r0:rs0_val_decode;

    assign rs1_val_decode_final = bypass_decode_mul_r1?mul_result_ex:
                                  bypass_decode_alu_r1?alu_result_ex:
                                  bypass_decode_mem_r1?memout_mem:
                                  bypass_decode_rob_r1?data_bypass_rob_r1:rs1_val_decode;
    
    assign rs0_decode = invalid_decode?{BITS_REG_SELECT{1'b0}}:instruction_rs0_decode;
    assign rs1_decode = invalid_decode?{BITS_REG_SELECT{1'b0}}:instruction_rs1_decode;
    
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
        .regwrite_mem_en(regwrite_mem_en_decode),
        .regwrite_alu_en(regwrite_alu_en_decode),
        .regwrite_mul_en(regwrite_mul_en_decode),
        .immediate(immediate_extended_decode),
        .memread_en(memread_en_decode),
        .memwrite_en(memwrite_en_decode),
        .mul_en(mul_en_decode),
        .is_byte(is_byte_decode),
        .is_branch(is_branch_decode),
        .branch_mask(branch_mask_decode),
        .is_jump(is_jump_decode),
        .is_exception(is_exception_inst_decode),
        .exception_type(exception_type_inst_decode),
        .hazard(hazard_decode),
        .execution_mode(exec_mode),
        .restore_context(restore_context),
        .read_from_csr(csr_en_read_decode),
        .write_to_csr(csr_en_write_decode),
        .csr_reg_num(csr_reg_num_decode),
        .write_tlb(tlb_we_decode)
    );
    
    // Registers File with 2 true read ports, 1 write port
    Register_File #(.BITS_REG_SELECT(BITS_REG_SELECT),.BITS_REG_DATA(BITS_DATA)) register_file (
        .clk(clk),
        .reset(reset),
        .rs(rs0_decode),
        .rt(rs1_decode),
        .dout0(rs0_val_decode),
        .dout1(rs1_val_decode),
        .din(rob_data),
        .reg_in(rob_rd),
        .we(rob_update_reg)
    );
    
    assign effective_address_decode = (memread_en_decode | memwrite_en_decode)?(rs0_val_decode_final+immediate_extended_decode):32'b0;
    assign data_store_decode = memwrite_en_decode?rs1_val_decode_final: //If we write in memory, we are store so take reg value
                                csr_en_read_decode?csr_outs[csr_reg_num_decode]:32'b0; //We take value from csr or 0
    assign is_exception_decode = ~prev_reset && ~hazard_decode && (is_exception_fetch_decode | is_exception_inst_decode);
    //Exceptions in previous stage are more important
    assign exception_type_decode = (~prev_reset && is_exception_fetch_decode)?exception_type_fetch_decode:exception_type_inst_decode;  
    assign inst_finished_decode = is_exception_decode | restore_context | csr_en_read_decode;
    
    /***********************************************************************************
    ************************************** ALU *****************************************
    ************************************************************************************/
   
    // DECODE to ALU
    Register_D_to_A #(.BITS_DATA(BITS_DATA),
        .BITS_ALU_OPCODE(BITS_ALU_OPCODE),
        .BITS_REG_SELECT(BITS_REG_SELECT),
        .BITS_PC(BITS_PC),
        .BITS_ROB_ENTRY_SELECT(BITS_ROB_ENTRY_SELECT)) stage_D_A(
        .clk(clk),
        .reset(reset),
        .enable(1'b1),//~stop_alu), 
        // Decode -> In
        .alu_en_in(~z & ~invalid_decode & alu_en_decode & ~is_exception_decode),
        .is_branch_in(~z & ~invalid_decode & is_branch_decode),
        .branch_mask_in((z | invalid_decode)?{3{1'b0}}:branch_mask_decode),
        .is_jump_in((~z & ~invalid_decode) & is_jump_decode),
        .alu_opcode_in(alu_opcode_decode),
        .rs0_val_in((z | invalid_decode)?{BITS_DATA-1{1'b0}}:rs0_val_decode_final), 
        .rs1_val_in((z | invalid_decode)?{BITS_DATA-1{1'b0}}:rs1_val_decode_final), 
        .immediate_in((z | invalid_decode)?{BITS_DATA-1{1'b0}}:immediate_extended_decode),
        .use_rs1_in((~z & ~invalid_decode) & use_rs1_decode),
        .regwrite_en_in((~z & ~invalid_decode) & regwrite_alu_en_decode),
        .rd_in((z | invalid_decode)?{BITS_REG_SELECT-1{1'b0}}:rd_decode),
        .save_pc_in(save_pc_decode),
        .rob_entry_in(rob_entry_decode),
        .invalidate_in(invalidate_pipeline | invalid_decode),
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
        .regwrite_en_out(regwrite_alu_en_ex),
        .rd_out(rd_ex),    
        .save_pc_out(save_pc_ex),
        .rob_entry_out(rob_entry_alu),
        .invalidate_out(invalid_alu)
    );
    
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
 
    assign z = (is_branch_ex & ~invalidate_pipeline & ~invalid_alu)?(|({gt_ex,lt_ex,eq_ex} & branch_mask_ex)):0;   
    assign branch_pc = save_pc_ex + immediate_extended_ex;    
    
    /***********************************************************************************
    ************************************** Memory **************************************
    ************************************************************************************/   
          
    // DECODE to MEM
    Register_D_to_M #(.BITS_ADDRESS(BITS_DATA),.BITS_EXCEPTION(BITS_EXCEPTION),
     .BITS_ROB_ENTRY_SELECT(BITS_ROB_ENTRY_SELECT)) stage_D_M(
       .clk(clk),
       .reset(reset),
       // Stop if we cannot access memory
       .enable(~stop_mem),     
       // Decode -> In
       .effective_address_in(effective_address_decode),
       .is_byte_in(is_byte_decode),
       .memread_en_in((~z & ~invalid_decode) & memread_en_decode & ~is_exception_decode),
       .memwrite_en_in((~z & ~invalid_decode) & memwrite_en_decode & ~is_exception_decode),
       .data_store_in((z | invalid_decode)?{BITS_DATA-1{1'b0}}:data_store_decode),
       .is_exception_in(is_exception_decode & (memread_en_decode | memwrite_en_decode)),
       .exception_type_in(exception_type_decode),
       .rob_entry_in(rob_entry_decode),
       .reg_rw_in((z | invalid_decode)?{BITS_REG_SELECT-1{1'b0}}:rd_decode),
       .invalidate_in(invalidate_pipeline | invalid_decode),
        // Out -> Mem
       .effective_address_out(effective_address_mem),
       .is_byte_out(is_byte_mem),
       .memread_en_out(memread_en_mem),
       .memwrite_en_out(memwrite_en_mem),
       .data_store_out(data_store_mem),
       .is_exception_out(is_exception_decode_mem),
       .exception_type_out(exception_type_decode_mem),
       .rob_entry_out(rob_entry_mem),
       .reg_rw_out(rd_mem),
       .invalidate_out(invalid_mem)
    );
    
    assign is_exception_inst_mem = 1'b0; //DUMMY, will be TLB miss
    assign exception_type_inst_mem = 0;
    //assign data_hit = 1'b1; //DUMMY substitute memory

    /* Signals to write in memory, they are taken from Reorder Buffer */
    assign mem_write_from_rob = rob_writeMem_en & ~rob_is_exception & rob_is_finished;
    assign address_mem_in = mem_write_from_rob?rob_address_mem:effective_address_mem;
    assign mem_data_write_rob = mem_write_from_rob?rob_data:data_store_mem;
    assign mem_is_byte_instruction = mem_write_from_rob?rob_is_byte:is_byte_mem;
    assign mem_finished = (memread_en_mem | memwrite_en_mem) & data_hit;
    
    Memory_System memory_hierarchy(
        .clk(clk),
        .reset(reset),
        .addr_data(address_mem_in),
        .addr_inst(pc_out),
        .hit_data(data_hit),
        .hit_inst(inst_hit),
        .is_byte(mem_is_byte_instruction),
        .we(rob_writeMem_en & ~invalidate_pipeline),
        .re_data(memread_en_mem & ~invalidate_pipeline),
        .re_inst(~reset), //Always fetching instruction
        .dout_data(memout_mem),
        .dout_inst(fetched_inst),
        .din(mem_data_write_rob),
        .invalidate_req_inst(z | invalidate_pipeline)
    );
    
    assign data_mem = memwrite_en_mem?data_store_mem:memout_mem;
    assign is_exception_mem = (is_exception_decode_mem | is_exception_inst_mem); //As always, prev exceptions are more important
    assign exception_type_mem = is_exception_decode_mem?exception_type_decode_mem:exception_type_inst_mem;
    
    /***********************************************************************************
    ************************************** MUL **************************************
    ************************************************************************************/    
    
    // DECODE to MUL
    Register_D_to_MUL #(.BITS_DATA(BITS_DATA),
    .BITS_REG_SELECT(BITS_REG_SELECT),
    .BITS_PC(BITS_PC),
    .BITS_ROB_ENTRY_SELECT(BITS_ROB_ENTRY_SELECT)) Stage_D_MUL (
        .clk(clk),
        .reset(reset),
        .enable(1'b1),
        .mul_en_in((~z & invalid_decode) & mul_en_decode & ~is_exception_decode),
        .rs0_val_in((z | invalid_decode)?{BITS_DATA-1{1'b0}}:rs0_val_decode_final),
        .rs1_val_in((z | invalid_decode)?{BITS_DATA-1{1'b0}}:rs1_val_decode_final),
        .regwrite_en_in((~z & ~invalid_decode) & regwrite_mul_en_decode),
        .rd_in((z | invalid_decode)?{BITS_REG_SELECT-1{1'b0}}:rd_decode),
        .save_pc_in((z | invalid_decode)?{BITS_DATA-1{1'b0}}:save_pc_decode),
        .rob_entry_in(rob_entry_decode),
        .mul_en_out(mul_en_ex),
        .rs0_val_out(mul_op0_ex),
        .rs1_val_out(mul_op1_ex),
        .regwrite_en_out(regwrite_mul_en_ex),
        .rd_out(rd_ex_m),
        .save_pc_out(save_pc_ex_m),
        .rob_entry_out(rob_entry_inst_mul),
        .invalidate_in(invalidate_pipeline | invalid_decode | z),
        .invalidate_out(invalid_initial_mul)
    );
    
    Multiplier #(.BITS_DATA(BITS_DATA),
    .BITS_ROB_ENTRY_SELECT(BITS_ROB_ENTRY_SELECT)) mul(
        .enable(mul_en_ex),
        .reset(reset),
        .clk(clk),
        .din0(mul_op0_ex),
        .din1(mul_op1_ex),
        .dout(mul_result_ex),
        .zero(mul_z_flag_ex),
        .finished(mul_finished),
        .rob_entry_in(rob_entry_inst_mul),
        .rob_entry_out(rob_entry_mul),
        .invalidate_in(invalidate_pipeline | invalid_initial_mul),
        .invalidate_out(invalid_final_mul)
    );
     
    /* Reorder Buffer connected to MEM and RegFile as outputs */
    ROB #(.BITS_ENTRY_SELECT(BITS_ROB_ENTRY_SELECT),
    .BITS_DATA(BITS_DATA),
    .BITS_ADDRESS(BITS_DATA),
    .BITS_REG_SELECT(BITS_REG_SELECT),
    .BITS_EXCEPTION(BITS_EXCEPTION)) reorder_buffer(
        .clk(clk),
        .reset(reset),
        .push(push_rob),
        .pop(pop_rob),
        .empty(rob_empty),
        .full(rob_full),
        .check_rs0(rs0_decode), // Register to check
        .check_rs1(rs1_decode),
        .hit_rs0(bypass_decode_rob_r0), // Possible bypass
        .hit_rs1(bypass_decode_rob_r1),
        .bypass_rs0_data(data_bypass_rob_r0), // Data to bypass
        .bypass_rs1_data(data_bypass_rob_r1), 
        .push_finished(inst_finished_decode), //If it is a store, then it is already finished
        .push_is_exception(is_exception_decode),
        .push_exception_type(exception_type_decode),
        .push_writeReg_en(regwrite_alu_en_decode | regwrite_mem_en_decode | regwrite_mul_en_decode),
        .push_rd(rd_decode),
        .push_pc(save_pc_decode),
        .push_data(data_store_decode), // if store, then we need that data, otherwise 0
        .push_address_mem(effective_address_decode),
        .push_is_byte(is_byte_decode),
        .push_writeMem_en(memwrite_en_decode),
        .commit_finished(rob_is_finished),
        .commit_is_exception(rob_is_exception),
        .commit_exception_type(rob_exception_type),
        .commit_writeReg_en(rob_writeReg_en),
        .commit_rd(rob_rd),
        .commit_pc(rob_pc),
        .commit_data(rob_data),
        .commit_address_mem(rob_address_mem),
        .commit_is_byte(rob_is_byte),
        .commit_writeMem_en(rob_writeMem_en),
        //operation finishes in the same cycle, we can use this signal as finished, additionally we can check whether if it is branch/jump
        .alu_inst_finished(alu_en_ex & ~invalidate_pipeline & ~invalid_alu), //& ~stop_alu), 
        .alu_inst_rob_entry(rob_entry_alu),
        .alu_inst_data(alu_result_ex),
        .mem_inst_finished(mem_finished), 
        .mem_inst_rob_entry(rob_entry_mem),
        .mem_inst_data(data_mem),
        .mem_inst_is_exception(is_exception_mem),
        .mem_inst_exception_type(exception_type_mem),
        .mul_inst_finished(mul_finished & ~invalidate_pipeline & ~invalid_final_mul),
        .mul_inst_rob_entry(rob_entry_mul),
        .mul_inst_data(mul_result_ex),
        .mem_inst_effective_addr(address_mem_in),
        .next_entry_number(rob_entry_decode),
        .recover_exception(invalidate_pipeline),
        .store_coming(rob_store_coming)
    ); 
    
    assign rob_update_reg = rob_writeReg_en && rob_is_finished && ~rob_is_exception;
    
endmodule
