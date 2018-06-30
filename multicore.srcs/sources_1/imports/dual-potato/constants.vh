/** INSTRUCTIONS */
//Accoding to ISA: NOP is encoded as ADDI r0,r0,0
// 32'b  FN7    RS1   RS0  FN3   RD   OPCODE
// 32'b???????_?????_?????_???_?????_???????

`define BYTE 8 

`define LOAD 8'b?0000011 //I-IMM
`define STORE 8'b?0100011 //S_IMM
`define ARIT_IMM 8'b?0010011 //I-IMM
`define ARIT_REG 8'b00110011 //Difference between MUL and ARIT is in fn7[0]
`define MUL 8'b10110011
`define IRET 8'b00000
`define TLBWRITE 8'b00000
`define BRANCH 8'b?1100011 //B-IMM
`define JAL 8'b?1101111   //J-IMM
`define CONTROL_TRANSF 7'b110??11 
`define NOP 32'h00000013
`define SYSTEM 8'b?1110011
// Next instructions share opcode which is SYSTEM
`define MRET 16'h3020  // Last 12 bits of instruction concatenate to fn3 
`define FENCE 16'h1050 //Las 12 bits of instruciton  concatenate to fn3 
`define CSRRW 16'h???1 // 1 indicates that instruction will read and write registers
`define CSRRS 16'h???2 // set bits on csr
`define CSRRC 16'h???3 // This only reads from csr

/** CSR register numbers */
`define CSR_PC 12'h000
`define CSR_PWS 12'h001
`define CSR_EPC 12'h002
`define CSR_ECAUSE 12'h003
`define CSR_BADVADDR 12'h004
`define CSR_ITYPE 12'h005
`define CSR_PEND_INT 12'h006
`define CSR_EPT 12'h007

/** EXCEPTION CODES according to ISA */
`define INSTRUCTION_MISSALIGNED 5'h00
`define INSTRUCTION_FAULT 5'h01
`define ILLEGAL_INSTRUCTION 5'h02
`define PRIVILEGED_INSTRUCTION 5'h03
`define SYSTEM_CALL 5'h06
`define LOAD_MISALIGNED 5'h08
`define STORE_MISALIGNED 5'h09
`define LOAD_FAULT 5'h0A
`define STORE_FAULT 5'h0B
`define EXTERNAL_INTERRUPT_0 5'h10
`define EXTERNAL_INTERRUPT_1 5'h11

/** PC ADDRESSES */
`define RESET_PC0 32'h00010000
`define RESET_PC1 32'h00020000

/** COHERENCE UNIT **/
`define BITS_COHERENCE_STATES 3 // 5 states (INVALID, WAIT_BUS_INVALID, VALID, WAIT_BUS_VALID, IV_DATA)
// States for the coherence unit
`define BITS_STATES 3 // 5 states (RESET, READY, WAIT_BUS, MISS)
`define INVALID 0
`define WAIT_COHERENCE_BUS_INVALID 1
`define VALID 2
`define WAIT_COHERENCE_BUS_VALID 3
`define IV_DATA 4
// Message types
`define BITS_COHERENCE_MSG_TYPES 2
`define IDLE 0
`define EVICT_DATA 1
`define READ_DATA 2
`define WRITE_DATA 3



