//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//
module CHIP #(                                                                                  //
    parameter BIT_W = 32                                                                        //
)(                                                                                              //
    // clock                                                                                    //
        input               i_clk,                                                              //
        input               i_rst_n,                                                            //
    // instruction memory                                                                       //
        input  [BIT_W-1:0]  i_IMEM_data,                                                        //
        output [BIT_W-1:0]  o_IMEM_addr,                                                        //
        output              o_IMEM_cen,                                                         //
    // data memory                                                                              //
        input               i_DMEM_stall,                                                       //
        input  [BIT_W-1:0]  i_DMEM_rdata,                                                       //
        output              o_DMEM_cen,                                                         //
        output              o_DMEM_wen,                                                         //
        output [BIT_W-1:0]  o_DMEM_addr,                                                        //
        output [BIT_W-1:0]  o_DMEM_wdata                                                        //
);                                                                                              //
//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameters
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any declaration
    // Constants

// RISC-V format
`define R_TYPE 7'b0110011 // arithmatic/logical ops
`define I_TYPE 7'b0010011 // immediates
`define I_JALR 7'b1100111
`define I_LOAD 7'b0000011
`define S_TYPE 7'b0100011 // store
`define B_TYPE 7'b1100011 // branch
`define U_TYPE 7'b0010111 // upper immediates
`define UJ_JAL 7'b1101111

// ALUCtrl signal
`define ADD  4'b0000
`define ADDI  4'b0001
`define SUB  4'b0010
`define SLTI  4'b0011
`define SLLI  4'b0100
`define SRAI  4'b0101
`define XOR  4'b0110
`define AND  4'b0111
`define MUL  4'b1000

// MemtoReg
`define MEM2REG_PC_PLUS_4 2'b00
`define MEM2REG_ALU 2'b01
`define MEM2REG_MEM 2'b10
`define MEM2REG_PC_PLUS_IMM 2'b11

// PCCtrl
`define PCCTRL_PC_PLUS_IMM 2'b00
`define PCCTRL_RS1_PLUS_IMM 2'b01
`define PCCTRL_PC_PLUS_4 2'b10

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
        reg [BIT_W-1:0] PC, next_PC;
        wire mem_cen, mem_wen;
        wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
        wire mem_stall;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (wen),          
        .rs1    (rs1),                
        .rs2    (rs2),                
        .rd     (rd),                 
        .wdata  (wdata),             
        .rdata1 (rdata1),           
        .rdata2 (rdata2)
    );

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // Todo: any combinational/sequential circuit

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
        end
        else begin
            PC <= next_PC;
        end
    end
endmodule

module type_ctrl(
    input   [6:0]   opcode,
    output  reg   is_branch,
    output  reg   [1:0] mem_to_reg,
    output  reg   [1:0] pc_ctrl,
    output  reg   mem_read,
    output  reg   mem_write,
    output  reg   alu_src,
    output  reg   reg_write
);
    always @(*) begin
        case(opcode)
            `R_TYPE : begin
                is_branch   = 0;
                mem_to_reg  = `MEM2REG_ALU;
                pc_ctrl     = `PCCTRL_PC_PLUS_4;
                mem_read    = 0;
                mem_write   = 0;
                alu_src     = `FROM_RS2;
                reg_write   = 1;
            end
            `I_TYPE : begin
                is_branch   = 0;
                mem_to_reg  = `MEM2REG_ALU;
                pc_ctrl     = `PCCTRL_PC_PLUS_4;
                mem_read    = 0;
                mem_write   = 0;
                alu_src     = `FROM_IMM;
                reg_write   = 1;
            end
            `I_JALR : begin
                is_branch   = 0;
                mem_to_reg  = `MEM2REG_PC_PLUS_4;
                pc_ctrl     = `PCCTRL_RS1_PLUS_IMM;
                mem_read    = 0;
                mem_write   = 0;
                alu_src     = `FROM_IMM;
                reg_write   = 1;
            end
            `I_LOAD : begin
                is_branch   = 0;
                mem_to_reg  = `MEM2REG_MEM;
                pc_ctrl     = `PCCTRL_PC_PLUS_4;
                mem_read    = 1;
                mem_write   = 0;
                alu_src     = `FROM_IMM;
                reg_write   = 1;
            end
            `S_TYPE : begin
                is_branch   = 0;
                mem_to_reg  = `MEM2REG_MEM;
                pc_ctrl     = `PCCTRL_PC_PLUS_4;
                mem_read    = 0;
                mem_write   = 1;
                alu_src     = `FROM_IMM;
                reg_write   = 0;
            end
            `B_TYPE : begin
                is_branch   = 1;
                mem_to_reg  = `MEM2REG_ALU;
                pc_ctrl     = `PCCTRL_PC_PLUS_IMM;
                mem_read    = 0;
                mem_write   = 0;
                alu_src     = `FROM_RS2;
                reg_write   = 0;
            end
            `U_TYPE : begin
                is_branch   = 0;
                mem_to_reg  = `MEM2REG_PC_PLUS_IMM;
                pc_ctrl     = `PCCTRL_PC_PLUS_4;
                mem_read    = 0;
                mem_write   = 0;
                alu_src     = `FROM_RS2;
                reg_write   = 1;
            end
            `UJ_JAL : begin
                is_branch   = 0;
                mem_to_reg  = `MEM2REG_PC_PLUS_4;
                pc_ctrl     = `PCCTRL_PC_PLUS_IMM;
                mem_read    = 0;
                mem_write   = 0;
                alu_src     = `FROM_IMM;
                reg_write   = 1;
            end
            default : begin
                is_branch   = 0;
                mem_to_reg  = 0;
                pc_ctrl     = 0;
                mem_read    = 0;
                mem_write   = 0;
                alu_src     = 0;
                reg_write   = 0;
            end
        endcase
    end
endmodule

module ALUControl(
    input   [6:0]   opcode,
    input   [2:0]   funct3,
    input   [6:0]   funct7,
    output  reg [3:0]   alu_ctrl
);
    always @(*) begin 
        case(opcode)
            `R_TYPE : begin
                if(funct7 == 7'b0000001)
                    alu_ctrl = `MUL;
                else begin
                    case(funct3)
                        3'b000: alu_ctrl = funct7 == 0 ? `ADD : `SUB;
                        3'b001: alu_ctrl = `SLL;
                        3'b010: alu_ctrl = `SLT;
                        3'b011: alu_ctrl = `SLTU;
                        3'b100: alu_ctrl = `XOR;
                        3'b101: alu_ctrl = funct7 == 0 ? `SRL : `SRA;
                        3'b110: alu_ctrl = `OR;
                        3'b111: alu_ctrl = `AND;
                    endcase
                end
            end
            `I_TYPE : begin
                case(funct3)
                    3'b000: alu_ctrl = `ADD;    // addi
                    3'b001: alu_ctrl = `SLL;    // slli
                    3'b010: alu_ctrl = `SLT;    // slti
                    3'b011: alu_ctrl = `SLTU;   // sltiu
                    3'b100: alu_ctrl = `XOR;    // xori
                    3'b101: alu_ctrl = funct7 == 0 ? `SRL : `SRA; // srli, srai
                    3'b110: alu_ctrl = `OR;     // or
                    3'b111: alu_ctrl = `AND;    // andi
                endcase
            end
            `B_TYPE : alu_ctrl = `SUB; // beq
            default: alu_ctrl = `ADD;
        endcase
    end
    

endmodule

module Reg_file(i_clk, i_rst_n, wen, rs1, rs2, rd, wdata, rdata1, rdata2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input i_clk, i_rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] wdata;
    input [addr_width-1:0] rs1, rs2, rd;

    output [BITS-1:0] rdata1, rdata2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign rdata1 = mem[rs1];
    assign rdata2 = mem[rs2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (rd == i)) ? wdata : mem[i];
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module MULDIV_unit(
    // TODO: port declaration
    
    );
    // Todo: HW2


endmodule

module Cache#(
        parameter BIT_W = 32,
        parameter ADDR_W = 32
    )(
        input i_clk,
        input i_rst_n,
        // processor interface
            input i_proc_cen,
            input i_proc_wen,
            input [ADDR_W-1:0] i_proc_addr,
            input [BIT_W-1:0]  i_proc_wdata,
            output [BIT_W-1:0] o_proc_rdata,
            output o_proc_stall,
        // memory interface
            output o_mem_cen,
            output o_mem_wen,
            output [ADDR_W-1:0] o_mem_addr,
            output [BIT_W-1:0]  o_mem_wdata,
            input [BIT_W-1:0] i_mem_rdata,
            input i_mem_stall
    );

    //---------------------------------------//
    //          default connection           //
    assign o_mem_cen = i_proc_cen;        //
    assign o_mem_wen = i_proc_wen;        //
    assign o_mem_addr = i_proc_addr;      //
    assign o_mem_wdata = i_proc_wdata;    //
    assign o_proc_rdata = i_mem_rdata;    //
    assign o_proc_stall = i_mem_stall;    //
    //---------------------------------------//

    // Todo: BONUS
endmodule