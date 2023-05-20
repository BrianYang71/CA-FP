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
        wire mem_cen, mem_wen; // mem_cen先不管(cache)
        wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
        wire mem_stall;

    // Instruction associated
        wire [4:0] rs1;
        wire [4:0] rs2;
        wire [4:0] rd;
        wire [6:0] opcode;
        wire [2:0] funct3;
        wire [6:0] funct7;
        wire [11:0] I_type_imm;
        wire [11:0] S_type_imm;
        wire [12:0] B_type_imm; 
        wire [19:0] U_type_imm;
        wire [20:0] J_type_imm; 

    // Type Control associated
        wire is_branch;
        wire [1:0] mem_to_reg;
        wire [1:0] pc_ctrl;
        wire mem_read;
        wire mem_write;
        wire alu_src;
        wire reg_write;

    // Memory associated
        wire [31:0] rs1_data;
        wire [31:0] rs2_data;
        wire [31:0] rd_data;



// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment
        assign PC = o_IMEM_addr;
        assign mem_wdata = rs2_data;
        assign mem_rdata = rd_data;

    // Instruction associated
        assign rs1 = i_IMEM_data[19:15];
        assign rs2 = i_IMEM_data[24:20];
        assign rd = i_IMEM_data[11:7];
        assign opcode = i_IMEM_data[6:0];
        assign funct3 = i_IMEM_data[14:12];
        assign funct7 = i_IMEM_data[31:25];
        assign I_type_imm = i_IMEM_data[31:20];
        assign S_type_imm = {i_IMEM_data[31:25], i_IMEM_data[11:7]};
        assign B_type_imm = {i_IMEM_data[31], i_IMEM_data[7], i_IMEM_data[30:25], i_IMEM_data[11:8], 1'b0};
        assign U_type_imm = i_IMEM_data[31:12];
        assign J_type_imm = {i_IMEM_data[31], i_IMEM_data[19:12], i_IMEM_data[20], i_IMEM_data[30:21], 1'b0};

    // Type Control associated
        assign mem_wen = reg_write;

    // ALU Control & ALU associated
        wire [3:0] alu_ctrl;
        wire [31:0] result_out;
        wire alu_ready; 
        reg [31:0] alu_B_input;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (mem_wen),          
        .rs1    (rs1),                
        .rs2    (rs2),                
        .rd     (rd),                 
        .wdata  (rd_data),             
        .rdata1 (rs1_data),           
        .rdata2 (rs2_data)
    );

    type_ctrl type_C(
        .opcode(opcode),
        .is_branch(is_branch),
        .mem_to_reg(mem_to_reg),
        .pc_ctrl(pc_ctrl),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .alu_src(alu_src),
        .reg_write(reg_write)
    );

    ALU alu(
        .clk(i_clk),
        .rst_n(i_rst_n),
        .A_input(rs1_data),
        .B_input(alu_B_input),
        .alu_ctrl(alu_ctrl),
        .result_out(result_out),
        .alu_ready(alu_ready)
    );

    ALUControl alu_C(
        .opcode(opcode),
        .funct3(funct3),
        .funct7(funct7),
        .alu_ctrl(alu_ctrl)
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
    
    // PC address
    always @(*) begin
        case(mem_to_reg)
            `MEM2REG_PC_PLUS_4 : wdata = PC + 4;
            `MEM2REG_ALU : wdata = result_out;
            `MEM2REG_MEM : wdata = //還沒寫
            `MEM2REG_PC_PLUS_IMM : wdata = ;
            default : wdata = 0;
        endcase
    end

    // Immediate

    // ALU input
    always @(*) begin
        case(alu_src)
            `FROM_RS2 : alu_B_input = {20'b0, I_type_imm};
            `FROM_IMM : alu_B_input = rs2_data;
            default :
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

module ALU(
    input clk,
    input rst_n,
    input [31:0] A_input,
    input [31:0] B_input,
    input [3:0] alu_ctrl,
    output [31:0] result_out,
    output alu_ready
);
    reg [31:0] alu_result;
    wire [63:0] muldiv_result;
    wire valid;
    wire mode;
    wire ready;

    assign alu_ready = (alu_ctrl == `MUL) ? ready : 1;
    assign result_out = (alu_ready) ? alu_result : 0;

    // valid, mode, ready(in module MulDiv)
    assign valid = (alu_ctrl == `MUL);
    assign mode = 0;

    MULDIV_unit muldiv(
        .clk(clk),
        .rst_n(rst_n),
        .valid(valid),
        .mode(mode),
        .A_input(A_input),
        .B_input(B_input),
        .ready(ready),
        .mul_output(muldiv_result) 
    );

    always @(*) begin
        case(alu_ctrl)
            `ADD : alu_result = A_input + B_input;
            `SUB : alu_result = A_input - B_input;
            `XOR : alu_result = A_input ^ B_input; 
            `AND : alu_result = A_input & B_input;
            `ADDI : alu_result = A_input + B_input;
            `SLTI : alu_result = (A_input < B_input) ? 1 : 0;
            `SLLI : alu_result = A_input << B_input;
            `SRAI : alu_result = $signed(A_input) >>> $signed(B_input);
            `MUL : alu_result = muldiv_result[31:0];
            default : alu_result = 0;
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
                        3'b000: alu_ctrl = (funct7 == 0 ? `ADD : `SUB);
                        3'b100: alu_ctrl = `XOR;
                        3'b111: alu_ctrl = `AND;
                    endcase
                end
            end
            `I_TYPE : begin
                case(funct3)
                    3'b000: alu_ctrl = `ADDI;   
                    3'b010: alu_ctrl = `SLTI;    
                    3'b001: alu_ctrl = `SLLI;    
                    3'b101: alu_ctrl = `SRAI;
                endcase
            end
            // `B_TYPE : alu_ctrl = `SUB; // beq
            default: alu_ctrl = `ADD;
        endcase
    end
endmodule

module MULDIV_unit(
    // TODO: port declaration
    input clk,
    input rst_n,
    input valid,
    input mode,
    input [31:0] A_input,
    input [31:0] B_input,
    output ready,
    output [63:0] mul_output
    );
    // Todo: HW2
    parameter IDLE = 2'd0;
    parameter MUL_state = 2'd1;
    parameter OUT = 2'd2;

    reg [1:0] state, next_state;
    reg [4:0] counter, next_counter;
    reg [31:0] first;
    reg [63:0] shift_reg, next_shift_reg;
    reg [64:0] buffer;


    assign mul_output = shift_reg;
    assign ready = (state==OUT) ? 1 : 0;

    // STATE
    always @(*) begin
        case(state)
            IDLE : begin
                if(valid) next_state = MUL;
                else next_state = IDLE;
            end
            MUL : next_state = (counter==31) ? OUT : MUL; 
            OUT : next_state = IDLE;
            default : next_state = IDLE;
        endcase
    end    

    // COUNTER
    always @(*) begin
        if(state==MUL) next_counter = counter + 1;
        else next_counter = 0;
    end

    // ALU output
    always @(*) begin
        first = (shift_reg[0])?(B_input):(32'b0);
    end

    // Shift Register
    always @(*) begin
        case(state)
            IDLE : begin
                if(valid) next_shift_reg = {32'b0, A_input};
                else next_shift_reg = 0;
            end
            MUL : begin
                buffer = ({first, 32'b0} + shift_reg) >> 1;
                next_shift_reg = buffer[63:0];
            end
            default : next_state = IDLE;
        endcase
    end

    // Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            counter <= 0;
            shift_reg <= 0;
        end
        else begin
            state <= next_state;
            counter <= next_counter;
            shift_reg <= next_shift_reg;
        end
    end

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