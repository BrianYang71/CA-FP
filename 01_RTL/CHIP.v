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
// Branch signal
`define BEQ  3'b000
`define BNE  3'b001
`define BLT  3'b100
`define BGE  3'b101

// MemtoReg
`define MEM2REG_PC_PLUS_4 2'b00
`define MEM2REG_ALU 2'b01
`define MEM2REG_MEM 2'b10
`define MEM2REG_PC_PLUS_IMM 2'b11

// PCCtrl
`define PCCTRL_PC_PLUS_IMM 2'b00
`define PCCTRL_RS1_PLUS_IMM 2'b01
`define PCCTRL_PC_PLUS_4 2'b10

`define FROM_RS2 1'b0
`define FROM_IMM 1'b1

//Top FSM
`define s_IDLE 3'd0
`define s_INSTRU 3'd1
`define s_MEMORY 3'd2
`define s_WRITE 3'd3
`define s_READ 3'd4
`define s_ALU 3'd5
`define s_OUT 3'd6

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


// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
        reg [BIT_W-1:0] PC, next_PC;
        wire mem_cen, mem_wen; // mem_cen先不管(cache)
        wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
        reg [31:0] o_DMEM_addr_reg;
        reg [31:0] o_DMEM_wdata_reg;
        reg [2:0] s, next_s;

    // Instruction associated
        wire [4:0] rs1;
        wire [4:0] rs2;
        wire [4:0] rd;
        wire [6:0] opcode;
        wire [2:0] funct3;
        wire [6:0] funct7;

    // IMMGen output
        wire [31:0] IMMGen_out;

    // Type Control associated
        wire is_branch;
        wire [1:0] mem_to_reg;
        wire [1:0] pc_ctrl;
        wire mem_read;
        wire mem_write;
        wire alu_src;
        wire reg_write_or_not;
        // wire Reg_write;
        reg Reg_write;

    // Memory associated
        wire [31:0] rs1_data;
        wire [31:0] rs2_data;
        reg [31:0] rd_data;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment
        assign o_IMEM_addr = PC;
        assign o_IMEM_cen = (s != `s_OUT) ? 1 : 0;
        assign o_DMEM_cen = (mem_write | mem_read) && (s==`s_READ || s==`s_WRITE);
        assign o_DMEM_wen = mem_write && (s==`s_READ || s==`s_WRITE);
        assign o_DMEM_addr = (mem_to_reg  == `MEM2REG_MEM) ? (rs1_data + IMMGen_out) : 0;
        assign o_DMEM_wdata = (o_DMEM_wen) ? rs2_data : 0;


    // Instruction associated
        assign rs1 = i_IMEM_data[19:15];
        assign rs2 = i_IMEM_data[24:20];
        assign rd = i_IMEM_data[11:7];
        assign opcode = i_IMEM_data[6:0];
        assign funct3 = i_IMEM_data[14:12];
        assign funct7 = i_IMEM_data[31:25];
        
        // assign Reg_write = (reg_write_or_not && (s==`s_OUT)) ? 1 : 0;      

    // B-type Jump associated
        wire if_jump;
    // ALU Control & ALU associated
        wire [3:0] alu_ctrl;
        wire [31:0] result_out;
        wire alu_ready; 
        reg [31:0] alu_B_input;

        //reg [3:0] stalldelay, next_stalldelay;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (Reg_write),          
        .rs1    (rs1),                
        .rs2    (rs2),                
        .rd     (rd),                 
        .wdata  (rd_data),             
        .rdata1 (rs1_data),           
        .rdata2 (rs2_data)
    );

    IMMGen immgen(
        .instruction(i_IMEM_data),
        .immediate(IMMGen_out)
    );

    type_ctrl type_C(
        .opcode(opcode),
        .is_branch(is_branch),
        .mem_to_reg(mem_to_reg),
        .pc_ctrl(pc_ctrl),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .alu_src(alu_src),
        .reg_write(reg_write_or_not)
    );

    B_type_jump b_type_jump(
        .A_input(rs1_data),
        .B_input(rs2_data),
        .funct3(funct3),
        .jump_or_not(if_jump)
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
            s <= `s_INSTRU;
        end
        
        else begin
            PC <= next_PC;
            s <= next_s;
        end
    end

    // always @()

    //FSM for the top level
    always @(*) begin
        case(s)
            // `s_IDLE : begin
            //     Reg_write = 0;
            //     next_s = `s_INSTRU;
            // end
            `s_INSTRU : begin
                if (mem_to_reg  == `MEM2REG_MEM) begin
                    Reg_write = 0;
                    // next_s = `s_MEMORY;
                    next_s = (mem_read) ? `s_READ : `s_WRITE;
                end
                else begin
                    Reg_write = 0;
                    next_s = `s_ALU;
                end
            end
            // `s_MEMORY : begin
            //     Reg_write = 0;
            //     next_s = (mem_read) ? `s_READ : `s_WRITE;
            // end
            `s_WRITE : begin
                if(!i_DMEM_stall) begin
                    Reg_write = reg_write_or_not;
                    next_s = `s_OUT;
                end
                else begin
                    Reg_write = 0;
                    next_s = `s_WRITE;
                end
            end
            `s_READ :  begin
                if(!i_DMEM_stall) begin
                    Reg_write = reg_write_or_not;
                    next_s = `s_OUT;
                end
                else begin
                    Reg_write = 0;
                    next_s = `s_READ;
                end
            end
            `s_ALU : begin
                if(!i_DMEM_stall && alu_ready) begin
                    Reg_write = reg_write_or_not;
                    next_s = `s_OUT;
                end
                else begin
                    Reg_write = 0;
                    next_s = `s_ALU;
                end
            end
            `s_OUT : begin
                Reg_write = 0;
                next_s = `s_INSTRU;
            end
            default : begin
                Reg_write = 0;
                next_s = `s_INSTRU;
            end
        endcase
    end
    
    // Choose data to write into reg
    always @(*) begin
        
        case(mem_to_reg)
            `MEM2REG_PC_PLUS_4 : rd_data = PC + 4;
            `MEM2REG_ALU : rd_data = result_out;
            `MEM2REG_MEM : rd_data = i_DMEM_rdata; // lw
            `MEM2REG_PC_PLUS_IMM : rd_data = PC + IMMGen_out;
            default : rd_data = 0;
        endcase
    end

    // Setting PC value 
    always @(*) begin   
        if (!o_IMEM_cen) begin     
            case(pc_ctrl)
                `PCCTRL_PC_PLUS_4 : next_PC = PC + 4;
                `PCCTRL_PC_PLUS_IMM : next_PC = (if_jump || opcode==`UJ_JAL || opcode==`I_JALR) ? (PC + IMMGen_out) : (PC + 4);
                `PCCTRL_RS1_PLUS_IMM : next_PC =  rs1_data + IMMGen_out;
                default : next_PC = PC;
            endcase
        end
        else begin
            next_PC = PC;
        end
    end

    // Choose ALU input
    always @(*) begin
        case(alu_src)
            `FROM_RS2 : alu_B_input = rs2_data;
            `FROM_IMM : alu_B_input = IMMGen_out;
            default : alu_B_input = rs2_data;
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

module IMMGen(
    input [31:0] instruction,
    output [31:0] immediate
);
    reg [31:0] extent_imm;
    assign immediate = extent_imm;

    always @(*) begin
        case(instruction[6:0]) 
            `R_TYPE : extent_imm = 32'b0;
            `I_TYPE : extent_imm = {{20{instruction[31]}}, instruction[31:20]};
            `I_JALR : extent_imm = {{20{instruction[31]}}, instruction[31:20]};
            `I_LOAD : extent_imm = {{20{instruction[31]}}, instruction[31:20]};
            `S_TYPE : extent_imm = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            `B_TYPE : extent_imm = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};
            `U_TYPE : extent_imm = {instruction[31:12], 12'b0};
            `UJ_JAL : extent_imm = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
            default : extent_imm = 32'b0;
        endcase
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
                is_branch   = 1;
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
                is_branch   = 0;
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
                is_branch   = 1;
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

module B_type_jump(
    input [31:0] A_input,
    input [31:0] B_input,
    input [2:0] funct3,
    output jump_or_not
);
    reg  jump_or_not_reg;
    assign jump_or_not = jump_or_not_reg;

    always @(*) begin
        case(funct3)
            `BEQ : jump_or_not_reg = ($signed(A_input) == $signed(B_input)) ? 1 : 0;
            `BNE : jump_or_not_reg = ($signed(A_input) != $signed(B_input)) ? 1 : 0;
            `BLT : jump_or_not_reg = ($signed(A_input) < $signed(B_input)) ? 1 : 0;
            `BGE : jump_or_not_reg = ($signed(A_input) >= $signed(B_input)) ? 1 : 0;
            default : jump_or_not_reg = 0;
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
                        default: alu_ctrl = `ADD;
                    endcase
                end
            end
            `I_TYPE : begin
                case(funct3)
                    3'b000: alu_ctrl = `ADDI;   
                    3'b010: alu_ctrl = `SLTI;    
                    3'b001: alu_ctrl = `SLLI;    
                    3'b101: alu_ctrl = `SRAI;
                    default: alu_ctrl = `ADDI;
                endcase
            end
            default: alu_ctrl = `ADD;
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
            `SLTI : alu_result = ($signed(A_input) < $signed(B_input)) ? 1 : 0;
            `SLLI : alu_result = A_input << B_input;
            `SRAI : alu_result = $signed(A_input) >>> $signed(B_input);
            `MUL : alu_result = muldiv_result[31:0];
            default : alu_result = 0;
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
    assign ready = (state == OUT) ? 1 : 0;

    // STATE
    always @(*) begin
        case(state)
            IDLE : begin
                if(valid) next_state = MUL_state;
                else next_state = IDLE;
            end
            MUL_state : next_state = (counter == 31) ? OUT : MUL_state; 
            OUT : next_state = IDLE;
            default : next_state = IDLE;
        endcase
    end    

    // COUNTER
    always @(*) begin
        if(state == MUL_state) next_counter = counter + 1;
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
            MUL_state : begin
                buffer = ({first, 32'b0} + shift_reg) >> 1;
                next_shift_reg = buffer[63:0];
            end
            default : begin
                next_shift_reg = 0;
            end
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
    
    // Implement: 64 blocks, directed cache
    // Tag: 24-bit, Index: 6-bit, Byte offset: 2-bit
    wire [23:0]     Tag;
    wire [7:0]      Index;
    assign Tag = i_proc_addr[31:8];
    assign Index = i_proc_addr[7:2] % 63;

    // Cache
    reg         hit_or_miss;
    reg         cache_valid[0:63], next_cache_valid[0:63];
    reg [23:0]  cache_tag[0:63], next_cache_tag[0:63];
    reg [31:0]  cache_data[0:63], next_cache_data[0:63];

    // FSM
    reg [2:0] state, next_state;
    parameter S_IDLE = 3'd0;
    parameter S_READ = 3'd1;
    parameter S_WRITE = 3'd2;
    parameter S_ALLO = 3'd3;
    // parameter S_IDLE = 3'd0;

    always @(*) begin
        case(state)
            S_IDLE : begin
                if (i_proc_cen && !i_proc_wen) next_state = S_READ;
                else if (i_proc_cen && i_proc_wen) next_state = S_WRITE;
                else next_state = S_IDLE; 
            end
            // 判斷
            S_READ : begin
                // Hit
                if(cache_valid[Index] && (Tag==cache_tag[Index])) begin
                    next_cache_valid[Index] = 1;
                    next_cache_tag[Index] = cache_tag[Index];
                    next_cache_data[Index] = cache_data[Index];
                    next_state = S_IDLE;
                end
                // Miss, Tag不同
                if(cache_valid[Index] && (Tag!=cache_tag[Index])) begin
                    next_cache_valid[Index] = 1;
                    next_cache_tag[Index] = Tag;
                    next_cache_data[Index] = i_mem_rdata;
                    next_state = S_WRITE;
                end
                // First Miss, 第一次開cache
                if(!cache_valid[Index] && !(Tag==cache_tag[Index])) begin
                    next_cache_valid[Index] = 1;
                    next_cache_tag[Index] = Tag;
                    next_cache_data[Index] = i_mem_rdata;
                    next_state = S_ALLO;
                end
                else next_state = S_READ;
            end
            S_WRITE : begin
            end
            S_ALLO : begin
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            for (idx=0; idx<64; idx = idx+1) begin
                cache_valid[idx] <= 1'b0;
                cache_tag[idx] <= 24'b0;
                cache_data[idx] <= 32'b0;
            end
        end
        else begin
            state <= next_state;
            for (idx=0; idx<64; idx = idx+1) begin
                cache_valid[idx] <= next_cache_valid[idx];
                cache_tag[idx] <= next_cache_tag[idx];
                cache_data[idx] <= next_cache_data[idx];
            end
        end
    end
    
endmodule