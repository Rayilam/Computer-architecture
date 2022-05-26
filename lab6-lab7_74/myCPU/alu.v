module alu(
  input         clk,
  input  [18:0] alu_op,
  input  [31:0] alu_src1,
  input  [31:0] alu_src2,
  output [31:0] alu_result,
  //lab6
  input         es_valid,
  output        div_finished
);

wire op_add;   //add operation
wire op_sub;   //sub operation
wire op_slt;   //signed compared and set less than
wire op_sltu;  //unsigned compared and set less than
wire op_and;   //bitwise and
wire op_nor;   //bitwise nor
wire op_or;    //bitwise or
wire op_xor;   //bitwise xor
wire op_sll;   //logic left shift
wire op_srl;   //logic right shift
wire op_sra;   //arithmetic right shift
wire op_lui;   //Load Upper Immediate
//lab6
wire op_mul;
wire op_mulh;
wire op_mulhu;
wire op_div; 
wire op_divu;
wire op_mod; 
wire op_modu;

// control code decomposition
assign op_add  = alu_op[ 0];
assign op_sub  = alu_op[ 1];
assign op_slt  = alu_op[ 2];
assign op_sltu = alu_op[ 3];
assign op_and  = alu_op[ 4];
assign op_nor  = alu_op[ 5];
assign op_or   = alu_op[ 6];
assign op_xor  = alu_op[ 7];
assign op_sll  = alu_op[ 8];
assign op_srl  = alu_op[ 9];
assign op_sra  = alu_op[10];
assign op_lui  = alu_op[11];
//lab6
assign op_mul  = alu_op[12];
assign op_mulh = alu_op[13];
assign op_mulhu= alu_op[14];
assign op_div  = alu_op[15];
assign op_divu = alu_op[16];
assign op_mod  = alu_op[17];
assign op_modu = alu_op[18];

wire [31:0] add_sub_result;
wire [31:0] slt_result;
wire [31:0] sltu_result;
wire [31:0] and_result;
wire [31:0] nor_result;
wire [31:0] or_result;
wire [31:0] xor_result;
wire [31:0] lui_result;
wire [31:0] sll_result;
wire [63:0] sr64_result;
wire [31:0] sr_result;
//lab6
wire [63:0] signed_prod;
wire [63:0] unsigned_prod;
wire [31:0] mul_result;
wire [63:0] div_result;
wire [63:0] divu_result;
wire [31:0] div_final_result;

// 32-bit adder
wire [31:0] adder_a;
wire [31:0] adder_b;
wire        adder_cin;
wire [31:0] adder_result;
wire        adder_cout;

assign adder_a   = alu_src1;
assign adder_b   = (op_sub | op_slt | op_sltu) ? ~alu_src2 : alu_src2;  //src1 - src2 rj-rk
assign adder_cin = (op_sub | op_slt | op_sltu) ? 1'b1      : 1'b0;
assign {adder_cout, adder_result} = adder_a + adder_b + adder_cin;

// ADD, SUB result
assign add_sub_result = adder_result;

// SLT result
assign slt_result[31:1] = 31'b0;   //rj < rk 1
assign slt_result[0]    = (alu_src1[31] & ~alu_src2[31])
                        | ((alu_src1[31] ~^ alu_src2[31]) & adder_result[31]);

// SLTU result
assign sltu_result[31:1] = 31'b0;
assign sltu_result[0]    = ~adder_cout;

// bitwise operation
assign and_result = alu_src1 & alu_src2;
assign or_result  = alu_src1 | alu_src2 ; ////*bug8, func error
assign nor_result = ~or_result;
assign xor_result = alu_src1 ^ alu_src2;
assign lui_result = alu_src2;

// SLL result
assign sll_result = alu_src1 << alu_src2[4:0];   //*bug6 rj << i5 
// SRL, SRA result
assign sr64_result = {{32{op_sra & alu_src1[31]}}, alu_src1[31:0]} >> alu_src2[4:0]; //*bug7 rj >> i5 

assign sr_result   = sr64_result[31:0];  //* bug7

//lab6
// MUL result
assign unsigned_prod = alu_src1 * alu_src2;
assign signed_prod   = $signed(alu_src1) * $signed(alu_src2);
assign mul_result    = {32{op_mul  }} & signed_prod[31:0]
                     | {32{op_mulh }} & signed_prod[63:32]
                     | {32{op_mulhu}} & unsigned_prod[63:32];
                    
// DIV result
reg    div_clear;
reg    divu_clear;
reg    divisor_tvalid;
reg    dividend_tvalid;
reg    divisor_tvalid_u;
reg    dividend_tvalid_u;
wire   div_result_valid;
wire   divu_result_valid;
wire   is_div;
wire   is_divu;

assign is_div = op_div || op_mod;
assign is_divu = op_divu || op_modu;
assign div_finished = div_result_valid || divu_result_valid;

div u_div(
  .aclk                   (clk),
  .s_axis_divisor_tdata   (alu_src2),
  .s_axis_divisor_tready  (divisor_tready),
  .s_axis_divisor_tvalid  (divisor_tvalid),
  .s_axis_dividend_tdata  (alu_src1),  
  .s_axis_dividend_tready (dividend_tready),
  .s_axis_dividend_tvalid (dividend_tvalid),
  .m_axis_dout_tdata      (div_result),
  .m_axis_dout_tvalid     (div_result_valid)
);

divu u_divu(
  .aclk                   (clk),
  .s_axis_divisor_tdata   (alu_src2),
  .s_axis_divisor_tready  (divisor_tready_u),
  .s_axis_divisor_tvalid  (divisor_tvalid_u),
  .s_axis_dividend_tdata  (alu_src1),  
  .s_axis_dividend_tready (dividend_tready_u),
  .s_axis_dividend_tvalid (dividend_tvalid_u),
  .m_axis_dout_tdata      (divu_result),
  .m_axis_dout_tvalid     (divu_result_valid)
);
assign div_final_result = {32{op_div }} & div_result[63:32]
                        | {32{op_divu}} & divu_result[63:32]
                        | {32{op_mod }} & div_result[31:0]
                        | {32{op_modu}} & divu_result[31:0];

always @(posedge clk) begin
    if(div_clear) begin
        divisor_tvalid <= 1'b0;
        dividend_tvalid <= 1'b0;
    end
    else if(divisor_tready && dividend_tready) begin
        divisor_tvalid <= 1'b0;
        dividend_tvalid <= 1'b0;    
    end 
    else if(es_valid && is_div) begin
        divisor_tvalid <= 1'b1;
        dividend_tvalid <= 1'b1;
    end else begin
        divisor_tvalid <= 1'b0;
        dividend_tvalid <= 1'b0; 
    end
end

always @(posedge clk) begin  
    if(divu_clear) begin
        divisor_tvalid_u <= 1'b0;
        dividend_tvalid_u <= 1'b0;
    end
    else if(divisor_tready_u && dividend_tready_u) begin
        divisor_tvalid_u <= 1'b0;
        dividend_tvalid_u <= 1'b0;    
    end  
    else if(es_valid && is_divu) begin
        divisor_tvalid_u <= 1'b1;
        dividend_tvalid_u <= 1'b1;
    end else begin
        divisor_tvalid_u <= 1'b0;
        dividend_tvalid_u <= 1'b0; 
    end
end

always @(posedge clk) begin
    if(div_result_valid) begin
        div_clear <= 1'b0;
    end
    else if(divisor_tready && divisor_tvalid && dividend_tready && dividend_tvalid) begin
        div_clear <= 1'b1;
    end
end

always @(posedge clk) begin
    if(divu_result_valid) begin
        divu_clear <= 1'b0;
    end
    else if(divisor_tready_u && divisor_tvalid_u && dividend_tready_u && dividend_tvalid_u) begin
        divu_clear <= 1'b1;
    end
end


// final result mux
assign alu_result = ({32{op_add|op_sub}} & add_sub_result)
                  | ({32{op_slt       }} & slt_result)
                  | ({32{op_sltu      }} & sltu_result)
                  | ({32{op_and       }} & and_result)
                  | ({32{op_nor       }} & nor_result)
                  | ({32{op_or        }} & or_result)
                  | ({32{op_xor       }} & xor_result)
                  | ({32{op_lui       }} & lui_result)
                  | ({32{op_sll       }} & sll_result)
                  | ({32{op_srl|op_sra}} & sr_result)//lab6
                  | ({32{op_mul|op_mulh|op_mulhu}} & mul_result)
                  | ({32{op_div|op_divu|op_mod|op_modu}} & div_final_result);

endmodule