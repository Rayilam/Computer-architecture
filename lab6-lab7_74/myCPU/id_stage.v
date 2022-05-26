`include "mycpu.h"

module id_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          es_allowin    ,
    output                         ds_allowin    ,
    //from fs
    input                          fs_to_ds_valid,
    input  [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus  ,
    //to es
    output                         ds_to_es_valid,
    output [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus  ,
    //to fs
    output [`BR_BUS_WD       -1:0] br_bus        ,
    //to rf: for write back
    input  [`WS_TO_RF_BUS_WD -1:0] ws_to_rf_bus  ,
    //lab4
    input  [`ES_TO_ID_BUS_WD -1:0] es_to_id_bus  ,
    input  [`MS_TO_ID_BUS_WD -1:0] ms_to_id_bus 
    //lab5
    //input  [`WS_TO_ID_BUS_WD -1:0] ws_to_id_bus
);

reg         ds_valid   ;
wire        ds_ready_go;

reg  [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus_r;

wire [31:0] ds_inst;
wire [31:0] ds_pc  ;
assign {ds_inst,
        ds_pc  } = fs_to_ds_bus_r;

wire        rf_we   ;
wire [ 4:0] rf_waddr;
wire [31:0] rf_wdata;
assign {rf_we   ,  //37:37
        rf_waddr,  //36:32
        rf_wdata   //31:0
       } = ws_to_rf_bus;

wire        br_taken;
wire [31:0] br_target;

wire [18:0] alu_op;
wire [ 4:0] load_op;
wire [ 2:0] st_op;
wire        src1_is_pc;
wire        src2_is_imm;
wire        dst_is_r1;
wire        gr_we;
wire        src_reg_is_rd;
wire [ 4:0] dest;
wire [31:0] rj_value;
wire [31:0] rkd_value;
wire [31:0] ds_imm;
wire [31:0] br_offs;
wire [31:0] jirl_offs;

wire [ 5:0] op_31_26;
wire [ 3:0] op_25_22;
wire [ 1:0] op_21_20;
wire [ 4:0] op_19_15;
wire [ 4:0] rd;
wire [ 4:0] rj;
wire [ 4:0] rk;
wire [11:0] i12;
wire [19:0] i20;
wire [15:0] i16;
wire [25:0] i26;

wire [63:0] op_31_26_d;
wire [15:0] op_25_22_d;
wire [ 3:0] op_21_20_d;
wire [31:0] op_19_15_d;
  
wire        inst_add_w; 
wire        inst_sub_w;  
wire        inst_slt;    
wire        inst_sltu;   
wire        inst_nor;    
wire        inst_and;    
wire        inst_or;     
wire        inst_xor;    
wire        inst_slli_w;  
wire        inst_srli_w;  
wire        inst_srai_w;  
wire        inst_addi_w; 
wire        inst_ld_w;  
wire        inst_st_w;   
wire        inst_jirl;   
wire        inst_b;      
wire        inst_bl;     
wire        inst_beq;    
wire        inst_bne;    
wire        inst_lu12i_w;
//lab6
wire        inst_slti;   
wire        inst_sltui;
wire        inst_andi;   
wire        inst_ori;   
wire        inst_xori;   
wire        inst_sll_w;  
wire        inst_srl_w;  
wire        inst_sra_w;
wire        inst_pcaddu12i;  
wire        inst_mul_w;  
wire        inst_mulh_w;
wire        inst_mulh_wu;
wire        inst_div_w;  
wire        inst_mod_w;  
wire        inst_div_wu; 
wire        inst_mod_wu;
//lab7
wire        inst_blt;
wire        inst_bge;
wire        inst_bltu;
wire        inst_bgeu;
wire        inst_ld_b;
wire        inst_ld_h;
wire        inst_ld_bu;
wire        inst_ld_hu;
wire        inst_st_b;
wire        inst_st_h;

wire        need_ui5;
wire        need_ui12;
wire        need_si12;
wire        need_si16;
wire        need_si20;
wire        need_si26;  
wire        src2_is_4;

wire [ 4:0] rf_raddr1;
wire [31:0] rf_rdata1;
wire [ 4:0] rf_raddr2;
wire [31:0] rf_rdata2;

wire        rj_eq_rd;
//lab4
//lab5
//wire es_rf_wen;
//wire ms_rf_wen;
//wire ws_rf_wen;
//lab6
wire es_rf_wen;
wire ms_rf_wen;
wire ws_rf_wen;
//wire es_data_from_mem;
wire es_ok;
wire ms_ok;
wire [4:0] es_rf_dest;
wire [4:0] ms_rf_dest;
wire [4:0] ws_rf_dest;
wire [31:0] es_value;
wire [31:0] ms_value;
assign {
    //es_rf_wen,
    //es_rf_dest
    //lab6
    es_rf_wen,
    //es_data_from_mem,
    es_rf_dest,
    es_value,
    es_ok
} = es_to_id_bus;
assign {
    //ms_rf_wen,
    //ms_rf_dest
    //lab6
    ms_rf_wen,
    //ms_ok,
    ms_rf_dest,
    ms_value
} = ms_to_id_bus;
// assign {
//     ws_rf_wen,
//     ws_rf_dest
// } = ws_to_id_bus;

assign br_bus       = {br_taken,br_target};

assign ds_to_es_bus = {alu_op      ,  //lab7 162:144  lab6 156:138   149:138
    load_op     ,  //lab7 143:139  137:137
    st_op       ,  //lab7 138:136
    src1_is_pc  ,  //lab7 135:135  136:136
    src2_is_imm ,  //lab7 134:134  135:135
    gr_we       ,  //lab7 133:133  134:134
    //mem_we      ,  //133:133
    dest        ,  //132:128
    ds_imm      ,  //127:96
    rj_value    ,  //95 :64
    rkd_value   ,  //63 :32
    ds_pc          //31 :0
   };

//lab6
//assign load_op = res_from_mem;
//lab4
//assign ds_ready_go    = 1'b1;
//assign load_op = res_from_mem;
//lab6
assign rk_read=inst_add_w|inst_sub_w|inst_slt|inst_sltu|
            inst_nor|inst_and|inst_or|inst_xor|inst_mul_w|
            inst_mulh_w|inst_mulh_wu|inst_div_w|inst_div_wu|
            inst_mod_w|inst_mod_wu|inst_sll_w|inst_srl_w|inst_sra_w;                 
assign rj_read=~inst_lu12i_w & ~inst_b & ~inst_bl & ~inst_pcaddu12i & ~inst_blt & ~inst_bge & ~inst_bltu & ~inst_bgeu;
assign rd_read=inst_beq | inst_bne | inst_st_w | inst_blt | inst_bge |
            inst_bltu | inst_bgeu;
//wire rk_read = inst_add_w | inst_sub_w | inst_slt | inst_sltu 
//wire rj_read = ~inst_lu12i_w & ~inst_b & ~inst_bl;
//wire rd_read = src_reg_is_rd;
//lab6
//lab5
assign rj_fwd_es=es_rf_wen && (es_rf_dest == rj) && rj != 0 && rj_read;
assign rj_fwd_ms=ms_rf_wen && (ms_rf_dest == rj) && (rj != 0) && rj_read;
assign rj_fwd_ws=rf_we && rf_waddr == rj && rj != 0 && rj_read;
assign rkd_fwd_es=es_rf_wen && es_rf_dest == rk && rk != 0 && rk_read||es_rf_wen && es_rf_dest == rd && rd != 0 && rd_read;
assign rkd_fwd_ms=ms_rf_wen && ms_rf_dest == rk && rk != 0 && rk_read||ms_rf_wen && ms_rf_dest == rd && rd != 0 && rd_read;
assign rkd_fwd_ws=rf_we && rf_waddr == rk && rk!=0 && rk_read || rf_we && rf_waddr == rd && rd != 0 && rd_read; 
assign rk_conflict=(rk!=0)
&&(rk_read)
&&( es_rf_wen & es_rf_dest == rk|| ms_rf_wen & ms_rf_dest == rk|| rf_we & ws_rf_dest == rk);

assign rj_conflict=(rj!=0)
&&(rj_read)
&&( es_rf_wen & es_rf_dest == rj|| ms_rf_wen & ms_rf_dest == rj|| rf_we & ws_rf_dest == rj);

assign rd_conflict=(rd!=0)
&&(rd_read)
&&( es_rf_wen & es_rf_dest == rd|| ms_rf_wen & ms_rf_dest == rd|| rf_we & ws_rf_dest == rd);

//assign ds_ready_go = !rj_conf && !rd_conf && !rk_conf;
//lab6
//lab5
assign ds_ready_go=~(rj_fwd_es&&es_rf_wen);

assign ds_allowin     = !ds_valid || ds_ready_go && es_allowin;
assign ds_to_es_valid = ds_valid && ds_ready_go;
//assign ds_ready_go = ~(es_data_from_mem && rj_fwd_es);


always @(posedge clk) begin
    if (reset) begin
        ds_valid <= 1'b0;
    end
    else if (ds_allowin) begin
        ds_valid <= fs_to_ds_valid;
    end 
    if (fs_to_ds_valid && ds_allowin) begin
        fs_to_ds_bus_r <= fs_to_ds_bus;
    end
end

assign op_31_26  = ds_inst[31:26];
assign op_25_22  = ds_inst[25:22];
assign op_21_20  = ds_inst[21:20];
assign op_19_15  = ds_inst[19:15];

assign rd   = ds_inst[ 4: 0];
assign rj   = ds_inst[ 9: 5];
assign rk   = ds_inst[14:10];

assign i12  = ds_inst[21:10];
assign i20  = ds_inst[24: 5];
assign i16  = ds_inst[25:10];
assign i26  = {ds_inst[ 9: 0], ds_inst[25:10]};

decoder_6_64 u_dec0(.in(op_31_26 ), .out(op_31_26_d ));
decoder_4_16 u_dec1(.in(op_25_22 ), .out(op_25_22_d ));
decoder_2_4  u_dec2(.in(op_21_20 ), .out(op_21_20_d ));
decoder_5_32 u_dec3(.in(op_19_15 ), .out(op_19_15_d ));

assign inst_add_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h00];
assign inst_sub_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h02];
assign inst_slt    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h04];
assign inst_sltu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h05];
assign inst_nor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h08];
assign inst_and    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h09];
assign inst_or     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0a];
assign inst_xor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0b];
assign inst_slli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h01];
assign inst_srli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h09];
assign inst_srai_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h11];
assign inst_addi_w = op_31_26_d[6'h00] & op_25_22_d[4'ha];
assign inst_ld_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h2];
assign inst_st_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h6];
assign inst_jirl   = op_31_26_d[6'h13];
assign inst_b      = op_31_26_d[6'h14];
assign inst_bl     = op_31_26_d[6'h15];
assign inst_beq    = op_31_26_d[6'h16];
assign inst_bne    = op_31_26_d[6'h17];
assign inst_lu12i_w= op_31_26_d[6'h05] & ~ds_inst[25];

//lab6
assign inst_slti   = op_31_26_d[6'h00] & op_25_22_d[4'h8];
assign inst_sltui  = op_31_26_d[6'h00] & op_25_22_d[4'h9];
assign inst_andi   = op_31_26_d[6'h00] & op_25_22_d[4'hd];
assign inst_ori    = op_31_26_d[6'h00] & op_25_22_d[4'he];
assign inst_xori   = op_31_26_d[6'h00] & op_25_22_d[4'hf];
assign inst_sll_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0e];
assign inst_srl_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0f];
assign inst_sra_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h10];
assign inst_mul_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h18];
assign inst_mulh_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h19];
assign inst_mulh_wu= op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h1a];
assign inst_div_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h00];
assign inst_mod_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h01];
assign inst_div_wu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h02];
assign inst_mod_wu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h03];
assign inst_pcaddu12i= op_31_26_d[6'h07] & ~ds_inst[25];


//lab7
assign inst_blt    = op_31_26_d[6'h18];
assign inst_bge    = op_31_26_d[6'h19];
assign inst_bltu   = op_31_26_d[6'h1a];
assign inst_bgeu   = op_31_26_d[6'h1b];
assign inst_ld_b   = op_31_26_d[6'h0a] & op_25_22_d[4'h0];
assign inst_ld_h   = op_31_26_d[6'h0a] & op_25_22_d[4'h1];
assign inst_ld_bu  = op_31_26_d[6'h0a] & op_25_22_d[4'h8];
assign inst_ld_hu  = op_31_26_d[6'h0a] & op_25_22_d[4'h9];
assign inst_st_b   = op_31_26_d[6'h0a] & op_25_22_d[4'h4];
assign inst_st_h   = op_31_26_d[6'h0a] & op_25_22_d[4'h5];

assign load_op[0] = inst_ld_w;
assign load_op[1] = inst_ld_b;
assign load_op[2] = inst_ld_h;
assign load_op[3] = inst_ld_bu;
assign load_op[4] = inst_ld_hu;
assign st_op[0] = inst_st_w;
assign st_op[1] = inst_st_b;
assign st_op[2] = inst_st_h;

assign alu_op[ 0] = inst_add_w | inst_addi_w | inst_jirl | inst_bl | inst_pcaddu12i | (load_op != 5'b0) | (st_op != 3'b0);
assign alu_op[ 1] = inst_sub_w;
assign alu_op[ 2] = inst_slt | inst_slti;
assign alu_op[ 3] = inst_sltu | inst_sltui;
assign alu_op[ 4] = inst_and | inst_andi;
assign alu_op[ 5] = inst_nor;
assign alu_op[ 6] = inst_or | inst_ori;
assign alu_op[ 7] = inst_xor | inst_xori;
assign alu_op[ 8] = inst_slli_w | inst_sll_w;
assign alu_op[ 9] = inst_srli_w | inst_srl_w;
assign alu_op[10] = inst_srai_w | inst_sra_w;
assign alu_op[11] = inst_lu12i_w;
assign alu_op[12] = inst_mul_w;
assign alu_op[13] = inst_mulh_w;
assign alu_op[14] = inst_mulh_wu;
assign alu_op[15] = inst_div_w;
assign alu_op[16] = inst_div_wu;
assign alu_op[17] = inst_mod_w;
assign alu_op[18] = inst_mod_wu;

assign need_ui5   = inst_slli_w | inst_srli_w | inst_srai_w;
assign need_ui12  = inst_andi | inst_ori | inst_xori;
//lab7
assign need_si12  = inst_addi_w | (load_op != 5'b0) | (st_op != 3'b0) | inst_slti | inst_sltui;
//assign need_si12  = inst_addi_w | inst_ld_w | inst_st_w | inst_slti | inst_sltui;
assign need_si16  = inst_jirl | inst_beq | inst_bne | inst_blt | inst_bge | inst_bltu | inst_bgeu;
assign need_si20  = inst_lu12i_w | inst_pcaddu12i;
assign need_si26  = inst_b | inst_bl;
assign src2_is_4  = inst_jirl | inst_bl;

assign ds_imm = {32{src2_is_4}} & 32'h4                                  |
                {32{need_si20}} & {i20, 12'b0}                           |  //i20[16:5]==i12[11:0]
                {32{need_ui5 || need_si12}} & {{20{i12[11]}}, i12[11:0]} |
                {32{need_ui12}} & {20'b0, i12[11:0]};

assign br_offs = need_si26 ? {{ 4{i26[25]}}, i26[25:0], 2'b0} : 
                             {{14{i16[15]}}, i16[15:0], 2'b0} ;

assign jirl_offs = {{14{i16[15]}}, i16[15:0], 2'b0};


//lab7
assign src_reg_is_rd = inst_beq | inst_bne | (st_op != 3'b0) | inst_blt | inst_bge | inst_bltu | inst_bgeu;
//assign src_reg_is_rd = inst_beq | inst_bne | inst_st_w;
assign src1_is_pc    = inst_jirl | inst_bl | inst_pcaddu12i;

assign src2_is_imm   = inst_slli_w | 
                       inst_srli_w |
                       inst_srai_w |
                       inst_addi_w |
                       //lab7
                 (load_op != 5'b0) | 
                   (st_op != 3'b0) |
                   //inst_ld_w   |
                   //inst_st_w   |
                       inst_lu12i_w|
                       inst_jirl   |
                       inst_bl     |
                       inst_slti   |
                       inst_sltui  |
                       inst_andi   | 
                       inst_ori    |
                       inst_xori   |
                       inst_pcaddu12i;



//assign res_from_mem  = inst_ld_w;
assign dst_is_r1     = inst_bl;
//lab7
assign gr_we         = ~inst_st_w & ~inst_st_b & ~inst_st_h & ~inst_beq & ~inst_bne 
                      & ~inst_blt & ~inst_bltu & ~inst_bge & ~inst_bgeu & ~inst_b;
//assign gr_we         = ~inst_st_w & ~inst_beq & ~inst_bne & ~inst_b;
//assign mem_we        = inst_st_w;
assign dest          = dst_is_r1 ? 5'd1 : rd;


assign rf_raddr1 = rj;
assign rf_raddr2 = src_reg_is_rd ? rd :rk;
regfile u_regfile(
    .clk    (clk      ),
    .raddr1 (rf_raddr1),
    .rdata1 (rf_rdata1),
    .raddr2 (rf_raddr2),
    .rdata2 (rf_rdata2),
    .we     (rf_we    ),
    .waddr  (rf_waddr ),
    .wdata  (rf_wdata )
    );

//assign rj_value  = rf_rdata1; 
//assign rkd_value = rf_rdata2;
assign rj_value = rj_fwd_es ? es_value:
                  rj_fwd_ms ? ms_value:
                  rj_fwd_ws ? rf_wdata:
                  rf_rdata1;
assign rkd_value = rkd_fwd_es ? es_value:
                   rkd_fwd_ms ? ms_value:
                   rkd_fwd_ws ? rf_wdata:
                   rf_rdata2;


//lab7
wire [31:0] adder_res;
wire        carry;
assign {carry, adder_res} = {1'b0, rj_value} + {1'b0, ~rkd_value} + 1'b1;
assign rj_lt_rd  = (rj_value[31] & ~rkd_value[31])
                | ((rj_value[31] ~^ rkd_value[31]) & adder_res[31]);
assign rj_ltu_rd = ~carry;

assign rj_eq_rd  = (rj_value == rkd_value); 

assign br_taken = (   inst_beq  &&  rj_eq_rd
                   || inst_bne  && !rj_eq_rd
                   || inst_blt  &&  rj_lt_rd
                   || inst_bge  && !rj_lt_rd
                   || inst_bltu &&  rj_ltu_rd
                   || inst_bgeu && !rj_ltu_rd
                   || inst_jirl
                   || inst_bl
                   || inst_b
                  ) && ds_valid; 
assign br_target = (inst_beq || inst_bne || inst_bl   || inst_b || 
                    inst_blt || inst_bge || inst_bltu || inst_bgeu) ? (ds_pc + br_offs) :
                                                        /*inst_jirl*/ (rj_value + jirl_offs);

endmodule