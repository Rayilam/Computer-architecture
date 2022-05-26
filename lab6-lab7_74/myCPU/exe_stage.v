
`include "mycpu.h"

module exe_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          ms_allowin    ,
    output                         es_allowin    ,
    //from ds
    input                          ds_to_es_valid,
    input  [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus  ,
    //to ms
    output                         es_to_ms_valid,
    output [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus  ,

    // data sram interface
    output        data_sram_en   ,
    output [ 3:0] data_sram_wen  ,
    output [31:0] data_sram_addr ,
    output [31:0] data_sram_wdata,
    //lab4
    output [`ES_TO_ID_BUS_WD -1:0] es_to_id_bus 
);

reg         es_valid      ;
wire        es_ready_go   ;

reg  [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus_r;
wire [18:0] es_alu_op     ;
wire        es_src1_is_pc ;
wire        es_src2_is_imm; 
wire        es_gr_we      ;
wire [ 4:0] es_dest       ;
wire [31:0] es_imm        ;
wire [31:0] es_rj_value   ;
wire [31:0] es_rkd_value  ;
wire [31:0] es_pc         ;

wire [ 4:0] es_load_op;
wire [ 2:0] es_st_op;
wire        es_res_from_mem;
wire        es_mem_we;

assign {es_alu_op      ,  //162:144
        es_load_op     ,  //143:139
        es_st_op       ,  //138:136
        es_src1_is_pc  ,  //135:135
        es_src2_is_imm ,  //134:134
        es_gr_we       ,  //133:133
        //es_mem_we    ,
        es_dest        ,  //132:128
        es_imm         ,  //127:96
        es_rj_value    ,  //95 :64
        es_rkd_value   ,  //63 :32
        es_pc             //31 :0
       } = ds_to_es_bus_r;

wire [31:0] es_alu_src1   ;
wire [31:0] es_alu_src2   ;
wire [31:0] es_alu_result ;

//lab4
//lab6
//wire es_ok = es_valid && es_gr_we;
wire es_ok = es_valid && es_res_from_mem;
wire[31:0] es_value = es_alu_result;
wire es_rf_wen = es_valid & es_gr_we;
wire es_rf_dest   = es_dest                    ;
//wire es_data_from_mem = es_valid && es_res_from_mem;
assign es_to_id_bus = {
    //!es_rf_wen,
    //!es_rf_dest
    //lab6
    //es_data_from_mem,
    es_rf_wen,
    es_dest,
    es_value,
    es_ok
};
//lab7
assign es_res_from_mem = (es_load_op != 5'b0);
//assign es_res_from_mem = es_load_op;
assign es_to_ms_bus = {es_load_op     ,  //74:70
                       es_gr_we       ,  //69:69
                       es_dest        ,  //68:64
                       es_alu_result  ,  //63:32
                       es_pc             //31:0
                      };

//lab6
assign is_div = es_alu_op[15] || es_alu_op[16] || es_alu_op[17] || es_alu_op[18];
assign es_ready_go    = !(is_div && !div_finished);
//assign es_ready_go    = 1'b1;
assign es_allowin     = !es_valid || es_ready_go && ms_allowin;
assign es_to_ms_valid =  es_valid && es_ready_go;

always @(posedge clk) begin
    if (reset) begin     
        es_valid <= 1'b0;
    end
    else if (es_allowin) begin 
        es_valid <= ds_to_es_valid;
    end

    if (ds_to_es_valid && es_allowin) begin
        ds_to_es_bus_r <= ds_to_es_bus;
    end
end

assign es_alu_src1 = es_src1_is_pc  ? es_pc[31:0] : 
                                      es_rj_value;
                                      
assign es_alu_src2 = es_src2_is_imm ? es_imm : 
                                      es_rkd_value;

alu u_alu(
    .clk         (clk          ),
    .alu_op      (es_alu_op    ),
    .alu_src1    (es_alu_src1  ),
    .alu_src2    (es_alu_src2  ),
    .alu_result  (es_alu_result),
    .es_valid    (es_valid     ),
    .div_finished(div_finished )
    );

//lab7
//st
//wire [31:0] st_data;
//wire [3:0]  st_strb;
//wire [3:0]  st_sel;
//wire op_st_b = es_st_op[1];
//wire op_st_h = es_st_op[2];
//decoder_2_4 decoder_st(
   // .in  (es_alu_result[1:0]),
   // .out (st_sel)
//);

//assign st_strb = op_st_h ? (st_sel[0]? 4'b0011 : 4'b1100) :
      //          op_st_b ?  st_sel                         :
         //       4'b1111;
//assign st_data = op_st_b ? {4{es_rkd_value[ 7:0]}} :
 //               op_st_h ? {2{es_rkd_value[15:0]}} :
  //               es_rkd_value[31:0];

//assign es_mem_we = (es_st_op != 3'b0);
//assign data_sram_en    = (es_res_from_mem || es_mem_we) && es_valid;
//assign data_sram_wen   = es_mem_we ? 4'hf : 4'h0;
//assign data_sram_addr  = {es_alu_result[31:2],2'b0};
//assign data_sram_wdata = st_data; //es_rkd_value;
wire [31:0] st_data;
wire [ 3:0] st_strb;
wire [ 3:0] st_sel;
decoder_2_4  u_dec_st(.in(es_alu_result[1:0]), .out(st_sel));
assign st_strb = { 4{es_st_op[0]}} & 4'b1111//st_w
               | { 4{es_st_op[1]}} & st_sel//st_b
               | { 4{es_st_op[2]}} & (st_sel[0] ? 4'b0011 : 4'b1100);//st_h
assign st_data = es_st_op[0]?  es_rkd_value:
                 es_st_op[1]? {4{es_rkd_value[7:0]}}:
                              {2{es_rkd_value[15:0]}};
assign es_mem_we       = (es_st_op != 3'b0);

assign data_sram_en    = (es_res_from_mem || es_mem_we) && es_valid;
assign data_sram_wen   = es_mem_we ? st_strb : 4'h0;
assign data_sram_addr  = {es_alu_result[31:2], 2'b0};
assign data_sram_wdata = st_data;

endmodule