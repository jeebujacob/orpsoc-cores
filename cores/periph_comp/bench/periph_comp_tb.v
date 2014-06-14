//`timescale 1ns/1ns

module tb_periph_comp;
// Outgoing Signals
    reg		s_wb_periph_clk_i	=1;
    reg		s_wb_periph_rst_i	=1;
    reg [31:0]	s_wb_periph_adr_i;
    reg [31:0]	s_wb_periph_dat_i;
    reg [3:0]	s_wb_periph_sel_i;
    reg		s_wb_periph_we_i;
    reg		s_wb_periph_cyc_i;
    reg		s_wb_periph_stb_i;
    reg  [2:0]	s_wb_periph_cti_i;
    reg  [1:0]	s_wb_periph_bte_i;
    
    //------Incoming signals--------//
    wire [31:0] s_wb_periph_dat_o;
    wire        s_wb_periph_ack_o;
    wire        s_wb_periph_err_o;
    wire        s_wb_periph_rty_o;
    
  always #5  		s_wb_periph_clk_i	<= ~s_wb_periph_clk_i;
  initial
  begin
  #20		s_wb_periph_rst_i	<= 0;
			s_wb_periph_we_i	<= 0;
			s_wb_periph_cyc_i	<= 0;
			s_wb_periph_stb_i	<= 0;
  
  #20 		s_wb_periph_adr_i	<= 32'h00000000;
		s_wb_periph_dat_i	<= 32'hDEADBEEF;
		s_wb_periph_sel_i	<= 4'hF;
		s_wb_periph_we_i	<= 1;
		s_wb_periph_cyc_i	<= 1;
		s_wb_periph_stb_i	<= 1;
		
  #20 		s_wb_periph_adr_i	<= 32'h00000001;
// 		s_wb_periph_dat_i	<= 32'hDEADBEEF;
		s_wb_periph_sel_i	<= 4'hF;
		s_wb_periph_we_i	<= 0;
		s_wb_periph_cyc_i	<= 1;
		s_wb_periph_stb_i	<= 1;
		
#20 		s_wb_periph_adr_i	<= 32'h00000000;
		s_wb_periph_dat_i	<= 32'hDEADBEEF;
		s_wb_periph_sel_i	<= 4'hF;
		s_wb_periph_we_i	<= 1;
		s_wb_periph_cyc_i	<= 1;
		s_wb_periph_stb_i	<= 1;
		
  #20 		s_wb_periph_adr_i	<= 32'h00000001;
// 		s_wb_periph_dat_i	<= 32'hDEADBEEF;
		s_wb_periph_sel_i	<= 4'hF;
		s_wb_periph_we_i	<= 0;
		s_wb_periph_cyc_i	<= 1;
		s_wb_periph_stb_i	<= 1;		
		
		
  end 		
		
periph_comp_top  periph1
			(
			
			 .wb_periph_clk_i(s_wb_periph_clk_i),
			  .wb_periph_rst_i(s_wb_periph_rst_i),
			  .wb_periph_adr_i(s_wb_periph_adr_i),
			  .wb_periph_dat_i(s_wb_periph_dat_i),
			  .wb_periph_sel_i(s_wb_periph_sel_i),
			  .wb_periph_we_i(s_wb_periph_we_i),
			  .wb_periph_cyc_i(s_wb_periph_cyc_i),
			  .wb_periph_stb_i(s_wb_periph_stb_i),
			  .wb_periph_cti_i(s_wb_periph_cti_i),
			  .wb_periph_bte_i(s_wb_periph_bte_i),
			  //------Ouptut Ports--------//
			  .wb_periph_dat_o(s_wb_periph_dat_o),
			  .wb_periph_ack_o(s_wb_periph_ack_o),
			  .wb_periph_err_o(s_wb_periph_err_o),
			  .wb_periph_rty_o(s_wb_periph_rty_o)
			
			);
		
  
 endmodule 
    