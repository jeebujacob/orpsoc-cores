 //%%					High Performance Computing Lab, Indian Institute of Technology, Bombay(IITB)			%%	
 //%%										Powai, Mumbai,India													%%
 //%=========================================================================================================%%
 // %%This is the Intellectual Property of High Performance Computing Laboratory,IIT Bombay, and hence 			%%
 // %%should not be used for any monetary benefits without the proper consent of the Institute. However			%%	
 // %%it can be used as reference related to academic activities. In the event of publication					%%
 // %%the following notice is applicable																		%% 
 // %%Copyright(c) 2014 HPC Lab,IIT Bombay.																		%%
 // %%The entire notice above must be reproduced on all authorized copies.										%%
 //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 //%% Project Name		: MTech Project "Study of FPGA based Cosystems"											%% 
 //%% File Name			: modelevaluator.v																		%%
 //%% Title 			: NonLinear Circuit Solver for ORPSoC v3.0(Fusesoc) based OR1200-generic System			%%
 //%% Author			: Jeebu Jacob Thomas																	%%
 //%% Description		:																						%%
 //%% Version			: v1.0																					%%
 //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
///////// 	Present Address map given to the peripheral is 
////////	Peripheral 	Base Address	:	x"9100000000"
////////	Peripheral Address Range	:   x"9100000000" - x"91040000" (256 K address range)
////////    Memory Modules Present		: 	2
////////    Memory Range				:   x"9100000000" - x"9100001FFF"(Mem0) ,  x"9100002000" - x"9100003FFF" (Mem1)  
///////		Dated						: 09-06-2014
 
 
 

module periph_comp_top
(
//********** Wishbone Ports for Communication with the Wishbone Bus********//////////////////////////
//*****All the wishbone signals as per the wishbone B3 specification are active high*********////////
    //------ Input ports----------
    input 		 wb_periph_clk_i,
    input 	 	 wb_periph_rst_i,
    input [31:0] wb_periph_adr_i,
    input [31:0] wb_periph_dat_i,
    input  [3:0] wb_periph_sel_i,
    input        wb_periph_we_i,
    input        wb_periph_cyc_i,
    input        wb_periph_stb_i,
    input  [2:0] wb_periph_cti_i,
    input  [1:0] wb_periph_bte_i,
    
    //------Ouptut Ports--------//
    output reg [31:0] wb_periph_dat_o, 
    output reg        wb_periph_ack_o,
    output reg        wb_periph_err_o,
    output reg        wb_periph_rty_o
	
////********************************************************//////////////////////////////////	
);
  localparam base_add_nodev		= 10; 
  localparam base_add_Gvalue		= 100;
  localparam depth 			= 2048;
  localparam sim_latency		= 3;
  localparam max_NR_iterations		= 2;

  localparam IDLE 			= 0;
  localparam EVAL_NGHBR_ENTER		= 1;
  localparam EVAL_NGHBR_BUSY		= 2;
  localparam EVAL_NGHBR_EXIT		= 3;
  localparam ACCUMULATE			= 4;
  localparam NEW_V_CALC			= 5;
  localparam UPDATE_POTENTIAL		= 6;
  localparam RETURN_POTENTIAL		= 7;
  
  
  
  wire valid;
  wire [10:0] periph_add ;
  reg [31:0] mem0 [0:depth-1];
  reg [31:0] mem1 [0:depth-1];
 
 ///////////**********State Variables Added Here********/////////////////////
  reg [4:0] state;
  
 
 
///************** Logic for Wishbone Bus Protocol************///////
  assign  valid = wb_periph_cyc_i & wb_periph_stb_i;
  
  initial 
  begin
  wb_periph_err_o <= 0;
  wb_periph_rty_o <= 0;
  wb_periph_ack_o <= 0;
  
  end


  ///////************Instantiations to different submodules*********///////////////////////////////// 
 
   
 ///////////***************************************************//////////////////////////////////////  
   
 //*******// Memory Read and Write//***********//
	
   wire [1:0] mem_select;	
   wire [31:0] hardware_start,neighbourcount,periph_reg2;
   reg [10:0] read_address1,read_address2;
   wire [10:0] fix_add, coeff_add;
   wire [31:0] wr_data; // Selecting the byte lines of the write data
   wire mem0_cs,mem1_cs, mem0_wr, mem1_wr;
   wire [31:0] mem0_rdata,mem1_rdata;
   assign periph_add = valid ? wb_periph_adr_i[12:2] : {11{1'b0}}; // 11 Bits for 2048 address lines since the address lines coming from the processor are byte addressible
   assign hardware_start = mem0[1024];
   assign neighbourcount = mem0[0];
   
   
   
   // assign wr_data[31:24] = wb_periph_sel_i[3] ? wb_periph_dat_i[31:24] : wb_periph_dat_o[31:24];
   // assign wr_data[23:16] = wb_periph_sel_i[2] ? wb_periph_dat_i[23:16] : wb_periph_dat_o[23:16];
   // assign wr_data[15: 8] = wb_periph_sel_i[1] ? wb_periph_dat_i[15: 8] : wb_periph_dat_o[15: 8];
   // assign wr_data[ 7: 0] = wb_periph_sel_i[0] ? wb_periph_dat_i[ 7: 0] : wb_periph_dat_o[ 7: 0];
   
   assign wr_data[31:24] = wb_periph_sel_i[3] ? wb_periph_dat_i[31:24] : {8{1'b0}};
   assign wr_data[23:16] = wb_periph_sel_i[2] ? wb_periph_dat_i[23:16] : {8{1'b0}};
   assign wr_data[15: 8] = wb_periph_sel_i[1] ? wb_periph_dat_i[15: 8] : {8{1'b0}};
   assign wr_data[ 7: 0] = wb_periph_sel_i[0] ? wb_periph_dat_i[ 7: 0] : {8{1'b0}};
  
   
   /////////////////********************Address Map of Peripheral************************////////////////////
   ////////////////32'h  9    1    X    X    X    X    X    X            -- X denotes dont care(Any value )
   ///////////////      1001 0001 XXXX XXXX XXXX XXXX XXXX XXXX
   ///////////////   (31)     (24)(23)                         (0) ///////////////////////////////////////////
   
   assign ram_we 		= wb_periph_we_i & wb_periph_ack_o;
   assign mem0_cs 		= ((wb_periph_adr_i[23:13] == 0) && valid) ? 1 : 0; // Enable for Memory0 ported as the lower 2048 address locations
   assign mem1_cs 		= ((wb_periph_adr_i[23:13] == 1) && valid) ? 1 : 0; // Enable for Memory1 ported as the upper 2048 address locations
   assign mem0_wr 		= ram_we & mem0_cs;
   assign mem1_wr		= ram_we & mem1_cs;
   assign mem0_rdata 		= mem0[read_address1];
   assign mem1_rdata 		= mem1[read_address2];
   assign mem_select		= {mem0_cs,mem1_cs};
   
   
   
  ///************************************************////////// 
  // Registers and Wires for FSM//
 ///************************************************//////////
   wire [31:0] Ivalue;
   reg [31:0]	countNumberofCycles;
   reg [4:0]	count_NR_iterations;
   reg [31:0]	neighbour_id = 0;
   reg [31:0]   counter = 0;
   reg [31:0]	coeff,fix;
   reg [31:0]	total_coeff,total_fix;
   reg [31:0]	new_v =	0;
   reg [31:0]	V_neighbour,G_neighbour;
   reg [31:0]  div_value,div_quotient;
   reg [31:0]  hardware_done;
   
   assign fix_add		= base_add_nodev + neighbour_id;
   assign coeff_add		= base_add_Gvalue + neighbour_id;
  
   assign Ivalue = mem0[9];

///******************Muxing the read_data to the output read data line*******//////////////////   
 always@(mem0_cs, mem1_cs, mem0_rdata, mem1_rdata )
	begin
	case (mem_select)
	
		2'b10 : 
			wb_periph_dat_o <= mem0_rdata;
		2'b01 : 
			wb_periph_dat_o <= mem1_rdata;	
			
		default:
				wb_periph_dat_o <= {32{1'b0}};
		
	endcase	
	  
	end  
/////***************************************************************/////////////////////////////		
    

 always@(posedge wb_periph_clk_i)  // For mem0
   begin
		if(mem0_wr) 
		begin 
			mem0[periph_add] <= wr_data;
		end 	
		read_address1 <= periph_add;
   
   end 
 
 always @(posedge wb_periph_clk_i) // For mem1
   begin
		if(mem1_cs) 
		begin 
			// mem1[periph_add] <= wr_data;  // Since this is a Hardware-Write Memory and the application can only read from it
		end 	
		read_address2 <= periph_add;
   
   end 
 //********//***********************//**************//
  //******//**Wishbone Signals Updation//***********//
  always @(posedge wb_periph_clk_i)
	begin
	
		if(wb_periph_rst_i)
		begin 
		  wb_periph_ack_o <= 0;
		end else
		begin 
		
		  wb_periph_ack_o <= valid & !(wb_periph_ack_o); 
		
		end 
	
	end
	


	
	
	
 always@(posedge wb_periph_clk_i)
	begin
		if(wb_periph_rst_i)
		begin 
		  state 		<= IDLE;
		  countNumberofCycles	<= 0;
		  count_NR_iterations	<= 0;
		  neighbour_id		<= 0;
		  counter		<= 0;
		  V_neighbour		<= 0;
		  G_neighbour		<= 0;
		  new_v			<= 0;
		  hardware_done		<= 0;
		  mem1[0]		<= 0;
	//******Blocking Assignments****////////	  
		  coeff			= 0;
		  fix			= 0;
		  total_coeff		= 0;
		  total_fix		= 0;
		  div_value		= 0;
		  div_quotient		= 0;
		
		  
		end else
		begin 
		
		 case(state)
			IDLE: //0
			begin
				count_NR_iterations 	<= 0;
				neighbour_id		<= 0;	
				if(hardware_start == 1)
				begin
					state	<= 	EVAL_NGHBR_ENTER;
				end else 
				begin
					state	<= 	IDLE;
				end 
			end
			EVAL_NGHBR_ENTER: //1
			begin
// 				hardware_done	<= 0;// Hardware Busy (Just for Simulation purpose)
				mem1[0]		<= 0;// Hardware Busy
				countNumberofCycles <= countNumberofCycles + 1;
					V_neighbour	<= mem0[fix_add];
					G_neighbour	<= mem0[coeff_add];
					state		<= EVAL_NGHBR_BUSY;
			
			end						
		
			EVAL_NGHBR_BUSY: //2
			begin	
				countNumberofCycles <= countNumberofCycles + 1;
				if(counter == sim_latency-1)
				begin
					counter <=	0;
					state	<=	EVAL_NGHBR_EXIT;
					
				end else
				begin
					counter  <=	counter + 1;
					state	 <= 	EVAL_NGHBR_BUSY;
					
				end
			end		
		
			EVAL_NGHBR_EXIT:   //3
			begin
				coeff	=	G_neighbour;
				fix	=	V_neighbour*G_neighbour;
				state	<= 	ACCUMULATE;
				countNumberofCycles <= countNumberofCycles + 1;
			end
			
			ACCUMULATE: //4
			begin
				countNumberofCycles 	<= countNumberofCycles + 1;
				total_coeff		= 	total_coeff + coeff;
				total_fix		=	total_fix	+ fix;
				
				
				if(neighbour_id == neighbourcount-1 )
				begin
					neighbour_id <= 0;
					div_value		= 	Ivalue + total_fix;
					state <= NEW_V_CALC;				
				end else
				begin	
					neighbour_id	<=  neighbour_id + 1;
					state <= EVAL_NGHBR_ENTER;
				end	
			end		
			
			
				
			NEW_V_CALC:	 //5
			begin	
			
				div_value	=	div_value - total_coeff	;
				if(div_value[31] == 1)
				begin
					state <= UPDATE_POTENTIAL;
				end
				else begin
					div_quotient	= div_quotient + 1;
					state	<=	NEW_V_CALC;
				end
			end	
			
				
			UPDATE_POTENTIAL: //6
			begin	
				new_v		<=	div_quotient;	
				div_quotient = 0;
				div_value = 0;
				if(count_NR_iterations	==	max_NR_iterations)
				begin
					state <= RETURN_POTENTIAL;
				end
				else begin
					count_NR_iterations	<= count_NR_iterations	+	1;
					state	<=	EVAL_NGHBR_ENTER;
					total_coeff	= 0;
					total_fix	= 0;

				end
			end	
			
			RETURN_POTENTIAL: //7
			begin
				total_coeff	= 0;
				total_fix	= 0;
				count_NR_iterations <= 0;
				mem1[1] 	<= new_v;
// 				hardware_done 	<= 1; //Hardware Computation is over( for Simulation only)
				mem1[0]		<= 1; //Hardware Computation is over
				state	<= IDLE;
			end
			default:
			begin
				count_NR_iterations <= 0;
				neighbour_id		<= 0;
			end		
		 endcase		
		end
	end	
	  
  endmodule     
	
    