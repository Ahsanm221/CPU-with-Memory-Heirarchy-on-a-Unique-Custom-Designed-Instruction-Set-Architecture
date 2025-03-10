`include "./config.sv"
`include "./constants.sv"
/* verilator lint_off UNPACKED */
/* verilator lint_off CASEX */
/* verilator lint_off CASEINCOMPLETE */
module datapath(input logic clk, reset, MemRead, MemWrite, MemToReg, Second_alu_en,
                input logic ALUSrc, RegWrite, Branch, Jump, swj,
					 input logic [1:0] ForwardA,
					 input logic [1:0] ForwardB,
					 input logic [31:0]	IMM,
					 input logic ForwardE,
					 input logic ForwardC,
					 input logic ForwardD,
					 output logic ExMemRegWrite,
					 output logic MemWbRegWrite,
					 output logic [4:0]	IdExrs1,
					 output logic [4:0]	IdExrs2,
					 output logic [4:0]	ExMemrd,
					 output logic [4:0]	ExMemrs3,
					 output logic [4:0]	MemWbrd,
					 output logic 	ExMemSecond_alu_en,
					 output logic [31:0] Instruction,
					 output logic [4:0] rs11,
					 output logic [4:0] rs22,
					 output logic [5:0] Op,
					 // stall signal
					 input logic cache_miss_stall,
					  //icache signals
					 output logic [`DRAM_ADDRESS_SIZE-1:0] icache_address,
					 output logic icache_valid,
					 input logic [31:0] icache_data_out,
					 //dcache signals
					 input logic [31:0] dcache_data_out, 
					 output logic [`DRAM_ADDRESS_SIZE-1:0] dcache_address, 
					 output logic [31:0] dcache_data_in, 
					 output logic [3:0] dcache_byte_en, 
					 output logic dcache_rw, 
					 output logic dcache_valid
					 );

	 parameter PCSTART = 512; //starting address of instruction memory
	 logic PCsrc;
	 //mem control and aluctrl
	 logic [3:0] aluctr;
	 logic [2:0] memctrl;
	 
	 always_comb begin
	 if ((Op == 6'b000011) | (Op == 6'b100011)) begin
	 aluctr = 4'b0;
	 memctrl = IfId.instruction[2:0];
	 end
	 else if (Op == 6'b111001) begin
	 memctrl = 3'b0;
	 aluctr = 4'b0;
	 end
	 else begin
	 aluctr = IfId.instruction[3:0];
	 memctrl = 3'b0;
	 end
	 end
	// Instruction memory internal storage, input address and output data bus signals
   logic [31:0] instmem_data;
	
	// Data memory internal storage, input address and output data bus signals
   logic [31:0] datamem_data;
	
	//multiplier signals
   logic [31:0] multiplier_out;
   logic multiplier_done;
   logic run_multiplier;
	logic pause_pipeline;
	
	//data and instruction memory 
		
   //internal signals 
	logic [31:0] PC; 				// Program Counter
	logic PCWrite;					// PC Write Enable 
	logic [31:0] OpA1; 			// Alu input 1
	logic [31:0] OpB1; 			// Alu input 2
	logic [3:0] ALUCtrl;
	logic [31:0] da; 				//read data 1
	logic [31:0] db; 				//read data 2
	logic [31:0] dc;				//read data 3 for mul and add
	logic [31:0] result; 			//output of ALU
	logic [31:0] ForwardA_data;		//input to ALU 1
	logic [31:0] ForwardB_data;		//input to ALUSrc Mux
	
	logic [1:0] aluctrl2;			// func 3 from IMM gen for mul and add
	logic [31:0] OpA2; 				//read data 1 for second ALU
	logic [31:0] OpB2; 				//read data 2 for second ALU
	logic [31:0] ALUResult2;		//output of 2nd ALU
   
	//Hazard Detection signals
	logic stall;
	logic flush;
	assign flush = IdEx.Jump | ExMem.Jump | (IdEx.Branch && result[0]) | (ExMem.Branch && ExMem.AluOut[0]);			
	
	// IF/ID register 
	logic IfId_Write;				//write enable signal for IFID
	
	struct {
		logic [31:0] instruction;
		logic [31:0] PCincremented; // PC 7 bit?
	} IfId;
	
	// ID/EX Register 
	struct {
		logic [31:0] PCincremented;
		logic MemRead; 					// control signals coming in ID/EX
		logic MemWrite;
		logic MemToReg;
		logic ALUSrc;
		logic RegWrite;
		logic Second_alu_en;			//Enable for second ALU 
		logic Branch;
		logic [31:0] da; 				// Read data 1 of register file
		logic [31:0] db;			 	// Read data 2 of register file
		logic [31:0] SignExtend; 		// Sign Extended offset
		logic [4:0] rs2; 				//rs2 address added for forwarding
		logic [4:0] rs1; 				// rs1 address replacing rt
		logic [4:0] rd;					// rd address
		logic [3:0] func2;				// for control of ALU
		logic [2:0] memctr;           // to control memory 
		logic [4:0] rs3; 				// rs1 address replacing rt
		logic Jump;
		logic swj;
		logic [4:0] rdwb;
	} IdEx;
	
	// EX/MEM Register
	struct {
		logic MemRead; 					// control signals coming in EX/MEM
		logic MemWrite;
		logic MemToReg;
		logic RegWrite;
		logic Branch;
		logic Second_alu_en;			//Enable for second ALU 
		logic [1:0] func3; 				//2 Lsbs of IMM generated for Mul add
		logic [3:0] func2;				// might be needed to control DataMem
		logic [2:0] memctr;           // to control memory 
		logic [4:0] rd; 	// Register Write Address (Rd or Rt)
		logic [31:0] AluOut; 			// output of ALU 
		logic [31:0] ForwardB_data; 	// WriteData to DataMemory
		logic [31:0] dc; 				// for input to the second ALU
		logic [31:0] jumpAddrss;
		logic [4:0] rs3; 				// rs1 address replacing rt
		logic Jump;
	} ExMem;
	
	// MEM/Wb Register
	struct {
		logic MemToReg; 				// control signals coming in MEM/WB
		logic RegWrite;
		logic [31:0] AluOut; 			// output of ALU 
		logic [31:0] DataMemRead; 		// Data Memory Read Output
		logic [4:0] rd; 	// Register Write Address (Rd or Rt)
	} MemWb;
	
//FETCH Stage +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Instruction Memory Address
	assign icache_address = PC[`DRAM_ADDRESS_SIZE-1:0];     
	//icash valid req
	always@ (posedge clk)begin
   if (reset) begin 
	icache_valid <= 0;
   end else begin 
	icache_valid <= 1;
   end 
   end	
    
	// Instruction Memory Read Logic
        assign instmem_data[31:24] = icache_data_out[7:0];
	assign instmem_data[23:16] = icache_data_out[15:8];
	assign instmem_data[15:8] = icache_data_out[23:16];
	assign instmem_data[7:0] = icache_data_out[31:24]; 
	
	always@ (posedge clk)begin
	   if (reset) begin
			IfId.PCincremented <= 0;
			IfId.instruction <=  0;
		end 
		else if (pause_pipeline) begin
	   end
		else if (IfId_Write) begin 
			IfId.PCincremented <= PC+4;
			IfId.instruction <= (flush) ? 0 : instmem_data;
		end
	end

	assign Op = IfId.instruction[31:26]; // Send OpCode to Control Unit	
//FETCH END ======================================================================
	

	
//DECODE Stage +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Register File description
	logic [31:0] RF[31:0];			// Register File array
	logic [31:0] RF_WriteData; 	// write data
	
	// Register File Read Logic 
	assign da = (IfId.instruction[20:16]!=0) ? RF[IfId.instruction[20:16]] : 0; //rs1
	assign db = (IfId.instruction[15:11]!=0) ? RF[IfId.instruction[15:11]] : 0; //rs2
	assign dc = (IdEx.rs3!=0) ? RF[IdEx.rs3] : 0;	//rs3
	assign RF_WriteData = (MemWb.MemToReg) ? MemWb.DataMemRead : MemWb.AluOut; 

	
	assign stall = ((IdEx.MemRead && ((IdEx.rd==IfId.instruction[20:16]) || (IdEx.rd==IfId.instruction[15:11]))) || (IdEx.Second_alu_en &&((IdEx.rd==IfId.instruction[20:16]) || (IdEx.rd==IfId.instruction[15:11])|| (IdEx.rd==IfId.instruction[10:6]))));
	assign PCWrite = ~stall;				
	assign IfId_Write = ~stall;
	
	//func 3 field assignment from the input IMM
	assign aluctrl2[1:0] = IMM[5:4];
	
	always@ (posedge clk)begin	
	if (reset)begin
	   IdEx.MemRead 		<= 1'b0;    
		IdEx.MemWrite 		<= 1'b0;
		IdEx.MemToReg 		<= 1'b0;
		IdEx.ALUSrc 		<= 1'b0;
		IdEx.RegWrite 		<= 1'b0;
		IdEx.Branch 		<= 1'b0;
		IdEx.PCincremented  <= 0;
		IdEx.Second_alu_en  <= 1'b0;
		IdEx.da <= 0;
		IdEx.db <= 0;
		// IMM gen output
		IdEx.SignExtend  <=  32'b0;
		IdEx.rd <= 5'b0;
		IdEx.rs1 <= 5'b0;   		
		IdEx.rs2 <= 5'b0;			
		IdEx.func2 <= 4'b0;		
		IdEx.rs3 <= 5'b0;
		IdEx.memctr <= 3'b0;
		IdEx.Jump <= 0;
		// fused swj
		IdEx.swj <= 0;
		//forwarding rd
		IdEx.rdwb <= 0;
	end 
	else if (stall | flush) begin
	// Control signals coming to ID/EX
		IdEx.MemRead 		<=  1'b0 ;    
		IdEx.MemWrite 		<=  1'b0 ;
		IdEx.MemToReg 		<=  1'b0 ;
		IdEx.ALUSrc 		<=  1'b0 ;
		IdEx.RegWrite 		<=  1'b0 ;
		IdEx.Branch 		<=  1'b0 ;
		IdEx.PCincremented  <=  0 ;
		IdEx.Second_alu_en  <=  1'b0 ;
		IdEx.da <= 0 ;
		IdEx.db <=  0 ;
		
		// IMM gen output
		IdEx.SignExtend [31:0] <= 0;
		IdEx.rd <= 5'b0;
		IdEx.rs1 <=  5'b0;   		
		IdEx.rs2 <=  5'b0;			
		IdEx.func2 <=  4'b0;		
		IdEx.rs3 <= 5'b0;
		IdEx.memctr <= 3'b0;
		IdEx.Jump <= Jump;
		// fused swj
		IdEx.swj <= swj;
		//forwarding
		IdEx.rdwb <= ExMem.rd;
	end
	else if (pause_pipeline) begin
	end
	else begin
		// Control signals coming to ID/EX
		IdEx.MemRead 		<= MemRead;    
		IdEx.MemWrite 		<= MemWrite;
		IdEx.MemToReg 		<= MemToReg;
		IdEx.ALUSrc 		<=  ALUSrc;
		IdEx.RegWrite 		<=  RegWrite;
		IdEx.Branch 		<=  Branch;
		IdEx.PCincremented  <=  IfId.PCincremented;
		IdEx.Second_alu_en  <=  Second_alu_en;
		IdEx.da <= (ForwardC) ? MemWb.AluOut : da;
		IdEx.db <= (ForwardD) ? MemWb.AluOut : db;
		
		// IMM gen output
		IdEx.SignExtend [31:0] <=  IMM[31:0];
		IdEx.rd <=  IfId.instruction[25:21];
		IdEx.rs1 <= IfId.instruction[20:16];   		
		IdEx.rs2 <= IfId.instruction[15:11];			
		IdEx.func2 <= aluctr;		
		IdEx.rs3 <= IfId.instruction[10:6];
		IdEx.memctr <= memctrl;
		IdEx.Jump <= Jump;
		// fused swj
		IdEx.swj <= swj;
		//forwarding
		IdEx.rdwb <= ExMem.rd;
		end
		
	end

	
// +++++++++++++++++++++++++++
      logic [31:0] jumpAddrsss;
		assign jumpAddrsss = (IdEx.swj) ? IdEx.PCincremented+{{(16){IdEx.SignExtend[31]}},IdEx.SignExtend[31:16]}:IdEx.PCincremented + IdEx.SignExtend; //Jump address 
		
// DECODE END ===================================================================



// EXECUTE Stage ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//Forwarding MUXes
	always_comb
	begin
		casex(ForwardA)
		2'b01: ForwardA_data = RF_WriteData;
		2'b10: ForwardA_data = ExMem.AluOut;
		2'b11: ForwardA_data = da;
		default: ForwardA_data = IdEx.da;
		endcase
	end
  	always_comb
	begin
		casex(ForwardB)
		2'b01: ForwardB_data = RF_WriteData;
		2'b10: ForwardB_data = ExMem.AluOut;
		2'b11: ForwardB_data = db;
		default: ForwardB_data = IdEx.db;
		endcase
	end
	// Data 1 = AluIn 1
	assign OpA1 = ForwardA_data;
	
	// Input of ALU Control
	//assign Function = IdEx.SignExtend[5:0];
	
	// Changing the names to get rid of the dot, outputs from the datapath to other modules
	assign Instruction = IfId.instruction;
	
	assign IdExrs1 = IdEx.rs1;		
	assign IdExrs2 = IdEx.rs2;
	assign ExMemSecond_alu_en = ExMem.Second_alu_en;
	assign ExMemrs3 = ExMem.rs3;
	assign ExMemrd = ExMem.rd;
	assign MemWbrd = MemWb.rd;
	assign ExMemRegWrite = ExMem.RegWrite;	//
	assign MemWbRegWrite = MemWb.RegWrite;	//control inputs for forwarding
	assign rs11 = IfId.instruction[20:16];
	assign rs22 = IfId.instruction[15:11];
	///////////////////////////////////////
	
	// AluSrc Mux
	always_comb begin 
	if (IdEx.swj) begin 
	   OpB1 = {{(16){IdEx.SignExtend[15]}},IdEx.SignExtend[15:0]};
	end 
	else if(IdEx.ALUSrc) // If it is not R type  use Sign Extended 
		OpB1 = IdEx.SignExtend;
	else 
		OpB1 = ForwardB_data; // Else Data 2 = AluIn 1  
	end  
	
	assign ALUCtrl = IdEx.func2[3:0];
	
	//
	assign pause_pipeline = ((run_multiplier & ~multiplier_done) | cache_miss_stall) ? 1'b1:1'b0;
	//mult
	multiplier multiplier (
      .clk(clk), .reset(reset),
      .a(OpA1[15:0]),
      .b(OpB1[15:0]),
      .start(run_multiplier),
      .product(multiplier_out),
      .done(multiplier_done)
);


assign run_multiplier = (IdEx.func2[3:0] == 4'b1100);
	// ALU ONE-----------------------------------------------------------
	always_comb
	begin
		case(ALUCtrl)
		4'b0000: result = OpA1 + OpB1;   					//add
		4'b0001: result = OpA1 - OpB1;						//sub	for bne and beq as well, with two zero flags
		4'b0010: result = OpA1 << OpB1[4:0];				//sll
		4'b0011: result = OpA1 >> OpB1[4:0];				//srl
		4'b0100: result = $signed(OpA1) >>> OpB1[4:0]; //sra
		4'b0101: result = OpA1 | OpB1 ;						//or
		4'b0111: result = OpA1 ^ OpB1;						//xor
		4'b1000: result = OpA1 < OpB1 ? 1:0;				//slt so also for blt
		//4'b1100: result = OpA1[15:0] * OpB1[15:0];	               //Mul
		//4'b1110: result = OpA1 / OpB1;	               //DIv
		4'b1001: result =(OpA1 == OpB1) ? 1 : 0;       //Equal
		4'b1101: result =(OpA1 != OpB1) ? 1 : 0;       //not Equal
		4'b0110: result =(OpA1 >= OpB1) ? 1 : 0;       //Greater Than or equal 
		4'b1111: result = OpA1 & OpB1;						//and 
		4'b1100: result = multiplier_out;
		default: result = OpA1 + OpB1 ;
		endcase
	end
	// ALU ONE END---------------------------------------------------------
	
	always@ (posedge clk)begin
	if (reset) begin
			// Control signals from ID/EX Stage
		ExMem.RegWrite <= 0; 
		ExMem.MemRead <= 0;
		ExMem.MemWrite <= 0;
		ExMem.MemToReg <= 0;
		ExMem.Branch <= 0;
		ExMem.ForwardB_data <= 0;	//outout of AluSrc 
		
		ExMem.dc <= 0;
		ExMem.func2 <= 0;				
		ExMem.func3 <= 0;
		ExMem.Second_alu_en <= 0;
		
		// Branch Address
		ExMem.jumpAddrss <= 0;
		
		// ALUOut to ExMem
		ExMem.AluOut <= 0;
		ExMem.rd <= 0;
		ExMem.rs3 <=  0;
		ExMem.Jump <= 0;
		
		//mem control
		ExMem.memctr <= 0;
	end
	else if (pause_pipeline) begin
	end
	else begin
		// Control signals from ID/EX Stage
		ExMem.RegWrite <= IdEx.RegWrite; 
		ExMem.MemRead <= IdEx.MemRead;
		ExMem.MemWrite <= IdEx.MemWrite;
		ExMem.MemToReg <= IdEx.MemToReg;
		ExMem.Branch <= IdEx.Branch;
		ExMem.ForwardB_data <= ForwardB_data;	//outout of AluSrc 
		
		ExMem.dc <= dc;
		ExMem.func2 <= IdEx.func2;				
		ExMem.func3 <= aluctrl2;
		ExMem.Second_alu_en <= IdEx.Second_alu_en;
		
		// Branch Address
		ExMem.jumpAddrss <= jumpAddrsss;
		
		// ALUOut to ExMem
		ExMem.AluOut <= result;
		ExMem.rd <= IdEx.rd;
		ExMem.rs3 <=  IdEx.rs3;
		ExMem.Jump <= IdEx.Jump;
		
		//mem control
		ExMem.memctr <= IdEx.memctr;
		end
	end
		
	
// EXECUTE END =================================================================	


// MEM STAGE +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		// Data Memory Read Logic
		assign dcache_valid = ExMem.MemRead || ExMem.MemWrite;
		assign dcache_rw = (ExMem.MemWrite) ? 1'b1:1'b0;                                        // read 0 write 1
		assign dcache_address = ExMem.AluOut[`DRAM_ADDRESS_SIZE-1:0];
/* verilator lint_off LATCH */		
always_comb begin
//if (ExMem.MemRead) begin 
casex(ExMem.memctr) 
      3'b000: begin  //LW
		 datamem_data = dcache_data_out;
		 end
		 3'b001: begin  //LH
		 datamem_data[31:16] = 16'b0;
		 datamem_data[15:0]  = dcache_data_out[15:0];
		 end
		 3'b010: begin  //LB
		 datamem_data[31:8] = 24'b0;
		 datamem_data[7:0]   = dcache_data_out[7:0];
		 end
		 3'b011: begin  //LHU
		 datamem_data[31:16]  = dcache_data_out[31:16];
		 datamem_data[15:0] = 16'b0;
		 end
		 3'b100: begin  //LBU
		 datamem_data[31:24] = 8'b0;	
		 datamem_data[23:16]   = dcache_data_out[23:16];	 
		 datamem_data[15:0] = 16'b0;
		 end
		 3'b101: begin  //LBU
                 datamem_data = 32'bx;	
		 end
		 3'b110: begin  //LBU
	         datamem_data = 32'bx;	
		 end
		 3'b111: begin  //LBU
		 datamem_data = 32'bx;
		 end
		endcase
		end
//		end
//		end
/* verilator lint_off LATCH */
	// ALU TWO-----------------------------------------------------------	
	
	assign OpA2 = ExMem.AluOut;
	assign OpB2 = (ForwardE) ? RF_WriteData : ExMem.dc;
	
	always_comb begin 
	if (ExMem.Second_alu_en) begin
		case(ExMem.func3)
		2'b00: ALUResult2 = OpA2 + OpB2; 			//add
		2'b01: ALUResult2 = OpA2 - OpB2; 			//sub
		2'b10: ALUResult2 = OpA2 >> OpB2[4:0]; 		//srl
		2'b11: ALUResult2 = OpA2 << OpB2[4:0]; 		//sll
		default: ALUResult2 = OpA2 + 0;
		endcase
	end
	else ALUResult2 = OpA2;
	end
	
	// ALU TWO END==================================================
		
	always@ (posedge clk)begin
	if (reset) begin
	// Control Signals
		MemWb.RegWrite <= 0; 
		MemWb.MemToReg <= 0;
		MemWb.AluOut <= 0;		//output from the mux of Alu2or1 to memwb
		MemWb.rd <= 0;
		MemWb.DataMemRead <= 0;
	end
	else if (pause_pipeline) begin
	end
	else begin
		// Control Signals
		MemWb.RegWrite <= ExMem.RegWrite; 
		MemWb.MemToReg <= ExMem.MemToReg;
		MemWb.AluOut <= ALUResult2;		//output from the mux of Alu2or1 to memwb
		MemWb.rd <= ExMem.rd;
		MemWb.DataMemRead <= datamem_data;
		end
		end
		//**********************************************//
		// Data Memory Write Data
		always @(posedge clk) begin
		if (ExMem.MemWrite && !pause_pipeline) begin
			dcache_data_in <= ExMem.ForwardB_data;
		end
		
	end
	
		//byte enable cache
		always_comb begin
		if(ExMem.memctr == 3'b000) dcache_byte_en = 4'b1111;
		else if(ExMem.memctr == 3'b001) dcache_byte_en = 4'b0011;
		else if (ExMem.memctr == 3'b010) dcache_byte_en = 4'b0001;
		else dcache_byte_en = 4'b0000;
		end


	// Data Memory Write Address
	
// MEM STAGE END ============================================================

	
	
// WRITE BACK STAGE +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
	// Register File Write Logic 
		always @(posedge clk) begin 	
		int i;		
		if(reset)begin
			for (i = 0; i < 32; i = i + 1) 
				 RF[i] <= 0;
		end
		else begin
			if (MemWb.RegWrite)  			
				RF[MemWb.rd] <= RF_WriteData; //MemtoReg Mux
		end	
		end
		
// WRITE BACK END ===========================================================
	
		
 assign PCsrc = ExMem.Jump | (ExMem.Branch && ExMem.AluOut[0]);

// PC LOGIC +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	always@ (posedge clk)begin			
		if(reset)
			PC <= PCSTART;
	   else if (pause_pipeline) begin
	   end
		else
		if (PCWrite) begin
			case(PCsrc)
				1'b0: PC <= PC+4;
				1'b1: PC <= ExMem.jumpAddrss; 
			endcase
		end
	end
	
// PC LOGIC END =============================================================
	
	
endmodule // datapath
/* verilator lint_on UNPACKED */
/* verilator lint_on CASEX */
/* verilator lint_on CASEINCOMPLETE */
