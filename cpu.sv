`include "./config.sv"
`include "./constants.sv"

module cpu(input logic clk,
               input logic 	   reset,
	       input logic 	   cache_miss_stall,
	       output logic [`DRAM_ADDRESS_SIZE-1:0]  icache_address,
	       output logic 	   icache_valid,
	       input logic [31:0]  icache_data_out,
	       input logic [31:0]  dcache_data_out, 
	       output logic [`DRAM_ADDRESS_SIZE-1:0] dcache_address, 
	       output logic [31:0] dcache_data_in, 
	       output logic [3:0]  dcache_byte_en, 
	       output logic 	   dcache_rw, 
	       output logic 	   dcache_valid,
	       output logic 	   regwritememwb);

	logic [5:0] Op;
	logic MemRead;
	logic MemWrite;
	logic MemToReg;
	logic ALUSrc;
	logic RegWrite;
	logic Branch;
	logic Jump;
	logic ExMemRegWrite; 
	logic MemWbRegWrite;
	logic Second_alu_en;
	logic Alu2or1;
	logic [4:0] IdExrs2;
	logic [4:0] IdExrs1;
	logic [4:0] ExMemrs3;
	logic [4:0] ExMemrd;
	logic [4:0] MemWbrd;
	logic ExMemSecond_alu_en;
	logic [1:0] ForwardA;
	logic [1:0]	ForwardB;
	logic ForwardE;
	logic [31:0] Instruction;
	logic [31:0]	IMM;
	logic swj;
	logic [4:0] rs11;
	logic [4:0] rs22;
	logic ForwardC;
	logic ForwardD;
	assign regwritememwb =MemWbRegWrite;


	control u0(.Op(Op), .MemRead(MemRead), .MemWrite(MemWrite), .MemToReg(MemToReg), .ALUSrc(ALUSrc), 
				.RegWrite(RegWrite), .Branch(Branch), .Jump(Jump), 
				.Second_alu_en(Second_alu_en), .swj(swj));

	datapath u1(.clk(clk), .reset(reset), .Op(Op), .MemRead(MemRead), .MemWrite(MemWrite), .MemToReg(MemToReg), 
				 .ALUSrc(ALUSrc), .RegWrite(RegWrite), .Branch(Branch), .Jump(Jump),  
				 .ExMemRegWrite(ExMemRegWrite), .MemWbRegWrite(MemWbRegWrite), .IdExrs1(IdExrs1), 
				 .IdExrs2(IdExrs2), .ExMemrs3(ExMemrs3), .ExMemrd(ExMemrd), .MemWbrd(MemWbrd),.ExMemSecond_alu_en(ExMemSecond_alu_en), .ForwardA(ForwardA), .ForwardB(ForwardB),
				 .Second_alu_en(Second_alu_en), .Instruction(Instruction), .IMM(IMM), .ForwardE(ForwardE), .swj(swj), .ForwardC(ForwardC), .ForwardD(ForwardD),
				 .rs11(rs11), .rs22(rs22), .icache_data_out(icache_data_out), .cache_miss_stall(cache_miss_stall), .icache_address(icache_address), .icache_valid(icache_valid), .dcache_data_out(dcache_data_out),
                                 .dcache_address(dcache_address), .dcache_data_in(dcache_data_in), .dcache_byte_en(dcache_byte_en), .dcache_rw(dcache_rw), .dcache_valid(dcache_valid));
	
	
	forwarding u3(.ExMemRegWrite(ExMemRegWrite), .MemWbRegWrite(MemWbRegWrite), .IdExrs1(IdExrs1), .IdExrs2(IdExrs2), .ExMemrs3(ExMemrs3),
						.ExMemrd(ExMemrd),  .MemWbrd(MemWbrd), .ExMemSecond_alu_en(ExMemSecond_alu_en),  .ForwardA(ForwardA), .ForwardB(ForwardB), .ForwardE(ForwardE) , .rs11(rs11), .rs22(rs22),
						.ForwardC(ForwardC), .ForwardD(ForwardD));
	
	imm_Gen u4(.Instruction(Instruction), .IMM(IMM));

endmodule
