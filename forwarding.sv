module forwarding(
				 input logic ExMemRegWrite, MemWbRegWrite,
				 input logic [4:0] IdExrs1,
				 input logic [4:0] IdExrs2,
				 input logic [4:0] ExMemrs3,
				 input logic [4:0] ExMemrd,
				 input logic [4:0] MemWbrd,
				 input logic ExMemSecond_alu_en,
				 input logic [4:0] rs11,
				 input logic [4:0] rs22,
				 output logic [1:0] ForwardA,
				 output logic ForwardE,
				 output logic ForwardC,
				 output logic ForwardD,
				 output logic [1:0]	ForwardB
				 );

			assign ForwardA = ((ExMemRegWrite) && (ExMemrd != 0) && (ExMemrd == IdExrs1)) ? 2'b10 :
									((MemWbRegWrite) && (MemWbrd != 0) && (MemWbrd == IdExrs1)) ? 2'b01 : 2'b00;

			assign ForwardB = ((ExMemRegWrite) && (ExMemrd != 0) && (ExMemrd == IdExrs2)) ? 2'b10 :
									((MemWbRegWrite) && (MemWbrd != 0) && (MemWbrd == IdExrs2)) ? 2'b01 : 2'b00;			
			assign ForwardE = ((MemWbRegWrite) && (ExMemSecond_alu_en) && (MemWbrd != 0) && (MemWbrd == ExMemrs3)) ? 1'b1 : 1'b0;	
			
			assign ForwardC = ((MemWbRegWrite) && (MemWbrd != 0) && (MemWbrd == rs11)) ? 1'b1 : 1'b0;	
			assign ForwardD = ((MemWbRegWrite) && (MemWbrd != 0) && (MemWbrd == rs22)) ? 1'b1 : 1'b0;	
			
endmodule
		
