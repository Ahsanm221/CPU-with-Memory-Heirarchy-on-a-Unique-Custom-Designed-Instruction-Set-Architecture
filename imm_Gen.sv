module imm_Gen(
		input logic [31:0] Instruction,
		output logic [31:0] IMM
		);


//made a new module for sign extension, since might need additional functionality
// later on when implementing new instructions	

always_comb 
	case (Instruction[31:26])	
		6'b010011:	//Immediate instruction format
		IMM = {Instruction[15]? 20'hFFFFF:20'b0 , Instruction[15:4]};
		6'b000011:	//Load Word 
		IMM = {Instruction[15]? 20'hFFFFF:20'b0 , Instruction[15:4]};
		6'b100011: 	//Store Word
		IMM = {Instruction[25]? 20'hFFFFF:20'b0 , Instruction[25:21], Instruction[10:4]};
		6'b000100:	//Branch Opcode might be changed later
		IMM = {Instruction[25]? 20'hFFFFF:20'b0 , Instruction[25:21], Instruction[10:4]};
		6'b000111:  //Multiply and addition
		IMM = {{(26){1'b0}},Instruction[5:0]};
		6'b000010:  //Jump
		IMM = {{(6){Instruction[25]}},Instruction[25:0]};
		6'b111001: //FSJ
		IMM = {{(8){Instruction[25]}},Instruction[25:21],Instruction[10:8],{(8){Instruction[7]}},Instruction[7:0]};
		///Might need to add, half word, or byte load store, for now controlling
		// with funct2
		default: IMM = {32'b0};
	endcase
endmodule