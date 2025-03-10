module control (input logic [5:0] Op,
					output logic MemRead, MemWrite, MemToReg, ALUSrc, swj,
					output logic RegWrite, Branch, Jump, Second_alu_en
				);

	always_comb begin 
	//Init Signals
				MemRead = 1'b0;  
				MemWrite = 1'b0;  
				MemToReg = 1'b0;  
				ALUSrc = 1'b0;  
	    		RegWrite = 1'b0;  
			   Branch = 1'b0;  
				Jump = 1'b0; 
				swj = 1'b0;
				Second_alu_en = 1'b0;
                  	
      case(Op)   
      6'b110011: begin // R-type 
                  RegWrite = 1'b1; 
						end 
	  6'b010011: begin // I-type
						ALUSrc = 1'b1;
                  RegWrite = 1'b1;  					
                  end  	
      6'b000011: begin // lw  
						//RegDst = 1'b0;
						ALUSrc = 1'b1; 
						MemToReg = 1'b1; 
						RegWrite = 1'b1; 
						MemRead = 1'b1;  
						end  
      6'b100011: begin // sw  
						ALUSrc = 1'b1;  
                  MemWrite = 1'b1;  
                  end 
      6'b000100: begin // B-type 
						Branch = 1'b1;     
                  end 
	  6'b000111: begin //fused mul and add
						RegWrite = 1'b1;
						Second_alu_en = 1'b1;
						end
		6'b111001: begin	//fused store and jump
		            swj = 1;
						ALUSrc = 1'b1;  		
                  MemWrite = 1'b1;  
						Jump = 1'b1;
						end
      6'b000010: begin // jump
						Jump = 1'b1;     				  
                 end 
					  
		default: begin 
					  MemRead = 1'b0;  
					  MemWrite = 1'b0;  
					  MemToReg = 1'b0;  
					  ALUSrc = 1'b0;  
					  RegWrite = 1'b0;    
					  Branch = 1'b0;  
					  Jump = 1'b0;  
					  Second_alu_en = 1'b0;

					end
						
      endcase
    end
endmodule
