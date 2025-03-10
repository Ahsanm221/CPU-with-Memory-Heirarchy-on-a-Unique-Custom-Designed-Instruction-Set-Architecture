// This memory core has been developed by Ali Muhtaroglu for education 
// in support of Computer Architecture / Organization courses at METU 
// Northern Cyprus Campus.

`include "./config.sv"
`include "./constants.sv"

module dram_array (
		     
		   input [`DRAM_ADDRESS_SIZE-1:0]   address,
		   input 			    clk,
		   inout wire [`DRAM_WORD_SIZE-1:0] data,
		   input 			    wren
);

   logic [7:0] 			  mem[0:2**(`DRAM_ADDRESS_SIZE)-1];
   logic [`DRAM_WORD_SIZE-1:0] 	  din;

   assign din = data;   

   /* verilator lint_off WIDTH */
   genvar i;
   generate
      for (i=0; i < `DRAM_WORD_SIZE/8; i++) begin
	 assign data[(i+1)*8-1:i*8] = (wren==0) ? mem[address+i] : (wren==1) ? `DRAM_WORD_SIZE'bz : `DRAM_WORD_SIZE'bx;
      end
   endgenerate
   /* verilator lint_on WIDTH */
   
    /* verilator lint_off WIDTH */
//   assign data = (wren==0) ? mem[address] : (wren==1) ? `DRAM_WORD_SIZE'bz : `DRAM_WORD_SIZE'bx;

   genvar j;
   generate
      for (j=0; j < `DRAM_WORD_SIZE/8; j++)  begin
	 always_ff @(posedge clk)
	   if (wren) begin
	      mem[address+j] <= din[(j+1)*8-1:j*8];
	   end
      end
   endgenerate
   /* verilator lint_on WIDTH */
//initial $readmemh("instruction_memory.dat", mem);
`ifdef DRAM_HEX
   initial $readmemh(`DRAM_HEX, mem);
`endif

endmodule

