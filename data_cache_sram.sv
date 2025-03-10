// This is the data sram unit for direct-mapped write-allocate, write-back data cache, 
// developed by Ali Muhtaroglu in support of education for Computer Architecture / Organization courses 
// at METU Northern Cyprus Campus.

`include "./config.sv"
`include "./constants.sv"

module data_cache_sram(input  clk,
		       input 			    we, // data request/command, e.g. RW
		       input [`DCACHE_INDEX-1:0]    index, // cache index
		       input [`DRAM_WORD_SIZE-1:0]  data_write[`DRAM_BLOCK_SIZE-1:0], //write port		       
		       output [`DRAM_WORD_SIZE-1:0] data_read[`DRAM_BLOCK_SIZE-1:0]); //read port

   bit [`DRAM_WORD_SIZE-1:0] dcache_sram[`DCACHE_SIZE-1:0][`DRAM_BLOCK_SIZE-1:0];
   
   initial begin
      for (int i=0; i<`DCACHE_SIZE; i++) 
	dcache_sram[i] = '{default:0};  // initialize all bits in cache to 0
   end
   
   assign data_read = dcache_sram[index];
   
   always_ff @(posedge(clk)) begin
      if (we)
	dcache_sram[index] <= data_write;
   end
   
endmodule

