// This is the sram unit for direct-mapped instruction cache developed by 
// Ali Muhtaroglu in support of education for Computer Architecture / Organization 
// courses at METU Northern Cyprus Campus.

`include "./config.sv"
`include "./constants.sv"

module inst_cache_sram(input  clk,
		       input 			    we, // data request/command, e.g. RW
		       input [`ICACHE_INDEX-1:0]    index, // cache index
		       input [`DRAM_WORD_SIZE-1:0]  data_write[`DRAM_BLOCK_SIZE-1:0], //write port		       
		       output [`DRAM_WORD_SIZE-1:0] data_read[`DRAM_BLOCK_SIZE-1:0]); //read port

   bit [`DRAM_WORD_SIZE-1:0] icache_sram[`ICACHE_SIZE-1:0][`DRAM_BLOCK_SIZE-1:0];
   
   initial begin
      for (int i=0; i<`ICACHE_SIZE; i++) 
	icache_sram[i] = '{default:0};  // initialize all bits in cache to 0
   end
   
   assign data_read = icache_sram[index];
   
   always_ff @(posedge(clk)) begin
      if (we)
	icache_sram[index] <= data_write;
   end
   
endmodule

