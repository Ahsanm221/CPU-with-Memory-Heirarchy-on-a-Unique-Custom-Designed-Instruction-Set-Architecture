// This is a direct-mapped write-allocate, write-back data cache controller fsm, 
// developed by Ali Muhtaroglu in support of education for Computer Architecture / Organization courses 
// at METU Northern Cyprus Campus.

`include "./config.sv"
`include "./constants.sv"



module dcache_controller (input  clk, reset,
			  input [`DRAM_ADDRESS_SIZE-1:0]  dcache_address, // cpu request address (CPU->cache)
			  input [`DRAM_WORD_SIZE-1:0] 	  dcache_data_in, // cpu request data (CPU->cache)
			  input [`DRAM_WORD_SIZE/8-1:0]   dcache_byte_en, // cpu request byte enable (CPU->cache)
			  input 			  dcache_rw, // cpu R/W request (CPU->cache)
			  input 			  dcache_valid, // cpu request valid (CPU->cache)
			  input [`DRAM_WORD_SIZE-1:0] 	  mem_data_in[`DRAM_BLOCK_SIZE-1:0], // memory read data (memory->cache)
			  input 			  mem_ready, // memory read data ready (memory->cache)    
			  output [`DRAM_ADDRESS_SIZE-1:0] mem_address, // cache request address (cache->memory)
			  output [`DRAM_WORD_SIZE-1:0] 	  mem_data_out[`DRAM_BLOCK_SIZE-1:0], // memory write data (cache->memory)
			  output 			  mem_rw, // R/W request to memory (cache->memory)
			  output 			  mem_valid, // request to memory valid (cache->memory)
			  output [`DRAM_WORD_SIZE-1:0] 	  dcache_data_out, // data to CPU (cache->CPU)
			  output 			  dcache_data_ready // data to CPU ready (cache->CPU)
			  );

   // log2 function quite useful for parametric design
   function automatic int log2 (input int n);
      if (n <=1) return 1; // abort function
      log2 = 0;
      while (n > 1) begin
	 n = n/2;
	 log2++;
      end
   endfunction; // log2

   // one-hot state assignment, and definition of next state and current state variables
   enum logic [2:0] {compare_tag=3'b001, allocate=3'b010, write_back= 3'b100} nstate, cstate;

   // interface signals to tag memory
   logic 	             dcache_valid_read;
   logic 	             dcache_dirty_read;
   logic [`DTAGMSB:`DTAGLSB] dcache_tag_read; 
   logic                     dcache_valid_write;
   logic                     dcache_dirty_write;
   logic [`DTAGMSB:`DTAGLSB] dcache_tag_write;   
   logic [`DCACHE_INDEX-1:0] dcache_tag_index;
   logic 		     dcache_tag_we;		     
   
   // interface signals to cache sram memory
   logic [`DRAM_WORD_SIZE-1:0] dcache_sram_read[`DRAM_BLOCK_SIZE-1:0];
   logic [`DRAM_WORD_SIZE-1:0] dcache_sram_write[`DRAM_BLOCK_SIZE-1:0];
   logic [`DCACHE_INDEX-1:0]   dcache_sram_index;
   logic                       dcache_sram_we;
   
   // temporary variables for cache controller output to CPU
   logic [`DRAM_WORD_SIZE-1:0] dcache_data_to_cpu;
   logic                       dcache_data_to_cpu_ready;			 
   
   // temporary variables for memory requests
   logic [`DRAM_ADDRESS_SIZE-1:0] mem_request_address;
   logic [`DRAM_WORD_SIZE-1:0]    mem_request_data[`DRAM_BLOCK_SIZE-1:0];
   logic 			  mem_request_rw;
   logic                          mem_request_valid;

   // temporary variables for writing bytes to cache
   logic  [`DRAM_WORD_SIZE-1:0]   dcache_sram_write_temp;
   
  
   assign mem_data_out = mem_request_data;
   assign mem_address = mem_request_address;
   assign mem_rw = mem_request_rw;
   assign mem_valid = mem_request_valid;  
   assign dcache_data_out = dcache_data_to_cpu;
   assign dcache_data_ready = dcache_data_to_cpu_ready;

   genvar i;
   generate
      for (i=0; i < `DRAM_WORD_SIZE/8; i++) begin
	 assign dcache_sram_write_temp[(i+1)*8-1:i*8] = (dcache_byte_en[i]==1) ? dcache_data_in[(i+1)*8-1:i*8] : dcache_sram_read[dcache_address[log2(`DRAM_BLOCK_SIZE)+1:2]][(i+1)*8-1:i*8];
      end
   endgenerate

   always_comb begin
   
   // default settings for all signals
      nstate = cstate;
      dcache_data_to_cpu = '{default:0};
      dcache_data_to_cpu_ready = '0;
      dcache_valid_write = '0;
      dcache_dirty_write = '0;
      dcache_tag_write = '{default:0};
      mem_request_valid = '0; 
      dcache_tag_we = '0;   

      // direct mapped cache index for cache tag memory and cache sram portion
      dcache_tag_index = dcache_address[log2(`DRAM_BLOCK_SIZE)+1+`DCACHE_INDEX:log2(`DRAM_BLOCK_SIZE)+2];
      dcache_sram_index = dcache_address[log2(`DRAM_BLOCK_SIZE)+1+`DCACHE_INDEX:log2(`DRAM_BLOCK_SIZE)+2];
      
      // default is reading cache line
      dcache_sram_we = '0;
      
      // access only the correct word and byte of the cache sram block when writing
      dcache_sram_write = dcache_sram_read;
      dcache_sram_write[dcache_address[log2(`DRAM_BLOCK_SIZE)+1:2]] = dcache_sram_write_temp;
      
      // access only the correct word of the cache sram block when reading
      dcache_data_to_cpu = dcache_sram_read[dcache_address[log2(`DRAM_BLOCK_SIZE)+1:2]];

      // main memory request address is a copy of cpu request address to cache
      mem_request_address = dcache_address;

      // when requesting write to main memory use cache sram content
      mem_request_data = dcache_sram_read;
      mem_request_rw = '0;

      // Cache FSM with four states:
      
      case(cstate)
	// compare tag state
	compare_tag : begin
	   /*If there is a CPU request, then compare cache tag*/
           /*cache hit (tag match and cache entry is valid)*/
	   if (dcache_valid && (dcache_address[`DTAGMSB:`DTAGLSB] == dcache_tag_read) && dcache_valid_read) begin
              dcache_data_to_cpu_ready = '1;
	      
              /*write hit*/
              if (dcache_rw) begin  
		 /*read/modify cache line*/
		 dcache_tag_we = '1; dcache_sram_we = '1;
		 /*no change in tag*/
		 dcache_tag_write =  dcache_tag_read; 
		 dcache_valid_write = '1;
		 /*cache line is dirty*/
		 dcache_dirty_write = '1;       
              end 
              /*ﬁnished*/
              nstate = compare_tag; 
	   end 
	   /*cache miss*/
	   else if (dcache_valid) begin 
              /*generate new tag*/
              dcache_tag_we = '1; 
              dcache_valid_write = '1;
              /*new tag*/
              dcache_tag_write = dcache_address[`DTAGMSB:`DTAGLSB];
              /*cache line is dirty if write*/
              dcache_dirty_write = dcache_rw;
	   
              /*generate memory request on miss*/
              mem_request_valid = '1; 
              /*compulsory miss or miss with clean block*/
              if (dcache_valid_read == 1'b0 || dcache_dirty_read == 1'b0)
		/*wait till a new block is allocated*/
		nstate = allocate;
	      else begin
		 /*miss with dirty line*/
		 /*write back address*/
		 mem_request_address = {dcache_tag_read, dcache_address[`DTAGLSB-1:0]};
		 mem_request_rw = '1;  
		 /*wait till write is completed*/
		 nstate = write_back;
	      end
	   end 
	end
	// state for allocating a new cache line before proceeding
	allocate: begin       
	   /*memory controller has responded*/
	   if (mem_ready) begin
              /*re-compare tag for write miss (need modify correct word)*/
              nstate = compare_tag; 
              dcache_sram_write = mem_data_in;
              /*update cache line data*/
              dcache_sram_we = '1;  
	   end 
	end
	// state writing back a dirty cache line before proceeding
	write_back : begin     
	   /*write back is completed*/
	   if (mem_ready) begin
           /*issue new memory request (allocating a new line)*/
              mem_request_valid = '1;    
              mem_request_rw = '0;              
              nstate = allocate; 
	   end
	end
	default: nstate = cstate;
      endcase
   end
   always_ff @(posedge(clk)) begin
      if (reset) 
	cstate <= compare_tag;  //reset to compare tag
      else 
	cstate <= nstate;
   end

   // data cache tag ram and sram instantiations:

   data_cache_tag data_cache_tag (
        .clk        (clk),
	.we           (dcache_tag_we),
        .index        (dcache_tag_index),
	.valid_in     (dcache_valid_write),
        .dirty_in     (dcache_dirty_write),
	.tag_in       (dcache_tag_write),
	.valid_out    (dcache_valid_read),
	.dirty_out    (dcache_dirty_read),
	.tag_out      (dcache_tag_read)
   );

   data_cache_sram data_cache_sram (
        .clk        (clk),
	.we           (dcache_sram_we),
        .index        (dcache_sram_index),
	.data_write   (dcache_sram_write),
        .data_read    (dcache_sram_read)
   );
   
endmodule
