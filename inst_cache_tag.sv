// This is the tag-ram unit for direct-mapped instruction cache developed by 
// Ali Muhtaroglu in support of education for Computer Architecture / Organization 
// courses at METU Northern Cyprus Campus.

`include "./config.sv"
`include "./constants.sv"

module inst_cache_tag(input  clk, //write clock
		      input 			 we, // tag request/command, e.g. RW
		      input [`ICACHE_INDEX-1:0]  index, // cache index
                      input 			 valid_in,
                      input [`ITAGMSB:`ITAGLSB]  tag_in, // input tag bits			
		      output 			 valid_out,
                      output [`ITAGMSB:`ITAGLSB] tag_out);  // output tag bits     

   bit icache_valid[`ICACHE_SIZE-1:0];
   bit [`ITAGMSB:`ITAGLSB] icache_tag[`ICACHE_SIZE-1:0];   

   initial begin
      icache_valid = '{default:0};  // initialize all valid bits in cache to 0
      icache_tag = '{default:0};    // initialize all tag bits in cache to 0       
   end
   
   assign tag_out = icache_tag[index];
   assign valid_out = icache_valid[index];
   
   always_ff @(posedge(clk)) begin
      if (we) begin
	 icache_tag[index] <= tag_in;
	 icache_valid[index] <= valid_in;
      end
   end
   
endmodule
